#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

import os
import base64
import binascii
import json
import shutil
import subprocess
import tarfile
import tempfile
import zipfile
import hashlib
from typing import Any, Dict
from urllib.parse import urlparse

import requests
import rclpy
from rclpy.node import Node
from muto_msgs.msg import StackManifest
from muto_msgs.srv import ProvisionPlugin

WORKSPACES_PATH = os.path.join("/tmp", "muto", "muto_workspaces")
ARTIFACT_STATE_FILE = ".muto_artifact.json"
from composer.utils.stack_parser import StackParser


class MutoProvisionPlugin(Node):
    """Plugin for setting up the workspace (decode, extract, build, install dependencies, etc.)"""

    def __init__(self):
        super().__init__("provision_plugin")

        self.current_stack: StackManifest | None = None
        self.is_up_to_date = False
        self.stack_parser = StackParser(self.get_logger())

        self.declare_parameter("ignored_packages", [""])
        self.ignored_packages = [
            pkg
            for pkg in self.get_parameter("ignored_packages")
            .get_parameter_value()
            .string_array_value
            if pkg
        ]

        # self.create_subscription(StackManifest, "composed_stack", self.get_stack, 10)
        self.provision_srv = self.create_service(
            ProvisionPlugin, "muto_provision", self.handle_provision
        )

    def get_stack(self, stack_msg: StackManifest):
        """Callback to receive the composed stack."""
        self.current_stack = stack_msg

    def _load_stack_manifest(self) -> Dict[str, Any]:
        """Return the current stack manifest as a dictionary if available."""
        if not self.current_stack:
            return {}

        raw_stack = getattr(self.current_stack, "stack", "")
        if not raw_stack:
            return {}

        if isinstance(raw_stack, bytes):
            # raw_stack = raw_stack.decode("utf-8")
            base64.b64decode(raw_stack)

        try:
            return json.loads(raw_stack)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().debug("Unable to decode stack manifest for provisioning.")
            return {}

    def _artifact_state_path(self, workspace_dir: str) -> str:
        return os.path.join(workspace_dir, ARTIFACT_STATE_FILE)

    def _load_artifact_state(self, workspace_dir: str) -> Dict[str, Any]:
        state_path = self._artifact_state_path(workspace_dir)
        if not os.path.exists(state_path):
            return {}
        try:
            with open(state_path, "r", encoding="utf-8") as state_file:
                return json.load(state_file)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                f"Failed to read artifact state from {state_path}: {exc}"
            )
            return {}

    def _write_artifact_state(self, workspace_dir: str, state: Dict[str, Any]) -> None:
        state_path = self._artifact_state_path(workspace_dir)
        try:
            with open(state_path, "w", encoding="utf-8") as state_file:
                json.dump(state, state_file)
        except OSError as exc:
            self.get_logger().warning(
                f"Failed to persist artifact state to {state_path}: {exc}"
            )

    def _reset_workspace(self, workspace_dir: str) -> None:
        if os.path.isdir(workspace_dir):
            shutil.rmtree(workspace_dir, ignore_errors=True)
        os.makedirs(workspace_dir, exist_ok=True)

    def _prepare_archive(self, manifest: Dict[str, Any], temp_dir: str) -> tuple[str, Dict[str, Any]]:
        artifact = manifest.get("launch", {})
        data_b64 = artifact.get("data")
        url = artifact.get("url")
        props = artifact.get("properties", {})
        metadata = manifest.get("metadata", {})

        if not data_b64 and not url:
            raise ValueError("Artifact specification must include either 'data' or 'url'.")

        if data_b64:
            filename = props.get("filename") or metadata.get("name") or "artifact.tar"
            archive_path = os.path.join(temp_dir, filename)
            try:
                decoded_bytes = base64.b64decode(data_b64, validate=True)
            except (binascii.Error, ValueError) as exc:
                raise ValueError(f"Invalid base64 data for artifact: {exc}") from exc

            with open(archive_path, "wb") as file_handle:
                file_handle.write(decoded_bytes)

            checksum = props.get("checksum")
            algorithm = props.get("algorithm", "sha256")
            if checksum:
                self._verify_checksum(archive_path, checksum, algorithm)

            digest = hashlib.sha256(decoded_bytes).hexdigest()
            metadata.update({
                "source": "inline",
                "data_sha256": digest,
            })

            return archive_path, metadata

        # URL-based artifact
        parsed = urlparse(url)
        filename = os.path.basename(parsed.path) or "artifact"
        archive_path = os.path.join(temp_dir, filename)

        if parsed.scheme in ("http", "https"):
            headers = dict(artifact.get("headers") or {})
            headers.setdefault("Accept", "application/octet-stream")
            headers.setdefault("User-Agent", "curl/7.88.1")
            timeout = artifact.get("timeout", 60)
            self.get_logger().info(f"Downloading artifact from {url}")
            try:
                with requests.get(
                    url,
                    stream=True,
                    headers=headers,
                    timeout=timeout,
                    allow_redirects=True,
                ) as response:
                    try:
                        response.raise_for_status()
                    except requests.HTTPError as exc:
                        if response.status_code == 404 and headers.get("Accept") == "application/octet-stream":
                            self.get_logger().warning(
                                "404 when requesting archive with Accept=application/octet-stream; retrying with '*/*'."
                            )
                            headers["Accept"] = "*/*"
                            response.close()
                            with requests.get(
                                url,
                                stream=True,
                                headers=headers,
                                timeout=timeout,
                                allow_redirects=True,
                            ) as retry_response:
                                retry_response.raise_for_status()
                                with open(archive_path, "wb") as file_handle:
                                    for chunk in retry_response.iter_content(chunk_size=8192):
                                        if chunk:
                                            file_handle.write(chunk)
                            metadata.update({"source": "url", "url": url})
                            return archive_path, metadata
                        else:
                            raise

                    with open(archive_path, "wb") as file_handle:
                        for chunk in response.iter_content(chunk_size=8192):
                            if chunk:
                                file_handle.write(chunk)
            except requests.RequestException as exc:
                raise RuntimeError(f"Failed to download artifact from {url}: {exc}")
        elif parsed.scheme in ("", "file"):
            source_path = url[7:] if parsed.scheme == "file" else url
            shutil.copy(source_path, archive_path)
        else:
            raise ValueError(f"Unsupported artifact URL scheme: {parsed.scheme}")

        checksum = artifact.get("checksum")
        if checksum:
            algorithm = artifact.get("algorithm", "sha256")
            self._verify_checksum(archive_path, checksum, algorithm)
            metadata["checksum"] = checksum
            metadata["algorithm"] = algorithm.lower()

        metadata.setdefault("source", "url")
        metadata.setdefault("url", url)

        return archive_path, metadata

    def _verify_checksum(self, file_path: str, expected: str, algorithm: str) -> None:
        algo = algorithm.lower()
        try:
            digest = hashlib.new(algo)
        except ValueError as exc:
            raise ValueError(f"Unsupported checksum algorithm '{algorithm}'") from exc

        try:
            with open(file_path, "rb") as file_handle:
                for chunk in iter(lambda: file_handle.read(8192), b""):
                    digest.update(chunk)
        except OSError as exc:
            raise RuntimeError(f"Failed to read archive for checksum verification: {exc}")

        actual = digest.hexdigest()
        if actual.lower() != expected.lower():
            raise ValueError(
                f"Checksum mismatch for {file_path}: expected {expected}, got {actual}"
            )

    def _extract_archive(self, archive_path: str, destination: str) -> None:
        if tarfile.is_tarfile(archive_path):
            with tarfile.open(archive_path, "r:*") as tar:
                tar.extractall(destination)
        elif zipfile.is_zipfile(archive_path):
            with zipfile.ZipFile(archive_path, "r") as zipped:
                zipped.extractall(destination)
        else:
            raise ValueError(f"Unsupported archive format for {archive_path}")

    def _flatten_single_directory(self, workspace_dir: str) -> None:
        entries = [entry for entry in os.listdir(workspace_dir)]
        if len(entries) != 1:
            return
        sole_entry = os.path.join(workspace_dir, entries[0])
        if not os.path.isdir(sole_entry):
            return
        for item in os.listdir(sole_entry):
            shutil.move(os.path.join(sole_entry, item), workspace_dir)
        shutil.rmtree(sole_entry, ignore_errors=True)

    def _move_contents(self, source_dir: str, destination_dir: str) -> None:
        for item in os.listdir(source_dir):
            shutil.move(os.path.join(source_dir, item), destination_dir)
        shutil.rmtree(source_dir, ignore_errors=True)

    def from_git(self, repo_url: str, branch: str = "main") -> None:
        """
        Clone or update a git repository, including its submodules.

        Args:
            repo_url (str): The URL of the git repository.
            branch (str): The branch to check out.
        """
        if not self.current_stack or not isinstance(self.current_stack, dict):
            self.get_logger().error("No valid current stack available.")
            return

        stack_name = self.current_stack.get("name", "default")
        target_dir = os.path.join(
            WORKSPACES_PATH, stack_name.replace(" ", "_")
        )

        if os.path.exists(os.path.join(target_dir, ".git")):
            self.update_repository(target_dir, branch)
        else:
            self.clone_repository(repo_url, target_dir, branch)

        submodules_up_to_date = self.checkout_and_check_submodules(target_dir, branch)
        self.is_up_to_date = self.is_up_to_date and submodules_up_to_date

    def from_archive(self, manifest: Dict[str, Any]) -> None:
        """Download and extract an archive described in the stack manifest."""
        if not self.current_stack:
            self.get_logger().error("No current stack available.")
            return
        artifact = manifest.get("launch", {})
        workspace_dir = self.get_workspace_dir()
        if not workspace_dir:
            return

        props = artifact.get("properties", {})  
        base_state = {
            "type": "archive",
            "subdir": props.get("subdir"),
            "flatten": props.get("flatten", True),
        }

        with tempfile.TemporaryDirectory() as temp_dir:
            archive_path, state_info = self._prepare_archive(manifest, temp_dir)

            desired_state = {**base_state, **state_info}
            current_state = self._load_artifact_state(workspace_dir)
            if current_state == desired_state:
                self.is_up_to_date = True
                self.get_logger().info("Archive workspace already up-to-date; skipping extraction.")
                return

            self.is_up_to_date = False
            self._reset_workspace(workspace_dir)
            self._extract_archive(archive_path, workspace_dir)
            self.get_logger().info(
                f"Workspace contents after extraction: {os.listdir(workspace_dir)}"
            )

        subdir = props.get("subdir")
        if subdir:
            resolved_subdir = os.path.join(workspace_dir, subdir)
            if os.path.isdir(resolved_subdir):
                self._move_contents(resolved_subdir, workspace_dir)
            else:
                self.get_logger().warning(
                    f"Artifact subdirectory '{subdir}' not found; continuing without flattening."
                )
        elif props.get("flatten", True):
            self._flatten_single_directory(workspace_dir)

        self._write_artifact_state(workspace_dir, {**base_state, **state_info})

    def clone_repository(self, repo_url: str, target_dir: str, branch: str):
        """Clone the repository and check out the specified branch."""
        if os.path.exists(target_dir):
            shutil.rmtree(target_dir)
        os.makedirs(target_dir, exist_ok=True)

        subprocess.run(
            ["git", "clone", "--recurse-submodules", repo_url, target_dir],
            check=True,
        )

        self.checkout_branch(target_dir, branch)
        self.is_up_to_date = False

    def update_repository(self, target_dir: str, branch: str):
        """Update the existing repository."""
        # Fetch updates from remote
        subprocess.run(
            ["git", "fetch", "--recurse-submodules", "origin"],
            check=True,
            cwd=target_dir,
        )

        # Check if local and remote commits differ
        try:
            local_commit = subprocess.check_output(
                ["git", "rev-parse", "@"],
                text=True,
                cwd=target_dir,
            ).strip()
            remote_commit = subprocess.check_output(
                ["git", "rev-parse", "@{u}"],
                text=True,
                cwd=target_dir,
            ).strip()

            if local_commit != remote_commit:
                self.is_up_to_date = False
            else:
                self.is_up_to_date = True
        except subprocess.CalledProcessError:
            self.get_logger().warning("HEAD not set. Checking out branch.")
            self.checkout_branch(target_dir, branch)
            self.is_up_to_date = False

        if not self.is_up_to_date:
            self.get_logger().info("Main repo is not up-to-date. Pulling changes.")
            subprocess.run(
                ["git", "checkout", branch],
                check=True,
                cwd=target_dir,
            )
            subprocess.run(
                ["git", "pull"],
                check=True,
                cwd=target_dir,
            )

    def checkout_branch(self, repo_dir: str, branch: str):
        """Check out the specified branch, falling back to 'main' if necessary."""
        try:
            subprocess.run(
                ["git", "checkout", branch],
                check=True,
                cwd=repo_dir,
            )
        except subprocess.CalledProcessError:
            self.get_logger().warning(
                f"Branch '{branch}' not found. Falling back to 'main'."
            )
            subprocess.run(
                ["git", "checkout", "main"],
                check=True,
                cwd=repo_dir,
            )

    def checkout_and_check_submodules(
        self, target_dir: str, branch: str = "main"
    ) -> bool:
        """
        Ensure all submodules are checked out to the specified branch and are up-to-date.

        Args:
            target_dir (str): The path to the main repository.
            branch (str): The branch to check out in submodules.

        Returns:
            bool: True if all submodules are up-to-date, False otherwise.
        """
        all_submodules_up_to_date = True

        try:
            submodules = (
                subprocess.check_output(
                    ["git", "submodule", "--quiet", "foreach", "echo $sm_path"],
                    text=True,
                    cwd=target_dir,
                )
                .strip()
                .split("\n")
            )

            for submodule in submodules:
                submodule_path = os.path.join(target_dir, submodule)
                self.checkout_branch(submodule_path, branch)

                try:
                    local_commit = subprocess.check_output(
                        ["git", "rev-parse", "@"],
                        text=True,
                        cwd=submodule_path,
                    ).strip()
                    remote_commit = subprocess.check_output(
                        ["git", "rev-parse", "@{u}"],
                        text=True,
                        cwd=submodule_path,
                    ).strip()

                    if local_commit != remote_commit:
                        all_submodules_up_to_date = False
                        self.get_logger().info(
                            f"Submodule '{submodule}' is not up-to-date. Pulling changes."
                        )
                        subprocess.run(
                            ["git", "pull"],
                            check=True,
                            cwd=submodule_path,
                        )
                except subprocess.CalledProcessError:
                    self.get_logger().warning(
                        f"Submodule '{submodule}' is not properly set up."
                    )
                    all_submodules_up_to_date = False

        except subprocess.CalledProcessError as e:
            self.get_logger().warning(f"Failed to checkout or update submodules: {e}")
            all_submodules_up_to_date = False

        return all_submodules_up_to_date

    def from_tar(self, tar_file_path: str):
        """Extract a compressed tar file (Not implemented)."""
        # Placeholder for future implementation

    def build_workspace(self):
        """Build the workspace using colcon."""
        colcon_command = [
            "colcon",
            "build",
            "--symlink-install",
        ]

        if self.ignored_packages:
            colcon_command += ["--packages-ignore"] + self.ignored_packages

        colcon_command += ["--cmake-args", "-DCMAKE_BUILD_TYPE=Release"]

        try:
            subprocess.run(
                colcon_command,
                check=True,
                cwd=self.get_workspace_dir(),
            )
        except subprocess.CalledProcessError:
            self.get_logger().warning("Build failed. Cleaning and retrying...")
            self.clean_build_workspace()
            subprocess.run(
                colcon_command,
                check=True,
                cwd=self.get_workspace_dir(),
            )

    def clean_build_workspace(self):
        """Remove build and install directories to clean the workspace."""
        workspace_dirs = ["build", "install"]
        for dir_name in workspace_dirs:
            dir_path = os.path.join(self.get_workspace_dir(), dir_name)
            if os.path.exists(dir_path):
                shutil.rmtree(dir_path)
                self.get_logger().info(
                    f"Removed {dir_name} directory to clean build workspace."
                )

    def install_dependencies(self):
        """Install dependencies scoped to the workspace with best-effort retries."""
        workspace_dir = self.get_workspace_dir()
        if not workspace_dir:
            self.get_logger().warning("Workspace directory unavailable; skipping rosdep.")
            return

        rosdep_update_cmd = [
            "rosdep",
            "update",
            "--rosdistro",
            os.environ.get("ROS_DISTRO", "humble"),
            "--include-eol-distros",
        ]

        try:
            subprocess.run(
                rosdep_update_cmd,
                check=True,
                cwd=workspace_dir,
            )
        except subprocess.CalledProcessError as exc:
            self.get_logger().warning(
                f"rosdep update failed ({exc}); continuing with cached data."
            )

        try:
            subprocess.run(
                [
                    "rosdep",
                    "install",
                    "--from-path",
                    workspace_dir,
                    "--ignore-src",
                    "-r",
                    "-y",
                ],
                check=False,
                cwd=workspace_dir,
            )
        except subprocess.CalledProcessError as exc:
            self.get_logger().warning(
                f"rosdep install encountered errors: {exc}."
            )

    def handle_provision(
        self, request: ProvisionPlugin.Request, response: ProvisionPlugin.Response
    ):
        """Service handler to prepare the workspace."""
        self.current_stack = self._safely_parse_stack(request.input.current.stack)
        try:
            if request.start:
                if self.current_stack:
                    # Parse payload and determine type
                    
                    
                    if self.current_stack :
                        metadata = self.current_stack.get("metadata", {})
                        content_type = None
                        if metadata is not None:
                            content_type = metadata.get("content_type")
                        self.get_logger().info(f"provisioning stack with content_type: {content_type}")
                        if content_type == "stack/archive":
                            self.from_archive(self.current_stack)
                        elif request.input.current.url:
                            self.from_git(
                                repo_url=request.input.current.url,
                                branch=request.input.current.branch,
                            )
                        else:
                            response.err_msg = (
                                "Stack does not define a repository or archive artifact."
                            )
                            response.success = False
                            self.get_logger().error(response.err_msg)
                            return response
                    elif request.input.current.url:
                        self.from_git(
                            repo_url=request.input.current.url,
                            branch=request.input.current.branch,
                        )
                    else:
                        response.err_msg = (
                            "Stack does not define a repository or archive artifact."
                        )
                        response.success = False
                        self.get_logger().error(response.err_msg)
                        return response

                    if not self.is_up_to_date:
                        self.get_logger().info(
                            "Workspace is NOT up-to-date. Updating..."
                        )
                        self.install_dependencies()
                        self.build_workspace()
                    else:
                        self.get_logger().info(
                            "Workspace is up-to-date. Skipping build and provisioning steps."
                        )
                    response.err_msg = "Successful"
                    response.success = True
                else:
                    response.err_msg = "No current stack received."
                    response.success = False
                    self.get_logger().error("No current stack received.")
            else:
                response.err_msg = "Start flag not set in request."
                response.success = False
                self.get_logger().warning(
                    "Start flag not set in provision plugin request."
                )
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            response.err_msg = f"Error: {e}"
            response.success = False
            
        response.output.current = request.input.current
        return response

    def get_workspace_dir(self) -> str:
        """Get the workspace directory for the current stack."""
        if not self.current_stack or not isinstance(self.current_stack, dict):
            self.get_logger().error("No valid current stack available.")
            return ""
        
        metadata = self.current_stack.get("metadata", {})
        name = metadata.get("name", self.current_stack.get("name", "default"))
        return os.path.join(WORKSPACES_PATH, name.replace(" ", "_"))

    def _safely_parse_stack(self, stack_string):
        """
        Safely parse a stack string to JSON. Returns dictionary if valid JSON, None otherwise.
        
        Args:
            stack_string (str): The stack string to parse
            
        Returns:
            dict or None: Parsed JSON dictionary or None if parsing fails
        """
        if not stack_string:
            return None
            
        try:
            parsed = json.loads(stack_string)
            if isinstance(parsed, dict):
                return parsed
            else:
                self.get_logger().warning(f"Stack string parsed to non-dict type: {type(parsed)}")
                return None
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warning(f"Failed to parse stack string as JSON: {e}")
            return None


def main():
    rclpy.init()
    provision_plugin = MutoProvisionPlugin()
    rclpy.spin(provision_plugin)
    provision_plugin.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
