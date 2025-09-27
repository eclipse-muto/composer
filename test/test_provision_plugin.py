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

import json
import os
import subprocess
import unittest
from unittest.mock import MagicMock, call, patch

import rclpy

from composer.plugins.provision_plugin import WORKSPACES_PATH, MutoProvisionPlugin


class TestMutoProvisionPlugin(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MutoProvisionPlugin()
        self.node.get_logger = MagicMock()
        # Don't set current_stack here - let handle_provision parse it from the request

    def tearDown(self):
        self.node.destroy_node()

    @patch.object(MutoProvisionPlugin, "build_workspace")
    @patch.object(MutoProvisionPlugin, "install_dependencies")
    @patch.object(MutoProvisionPlugin, "from_git")
    def test_handle_provision_start_true_not_up_to_date(
        self, mock_from_git, mock_install_dependencies, mock_build_workspace
    ):
        request = MagicMock()
        request.start = True
        request.input.current.url = "http://github.com/composer.git"
        request.input.current.branch = "main"
        # Use structure matching talker-listener-xarchive.json
        stack_data = {
            "metadata": {
                "name": "Test Stack",
                "description": "A test stack",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [{"name": "test_node"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        response = MagicMock()
        self.node.is_up_to_date = False

        self.node.handle_provision(request, response)

        mock_from_git.assert_called_once_with(
            repo_url=request.input.current.url,
            branch=request.input.current.branch,
        )
        mock_install_dependencies.assert_called_once()
        mock_build_workspace.assert_called_once()
        self.assertEqual(response.err_msg, "Successful")
        self.assertTrue(response.success)

    @patch.object(MutoProvisionPlugin, "build_workspace")
    @patch.object(MutoProvisionPlugin, "install_dependencies")
    @patch.object(MutoProvisionPlugin, "from_git")
    def test_handle_provision_start_true_up_to_date(
        self, mock_from_git, mock_install_dependencies, mock_build_workspace
    ):
        request = MagicMock()
        request.start = True
        request.input.current.url = "http://github.com/composer.git"
        request.input.current.branch = "main" 
        # Use structure matching talker-listener-xarchive.json
        stack_data = {
            "metadata": {
                "name": "Test Stack",
                "description": "A test stack",
                "content_type": "stack/json"
            },
            "launch": {
                "node": [{"name": "test_node"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        response = MagicMock()
        self.node.is_up_to_date = True

        self.node.handle_provision(request, response)

        mock_from_git.assert_called_once_with(
            repo_url=request.input.current.url,
            branch=request.input.current.branch,
        )
        mock_install_dependencies.assert_not_called()
        mock_build_workspace.assert_not_called()
        self.assertEqual(response.err_msg, "Successful")
        self.assertTrue(response.success)

    @patch.object(MutoProvisionPlugin, "build_workspace")
    @patch.object(MutoProvisionPlugin, "install_dependencies")
    @patch.object(MutoProvisionPlugin, "from_archive")
    def test_handle_provision_archive_not_up_to_date(
        self, mock_from_archive, mock_install_dependencies, mock_build_workspace
    ):
        request = MagicMock()
        request.start = True
        request.input.current.url = ""
        response = MagicMock()
        # Use real archive structure from talker-listener-xarchive.json
        artifact_manifest = {
            "metadata": {
                "name": "Muto Simple Talker-Listener Stack",
                "description": "A simple talker-listener stack example",
                "content_type": "stack/archive"
            },
            "launch": {
                "data": "ZHVtbXk=",  # base64 encoded dummy data
                "properties": {
                    "algorithm": "sha256",
                    "checksum": "553fd2dc7d0eb41e7d65c467d358e7962d3efbb0e2f2e4f8158e926a081f96d0",
                    "launch_file": "launch/talker_listener.launch.py",
                    "command": "launch",
                    "flatten": True
                }
            }
        }
        request.input.current.stack = json.dumps(artifact_manifest)

        def mark_dirty(_):
            self.node.is_up_to_date = False

        mock_from_archive.side_effect = mark_dirty

        self.node.handle_provision(request, response)

        mock_from_archive.assert_called_once()
        mock_install_dependencies.assert_called_once()
        mock_build_workspace.assert_called_once()
        self.assertTrue(response.success)

    @patch.object(MutoProvisionPlugin, "build_workspace")
    @patch.object(MutoProvisionPlugin, "install_dependencies")
    @patch.object(MutoProvisionPlugin, "from_archive")
    def test_handle_provision_archive_up_to_date(
        self, mock_from_archive, mock_install_dependencies, mock_build_workspace
    ):
        request = MagicMock()
        request.start = True
        request.input.current.url = ""
        response = MagicMock()
        # Use real archive structure from talker-listener-xarchive.json
        artifact_manifest = {
            "metadata": {
                "name": "Muto Simple Talker-Listener Stack",
                "description": "A simple talker-listener stack example",
                "content_type": "stack/archive"
            },
            "launch": {
                "data": "ZHVtbXk=",  # base64 encoded dummy data
                "properties": {
                    "algorithm": "sha256",
                    "checksum": "553fd2dc7d0eb41e7d65c467d358e7962d3efbb0e2f2e4f8158e926a081f96d0",
                    "launch_file": "launch/talker_listener.launch.py",
                    "command": "launch",
                    "flatten": True
                }
            }
        }
        request.input.current.stack = json.dumps(artifact_manifest)

        def mark_clean(_):
            self.node.is_up_to_date = True

        mock_from_archive.side_effect = mark_clean

        self.node.handle_provision(request, response)

        mock_from_archive.assert_called_once()
        mock_install_dependencies.assert_not_called()
        mock_build_workspace.assert_not_called()
        self.assertTrue(response.success)

    def test_handle_provision_no_artifact_or_repo(self):
        request = MagicMock()
        request.start = True
        request.input.current.url = ""
        # Use proper structure but no archive data and no repo URL
        stack_data = {
            "metadata": {
                "name": "Test Stack",
                "description": "A test stack without repo or archive"
            },
            "launch": {
                "node": [{"name": "test_node"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        response = MagicMock()

        result = self.node.handle_provision(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "Stack does not define a repository or archive artifact.")

    def test_handle_provision_start_false(self):
        request = MagicMock()
        request.start = False
        response = MagicMock()

        self.node.handle_provision(request, response)

        self.assertEqual(response.err_msg, "Start flag not set in request.")
        self.assertFalse(response.success)

    def test_handle_provision_no_current_stack(self):
        request = MagicMock()
        request.start = True
        response = MagicMock()
        self.node.current_stack = None

        self.node.handle_provision(request, response)

        self.assertEqual(response.err_msg, "No current stack received.")
        self.assertFalse(response.success)

    @patch.object(MutoProvisionPlugin, "from_git")
    def test_handle_provision_exception(self, mock_from_git):
        request = MagicMock()
        request.start = True
        request.input.current.url = "http://github.com/composer.git" 
        request.input.current.branch = "main"
        # Use proper structure
        stack_data = {
            "metadata": {
                "name": "Test Stack",
                "description": "A test stack"
            },
            "launch": {
                "node": [{"name": "test_node"}]
            }
        }
        request.input.current.stack = json.dumps(stack_data)
        response = MagicMock()
        mock_from_git.side_effect = Exception("Test Exception")

        self.node.handle_provision(request, response)

        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "Error: Test Exception")

    @patch("os.makedirs")
    @patch("shutil.rmtree")
    @patch("os.path.exists")
    @patch.object(MutoProvisionPlugin, "checkout_branch")
    @patch("subprocess.run")
    def test_clone_repository(
        self,
        mock_subprocess_run,
        mock_checkout_branch,
        mock_path_exists,
        mock_shutil_rmtree,
        mock_os_makedirs,
    ):
        target_dir = "/mock/target/dir"
        repo_url = "http://github.com/composer.git"
        branch = "main"
        mock_path_exists.return_value = True

        self.node.clone_repository(repo_url, target_dir, branch)

        mock_shutil_rmtree.assert_called_once_with(target_dir)
        mock_os_makedirs.assert_called_once_with(target_dir, exist_ok=True)
        mock_subprocess_run.assert_any_call(
            ["git", "clone", "--recurse-submodules", repo_url, target_dir],
            check=True,
        )
        mock_checkout_branch.assert_called_once_with(target_dir, branch)
        self.assertFalse(self.node.is_up_to_date)

    @patch("subprocess.check_output")
    @patch("subprocess.run")
    def test_update_repository_up_to_date(self, mock_subprocess_run, mock_check_output):
        target_dir = "/mock/target/dir"
        branch = "main"
        mock_check_output.side_effect = ["abc123", "abc123"]

        self.node.update_repository(target_dir, branch)

        mock_subprocess_run.assert_any_call(
            ["git", "fetch", "--recurse-submodules", "origin"],
            check=True,
            cwd=target_dir,
        )
        self.assertTrue(self.node.is_up_to_date)

    @patch("subprocess.check_output")
    @patch("subprocess.run")
    def test_update_repository_not_up_to_date(
        self, mock_subprocess_run, mock_check_output
    ):
        target_dir = "/mock/target/dir"
        branch = "main"
        mock_check_output.side_effect = ["start", "stop"]

        self.node.update_repository(target_dir, branch)

        mock_subprocess_run.assert_any_call(
            ["git", "fetch", "--recurse-submodules", "origin"],
            check=True,
            cwd=target_dir,
        )
        self.assertFalse(self.node.is_up_to_date)
        mock_subprocess_run.assert_any_call(
            ["git", "checkout", branch],
            check=True,
            cwd=target_dir,
        )
        mock_subprocess_run.assert_any_call(
            ["git", "pull"],
            check=True,
            cwd=target_dir,
        )

    @patch("subprocess.run")
    def test_checkout_branch_success(self, mock_subprocess_run):
        repo_dir = "/mock/repo/dir"
        branch = "feature"

        self.node.checkout_branch(repo_dir, branch)

        mock_subprocess_run.assert_called_once_with(
            ["git", "checkout", branch],
            check=True,
            cwd=repo_dir,
        )

    @patch("os.path.exists")
    @patch("shutil.rmtree")
    def test_clean_build_workspace(self, mock_rmtree, mock_exists):
        mock_exists.return_value = True
        self.node.get_workspace_dir = MagicMock(return_value="/mock/workspace")

        self.node.clean_build_workspace()

        calls = [call("/mock/workspace/build"), call("/mock/workspace/install")]
        mock_rmtree.assert_has_calls(calls, any_order=True)

    @patch("subprocess.run")
    def test_build_workspace(self, mock_subprocess_run):
        self.node.ignored_packages = []
        self.node.get_workspace_dir = MagicMock(return_value="/mock/workspace")

        self.node.build_workspace()

        expected_command = [
            "colcon",
            "build",
            "--symlink-install",
            "--cmake-args",
            "-DCMAKE_BUILD_TYPE=Release",
        ]
        mock_subprocess_run.assert_called_with(
            expected_command,
            check=True,
            cwd="/mock/workspace",
        )

    @patch("subprocess.run")
    def test_install_dependencies(self, mock_subprocess_run):
        self.node.get_workspace_dir = MagicMock(return_value="/mock/workspace")

        self.node.install_dependencies()

        expected_calls = [
            call(
                [
                    "rosdep",
                    "update",
                    "--rosdistro",
                    os.environ.get("ROS_DISTRO", "humble"),
                    "--include-eol-distros",
                ],
                check=True,
                cwd="/mock/workspace",
            ),
            call(
                [
                    "rosdep",
                    "install",
                    "--from-path",
                    "/mock/workspace",
                    "--ignore-src",
                    "-r",
                    "-y",
                ],
                check=False,
                cwd="/mock/workspace",
            ),
        ]
        mock_subprocess_run.assert_has_calls(expected_calls)

    def test_get_workspace_dir(self):
        self.node.current_stack = {"name": "Test Stack"}

        workspace_dir = self.node.get_workspace_dir()

        expected_dir = os.path.join(WORKSPACES_PATH, "Test_Stack")
        self.assertEqual(workspace_dir, expected_dir)

    def test_get_workspace_dir_no_stack(self):
        self.node.current_stack = None

        workspace_dir = self.node.get_workspace_dir()

        self.assertEqual(workspace_dir, "")

    @patch("os.path.exists")
    @patch("os.path.join")
    @patch.object(MutoProvisionPlugin, "checkout_and_check_submodules")
    @patch.object(MutoProvisionPlugin, "clone_repository")
    @patch.object(MutoProvisionPlugin, "update_repository")
    def test_from_git(
        self,
        mock_update_repository,
        mock_clone_repository,
        mock_checkout_and_check_submodules,
        mock_join,
        mock_exist,
    ):
        self.node.current_stack = {"name": "Test Stack"}
        mock_exist.return_value = True  # Mock that .git directory exists
        repo_url = "http://github.com/composer.git"
        branch = "main"

        self.node.from_git(repo_url, branch)
        mock_update_repository.assert_called_once_with(mock_join.return_value, "main")
        mock_clone_repository.assert_not_called()
        mock_checkout_and_check_submodules.assert_called_once_with(mock_join.return_value, "main")
        mock_join.assert_called()
        mock_exist.assert_called_once()

    @patch("os.path.exists")
    @patch("os.path.join")
    @patch.object(MutoProvisionPlugin, "checkout_and_check_submodules")
    @patch.object(MutoProvisionPlugin, "clone_repository")
    @patch.object(MutoProvisionPlugin, "update_repository")
    def test_from_git_no_current_stack(
        self,
        mock_update_repository,
        mock_clone_repository,
        mock_checkout_and_check_submodules,
        mock_join,
        mock_exist,
    ):
        self.node.current_stack = None
        repo_url = "http://github.com/composer.git"
        branch = "main"
        self.node.from_git(repo_url, branch)

        mock_update_repository.assert_not_called()
        mock_clone_repository.assert_not_called()
        mock_checkout_and_check_submodules.assert_not_called()
        mock_join.assert_not_called()
        mock_exist.assert_not_called()

    @patch("subprocess.check_output")
    @patch("subprocess.run")
    def test_all_submodules_up_to_date(self, mock_run, mock_check_output):
        mock_check_output.side_effect = [
            "submodule1\nsubmodule2",
            "local_commit_1",
            "local_commit_1",
            "local_commit_2",
            "local_commit_2",
        ]

        target_dir = self.node.get_workspace_dir()
        result = self.node.checkout_and_check_submodules(
            target_dir, branch="test_branch"
        )

        self.assertTrue(result)
        calls = [call.args for call in mock_run.call_args_list]
        checkout_calls = [c for c in calls if "checkout" in c[0]]
        self.assertEqual(len(checkout_calls), 2)

    @patch("subprocess.check_output")
    @patch("subprocess.run")
    def test_submodule_not_up_to_date(self, mock_run, mock_check_output):
        mock_check_output.side_effect = [
            "submodule1",
            "local_commit_1",
            "remote_commit_1_different",
        ]

        target_dir = self.node.get_workspace_dir()
        result = self.node.checkout_and_check_submodules(
            target_dir, branch="test_branch"
        )

        self.assertFalse(result)

        calls = [call.args for call in mock_run.call_args_list]
        pull_calls = [c for c in calls if "pull" in c[0]]
        self.assertEqual(len(pull_calls), 1)

    @patch("subprocess.check_output")
    @patch("subprocess.run")
    def test_submodule_checkout_failure(self, mock_run, mock_check_output):
        def side_effect_check_output(args, text=True, cwd=None):
            if "rev-parse" in args:
                raise subprocess.CalledProcessError(1, args, "Error message")
            return "submodule1"

        mock_check_output.side_effect = side_effect_check_output

        target_dir = self.node.get_workspace_dir()
        result = self.node.checkout_and_check_submodules(
            target_dir, branch="test_branch"
        )

        self.assertFalse(result)
        # The actual path should be an os.path.join result
        expected_submodule_path = os.path.join(target_dir, "submodule1")
        mock_run.assert_called_once_with(
            ["git", "checkout", "test_branch"],
            check=True,
            cwd=expected_submodule_path,
        )

    @patch("subprocess.check_output")
    @patch("subprocess.run")
    def test_no_submodules_or_command_fail(self, mock_run, mock_check_output):
        mock_check_output.side_effect = subprocess.CalledProcessError(
            1, ["git", "submodule", "--quiet", "foreach", "echo $sm_path"], "Error"
        )

        target_dir = self.node.get_workspace_dir()
        result = self.node.checkout_and_check_submodules(
            target_dir, branch="test_branch"
        )

        self.assertFalse(result)
        mock_run.assert_not_called()

    @patch.object(MutoProvisionPlugin, "from_git")
    def test_handle_provision_stack_json_content_type(self, mock_from_git):
        """Test handle_provision with stack/json content_type falls back to git."""
        request = MagicMock()
        request.start = True
        request.input.current.url = "http://example.com/repo.git"
        request.input.current.branch = "main"
        response = MagicMock()
        
        # Mock is_up_to_date to be False so dependencies and build are called
        self.node.is_up_to_date = False
        
        # Mock the dependencies and build methods
        with patch.object(self.node, 'install_dependencies') as mock_install, \
             patch.object(self.node, 'build_workspace') as mock_build:
            
            # Payload with stack/json content_type using proper structure
            json_manifest = {
                "metadata": {
                    "name": "test-json-stack",
                    "description": "A test JSON stack",
                    "content_type": "stack/json"
                },
                "launch": {
                    "node": [{"name": "test_node"}]
                }
            }
            request.input.current.stack = json.dumps(json_manifest)

            self.node.handle_provision(request, response)

            # Should call from_git since stack/json doesn't have special provision handling
            mock_from_git.assert_called_once_with(
                repo_url="http://example.com/repo.git",
                branch="main"
            )
            mock_install.assert_called_once()
            mock_build.assert_called_once()
            self.assertTrue(response.success)

    @patch.object(MutoProvisionPlugin, "from_git")
    def test_handle_provision_unknown_content_type(self, mock_from_git):
        """Test handle_provision with unknown content_type falls back to git."""
        request = MagicMock()
        request.start = True
        response = MagicMock()
        
        # Set actual values instead of letting MagicMock create placeholders
        request.input.current.url = "http://example.com/repo.git"
        request.input.current.branch = "main"
        
        # Mock the dependencies and build methods since they might be called
        with patch.object(self.node, 'install_dependencies') as mock_install, \
             patch.object(self.node, 'build_workspace') as mock_build:
            
            # Set is_up_to_date to False so we can track the install/build calls
            self.node.is_up_to_date = False
        
            # Payload with unknown content_type
            unknown_manifest = {
                "metadata": {
                    "name": "test-unknown-stack",
                    "content_type": "unknown/type"
                },
                "custom": {
                    "data": "some_data"
                }
            }
            request.input.current.stack = json.dumps(unknown_manifest)

            self.node.handle_provision(request, response)

            # Should call from_git since unknown content_type doesn't have special handling
            mock_from_git.assert_called_once_with(
                repo_url="http://example.com/repo.git",
                branch="main"
            )
            mock_install.assert_called_once()
            mock_build.assert_called_once()
            self.assertTrue(response.success)

    @patch.object(MutoProvisionPlugin, "from_git")
    def test_handle_provision_missing_content_type(self, mock_from_git):
        """Test handle_provision with missing content_type falls back to git."""
        request = MagicMock()
        request.start = True
        request.input.current.url = "http://example.com/repo.git"
        request.input.current.branch = "main"
        response = MagicMock()
        
        # Payload without content_type
        no_content_type_manifest = {
            "metadata": {
                "name": "test-no-content-type-stack"
            },
            "launch": {
                "data": "ZHVtbXk="
            }
        }
        request.input.current.stack = json.dumps(no_content_type_manifest)

        self.node.handle_provision(request, response)

        # Should call from_git since missing content_type doesn't trigger archive handling
        mock_from_git.assert_called_once_with(
            repo_url="http://example.com/repo.git",
            branch="main"
        )
        self.assertTrue(response.success)

    @patch.object(MutoProvisionPlugin, "from_git")
    def test_handle_provision_invalid_json_payload(self, mock_from_git):
        """Test handle_provision with invalid JSON payload falls back to git."""
        request = MagicMock()
        request.start = True
        request.input.current.url = "http://example.com/repo.git"
        request.input.current.branch = "main"
        response = MagicMock()
        
        # Invalid JSON in stack field - becomes None when parsed
        request.input.current.stack = "invalid json {"

        self.node.handle_provision(request, response)

        # Should NOT call from_git since parsed stack will be None for invalid JSON
        mock_from_git.assert_not_called()
        self.assertFalse(response.success)
        self.assertEqual(response.err_msg, "No current stack received.")

    @patch.object(MutoProvisionPlugin, "from_git")
    def test_handle_provision_missing_content_type(self, mock_from_git):
        """Test handle_provision with missing content_type falls back to git."""
        request = MagicMock()
        request.start = True
        response = MagicMock()
        
        # Set actual values instead of letting MagicMock create placeholders
        request.input.current.url = "http://example.com/repo.git"
        request.input.current.branch = "main"
        
        # Mock the dependencies and build methods since they might be called
        with patch.object(self.node, 'install_dependencies') as mock_install, \
             patch.object(self.node, 'build_workspace') as mock_build:
            
            # Set is_up_to_date to False so we can track the install/build calls
            self.node.is_up_to_date = False
        
            # Payload without content_type - parser returns None
            no_content_type_manifest = {
                "metadata": {
                    "name": "test-no-content-type-stack"
                },
                "launch": {
                    "data": "ZHVtbXk="
                }
            }
            request.input.current.stack = json.dumps(no_content_type_manifest)

            self.node.handle_provision(request, response)

            # Should call from_git since parser returns None for unknown format
            mock_from_git.assert_called_once_with(
                repo_url="http://example.com/repo.git",
                branch="main"
            )
            mock_install.assert_called_once()
            mock_build.assert_called_once()
            self.assertTrue(response.success)

    @patch.object(MutoProvisionPlugin, "from_git")
    def test_handle_provision_no_metadata_falls_back_to_git(self, mock_from_git):
        """Test handle_provision with payload missing metadata falls back to git."""
        request = MagicMock()
        request.start = True
        request.input.current.url = "http://example.com/repo.git"
        request.input.current.branch = "main"
        response = MagicMock()
        
        # Payload without metadata - parser returns None
        no_metadata_payload = {
            "launch": {
                "data": "ZHVtbXk="
            }
        }
        request.input.current.stack = json.dumps(no_metadata_payload)

        self.node.handle_provision(request, response)

        # Should call from_git since parser returns None for unknown format
        mock_from_git.assert_called_once_with(
            repo_url="http://example.com/repo.git",
            branch="main"
        )
        self.assertTrue(response.success)

    def test_handle_provision_stack_archive_no_url_fails(self):
        """Test handle_provision fails when stack/archive has no data or URL."""
        request = MagicMock()
        request.start = True
        request.input.current.url = ""  # No URL
        response = MagicMock()
        
        # Archive payload with no data and no URL
        archive_manifest = {
            "metadata": {
                "name": "test-archive-no-data-url",
                "content_type": "stack/archive"
            },
            "launch": {
                "properties": {
                    "filename": "dummy.tar.gz"
                }
            }
        }
        request.input.current.stack = json.dumps(archive_manifest)

        self.node.handle_provision(request, response)

        # Should fail because archive specification must include data or url
        self.assertFalse(response.success)
        self.assertIn("must include either 'data' or 'url'", response.err_msg)

    def test_handle_provision_stack_archive_no_url_fails(self):
        """Test handle_provision fails when stack/archive has no data or URL."""
        request = MagicMock()
        request.start = True
        response = MagicMock()
        
        # Archive payload with no data and no URL in launch
        archive_manifest = {
            "metadata": {
                "name": "test-archive-no-data-url",
                "content_type": "stack/archive"
            },
            "launch": {
                "properties": {
                    "filename": "dummy.tar.gz"
                }
            }
        }
        request.input.current.stack = json.dumps(archive_manifest)
        request.input.current.source = json.dumps({
            "url": ""  # No URL
        })

        self.node.handle_provision(request, response)

        # Should fail because archive specification must include data or url
        self.assertFalse(response.success)
        self.assertIn("must include either 'data' or 'url'", response.err_msg)

    @patch.object(MutoProvisionPlugin, "from_git")
    def test_handle_provision_no_metadata_falls_back_to_git(self, mock_from_git):
        """Test handle_provision with payload missing metadata falls back to git."""
        request = MagicMock()
        request.start = True
        response = MagicMock()
        
        # Set actual values instead of letting MagicMock create placeholders
        request.input.current.url = "http://example.com/repo.git"
        request.input.current.branch = "main"
        
        # Mock the dependencies and build methods since they might be called
        with patch.object(self.node, 'install_dependencies') as mock_install, \
             patch.object(self.node, 'build_workspace') as mock_build:
            
            # Set is_up_to_date to False so we can track the install/build calls
            self.node.is_up_to_date = False
        
            # Payload without metadata
            no_metadata_payload = {
                "launch": {
                    "data": "ZHVtbXk="
                }
            }
            request.input.current.stack = json.dumps(no_metadata_payload)

            self.node.handle_provision(request, response)

            # Should call from_git since no metadata means no content_type check
            mock_from_git.assert_called_once_with(
                repo_url="http://example.com/repo.git",
                branch="main"
            )
            mock_install.assert_called_once()
            mock_build.assert_called_once()
            self.assertTrue(response.success)
