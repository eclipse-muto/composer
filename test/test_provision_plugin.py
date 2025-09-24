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
        self.node.current_stack = MagicMock()
        self.node.current_stack.name = "Test Stack"
        self.node.current_stack.url = "http://example.com/repo.git"
        self.node.current_stack.branch = "main"
        self.node.current_stack.stack = json.dumps({})
        self.node.current_stack.source = json.dumps({})

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
        response = MagicMock()
        self.node.current_stack = MagicMock()
        self.node.current_stack.url = "http://github.com/composer.git"
        self.node.current_stack.branch = "main"
        self.node.current_stack.stack = json.dumps({})
        self.node.current_stack.source = json.dumps({})
        self.node.current_stack.name = "Test Stack"
        self.node.is_up_to_date = False

        self.node.handle_provision(request, response)

        mock_from_git.assert_called_once_with(
            repo_url=self.node.current_stack.url,
            branch=self.node.current_stack.branch,
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
        response = MagicMock()
        self.node.current_stack = MagicMock()
        self.node.current_stack.url = "http://github.com/composer.git"
        self.node.current_stack.branch = "main"
        self.node.current_stack.stack = json.dumps({})
        self.node.current_stack.source = json.dumps({})
        self.node.current_stack.name = "Test Stack"
        self.node.is_up_to_date = True

        self.node.handle_provision(request, response)

        mock_from_git.assert_called_once_with(
            repo_url=self.node.current_stack.url,
            branch=self.node.current_stack.branch,
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
        response = MagicMock()
        artifact_manifest = {
            "artifact": {
                "type": "archive",
                "data": "ZHVtbXk=",
            }
        }
        self.node.current_stack.stack = json.dumps(artifact_manifest)
        self.node.current_stack.url = ""

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
        response = MagicMock()
        artifact_manifest = {
            "artifact": {
                "type": "archive",
                "data": "ZHVtbXk=",
            }
        }
        self.node.current_stack.stack = json.dumps(artifact_manifest)
        self.node.current_stack.url = ""

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
        response = MagicMock()
        self.node.current_stack = MagicMock()
        self.node.current_stack.url = ""
        self.node.current_stack.stack = json.dumps({})
        self.node.current_stack.name = "Test Stack"

        result = self.node.handle_provision(request, response)

        self.assertFalse(result.success)
        self.assertEqual(
            result.err_msg,
            "Stack does not define a repository or archive artifact.",
        )

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
        response = MagicMock()
        self.node.current_stack = MagicMock()
        self.node.current_stack.stack = json.dumps({})
        self.node.current_stack.name = "Test Stack"
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
                    "humble",
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
        self.node.current_stack = MagicMock()
        self.node.current_stack.name = "Test Stack"

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
        self.node.current_stack = MagicMock()
        repo_url = "http://github.com/composer.git"
        branch = "main"

        self.node.from_git(repo_url, branch)
        mock_update_repository.assert_called_once_with(mock_join(), "main")
        mock_clone_repository.assert_not_called()
        mock_checkout_and_check_submodules.assert_called_once_with(mock_join(), "main")
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
        mock_run.assert_called_once_with(
            ["git", "checkout", "test_branch"],
            check=True,
            cwd="/var/tmp/muto_workspaces/Test_Stack/submodule1",
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
