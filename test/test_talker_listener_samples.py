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

"""
Regression tests for talker-listener sample payloads.
These tests deserialize each sample, feed it through StackParser.parse_payload(),
and assert that the resulting StackManifest objects match the expected talker/listener topology.
"""

import json
import os
import unittest
from pathlib import Path

from composer.utils.stack_parser import StackParser, create_stack_parser


class TestTalkerListenerSamples(unittest.TestCase):
    """Regression tests for talker-listener sample payloads."""

    @classmethod
    def setUpClass(cls):
        """Load all sample files from docs/samples/talker-listener/"""
        # Find the samples directory relative to the repo root
        cls.samples_dir = Path(__file__).parent.parent.parent.parent / "docs" / "samples" / "talker-listener"
        cls.parser = create_stack_parser()

    def _load_sample(self, filename: str) -> dict:
        """Load a sample JSON file."""
        sample_path = self.samples_dir / filename
        self.assertTrue(sample_path.exists(), f"Sample file {filename} not found at {sample_path}")
        with open(sample_path, 'r') as f:
            return json.load(f)

    def test_talker_listener_json_format(self):
        """Test that talker-listener-json.json parses correctly as stack/json type."""
        payload = self._load_sample("talker-listener-json.json")

        # Parse payload
        parsed = self.parser.parse_payload(payload)

        self.assertIsNotNone(parsed, "Parser should successfully parse JSON stack")
        self.assertIsInstance(parsed, dict)

        # Verify metadata
        metadata = parsed.get("metadata", {})
        self.assertEqual(metadata.get("content_type"), "stack/json")
        self.assertEqual(metadata.get("name"), "Muto Simple Talker-Listener Stack")

        # Verify launch structure
        launch = parsed.get("launch", {})
        self.assertIsNotNone(launch, "JSON stack should have 'launch' section")

        # Verify nodes in launch
        nodes = launch.get("node", [])
        self.assertEqual(len(nodes), 2, "Should have exactly 2 nodes (talker and listener)")

        # Verify talker node
        talker = next((n for n in nodes if n.get("name") == "talker"), None)
        self.assertIsNotNone(talker, "Should have talker node")
        self.assertEqual(talker.get("pkg"), "demo_nodes_cpp")
        self.assertEqual(talker.get("exec"), "talker")

        # Verify listener node
        listener = next((n for n in nodes if n.get("name") == "listener"), None)
        self.assertIsNotNone(listener, "Should have listener node")
        self.assertEqual(listener.get("pkg"), "demo_nodes_cpp")
        self.assertEqual(listener.get("exec"), "listener")

    def test_talker_listener_json_validation(self):
        """Test that talker-listener-json.json passes validation."""
        payload = self._load_sample("talker-listener-json.json")
        parsed = self.parser.parse_payload(payload)

        self.assertTrue(
            self.parser.validate_stack(parsed),
            "Talker-listener JSON stack should pass validation"
        )

    def test_talker_listener_archive_format(self):
        """Test that talker-listener-xarchive.json parses correctly as stack/archive type."""
        payload = self._load_sample("talker-listener-xarchive.json")

        # Parse payload
        parsed = self.parser.parse_payload(payload)

        self.assertIsNotNone(parsed, "Parser should successfully parse archive stack")
        self.assertIsInstance(parsed, dict)

        # Verify metadata
        metadata = parsed.get("metadata", {})
        self.assertEqual(metadata.get("content_type"), "stack/archive")
        self.assertEqual(metadata.get("name"), "Muto Simple Talker-Listener Stack")

        # Verify launch structure for archive
        launch = parsed.get("launch", {})
        self.assertIsNotNone(launch, "Archive stack should have 'launch' section")

        # Verify archive-specific properties
        data = launch.get("data")
        self.assertIsNotNone(data, "Archive stack should have base64 encoded data")
        self.assertIsInstance(data, str)

        properties = launch.get("properties", {})
        self.assertIsNotNone(properties, "Archive stack should have properties")
        self.assertEqual(properties.get("launch_file"), "launch/talker_listener.launch.py")
        self.assertEqual(properties.get("algorithm"), "sha256")
        self.assertIsNotNone(properties.get("checksum"), "Should have checksum")

    def test_talker_listener_archive_validation(self):
        """Test that talker-listener-xarchive.json passes validation."""
        payload = self._load_sample("talker-listener-xarchive.json")
        parsed = self.parser.parse_payload(payload)

        self.assertTrue(
            self.parser.validate_stack(parsed),
            "Talker-listener archive stack should pass validation"
        )

    def test_talker_listener_json_instance_format(self):
        """
        Test that talker-listener-json-instance.json is a Symphony instance, not a stack.
        Instance files reference solutions and are not directly parseable as stacks.
        """
        try:
            payload = self._load_sample("talker-listener-json-instance.json")
        except AssertionError:
            self.skipTest("Sample file talker-listener-json-instance.json not found")

        # Instance files have metadata and spec.solution, not launch definitions
        self.assertIn("spec", payload)
        self.assertIn("solution", payload.get("spec", {}))
        # StackParser should return None for instance files (they're not stacks)
        parsed = self.parser.parse_payload(payload)
        self.assertIsNone(parsed, "Instance files should not parse as stacks")

    def test_talker_listener_archive_instance_format(self):
        """
        Test that talker-listener-xarchive-instance.json is a Symphony instance, not a stack.
        Instance files reference solutions and are not directly parseable as stacks.
        """
        try:
            payload = self._load_sample("talker-listener-xarchive-instance.json")
        except AssertionError:
            self.skipTest("Sample file talker-listener-xarchive-instance.json not found")

        # Instance files have metadata and spec.solution, not launch definitions
        self.assertIn("spec", payload)
        self.assertIn("solution", payload.get("spec", {}))
        # StackParser should return None for instance files (they're not stacks)
        parsed = self.parser.parse_payload(payload)
        self.assertIsNone(parsed, "Instance files should not parse as stacks")

    def test_all_samples_are_valid_json(self):
        """Test that all sample files are valid JSON."""
        sample_files = list(self.samples_dir.glob("*.json"))
        self.assertGreater(len(sample_files), 0, "Should have sample files")

        for sample_file in sample_files:
            with self.subTest(file=sample_file.name):
                with open(sample_file, 'r') as f:
                    try:
                        data = json.load(f)
                        self.assertIsInstance(data, dict)
                    except json.JSONDecodeError as e:
                        self.fail(f"Invalid JSON in {sample_file.name}: {e}")

    def test_topology_nodes_are_consistent(self):
        """
        Verify that the talker-listener topology is consistent across formats.
        Both JSON and archive formats should define the same logical nodes.
        """
        json_payload = self._load_sample("talker-listener-json.json")
        json_parsed = self.parser.parse_payload(json_payload)

        # Extract node names from JSON format
        json_nodes = json_parsed.get("launch", {}).get("node", [])
        json_node_names = {n.get("name") for n in json_nodes}

        self.assertIn("talker", json_node_names)
        self.assertIn("listener", json_node_names)

        # The archive format contains a compressed workspace with the launch file
        # We verify it has the expected structure for deployment
        archive_payload = self._load_sample("talker-listener-xarchive.json")
        archive_parsed = self.parser.parse_payload(archive_payload)

        props = archive_parsed.get("launch", {}).get("properties", {})
        launch_file = props.get("launch_file", "")
        self.assertIn("talker_listener", launch_file.lower())


class TestStackParserIntegration(unittest.TestCase):
    """Integration tests for StackParser with various payload formats."""

    def setUp(self):
        self.parser = create_stack_parser()

    def test_parse_payload_returns_none_for_invalid_input(self):
        """Test that parse_payload returns None for invalid inputs."""
        self.assertIsNone(self.parser.parse_payload(None))
        self.assertIsNone(self.parser.parse_payload("not a dict"))
        self.assertIsNone(self.parser.parse_payload([]))

    def test_parse_payload_with_value_key_returns_as_is(self):
        """Test that payloads with 'value' key are returned unchanged."""
        payload = {"value": {"some": "data"}, "other": "field"}
        result = self.parser.parse_payload(payload)
        self.assertEqual(result, payload)

    def test_validate_stack_returns_false_for_empty_dict(self):
        """Test validation returns false for empty stack."""
        self.assertFalse(self.parser.validate_stack({}))

    def test_validate_stack_returns_true_for_valid_stack(self):
        """Test validation returns true for valid stack with nodes."""
        stack = {
            "node": [{"name": "test", "pkg": "test_pkg", "exec": "test_exec"}]
        }
        self.assertTrue(self.parser.validate_stack(stack))


if __name__ == "__main__":
    unittest.main()
