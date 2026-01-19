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

import logging
import subprocess
import shlex
import yaml
import re

logger = logging.getLogger(__name__)


class Param:
    def __init__(self, stack, manifest=None):
        if manifest is None:
            manifest = {}

        self.stack = stack
        self.manifest = manifest
        self.name = manifest.get('name', '')
        self.value = self._resolve_value(manifest, stack)
        self.sep = manifest.get('sep', '')
        self.from_file = manifest.get('from', '')
        self.namespace = manifest.get('namespace', '/')
        self.command = manifest.get('command', '')

    def _resolve_value(self, manifest, stack):
        """Resolve the value of the parameter from various sources."""
        if 'from' in manifest:
            return self._resolve_from_file(stack.resolve_expression(manifest['from']))
        if 'command' in manifest:
            return self._execute_command(stack.resolve_expression(manifest['command']))
        return self._parse_value(manifest.get('value'))

    def _resolve_from_file(self, filepath):
        """Fetch and return the content of the specified file."""
        with open(filepath, 'r') as file:
            try:
                yaml_contents = yaml.safe_load(file)
                # FIXME: Below pattern matches everything. So it will load every parameter in yaml without looking at the relevant node name. 
                pattern = re.compile(r'/.*?') 
                matching_key = next(key for key in yaml_contents.keys() if pattern.match(key))
                ros_parameters = yaml_contents.get(matching_key, {}).get('ros__parameters', {})
                return ros_parameters

            except yaml.YAMLError as e:
                logger.error(f"Yaml read error: {e}")
            except Exception as e:
                logger.error(f"Failed to read from file '{filepath}': {e}")
                return None
            

    def _execute_command(self, command):
        """Execute the specified command and return its output."""
        try:
            return subprocess.check_output(shlex.split(command), text=True).strip()
        except subprocess.CalledProcessError as e:
            logger.error(f"Command execution failed: {e}")
            return None

    def _parse_value(self, value):
        """Parse the given value into the appropriate data type."""
        if isinstance(value, str):
            if value.lower() == 'true':
                return True
            if value.lower() == 'false':
                return False
            try:
                return int(value)
            except ValueError:
                try:
                    return float(value)
                except ValueError:
                    return value
        return value

    def toManifest(self):
        """Convert this Param instance into a manifest dictionary."""
        return {
            "name": self.name,
            "value": self.value,
            "sep": self.sep,
            "from": self.from_file,
            "namespace": self.namespace,
            "command": self.command,
        }

    def __eq__(self, other):
        """Check equality based on the attributes of the Param instance."""
        return isinstance(other, Param) and all(
            getattr(self, attr) == getattr(other, attr) for attr in [
                'name', 'value', 'from_file', 'namespace', 'command'
            ])

    def __hash__(self):
        """Generate a hash value for this Param instance."""
        return hash((self.name, self.value, self.from_file, self.namespace, self.command))