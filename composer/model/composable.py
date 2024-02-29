#
#  Copyright (c) 2023 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#
#

import os
import composer.model.node as node

class Container:
    def __init__(self, stack, manifest=None):
        if manifest is None:
            manifest = {}

        self.stack = stack
        self.manifest = manifest
        self.package = manifest.get('package', '')
        self.executable = manifest.get('executable', '')
        self.name = manifest.get('name', '')
        self.namespace = manifest.get('namespace', os.getenv('MUTONS'))
        self.output = manifest.get('output', 'screen')
        self.nodes = [node.Node(stack, nDef, self) for nDef in manifest.get('node', [])]
        self.remap = manifest.get('remap', [])

    def to_manifest(self):
        return {
            "package": self.package,
            "executable": self.executable,
            "name": self.name,
            "namespace": self.namespace,
            "node": [n.toManifest() for n in self.nodes],
            "output": self.output,
            "remap": self.remap
        }

    def resolve_namespace(self):
        ns_prefix = '/' if not self.namespace.startswith('/') else ''
        ns_suffix = '/' if not self.namespace.endswith('/') else ''
        return f"{ns_prefix}{self.namespace}{ns_suffix}{self.name}/"

    def __eq__(self, other):
        if not isinstance(other, Container):
            return False
        return (self.package == other.package and
                self.name == other.name and
                self.namespace == other.namespace and
                self.executable == other.executable and
                len(self.nodes) == len(other.nodes) and
                all(n == on for n, on in zip(self.nodes, other.nodes)))

    def __hash__(self):
        node_hash = sum(hash(n) for n in self.nodes)
        return hash((self.package, self.name, self.namespace, self.executable, node_hash))

