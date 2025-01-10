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
        self.namespace = manifest.get('namespace', os.getenv('MUTONS', default=''))
        self.output = manifest.get('output', 'screen')
        self.nodes = [node.Node(stack, nDef, self) for nDef in manifest.get('node', [])]
        self.remap = manifest.get('remap', [])
        self.action = manifest.get('action', '')

    def toManifest(self):
        return {
            "package": self.package,
            "executable": self.executable,
            "name": self.name,
            "namespace": self.namespace,
            "node": [n.toManifest() for n in self.nodes],
            "output": self.output,
            "remap": self.remap,
            "action": self.action
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
                self.executable == other.executable)
                

    def __hash__(self):
        return hash((self.package, self.name, self.namespace, self.executable))
