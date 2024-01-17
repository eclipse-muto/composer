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
import composer.node as node


class Container(object):
    def __init__(self, stack, manifest={}):
        self.stack = stack
        self._manifest = manifest
        self._pkg = manifest.get('package', '')
        self._exec = manifest.get('executable', '')
        self._name = manifest.get('name', '')
        muto_ns = os.getenv('MUTONS')
        self._namespace = manifest.get('namespace', muto_ns)
        self._output = manifest.get('output', 'log')
        self.initialize()

    def toManifest(self):
        manifest = {
            "package": self._pkg,
            "executable": self._exec,
            "name": self._name,
            "namespace": self._namespace,
            "node": [],
            "output": self._output
        }
        for n in self._node:
            manifest["node"].append(n.toManifest())
        return manifest

    def initialize(self):
        self._node = []
        for nDef in self.manifest.get('node', []):
            sn = node.Node(self.stack, nDef, self)
            self._node.append(sn)

    def launch(self):
        parameters = []
        for p in self.param:
            parameters.append({p.name: p.value})

    def resolve_namespace(self):
        ns = self.namespace
        if not ns.startswith('/'):
            ns = '/' + ns
        if ns.endswith('/'):
            return ns + self.name + '/'
        return ns + '/' + self.name + '/'

    def __eq__(self, other):
        """Overrides the default implementation"""
        if isinstance(other, Container):
            if self.package != other.package:
                return False
            if self.name != other.name:
                return False
            if self.namespace != other.namespace:
                return False
            if self.executable != other.executable:
                return False
            if len(self.node) != len(other.node):
                return False 
            for n in self.node: 
                fail = True
                for on in other.node:
                    if n == on:
                      fail = False
                if fail:
                    return False
            return True
        return False

    def __ne__(self, other):
        """Overrides the default implementation (unnecessary in Python 3)"""
        return not self.__eq__(other)

    def __hash__(self):
        h = 7
        if self.package is not None:
            h = 31 * h + hash(self.package)
        if self.name is not None:
            h = 31 * h + hash(self.name)
        if self.namespace is not None:
            h = 31 * h + hash(self.namespace)
        if self.executable is not None:
            h = 31 * h + hash(self.executable)
        for n in self.node: 
            h = 31 * h + hash(n)
        return h

    @property
    def node(self):
        return self._node

    @property
    def package(self):
        return self._pkg

    @package.setter
    def package(self, n):
        self._pkg = n

    @property
    def executable(self):
        return self._exec

    @executable.setter
    def executable(self, n):
        self._exec = n

    @property
    def namespace(self):
        return self._namespace

    @namespace.setter
    def namespace(self, n):
        self._namespace = n

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, n):
        self._name = n

    @property
    def output(self):
        return self._output

    @output.setter
    def output(self, n):
        self._output = n

    @property
    def action(self):
        return self._action

    @action.setter
    def action(self, n):
        self._action = n

    @property
    def manifest(self):
        return self._manifest
