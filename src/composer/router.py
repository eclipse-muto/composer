#!/usr/bin/env python
#
#  Copyright (c) 2022 Composiv.ai, Eteration A.S. and others
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
from composer.pipeline import Pipeline
class Router(object):

    def __init__(self, device, pipelines):
        self.device = device
        self.pipelines = pipelines

    def route(self,action,payload):
        print("Compose route:", action, payload)
        if 'register' == action:
            self.device.register(payload)
        if self.pipelines.get(action) is not None :
            self.device.bootstrap() # reobtain current stack from twin
            currentStack = self.device.currentStack
            manifest = {}
            if currentStack :
                
                manifest = currentStack.manifest
            self.pipelines[action].execute(action, manifest, payload)


