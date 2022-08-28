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

import rospy
import json

import core.ditto.twin as twin
import core.model.edge_device as edge
import core.model.stack as stack

from muto_msgs.msg import PluginResponse, StackManifest, PlanManifest
from muto_msgs.srv import ComposePlugin

class MutoDefaultComposePlugin(object):
    """
    """

    def __init__(self):

        #self.composiv = rospy.get_param("muto_composer_plugin/composiv")
        #self.mqtt_client.on_connect = lambda xx, userdata, flags,rc: self.on_mqtt_connect(userdata, flags,rc)
        self.service = rospy.Service('muto_compose', ComposePlugin, self.handle_compose)
        self.muto = rospy.get_param("composer_plugin/muto")
        self.edge_device = None
        #   Subscribers
        self.bootstrap(self.muto)


    def bootstrap(self, context):
        self.twin = twin.Twin(node='muto_composer_plugin',config=context, publisher=None)
        self.edge_device = edge.EdgeDevice(self.twin, context)

    def handle_compose(self,req):
        plan = req.input
        next  = plan.next

        st =  json.loads(plan.current.stack)
        currentStack = stack.Stack(self.edge_device, st, None)
        st =  json.loads(plan.next.stack)
        if not st.get('stackId', None) is None:
            manifest = self.edge_device.stack(st['stackId'])
            nextStack = stack.Stack(self.edge_device, manifest, None)
        else:
            nextStack = stack.Stack(self.edge_device, st, None)

        merged = currentStack.merge(nextStack)


        planned = StackManifest()
        planned.type = "json"
        planned.stack = json.dumps(merged.manifest)
        result = PluginResponse(resultCode=0, errorMessage="", errorDescription="")

        plan.planned = planned
        plan.result = result

        return plan



def main():
    rospy.init_node(name="composer_plugin",anonymous=True)
    C = MutoDefaultComposePlugin()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main()
