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
import json
import rospy
import core.ditto.twin as twin
import core.model.edge_device as edge
from std_msgs.msg import String



from muto_msgs.msg import PluginResponse, StackManifest, PlanManifest
from muto_msgs.srv import ComposePlugin

class MutoDefaultLaunchPlugin(object):
    """
    """

    def __init__(self):

        self.muto = rospy.get_param("launch_plugin/muto")
        self.device = None
        # Subscribers
        self.twinPublisher = rospy.Publisher(rospy.get_param("muto_composer/muto/twin_topic"), String, queue_size=100)
        self.bootstrap(self.muto)
        self.service = rospy.Service('muto_apply_stack', ComposePlugin, self.handle_apply)
        self.service = rospy.Service('muto_kill_stack', ComposePlugin, self.handle_kill)
        self.service = rospy.Service('muto_start_stack', ComposePlugin, self.handle_start)
        self.service = rospy.Service('muto_restart_stack', ComposePlugin, self.handle_restart)


    def bootstrap(self, context):
        if  self.device == None:
            self.twin = twin.Twin(node='muto_composer',config=context, publisher=self.twinPublisher)
            self.device = edge.EdgeDevice(self.twin, context)

    def handle_apply(self,req):
        plan = req.input
 

        if plan.planned:
            # context = json.loads(msg.context)
            # self.bootstrap(context)
            stack =  json.loads(plan.planned.stack)
            #Apply is different than start kill etc. It will use the node "action"
            self.device.apply(stack) 

            result = PluginResponse(resultCode=0, errorMessage="", errorDescription="")
            plan.result = result

            return plan

        result = PluginResponse(resultCode=1000, errorMessage="Could not handle launch", errorDescription="Could not handle launch")
        plan.result = result
        return plan


    def handle_start(self,req):
        plan = req.input
 

        if plan.planned:
            # context = json.loads(msg.context)
            # self.bootstrap(context)
            stack =  json.loads(plan.planned.stack)
            #Apply is different than start kill etc. It will use the node "action"
            self.device.activate(stack) 

            result = PluginResponse(resultCode=0, errorMessage="", errorDescription="")
            plan.result = result

            return plan

        result = PluginResponse(resultCode=1000, errorMessage="Could not handle launch", errorDescription="Could not handle launch")
        plan.result = result
        return plan

    def handle_restart(self,req):
        plan = req.input
 

        if plan.planned:
            # context = json.loads(msg.context)
            # self.bootstrap(context)
            stack =  json.loads(plan.planned.stack)
            #Apply is different than start kill etc. It will use the node "action"
            self.device.restart(stack) 

            result = PluginResponse(resultCode=0, errorMessage="", errorDescription="")
            plan.result = result

            return plan

        result = PluginResponse(resultCode=1000, errorMessage="Could not handle launch", errorDescription="Could not handle launch")
        plan.result = result
        return plan

    def handle_kill(self,req):
        plan = req.input
 

        if plan.planned:
            # context = json.loads(msg.context)
            # self.bootstrap(context)
            try:
                stack =  json.loads(plan.planned.stack)
                #Apply is different than start kill etc. It will use the node "action"
                self.device.kill(stack) 
            except Exception as error:
              print('Cannot parse stack: {}'.format(error))

            result = PluginResponse(resultCode=0, errorMessage="", errorDescription="")
            plan.result = result

            return plan

        result = PluginResponse(resultCode=1000, errorMessage="Could not handle launch", errorDescription="Could not handle launch")
        plan.result = result
        return plan




def main():
    rospy.init_node(name="launch_plugin",anonymous=True)
    C = MutoDefaultLaunchPlugin()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main()
