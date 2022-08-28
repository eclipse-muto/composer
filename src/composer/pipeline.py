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
from muto_msgs.msg import StackManifest, PlanManifest
from muto_msgs.srv import ComposePlugin

PLUGINS = {
    "ComposePlugin": ComposePlugin
}

class Pipeline(object):

    def __init__(self, device, name, steps, compensation=None):

        # Initialize from pipeline parameter
        self.device = device
        self.actions = {}
        self.actions[name] = steps
        self.compensation = compensation

    def execute(self,command, current, next):
        print("Execute pipeline for commnd:", command)
        plan = PlanManifest()
        cstack =  StackManifest()
        cstack.type = "json"
        cstack.stack = json.dumps(current)
        plan.current = cstack

        pstack =  StackManifest()
        pstack.type = "json"
        pstack.stack = json.dumps(next)
        plan.next = pstack
        plan.pipeline = command

        pipeline = self.actions[command]
        for items in pipeline:
            if items["sequence"]:
                for step in items["sequence"]:
                    try:
                        response =  self.executeStep(plan, step)
                        if response.output.result.resultCode == 0:
                            #Input for the next step is the ouput of this step
                            plan = response.output
                            print("Step passed", step, plan)
                    except rospy.ServiceException as e:
                        print("Step failed:",step,e)
                        if not self.compensation is None :
                            for step in self.compensation :
                                self.executeStep(plan, step)
                        return


    def executeStep(self, plan, step):
        call = rospy.ServiceProxy(step["service"], PLUGINS[step["plugin"]])
        # action = getattr(self.device, step["action"])
        response = call(plan)
        if response.output.result.resultCode == 0:
            # Step failed return an error message
            # and run rollback steps if any
            print("Step failed", step)

        return response
