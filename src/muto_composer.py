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
import uuid

import rospy
import paho.mqtt.client as mqtt
import core.ditto.twin as twin
import composer.router as router
import core.model.edge_device as edge


from muto_msgs.msg import MutoAction
from std_msgs.msg import String
from composer.pipeline import Pipeline


class MutoComposer(object):
    """
    The class that handles muto composer
    """

    def __init__(self):

        self.muto = rospy.get_param("muto_composer/muto")
        self.edge_device = None
        #   Subscribers
        self.twinPublisher = rospy.Publisher(rospy.get_param("muto_composer/muto/twin_topic"),String, queue_size=100)
        self.bootstrap(self.muto)
        rospy.Subscriber(rospy.get_param("muto_composer/muto/stack_topic"),MutoAction, self.on_stack_callback, queue_size=100)




    def bootstrap(self, context):
        if  self.edge_device == None:
            self.twin = twin.Twin(node='muto_composer',config=context, publisher=self.twinPublisher)
            self.edge_device = edge.EdgeDevice(self.twin, context)
            self.pipelines = {}
            pipelinesSpec = context["pipelines"]
            for pipelineItem in pipelinesSpec:
                pipeline = Pipeline(self.edge_device, pipelineItem["name"], pipelineItem["pipeline"], pipelineItem.get("compensation",None))
                self.pipelines[pipelineItem["name"]] = pipeline


            self.edge_device.bootstrap()
            self.router = router.Router(self.edge_device, self.pipelines)

    # Input data is AckermannDriveStamped message from nav topic
    # Publishes velocity and steering angle to twin channel
    def on_stack_callback(self, msg):
        if msg:
            # context = json.loads(msg.context)
            # self.bootstrap(context)
            stack =  json.loads(msg.payload)
            self.router.route(msg.method, stack)



def main():
    rospy.init_node(name="muto_composer",anonymous=True)
    C = MutoComposer()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main()
