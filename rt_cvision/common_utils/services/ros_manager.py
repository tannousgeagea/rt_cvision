#!/usr/bin/env python3

"""
 * Copyright (C) WasteAnt - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential
 * See accompanying/further LICENSES below or attached
 * Created by Tannous Geagea <tannous.geagea@wasteant.com>, December 2024
 * Edited by:
 *
"""

import json
import rospy
import logging
import message_filters
from datetime import datetime
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage


def default_process_messages(messages):
    print(f'DEFAULT CALLBACK: {messages.keys()}')
    

class ROSManager:
    def __init__(
        self,
        params=None,
        topics=None,
        msg_type=None,
        callback=None,
        processor_node_name=None,
    ):
        self.bridge = CvBridge()
        
        self.params = params if not params is None else {}
        self.topics = self.params['topics'] if 'topics' in self.params.keys() else topics
        self.msg_type = self.params['msg_type'] if 'msg_type' in self.params.keys() else msg_type
        self.processor_node_name = self.params["processor_node_name"] if "processor_node_name" in self.params.keys() else processor_node_name
        
        self.callback = callback
        
        if isinstance(self.topics, str):
            self.topics = [self.topics]

        if isinstance(self.msg_type, str):
            self.msg_type = [self.msg_type]

        if self.msg_type:
            self.msg_type = [self.checkMsgType(msg) for msg in self.msg_type]
        
        self.processor_result_publisher = None
        self.processor_image_publisher = None

    def init_node(self):
        """initialize ros node"""
        try:
            logging.info("Init %s" % self.processor_node_name)
            rospy.init_node(self.processor_node_name, anonymous=True)
        except Exception as err:
            logging.error("Unexpected Error where initializing ros nore: %s" % err)
            
    def listener_on(self, queue_size=10):
        try:
            assert not self.topics is None, f'No topics were Found !'
            assert not self.msg_type is None, f'msg_type are None'
            assert not self.callback is None, f'no callback function were given'
            
            print(f"Listening to Topics: {self.topics}")
            print(f"MSG Type: {self.msg_type}")
            if self.topics:
                self.listen_to_topic(topic=self.topics[0], msg_type=self.msg_type[0], queue_size=queue_size)
            else:
                logging.warning("No topic was Found")
        except Exception as err:
            logging.error("Unexpected Error where listening: %s" % err)


    def listen_to_topic(self, topic, msg_type, queue_size:int=10):
        """Listen to exactly one topic"""
        try:
            rospy.Subscriber(topic, msg_type, self.callback, queue_size=queue_size)
            rospy.spin()
        except Exception as err:
            logging.error(
                "Unexpected Error while listening to topic %s: %s"
                % (self.topics[0], err)
            )

    def pub_result(self, result, header, pub_topic, msg_type, queue_size=10):
        """
        This function publishes an message to a designated topic on the ROS network.
        The message contains information about a processing result or event and is intended for dashboard visualization.

        Args:
            result (json): result message to be published.
            header (rospy.Header): The header information associated with the alarm message, typically containing timestamp and frame ID.

        Returns:
            None: The function publishes the alarm message to the specified ROS topic for visualization.
        """
        try:
            if self.processor_result_publisher == None:
                self.processor_result_publisher = rospy.Publisher(
                    pub_topic, msg_type, queue_size=queue_size
                )

            msg = msg_type()
            msg.header = header
            msg.data = str(result)
            self.processor_result_publisher.publish(msg)

        except Exception as err:
            logging.error("Unexpected error in publishing: %s" % err)

    def checkMsgType(self, msg):
        if msg == "image":
            return Image
        elif msg == "compressed_image":
            return CompressedImage
        else:
            raise ValueError(f"Message type is not defined {msg}")
        
    @property 
    def message_mapping(self, msg):
        return {
            Image: self.bridge.imgmsg_to_cv2(msg, 'bgr8'),
            CompressedImage: self.bridge.compressed_imgmsg_to_cv2(msg),
        }
        
        
        