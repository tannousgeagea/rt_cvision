
import logging
import numpy as np
from common_utils.services.ros_manager import (
    ROSManager, 
    rospy, 
    Header, 
    Image
)

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class ImagePublisher(ROSManager):
    def __init__(self, processor_node_name:str):
        super(ImagePublisher, self).__init__(
            params=None, topics=None, msg_type=None, processor_node_name=processor_node_name
        )

    def publish_img(self, image, header, pub_topic, queue_size=1):
        success = False
        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            img_msg.header = header
            if self.processor_image_publisher == None:
                self.processor_image_publisher = rospy.Publisher(
                    pub_topic, 
                    Image, 
                    queue_size=queue_size
                    )
                
            self.processor_image_publisher.publish(img_msg)  
            success = True
        except Exception as err:
            logging.error(f"Error while publishing image to {pub_topic} in publish_img: {err}")
            print(err)
            
        return success


ros_manager = ImagePublisher(processor_node_name='visualizer')
ros_manager.init_node()

def execute(cv_image:np.ndarray, topic:str, queue_size:int=1, publish_rate:int=30):
    success = False
    try:
        header = Header()
        header.stamp = rospy.Time.now()
        rate = rospy.Rate(publish_rate)
        success = ros_manager.publish_img(cv_image, header, topic, queue_size)
        rate.sleep()
        
    except Exception as err:
        logging.error(f"Error while publishing image to {topic} in visualize module: {err}")
        print(err)
    return success