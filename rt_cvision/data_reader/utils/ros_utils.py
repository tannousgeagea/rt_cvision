import logging
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
cv_bridge = CvBridge()

def message_mapping(msg):
    """
    Converts a ROS (Robot Operating System) image message to an OpenCV image format using the appropriate 
    cv_bridge conversion function based on the message type.

    This function supports two types of ROS image messages: `sensor_msgs/Image` and `sensor_msgs/CompressedImage`.
    It identifies the type of the input message and converts it to the OpenCV image format using the corresponding 
    cv_bridge method. If the message type is not supported, it prints an error message.

    Parameters:
    - msg (sensor_msgs/Image or sensor_msgs/CompressedImage): The ROS image message to be converted.

    Returns:
    - tuple: A tuple containing two elements:
        - key (str): A string key 'cv_image' indicating the type of the output value.
        - value (numpy.ndarray): The image array in OpenCV format (BGR8 for Image, appropriate format for CompressedImage).
      Returns (None, None) if the message type is unsupported.

    Raises:
    - TypeError: If `msg` is neither a `sensor_msgs/Image` nor a `sensor_msgs/CompressedImage`.

    Example Usage:
    ```python
    import sensor_msgs.msg
    from cv_bridge import CvBridge
    cv_bridge = CvBridge()

    # Example for sensor_msgs/Image
    image_msg = sensor_msgs.msg.Image()
    key, cv_image = message_mapping(image_msg)
    if cv_image is not None:
        # cv_image can now be used with OpenCV functions
        print("Received an Image message.")

    # Example for sensor_msgs/CompressedImage
    compressed_image_msg = sensor_msgs.msg.CompressedImage()
    key, cv_image = message_mapping(compressed_image_msg)
    if cv_image is not None:
        # cv_image can now be used with OpenCV functions
        print("Received a Compressed Image message.")
    ```

    Note:
    This function is designed specifically for use within the ROS ecosystem where sensor_msgs and cv_bridge are commonly used modules.
    """
    
    if isinstance(msg, Image):
        key = 'cv_image'
        value = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
    elif isinstance(msg, CompressedImage):
        key = 'cv_image'
        value = cv_bridge.compressed_imgmsg_to_cv2(msg)
    else:
        raise ValueError('Undefined msg type: %s' % {type(msg)})

    return key, value


def extract_data_from_topic(msg, messages:dict={}):
    """
    Processes a ROS message to extract data and stores it in a dictionary using a key-value pair
    extracted by the `message_mapping` function. This function attempts to handle and log any exceptions
    that might occur during the message processing, ensuring that the function does not fail silently.

    Parameters:
    - msg (sensor_msgs/Image or sensor_msgs/CompressedImage): The ROS message that needs to be processed.
      This message should be compatible with the `message_mapping` function.
    - messages (dict, optional): A dictionary where the extracted data will be stored. Defaults to an empty dictionary.

    Returns:
    - dict: The updated dictionary containing the new key-value pair if the message was successfully processed.
      If an exception occurs, the original dictionary is returned with no changes.

    Raises:
    - None directly by this function but could log errors through the `logging` module if exceptions occur during
      message processing.

    Example Usage:
    ```python
    import sensor_msgs.msg
    from cv_bridge import CvBridge
    cv_bridge = CvBridge()

    # Assuming message_mapping and logging have been correctly set up
    image_msg = sensor_msgs.msg.Image()
    messages_dict = {}
    updated_messages = extract_data_from_topic(image_msg, messages_dict)
    if 'cv_image' in updated_messages:
        print("Image data extracted and stored in dictionary.")
    else:
        print("Failed to extract image data.")
    ```

    Notes:
    - It is important to handle exceptions inside this function to avoid disrupting the flow in applications where
      this function is a part of a larger data processing pipeline.
    - This function uses logging to report errors, which requires the `logging` module to be properly configured
      in the main application to capture these error logs.
    - The `messages` dictionary is used to accumulate results over multiple calls, which can be particularly
      useful in continuous data processing scenarios, such as reading from a live video stream.
    """
    
    try:
    
      if not "header" in messages.keys():
          messages["timestamp"] = str(msg.header.stamp)
          
      k, v = message_mapping(msg)
    
      if not k in messages.keys():
          messages[k] = v
      else:
          print(f'{k} already extracted')
    except Exception as err:
      logging.error(f'Error extracting data from ros message. Type of msg {type(msg)}: {err}')

    return messages