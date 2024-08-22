# wa_research_object_segmentation_classifier

this package is used to process images stored in rosbag / or published through a ros topic in live mode.

# Installation
```

git clone -b develop ssh://git@212.201.48.55:33033/wasteavengers/wa_research_object_segmentation_classifier.git

cd wa_research_object_segmentation_classifier

docker compose up --build -d   # for the first time

or

docker compose up -d


```

# Run

```
docker ps
```

you should get something like this

```
CONTAINER ID   IMAGE                                                       COMMAND                  CREATED          STATUS          PORTS     NAMES
04bdb85c9580   wasteant/wasteant:cvision-dl-ops-ubuntu.20.04-cuda.11.5.2   "/bin/sh -c '/bin/ba…"   32 minutes ago   Up 32 minutes             cvision-dl-ops-dev-core
2d4fce4736f0   ros:noetic-ros-core                                         "/ros_entrypoint.sh …"   32 minutes ago   Up 32 minutes             cvision-dl-ops-dev-roscore
```

then

```
docker exec -it cvision-dl-ops-dev-core bash


cd /home/appuser/src

python3 main.py
```

you should expect this output

```
2024-07-03 10:00:57,761 - INFO - PARAMETERS:
2024-07-03 10:00:57,761 - INFO - weights                   	 yolov8.brewa_blu_s01_v0.2.pt
2024-07-03 10:00:57,761 - INFO - conf                      	 0.05
2024-07-03 10:00:57,761 - INFO - mode                      	 detect
2024-07-03 10:00:57,761 - INFO - processor_node_name       	 processor_waste_impurity_detection_execution
2024-07-03 10:00:57,761 - INFO - processor_pub_topic       	 /processor_waste_impurity_detection_execution/result
2024-07-03 10:00:57,761 - INFO - topics                    	 ['/stereo_ueye_cam_1/right/image_raw/compressed']
2024-07-03 10:00:57,761 - INFO - msg_type                  	 ['compressed_image']
2024-07-03 10:00:57,761 - INFO - show_roi                  	 True
2024-07-03 10:00:57,761 - INFO - estimate_object_size      	 True
2024-07-03 10:00:57,761 - INFO - show_object_size          	 True
2024-07-03 10:00:57,761 - INFO - show_tracker_id           	 False
2024-07-03 10:00:57,761 - INFO - show_legend               	 True
2024-07-03 10:00:57,761 - INFO - save_image                	 True
2024-07-03 10:00:57,761 - INFO - save_txt                  	 False
2024-07-03 10:00:57,761 - INFO - object_length_threshold   	 [0, 0.5, 1.0, 2]
2024-07-03 10:00:57,761 - INFO - object_length_color_threshold 	 [[0, 255, 0], [0, 255, 255], [0, 165, 255], [0, 0, 255]]
2024-07-03 10:00:57,761 - INFO - object_size_factor        	 0.002
2024-07-03 10:00:58,502 - INFO - Init processor_waste_impurity_detection_execution
Listening to Topics: ['/stereo_ueye_cam_1/right/image_raw/compressed']
MSG Type: [<class 'sensor_msgs.msg._CompressedImage.CompressedImage'>]

```

as you can see now, we are Listening to topic /stereo_ueye_cam_1/right/image_raw/compressed with msg_type CompressedImage. To Change the topic and the msg_type please refer to the Configuration below.


## Play RosBag
from anywhere in your host, or as long you are in the same netword of roscore (for now http://localhost:11311), to change it check docker-compose.yml, run from a different terminal:

```
rosbag play bagfile.bag
```

the images or files are saved in /home/appuser/data or at your host at the same lever as the src folder


# [Configuration](http://212.201.48.55:800/wasteavengers/wa_research_object_segmentation_classifier/-/blob/develop/src/config/dev.yaml)

## Model Configuration
- **weights**: Specifies the pre-trained weights file for the YOLOv8 model.
  - `yolov8.brewa_blu_s01_v0.2.pt`
- **conf**: Sets the confidence threshold for detections.
  - `0.05`
- **mode**: Determines the operational mode of the model.
  - `detect` (alternative: `track`)

## ROS Configuration
- **processor_node_name**: Name of the ROS node responsible for processing waste impurity detection.
  - `processor_waste_impurity_detection_execution`
- **processor_pub_topic**: The topic on which the processor node publishes results.
  - `/processor_waste_impurity_detection_execution/result`
- **topics**: List of topics to subscribe to for receiving input images.
  - `/stereo_ueye_cam_1/right/image_raw/compressed`
- **msg_type**: The type of message being processed.
  - `compressed_image`

## Display and Output Settings
- **show_roi**: Boolean flag to show the region of interest.
  - `true`
- **estimate_object_size**: Enables the estimation of object size.
  - `true`
- **show_object_size**: Displays the size of the detected objects.
  - `true`
- **show_tracker_id**: Toggles the display of tracker IDs.
  - `false`
- **show_legend**: Displays a legend on the output image.
  - `true`
- **save_image**: Saves the output image with detections.
  - `true`
- **save_txt**: Saves the detection results in a text file.
  - `false`

## Object Length and Color Thresholds
- **object_length_threshold**: Defines thresholds for object lengths in meters.
  - `[0, 0.5, 1.0, 2.0]`
- **object_length_color_threshold**: Specifies the color associated with each length threshold in BGR format.
  - `[0, 255, 0]` for objects between 0 and 0.5 meters
  - `[0, 255, 255]` for objects between 0.5 and 1.0 meters
  - `[0, 165, 255]` for objects between 1.0 and 2.0 meters
  - `[0, 0, 255]` for objects larger than 2.0 meters

## Additional Parameters
- **object_size_factor**: A factor used to scale the size of the objects.
  - `0.002`
