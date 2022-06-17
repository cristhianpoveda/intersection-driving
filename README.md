# Intersection driving for Duckiebots DB18.

**NOTE:** Based on duckietown/template-ros.

**Considerations:** 

* Road users: vehicles (duckiebtos) and pedestrians (duckies).
* Static scenarios.
* Intersections of 4 perpendicular branches, planar ground and given map.
* Vehicles just perform 90° turns.
* Pedestrians only cross the street between adjacent blocks.
* Priority norms accoring to Colombia's National Transit Code.
* Negotiation refers to avoid collisions inside conflict zones without muli-agent communication.

![Alt text](/repoImages/Intersection.jpg?raw=true "Optional Title")

## 

### Duckiebot setup
For ensuring real time execution during the object detection functionality the following parameters have to be tuned until there is no delay on the image topic.

`rosparam set /ROBOTNAME.local/camera_node/framerate fps` fps = camera framerate in fps.

`rosparam set /ROBOTNAME.local/camera_node/res_h resH` resH = desired camera image height.

`rosparam set /ROBOTNAME.local/camera_node/res_w resW` resW = desired camera image width.

### 1. Stop sign detector >> package: stop finder

The first step to cross an intersection is to detect where it is. In duckietown stop signs are solid red lines located in the ground at the point where a lane has reached an intersection.

This package is in charge of detecting the stop sign located at the robot's current lane and estimating the horizontal distance to it. For this purpose, it uses open cv tools to process the image, and the distance is calculated with a function obtained experimentally. To take into account, received images are cropped because ground is considered to be planar, so that just the lane in front of the robot is analysed in order to minimize detection mistakes.

The processed image is obtained by subscribing to the duckiebot's camera driver topic. Results are published into 2 separate topics containing an image with the line detection and the estimated distance respectively.

![](https://github.com/cristhianpoveda/intersection-driving/blob/v2/repoImages/stopFinder.gif)

This test was performed in real time running on a duckiebot at 20fps with an image resolution of (416, 416)px.

### 2. Road users detection >> package: detection

A key factor to interact with the envirnoment is to detect who else is using the road space in order to avoid collisions, and this task must be done in  real time. Therefore, model precision and RAM ussage in the robot are taken into account.

This is done by implementing the Edge Impulse FOMO model for object detection (input image resolution 320 x 320 px) as a c++ library running directly on the duckiebot without the need of any external dependency. Model's input is a 1d array containing the flattened image pixels in format (0xRRGGBB); this array is created in the python script image_processor.py by subscribing to the camera node and processed using opencv tools.

#### Model metrics

![Alt text](/repoImages/Models.jpg?raw=true "Optional Title")

**Note:** Models were tested on the duckiebot's raspberry pi 3 (1GB RAM) by running the following c++ codes containing: inference script and model library.

![Alt text](/repoImages/InferenceT.jpg?raw=true "Optional Title")
Inference time by the duckiebot using a previous loaded image. First with quantized (int 8) model and then with float32 model.

**FOMO model c++ library repos**

[float 32 model repo](https://github.com/cristhianpoveda/prueba_c)

[Quantized (int 8) model repo](https://github.com/cristhianpoveda/prueba_quant)

#### Object detection models tested
