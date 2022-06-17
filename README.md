# Intersection driving for Duckiebots DB18.

**NOTE:** Based on duckietown/template-ros.

**Considerations:** 

* Road users: vehicles (duckiebtos) and pedestrians (duckies).
* Static scenarios.
* Intersections of 4 perpendicular branches, planar ground and given map.
* Vehicles just perform 90° turns.
* Pedestrians only cross the street between adjacent blocks.
* Priority norms accoring to Colombia's National Transit Code.

## 

### 1. Stop sign detector >> package: stop finder

The first step to cross an intersection is to detect where it is. In duckietown stop signs are solid red lines located in the ground at the point where a lane has reached an intersection.

intersection view.

This package is in charge of detecting the stop sign located at the robot's current lane and estimating the horizontal distance to it. For this purpose, it uses open cv tools to process the image, and the distance is calculated with a function obtained experimentally. To take into account, received images are cropped because ground is considered to be planar, so that just the lane in front of the robot is analysed in order to minimize detection mistakes.

The processed image is obtained by subscribing to the duckiebot's camera driver topic. Results are published into 2 separate topics containing an image with the line detection and the estimated distance respectively.

test gif.

This test was performed in real time running on a duckiebot at 20fps with an image resolution of (416, 416)px.

### 4. Place your code

Place your code in the directory `/packages/` of
your new repository
