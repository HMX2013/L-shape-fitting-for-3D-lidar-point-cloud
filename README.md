# Lidar Cluster Shape Estimation
L-shape fitting implementation of the paper:

## Reference
* Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners}
* 


### How to use
* `roslaunch lidar_shape_estimation L_shape_estimation_clustering.launch`

Launch files also include the visualization node.

## Requirements

1. LiDAR data segmented. 
1. Objects 

## Parameters

|Parameter| Type| Description|Default|
----------|-----|--------|---|
|`input`|*String*|Topic name containing the objects detected by the Lidar in 3D space.|`/detection/lidar_detector/objects`|
|`output`|*String*|Topic name containing the objects with the shape estimated in 3D space.|`/detection/lidar_shape_estimation/objects`|

## Usage example

1. Launch a ground filter algorithm from the `Points Preprocessor` section in the **Sensing** tab. (adjust the parameters to your vehicle setup).
1. Launch a Lidar Detector from the Computing tab.
1. Launch this node.

## Node info

```
Node [/lidar_shape_estimation]
Publications: 
 * /detection/shape_estimation/objects [autoware_msgs/DetectedObjectArray]

Subscriptions: 
 * /detection/lidar_detector/objects [autoware_msgs/DetectedObjectArray]
 
-------------------------
Node [/detection/shape_estimation/shape_estimation_visualization]
Publications: 
 * /detection/shape_estimation/objects_markers [visualization_msgs/MarkerArray]

Subscriptions: 
 * /detection/shape_estimation/objects [autoware_msgs/DetectedObjectArray]
```