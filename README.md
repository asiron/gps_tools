# gps_tools
ROS package for converting GPS measurements (NavSatFix) into local Euclidean frame using WGS-84 ellipsoid

Copyright (C) 2017 - Maciej Żurad, University of Luxembourg

### Overview

This packages is composed of 2 ROS nodes:

- `gps_reference.py` - loads and saves a NavSatFix message (serialized to YAML format), which is the GPS reference used to convert other GPS measurements into Euclidean space
- `gps_to_local_euclidean.py` - takes a GPS reference and current GPS measurement and outputs the position in Euclidean space as a `PointStamped` msg

Usually, you have an `/earth` coordinate frame and a `/world` (`/map`) coordinate frame. By saving GPS reference and your `/earth` to `/world` transform, you can consistently reproduce your global `/world` frame after rebooting, or even coming back to the site. 
It helps a lot when you need to evaluate algorithms and need ground truth data!

### Sample usage

Initially, you want to set your GPS reference so launch both nodes (without autoset):

```
$ roslaunch gps_tools gps_reference.launch earth_frame_id:=earth autoset_geo_reference:=false gps_reference_output_directory:=/my/path/to/saved/references
$ roslaunch gps_tools gps_to_local_euclidean.launch earth_frame_id:=earth local_euclidean_position_topic:=local_position
```
Now, once your confident that GPS condition are good (no multipath, clear sky, etc), save your reference by calling
```
$ rosservice call set_geo_reference
```
The node will take 30 measurements, calculate covariance matrix and if it's above thresholds, it won't set the reference and dump it to a file.
Otherwise, the message will be saved to the previously specified directory with a filename: `gps_reference_%Y%m%d-%H%M%S.yml` e.g. `gps_reference_20180920-113000.yml`
You can then load this reference (for example after reboot) with:
```
$ roslaunch gps_tools gps_reference.launch earth_frame_id:=earth autoset_geo_reference:=false autoset_geo_reference_file:=gps_reference_20180920-113000 gps_reference_output_directory:=/my/path/to/saved/references
```

### Python dependecies

You have to have the following Python dependencies installed:
```
pip install nvector numpy tqdm

```
