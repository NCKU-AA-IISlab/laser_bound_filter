# laser_bound_filter

## Input
-> topic: /scan (sensor_msgs/LaserScan)  
-> param: lower_bound (in rad)  
-> param: upper_bound (in rad)  
-> param: inner_bound_flag (bool), for determining whether or not the inner bound cut mode will be set (see below for detail)  
  
## Output
-> topic: /scan_bound  
Notice:  
for outer bound mode, the frame_id of output msg will be replaced as "laser_bound"  
in this case, you need to publish static tf to link between /laser and /laser_bound  

## Demo
Please run $ roslaunch laser_bound_filter rplidar_bound_filter.launch

## Mode Selection
This node supports two different bound cutting modes: inner bound v.s. outer bound.  
#### inner bound:  
The remaining area (the area you want to keep) is in the middle of overall range, for example,   
the overall range of laser scan is -150 ~ 150 (deg), only -60 ~ 60 is the wanted area, in this situation, please use this mode.  
#### outer bound:
In contrast, the outer mode is for the usage of cutting area in the middle. To use this mode, the laser scanner have to   
be able to provide 360-deg scan data. For instance, the unwanted are is from -80 ~ 70, then this node will rearrange data   
from - 100 ~ 110. Hence, the external static tf is required to reproject output scan result to original direction.

