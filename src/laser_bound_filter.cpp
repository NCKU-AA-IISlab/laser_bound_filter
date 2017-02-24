/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, (ROC) Advanced Robotics, Ltd.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Advanced Robotics, Ltd. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: HaoChih, LIN
* Email: f44006076@gmail.com
*********************************************************************/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

#define PI_RAD 3.14159265358979323846264338327950288

class LaserFilter
{
  public:
    LaserFilter();
    double lower_bound_;
    double upper_bound_;
    bool inner_bound_flag_;

  private:
    ros::NodeHandle nh_;                          
    ros::Subscriber scan_sub;			     
    ros::Publisher scan_pub;                   
    
    void scan_callback(const sensor_msgs::LaserScan& scan_origin);
};


//-------Set up all members in the "ArdroneControl class"------
LaserFilter::LaserFilter()
{
  lower_bound_ = -1.95;
  upper_bound_ = 1.95;
  inner_bound_flag_ = false;
  nh_.param("lower_bound", lower_bound_, lower_bound_ ); //rad
  nh_.param("upper_bound", upper_bound_, upper_bound_ ); //rad
  nh_.param("inner_bound_flag", inner_bound_flag_, inner_bound_flag_ ); //rad
  if(lower_bound_ <= -PI_RAD)
  {
    lower_bound_ = -PI_RAD;
    ROS_ERROR("The lower bound is smaller than minimum, reset as -PI");
  }
  if(upper_bound_ >= PI_RAD)
  {
    upper_bound_ = PI_RAD;
    ROS_ERROR("The upper bound is larger than maximum, reset as PI");
  }

  scan_sub = nh_.subscribe("/scan", 1, &LaserFilter::scan_callback, this);
  scan_pub = nh_.advertise<sensor_msgs::LaserScan>("/scan_bound",50, true);
}


void LaserFilter::scan_callback(const sensor_msgs::LaserScan& scan_origin)
{
  sensor_msgs::LaserScan scan_bound;
  unsigned int scan_ori_size = scan_origin.ranges.size();
  scan_bound.header = scan_origin.header;
  scan_bound.angle_increment = scan_origin.angle_increment;
  scan_bound.time_increment = scan_origin.time_increment;
  scan_bound.scan_time = scan_origin.scan_time;
  scan_bound.range_min = scan_origin.range_min;
  scan_bound.range_max = scan_origin.range_max;
  
  int resize_value = (int)(   (abs(scan_origin.angle_max) + abs(scan_origin.angle_min) - (abs(upper_bound_) + abs(lower_bound_))) / scan_origin.angle_increment );
  if(resize_value >= scan_ori_size)
    resize_value = scan_ori_size;
  scan_bound.ranges.resize(resize_value);
  scan_bound.intensities.resize(resize_value);
  
  if(!inner_bound_flag_)
    scan_bound.header.frame_id = "laser_bound";
  
  unsigned int count = 0;

  if(inner_bound_flag_)
  {
    float current_angle = (scan_origin.angle_min);
    scan_bound.angle_min = lower_bound_;
    scan_bound.angle_max = upper_bound_;
    for(unsigned int i = 0; i < scan_ori_size; ++i)
    {
      if(current_angle >= lower_bound_ && current_angle <= upper_bound_)
      {
        scan_bound.intensities[count] = scan_origin.intensities[i];
        scan_bound.ranges[count] = scan_origin.ranges[i];
        count++;
      }
      
      if(current_angle > upper_bound_ || count >= resize_value)
        break;
      current_angle = current_angle + scan_bound.angle_increment;
    }//end of for-loop
  }
  else //For outer bound mode, only support 360 laser scan
  {
    int first_half_id  = abs( (int)( (-PI_RAD - lower_bound_)/scan_bound.angle_increment  ) );
    int second_half_id = abs( (int)( ( PI_RAD - upper_bound_)/scan_bound.angle_increment  ) );
    scan_bound.angle_min = -PI_RAD - lower_bound_;
    scan_bound.angle_max =  PI_RAD - upper_bound_;
    //int second_start_id = (int)( (scan_origin.range_min + upper_bound_)/scan_bound.angle_increment );
    for(unsigned int i = 0; i < first_half_id; ++i)
    {
        scan_bound.intensities[count] = scan_origin.intensities[first_half_id - i];
        scan_bound.ranges[count] = scan_origin.ranges[first_half_id - i];
        count++;
    }
    for(unsigned int i = 0; i < second_half_id; ++i)
    {
        scan_bound.intensities[count] = scan_origin.intensities[scan_ori_size - i];
        scan_bound.ranges[count] = scan_origin.ranges[scan_ori_size - i];
        count++;
        if(count >= resize_value)
          break;
    }
  }

  // Publish the result
  scan_bound.ranges.resize(count);
  scan_bound.intensities.resize(count);
  scan_pub.publish(scan_bound);
}


//------------Main function-----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Laser_Bound_Filter");
  ROS_INFO("=== Laser Bound Filter Start ===");  
  LaserFilter laser_filter;

  while( ros::ok() )
    ros::spin();

  return EXIT_SUCCESS;
}
