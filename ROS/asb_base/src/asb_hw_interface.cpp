/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the Ommp
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <asb_base/asb_hw_interface.h>

namespace asb_base
{
AsbHWInterface::AsbHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
    ROS_INFO_NAMED("Asb_hw_interface", "AsbHWInterface Ready.");

    //confusing with nh_.
    wheel_vel_pub = nh_.advertise<std_msgs::Float32MultiArray>("set_vel", 10);
    wheel_enc_sub = nh_.subscribe("/encoder_ticks", 10, &AsbHWInterface::enc_ticks_CB, this); //queue 10 whats "this" sending
}

void AsbHWInterface::read(ros::Duration &elapsed_time)
{
    double wheel_angles[4];
    double wheel_angle_deltas[4];
    for (int i = 0; i < 4; i++)
    {
        wheel_angles[i] = ticksToAngle(curr_enc[i]); //radians travelled 
        //change in position (rad)
        wheel_angle_deltas[i] = wheel_angles[i] - joint_position_[i]; //delta radians travelled
        joint_position_[i] += wheel_angle_deltas[i]; //position is deltas + last position in radians, it will calculate the net position
        joint_velocity_[i] = wheel_angle_deltas[i] / elapsed_time.toSec(); //
    }
}

void AsbHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  this -> cmd_to_setpoint();
  // Transform From joint_velocity_command to setpoint
  vel_setpoint_cmd_array.data.push_back(vel_set_cmd[0]);
  vel_setpoint_cmd_array.data.push_back(vel_set_cmd[1]);
  wheel_vel_pub.publish(vel_setpoint_cmd_array);
  // Clear Array
  vel_setpoint_cmd_array.data.clear(); 
}

void AsbHWInterface::enforceLimits(ros::Duration &period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  
  // Enforces velocity and acceleration limits
  vel_jnt_sat_interface_.enforceLimits(period);
}

void AsbHWInterface::enc_ticks_CB(const std_msgs::Int64MultiArray::ConstPtr &enc_msg) //need to look into enc_msg
{
  curr_enc[0] = enc_msg->data[0]; //FL
  curr_enc[1] = enc_msg->data[1]; //BL
  curr_enc[2] = enc_msg->data[2]; //FR
  curr_enc[3] = enc_msg->data[3]; //BR
}

//feel like this should be changed with ticks per meter measure, also need to be careful about long -> int
double AsbHWInterface::ticksToAngle(const int &ticks) const
{
  double angle = (double)ticks * ((2.0*M_PI) / 11231); //256 pulses per channel x 2 x 2 x 10.71 = 10967 
  return angle; //(experimental value is close so trying it to match with arduino)
}

void AsbHWInterface::cmd_to_setpoint() //TEST, assumed it sends a rad/s, is this really the case? 
{
  // On the same side they recieve the same command, rad/s to m/s is v = rw 
  vel_set_cmd[0] = joint_velocity_command_[0] * 0.09525;  // back left -> = left
  vel_set_cmd[1] = joint_velocity_command_[2] * 0.09525;  // back right -> = right
}
} //namespace asb_base