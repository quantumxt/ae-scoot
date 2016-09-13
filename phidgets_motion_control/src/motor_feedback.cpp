/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets motor control HC
 *  Copyright (c) 2010, Bob Mottram
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

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <phidgets_api/phidget.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
//#include <sensor_msgs/BatteryState.h>


ros::Publisher supplyVoltagePub;
ros::Publisher motorCurrentPub;
ros::Publisher motorBackEMFPub;
ros::Publisher batteryStatePub;

CPhidgetMotorControlHandle mcphid = 0;
bool initalized = false;

int serialNumber = -1;
bool enableBackEMF = false;

int AttachHandler(CPhidgetHandle phid, void *userptr){
  int serial_number;
  const char *name;

  CPhidget_getDeviceName (phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);
  ROS_INFO("%s Serial number %d attached!", name, serial_number);

  return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr){
  int serial_number;
  const char *name;

  CPhidget_getDeviceName (phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);
  ROS_INFO("%s Serial number %d detached!", name, serial_number);

  return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *Description){
    ROS_ERROR("Error handled. %d - %s", ErrorCode, Description);
    return 0;
}

int display_properties(CPhidgetMotorControlHandle phid){
  int serialNo, version, numEncoders, numInputs, numMotors, numSensors, ratiometric;
  const char* ptr;

  CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
  CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
  CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

  CPhidgetMotorControl_getEncoderCount(phid, &numEncoders);
  CPhidgetMotorControl_getInputCount(phid, &numInputs);
  CPhidgetMotorControl_getMotorCount(phid, &numMotors);
  CPhidgetMotorControl_getSensorCount(phid, &numSensors);
  CPhidgetMotorControl_getRatiometric(phid, &ratiometric);

  ROS_INFO("%s\n", ptr);
  ROS_INFO("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
  ROS_INFO("# Encoders: %d\n", numEncoders);
  ROS_INFO("# Inputs: %d\n", numInputs);
  ROS_INFO("# Motors: %d\n", numMotors);
  ROS_INFO("# Sensors: %d\n", numSensors);
  ROS_INFO("Ratiometric: %d\n", ratiometric);

	return 0;
}

bool attach(CPhidgetMotorControlHandle &phid, int serial_number){

  //create the InterfaceKit object
	CPhidgetMotorControl_create(&phid);

	//Setup life-cycle handlers
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)phid, ErrorHandler, NULL);

  // Open
	CPhidget_open((CPhidgetHandle)phid, serial_number);

  return true;
}


void disconnect(CPhidgetMotorControlHandle &phid){
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

std_msgs::Float32 publishSupplyVoltage(CPhidgetMotorControlHandle &phid) {
  double supplyVoltage = 0;
  std_msgs::Float32 msg;
  CPhidgetMotorControl_getSupplyVoltage(phid, &supplyVoltage);

  msg.data = supplyVoltage;

  supplyVoltagePub.publish(msg);
  return msg;
}


void timerCallback(const ros::TimerEvent& event) {
  if(!initalized){ return; }



  if(supplyVoltagePub.getNumSubscribers() > 0 /*|| batteryStatePub.getNumSubscribers() > 0*/){
    std_msgs::Float32 voltsMsg = publishSupplyVoltage(mcphid);
    /*sensor_msgs::BatteryState battMsg;

    battMsg.header.stamp = ros::Time::now();
    battMsg.voltage = voltsMsg.data;
    battMsg.current = nanf();
    battMsg.charge = nanf();
    battMsg.capacity = nanf();
    battMsg.design_capacity = nanf();
    battMsg.percentage = nanf();

    batteryStatePub.publish(battMsg);*/
  }

  /*if(motorCurrentPub.getNumSubscribers() > 0){
    publishMotorCurrents(mcphid);
  }

  if(motorBackEMFPub.getNumSubscribers() > 0){
    publishBackEMF(mcphid);
  }*/
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "phidgets_motor_gpio");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  supplyVoltagePub = n.advertise<std_msgs::Float32>("/cl4_motor_gpio/supply_voltage", 1, true);
  motorCurrentPub = n.advertise<std_msgs::Float32MultiArray>("/cl4_motor_gpio/motor_current", 1, true);
  motorBackEMFPub = n.advertise<std_msgs::Float32MultiArray>("/cl4_motor_gpio/motor_back_emf", 1, true);
  //batteryStatePub = n.advertise<sensor_msgs::BatteryState>("/battery", 1, true);

  // Load parameters
  nh.getParam("serial", serialNumber);
  nh.param("enableBackEMF", enableBackEMF, false);

  if (attach(mcphid, serialNumber)) {

    display_properties(mcphid);
    initalized = true;

    ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);
    ros::spin();
    disconnect(mcphid);
  }

  return 0;
}
