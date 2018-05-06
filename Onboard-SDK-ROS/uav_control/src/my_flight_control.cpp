/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */
#include <iostream>
#include <stdio.h>
#include <fstream>

#include "uav_control/uav_task.h"
#include "uav_control/uav_flight_control.h"
#include "dji_sdk/dji_sdk.h"

//defines


// namespace
using namespace std;

//global variables
const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;

ros::Publisher UAVstatePub;


// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Vector3     localOffset;
geometry_msgs::Quaternion current_atti;

Mission square_mission;

float uav_fly_yaw = 0;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_control");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);


  //net interface pub and sub
  ros::Subscriber networkSub      = nh.subscribe("/network/control", 10, &network_state_callback);
  UAVstatePub =                     nh.advertise<std_msgs::UInt8>("/uav_control/control", 10);
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();

  if(task_init() == false)
  {
  	cout << "task init fails!" << endl;
  	return 0;
  }
  ros::spinOnce();
	//set_local_position();
  UAV_get_start_location();

  test_menu();

  ros::spin();
  return 0;
}

void UAV_state_publish(Flight_state state)
{
  std_msgs::UInt8 uav_msg;
  uav_msg.data = state;
  UAVstatePub.publish(uav_msg);
}

void UAV_get_start_location(void)
{
	square_mission.start_gps_location = current_gps;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void Mission::step()
{
  static int info_counter = 0;
  
  float speedFactor         = uav_config.fly_speed;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

  //zCmd = start_gps_location.altitude + target_offset_z;
  zCmd = target_offset_z;

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    cout << "*********************************************************************" << endl;
    cout << "target_x:" << target_offset_x << " target_y:" << target_offset_y << " target_z:" << target_offset_z << endl;
    cout << "current_x:" << localOffset.x << " current_y:" << localOffset.y << " current_z:" << localOffset.z << endl;
    cout << "dx:" << xOffsetRemaining << " dy:" << yOffsetRemaining << " dz:" << zOffsetRemaining << " dyaw:" << (yawInRad - yawDesiredRad) << endl;
    cout << "start_gps_z:" << start_gps_location.altitude << " current_gps_z:" << current_gps.altitude<<" target_gps_z:" << zCmd << endl;
  }


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter > 50)
  {
    ROS_INFO("##### Route finished....");
    finished = true;
    return;
  }
  else if(break_counter > 0)
  {
    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
    break_counter++;
    return;
  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;


    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    ctrlPosYawPub.publish(controlPosYaw);
  }

  if (std::abs(xOffsetRemaining) < 0.5 &&
      std::abs(yOffsetRemaining) < 0.5 &&
      std::abs(zOffsetRemaining) < 1 &&
      std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter
    inbound_counter ++;
    cout << "in!!!"<<endl;
  }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
      cout << "out!!!"<<endl;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter >10)
  {
    //ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > 50)
  {
    //ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;
  }
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;

  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {
    start_time = ros::Time::now();
    task_state_machine();
  }
}

bool uav_landing(void)
{
  return takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND);
}

void continue_flying(void)
{
  square_mission.step();
}

void uav_set_target(Waypoint point)
{
  square_mission.reset();
  
  square_mission.setTarget(point.x, point.y, point.z, uav_fly_yaw);
}
bool check_finished(void)
{
  if(square_mission.finished)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool check_landing(void)
{
  if(flight_status == DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 0.5)
  {
  	cout << "take off detal hight:" << current_gps.altitude - home_altitude << endl;
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}

geometry_msgs::Vector3 get_current_position(void)
{
  return localOffset;
}


bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  //uav_fly_yaw = rad2deg * toEulerAngle(current_atti).z;
}


bool set_target_yaw()
{
  uav_fly_yaw = rad2deg * toEulerAngle(current_atti).z;
}
