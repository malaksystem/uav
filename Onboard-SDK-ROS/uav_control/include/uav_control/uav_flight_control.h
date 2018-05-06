/** @file demo_flight_control.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef UAV_FLIGHT_CONTROL_H
#define UAV_FLIGHT_CONTROL_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#include "network_control/network_control.h"

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))





typedef struct Waypoint
{
  float x;
  float y;
  float z;
}Waypoint;



/*!
 * @brief a bare bone state machine to track the stage of the mission
 */
class Mission
{
public:
  // The basic state transition flow is:
  // 0---> 1 ---> 2 ---> ... ---> N ---> 0
  // where state 0 means the mission is note started
  // and each state i is for the process of moving to a target point.
  int state;

  int inbound_counter;
  int outbound_counter;
  int break_counter;

  float target_offset_x;
  float target_offset_y;
  float target_offset_z;
  float target_yaw;
  sensor_msgs::NavSatFix start_gps_location;

  bool finished;

  Mission() : state(0), inbound_counter(0), outbound_counter(0), break_counter(0),
              target_offset_x(0.0), target_offset_y(0.0), target_offset_z(0.0),
              finished(false)
  {
  }

  void step();

  void setTarget(float x, float y, float z, float yaw)
  {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw      = yaw;
  }

  void reset()
  {
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    finished = false;
  }

};

void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin);

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

bool takeoff_land(int task);

bool obtain_control();

bool is_M100();

bool monitoredTakeoff();

bool M100monitoredTakeoff();


void continue_flying(void);
void uav_set_target(Waypoint point);
bool check_finished(void);
bool check_landing(void);
bool uav_landing(void);
void UAV_state_publish(Flight_state state);
geometry_msgs::Vector3 get_current_position(void);
void UAV_get_start_location(void);
bool set_local_position();
bool set_target_yaw();

#endif // UAV_CONTROL_H
