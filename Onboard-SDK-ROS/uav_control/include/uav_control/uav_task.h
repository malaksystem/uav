#ifndef __UAV_TASK_H
#define __UAV_TASK_H

#include "std_msgs/UInt8.h"



typedef struct
{
    int max_point;
    uint fly_height;
    float fly_speed;
} configuration;

extern configuration uav_config;

void network_state_callback(const std_msgs::UInt8::ConstPtr& msg);
int test_menu(void);
void task_state_machine(void);
bool task_init(void);
void uav_clime(void);

#endif


