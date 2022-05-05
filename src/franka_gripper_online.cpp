#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>

double width =0;
double flag =0;

void chatterCallback(const std_msgs::Float32::ConstPtr& msg)
{
  width=msg->data;
  flag = 1;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gripper");


  ros::NodeHandle n;
  ros::Rate loop_rate(30);

  ros::Subscriber sub = n.subscribe("gripper_online", 1000, chatterCallback);
  ros::Publisher pub = n.advertise<franka_gripper::MoveActionGoal>("/franka_gripper/move/goal", 1000);

  franka_gripper::MoveActionGoal msg;
  msg.goal.speed = 1;

  while (ros::ok())
  {
   if(flag==1)
   {
     msg.goal.width = width;
     
     pub.publish(msg);
     flag = 0;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  




  //ros::spin();


  return 0;
}

