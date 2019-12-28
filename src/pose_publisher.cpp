#include"ros/ros.h"
#include"geometry_msgs/Pose.h"
#include"geometry_msgs/Point.h"
#include"geometry_msgs/PoseWithCovarianceStamped.h"
#include"sensor_msgs/LaserScan.h"
#include"people_msgs/People.h"
#include"people_msgs/Person.h"
#include <move_base_msgs/MoveBaseAction.h>
#include"std_msgs/Float64.h"
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64MultiArray.h"
#include<cmath>
#include<iostream>
#include<vector>
using namespace std;

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("/move_base", true);
    

void goal_callback(const geometry_msgs::Pose::ConstPtr & _pose)
{    
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = _pose->position.x;
    goal.target_pose.pose.position.y = _pose->position.y;
    goal.target_pose.pose.orientation.z = _pose->orientation.z;
    goal.target_pose.pose.orientation.w = _pose->orientation.w;
    ac.sendGoal(goal);

    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("tracking");
    else
        ROS_INFO("no traget");

}

int main (int argc, char ** argv)
{

    ros::init(argc,argv,"pose_publisher_v2");
    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("pose_queue",10,goal_callback);
    ros::Duration(0.5).sleep(); 
    ros::spin();
}