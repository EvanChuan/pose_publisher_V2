#include"ros/ros.h"
#include"geometry_msgs/Pose.h"
#include"geometry_msgs/Point.h"
#include"geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/Twist.h>
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

#define X_WEIGHT 1
#define Y_WEIGHT 1
#define RADIUS 3 

people_msgs::Person legs_published;

geometry_msgs::Pose bot_pos;
double heading_angle=0;
bool tracking=false;

void leg_detector_callback( const people_msgs::People::ConstPtr & legs)
{
    double x_vel,y_vel,final_vel,x_tmp,y_tmp,final_tmp,distance;
    int length = legs->people.size();
    vector<people_msgs::Person> closer_legs;
    cout <<"detecting "<<length<<" legs"<<endl;
    if(length!=0)
    {
        for(int i=0;i<length;++i)
        {
            distance=pow(legs->people[i].position.x-bot_pos.position.x,2)+pow(legs->people[i].position.y-bot_pos.position.y,2);
            distance=sqrt(distance);
            if(distance<=RADIUS)//Only consider people within radius
            {
               closer_legs.push_back(legs->people[i]); 
            }
        }
    }
    
    length = closer_legs.size();
    //cout << "legs within "<<RADIUS<<" :"<<length<<endl;
    if(length!=0)
    {
        x_vel=abs(legs->people[0].velocity.x);
        y_vel=abs(legs->people[0].velocity.y);
        final_vel=X_WEIGHT*x_vel+Y_WEIGHT*y_vel;
        legs_published=legs->people[0];

        for(int i=1;i<length;++i)
        {
            x_tmp=abs(legs->people[i].velocity.x);
            y_tmp=abs(legs->people[i].velocity.y);
            final_tmp=X_WEIGHT*x_tmp+Y_WEIGHT*y_tmp;

            if(final_tmp>final_vel)
            {
                final_vel=final_tmp;
                legs_published=legs->people[i];
            }
        }
        tracking=true;
    }
    else 
    {
        tracking=false; 
    }

    //***************************************Add command velocities*********************************************
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    double rot_vel_scale = 10.0;//need test
    double lin_vel_scale = 5.0;//need test

    double x_arr=(legs_published.position.x-bot_pos.position.x)*0.7;
    double y_arr=(legs_published.position.y-bot_pos.position.y)*0.7;
    distance=sqrt(x_arr*x_arr+y_arr*y_arr);
    distance *= cos(heading_angle);//only consider distance in forward direction

    if(tracking)//if not tracking all velocites are 0
    {
        if(abs(heading_angle) > 3.14/18)//heading error > 10 degree
        {
            cmd->angular.z = heading_angle*rot_vel_scale;//P controller
        }
        if(distance > 0.1)//distance to goal > 10cm
        {
            cmd->linear.x = distance*lin_vel_scale;//P controller
        }
        cout << "Rotation speed: " << cmd->angular.z << endl;
        cout << "Linear speed: " << cmd->linear.x << endl;
    }

    cmdpub.publish(cmd);
    //***********************************************************************************************************
}

void headercallback(const std_msgs::Float64MultiArray::ConstPtr & msg)
{
  heading_angle=(msg->data[1]+msg->data[0])/2;
}


void robot_position_callback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos)
{
    bot_pos=pos->pose.pose;
}

int main (int argc, char ** argv)
{
    ros::init(argc,argv,"velocity_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub_3=n.subscribe("amcl_pose",1,robot_position_callback);
    ros::Subscriber sub_1=n.subscribe("people",5,leg_detector_callback);
    ros::Subscriber sub_2=n.subscribe("tracker_angles",1,headercallback);
    ros::Publisher cmdpub = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1);

    ros::spin();
    return 0;
}