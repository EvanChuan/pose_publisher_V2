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

#define X_WEIGHT 1
#define Y_WEIGHT 1
#define RADIUS 3 

people_msgs::Person legs_published;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
            if(distance<=RADIUS)
            {
               closer_legs.push_back(legs->people[i]); 
            }
        }
    }
    
    length = closer_legs.size();
    cout << "legs within "<<RADIUS<<" :"<<length<<endl;
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
    double x_prev = 0, y_prev = 0;

    ros::init(argc,argv,"velocity_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub_3=n.subscribe("amcl_pose",1,robot_position_callback);
    ros::Subscriber sub_1=n.subscribe("people",5,leg_detector_callback);
    ros::Subscriber sub_2=n.subscribe("tracker_angles",1,headercallback);
    

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("/move_base", true);
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    while(ros::ok())
    {
        ros::Duration(1).sleep();
        if(tracking)
        {
            double x_arr=(legs_published.position.x-bot_pos.position.x)*0.7;
            double y_arr=(legs_published.position.y-bot_pos.position.y)*0.7;

            double angle_on_map = 2*asin(bot_pos.orientation.z);


            double x_final=bot_pos.position.x+x_arr;
            double y_final=bot_pos.position.y+y_arr;
            double angle_final = angle_on_map + heading_angle;

            double x_dif = x_final-x_prev;
            double y_dif = y_final-y_prev;
            if( (x_dif*x_dif+y_dif*y_dif)>0.15*0.15 )//goals are not too close
            {
                move_base_msgs::MoveBaseGoal goal;

                //we'll send a goal to the robot to move 1 meter forward
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = x_final;
                goal.target_pose.pose.position.y = y_final;
                goal.target_pose.pose.orientation.z = sin(angle_final/2);
                goal.target_pose.pose.orientation.w = cos(angle_final/2);

                x_prev = x_final;
                y_prev = y_final;

                cout <<"x_final"<< x_final <<endl;
                cout <<"y_final"<< y_final <<endl;
                cout <<"angle_final"<< angle_final/3.14*180 << endl;


                ac.sendGoal(goal);

                //ac.waitForResult();
                /*
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO("tracking");
                else
                    ROS_INFO("no traget");*/
            }
        }
        ros::spinOnce();
    }
    
   


}


