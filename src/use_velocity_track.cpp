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
#include<std_msgs/Bool.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

using namespace std;

#define X_WEIGHT 1
#define Y_WEIGHT 1
#define RADIUS 2.5 

people_msgs::Person legs_published;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Pose bot_pos;
double heading_angle=0;
bool tracking=false;
bool has_tracker=false;
int box_num=0;

int distance_weight(int distance)
{
    if(distance<0.6) return 0.9;
    else if(distance>=0.6&&distance<=1.5) return 1.2;
    else return 0.3; 
}

void leg_detector_callback( const people_msgs::People::ConstPtr & legs)
{
    
    double x_vel,y_vel,final_vel=0,x_tmp,y_tmp,final_tmp,distance;
    int length = legs->people.size();
    vector<people_msgs::Person> closer_legs;
    cout <<"detecting "<<length<<" legs"<<endl;
    if(length!=0)
    {
        for(int i=0;i<length;++i)
        {
            distance=pow(legs->people[i].position.x-bot_pos.position.x,2)+pow(legs->people[i].position.y-bot_pos.position.y,2);
            distance=sqrt(distance);
            if(distance<=2 &&distance>=0.4)
            {
               closer_legs.push_back(legs->people[i]); 
            }
        }
    }
    
    length = closer_legs.size();
    cout << "legs within "<<RADIUS<<" :"<<length<<endl;

    if(length!=0&&box_num!=0)
    {

        for(int i=0;i<length;++i)
        {
            distance=pow(closer_legs[i].position.x-bot_pos.position.x,2)+pow(closer_legs[i].position.y-bot_pos.position.y,2);
            distance=sqrt(distance);
            x_tmp=abs(closer_legs[i].velocity.x);
            y_tmp=abs(closer_legs[i].velocity.y);
            final_tmp=250*(x_tmp)+250*(y_tmp)+150*distance_weight(distance);

            if(final_tmp>final_vel)
            {
                final_vel=final_tmp;
                legs_published=closer_legs[i];
            }
        }
        if(final_vel <= 50)//leg velocity minimum
        {
            tracking = false;
            return;
        }

        cout<<"ready to track"<<endl;
        tracking=true;
        return;
    }


    tracking=false;
    
}

void headercallback(const std_msgs::Float64MultiArray::ConstPtr & msg)
{
  heading_angle=(msg->data[1]+msg->data[0])/2;
}


void robot_position_callback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos)
{
    bot_pos=pos->pose.pose;
}


void tracker_callback(const std_msgs::Bool::ConstPtr & judge)
{
    if(judge->data) has_tracker=true;
}

void bounding_boxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr & msg)
{
    if(msg->bounding_boxes.size()!=0) 
    {
        box_num=msg->bounding_boxes.size();
        cout << box_num <<endl;
        return;
    }

    box_num=0;
}

int main (int argc, char ** argv)
{
    double x_prev = 0, y_prev = 0;
    ros::init(argc,argv,"velocity_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub_3=n.subscribe("amcl_pose",1,robot_position_callback);
    ros::Subscriber sub_5=n.subscribe("/darknet_ros/bounding_boxes",1,bounding_boxes_callback);
    ros::Subscriber sub_1=n.subscribe("people",5,leg_detector_callback);
    ros::Subscriber sub_2=n.subscribe("tracker_angles",1,headercallback);
    ros::Subscriber sub_4=n.subscribe("has_tracker",1,tracker_callback);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("/move_base", true);
    //wait for the action server to come up
    /*while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }*/
    ros::Rate loop_rate(8);
    double angle_on_map;
    while(ros::ok())
    {
        if(tracking)
        {
            double x_arr=(legs_published.position.x-bot_pos.position.x)*0.7;
            double y_arr=(legs_published.position.y-bot_pos.position.y)*0.7;

            angle_on_map = 2*asin(bot_pos.orientation.z);

            double x_final=bot_pos.position.x+x_arr;
            double y_final=bot_pos.position.y+y_arr;
            double angle_final = angle_on_map + 1.6*heading_angle;
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

            }
        }
        /*else
        {
            if(has_tracker&&abs(heading_angle)>20*3.17/180)
            {
                move_base_msgs::MoveBaseGoal goal_find;
                double turn_fifty_degree;
                if(heading_angle>0) turn_fifty_degree=angle_on_map+30*3.14/180;
                else turn_fifty_degree=angle_on_map-30*3.14/180;

                goal_find.target_pose.header.frame_id = "map";
                goal_find.target_pose.header.stamp = ros::Time::now();
                goal_find.target_pose.pose.position.x = bot_pos.position.x;
                goal_find.target_pose.pose.position.y = bot_pos.position.y;
                goal_find.target_pose.pose.orientation.z = sin(turn_fifty_degree/2);
                goal_find.target_pose.pose.orientation.w = cos(turn_fifty_degree/2);
                cout << "finding"<<endl;
                ac.sendGoal(goal_find);
                ros::Duration(0.6).sleep();
            }

            else
            {
                move_base_msgs::MoveBaseGoal goal_no;
                angle_on_map = 2*asin(bot_pos.orientation.z);
                double find_angle=angle_on_map+30*3.14/180;
                cout << "find_angle_counterclock "<<find_angle<<endl;
                //we'll send a goal to the robot to move 1 meter forward
                goal_no.target_pose.header.frame_id = "map";
                goal_no.target_pose.header.stamp = ros::Time::now();
                goal_no.target_pose.pose.position.x = bot_pos.position.x;
                goal_no.target_pose.pose.position.y = bot_pos.position.y;
                goal_no.target_pose.pose.orientation.z = sin(find_angle/2);
                goal_no.target_pose.pose.orientation.w = cos(find_angle/2);

                ac.sendGoal(goal_no);

                ros::Duration(0.6).sleep();

                find_angle-=60*3.14/180;
                cout << "find_angle"<<find_angle<<endl;
                goal_no.target_pose.header.stamp = ros::Time::now();
                goal_no.target_pose.pose.position.x = bot_pos.position.x;
                goal_no.target_pose.pose.position.y = bot_pos.position.y;
                goal_no.target_pose.pose.orientation.z = sin(find_angle/2);
                goal_no.target_pose.pose.orientation.w = cos(find_angle/2);

                ac.sendGoal(goal_no);

                ros::Duration(0.6).sleep();
            }*/

        
        ros::spinOnce();
        loop_rate.sleep();
    }
}




