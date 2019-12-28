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
#include<queue>
using namespace std;

#define X_WEIGHT 1.5
#define Y_WEIGHT 1
#define RADIUS 2 
people_msgs::Person legs_published;
geometry_msgs::Pose bot_pos;
geometry_msgs::Pose _pose;
double heading_angle=0;
bool tracking=false;

void pose_callback( const people_msgs::People::ConstPtr & legs)
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

    int vel=0;
    double angle_final;
    if(length!=0)
    {

        for(int i=0;i<length;++i)
        {
            x_vel=abs(legs->people[i].velocity.x);
            y_vel=abs(legs->people[i].velocity.y);

            double angle_on_map = 2*asin(bot_pos.orientation.z);
            angle_final = angle_on_map + heading_angle;

            x_tmp=x_vel*cos(angle_final);
            y_tmp=y_vel*cos(angle_final+1.57);

            final_tmp=X_WEIGHT*x_tmp+Y_WEIGHT*y_tmp;

            if(final_tmp>vel)
            {
                vel=final_tmp;
                legs_published=legs->people[i];
            }
        }

        if(vel>0.5)
        {
            tracking=true;
            double x_arr=(legs_published.position.x-bot_pos.position.x)*0.7;
            double y_arr=(legs_published.position.y-bot_pos.position.y)*0.7;

            double x_final=bot_pos.position.x+x_arr;
            double y_final=bot_pos.position.y+y_arr;

            _pose.position.x=x_final;
            _pose.position.y=y_final;
            _pose.orientation.z=sin(angle_final/2);
            _pose.orientation.w=cos(angle_final/2);
        }
        
        else tracking=false;
       
    }

    else 
    {
        tracking=false;  
    }

}


void robot_position_callback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos)
{
    bot_pos=pos->pose.pose;
}

void headercallback(const std_msgs::Float64MultiArray::ConstPtr & msg)
{
  heading_angle=(msg->data[1]+msg->data[0])/2;
}

int main (int argc, char ** argv)
{
    ros::init(argc,argv,"pose_queue");
    ros::NodeHandle n;
    ros::Subscriber sub_3=n.subscribe("amcl_pose",1,robot_position_callback);
    ros::Subscriber sub_2=n.subscribe("tracker_angles",1,headercallback);
    ros::Subscriber sub_1=n.subscribe("people",5,pose_callback);
    ros::Publisher pub=n.advertise<geometry_msgs::Pose>("pose_queue",10);
    if(tracking=true)
    {
        pub.publish(_pose);
    }

    ros::Duration(0.5).sleep();
    ros::spin();
}










    