#include"ros/ros.h"
#include"geometry_msgs/Pose.h"
#include"geometry_msgs/Point.h"
#include"geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
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
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include"sensor_msgs/LaserScan.h"
using namespace std;

ros::Publisher cmdpub;

enum danger_detect
{
    RIGHT,
    LEFT,
    none
};

class PD_controller
{
    public:

    void set_error_prev(double diff){error_prev=diff;}
    void set_error(double distance){error=distance-actual_value;}
    void set_actual_value(double tmp){actual_value=tmp;}
    double show_error(){return error;}
    double show_diff(){return error-error_prev;}
    double show_actual_value(){return actual_value;}

    double data_now=0;
    private:
    double error=0;
    double error_prev=0;
    double actual_value=0;
    
};

danger_detect judge=none;
danger_detect find_man=none;

people_msgs::Person legs_published;

geometry_msgs::Pose bot_pos;
double heading_angle=0;
bool tracking=false;
int box_num=0;

double front_range;


int distance_weight(int distance)
{
    if(distance<0.7) return 0.6;
    else if(distance>=0.7&&distance<=2) return 1.2;
    else return 0.3; 
}

void leg_detector_callback( const people_msgs::People::ConstPtr & legs)
{
    
    double x_vel,y_vel,final_vel=0,x_tmp,y_tmp,final_tmp,distance;
    int length = legs->people.size();
    vector<people_msgs::Person> closer_legs;
    if(length!=0)
    {
        for(int i=0;i<length;++i)
        {
            distance=pow(legs->people[i].position.x-bot_pos.position.x,2)+pow(legs->people[i].position.y-bot_pos.position.y,2);
            distance=sqrt(distance);
            if(distance<=2.5 &&distance>=0.5)
            {
               closer_legs.push_back(legs->people[i]); 
            }
        }
    }
    
    length = closer_legs.size();

    if(length!=0&&box_num!=0)
    {

        for(int i=0;i<length;++i)
        {
            distance=pow(closer_legs[i].position.x-bot_pos.position.x,2)+pow(closer_legs[i].position.y-bot_pos.position.y,2);
            distance=sqrt(distance);
            x_tmp=abs(closer_legs[i].velocity.x);
            y_tmp=abs(closer_legs[i].velocity.y);
            final_tmp=400*(x_tmp)+400*(y_tmp)+100*distance_weight(distance);

            if(final_tmp>final_vel)
            {
                final_vel=final_tmp;
                legs_published=closer_legs[i];
            }
        }
    
        if(final_vel <= 160)//weight minimum
        {
            tracking = false;
        }

        else
        {
            tracking=true;
            ROS_INFO("tracking");
        }
    }
    


    else   tracking=false;
    


    //***************************************Add command velocities*********************************************
    geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

    double rot_vel_scale = 5;//need test
    double diff_rot_vel_scale=2;
    double lin_vel_scale = 0.9;//need test
    double diff_vel_scale=0.8;
    double pro_error;
    double diff_error;
    double total_error;
    PD_controller control;
    PD_controller angle_control;

    double x_arr=(legs_published.position.x-bot_pos.position.x)*0.62;
    double y_arr=(legs_published.position.y-bot_pos.position.y)*0.62;
    distance=sqrt(x_arr*x_arr+y_arr*y_arr);
    distance *= cos(heading_angle);//only consider distance in forward direction

    if(tracking)//if not tracking all velocites are 0
    {
        if(abs(heading_angle) > 3.14/18)//heading error > 10 degree
        {
            
            angle_control.data_now=heading_angle;
            pro_error=angle_control.show_error();
            angle_control.set_error(heading_angle);
            diff_error=angle_control.show_diff();
            total_error=rot_vel_scale*pro_error+diff_rot_vel_scale*diff_error; //PD controller
            cmd->angular.z = total_error;
            
            if(total_error>0.3) 
            { 
                find_man=RIGHT;
            }
            else if(total_error<-0.3) 
            {
                find_man=LEFT;
            }
            else find_man=none;
            angle_control.set_error_prev(pro_error);
            angle_control.set_actual_value(total_error);
            if(abs(cmd->angular.z) > 0.62)
            {
                if(total_error>0) cmd->angular.z = 0.62;
                else cmd->angular.z = -0.62;
            }
                 
        }
        if(distance > 0.4)//distance to goal > 40cm
        {
            pro_error=control.show_error();
            control.set_error(distance);
            diff_error=control.show_diff();
            
            total_error=lin_vel_scale*pro_error+diff_vel_scale*diff_error;//PD controller
            cmd->linear.x = total_error;
            control.set_error_prev(pro_error);
            control.set_actual_value(total_error);
            
            if(cmd->linear.x > 0.72)
                 cmd->linear.x = 0.72;
        }

        cout << "Rotation speed: " << cmd->angular.z << endl;
        cout << "Linear speed: " << cmd->linear.x << endl;
    }

    else if(!tracking && find_man !=none)
    {
        
            cout << "finding"<<endl;
        
            if(find_man==RIGHT) 
            {
                cmd->angular.z=0.22;
            }
            else 
            {
                cmd->angular.z=-0.22;
            }
        
    }

     if(cmd->linear.x>0.6&&front_range<0.75)
    {
        cmd->linear.x = -0.05;
        find_man=none;
        cout << "EM STOP\n";
    }


    if(front_range < 0.45 && !isnan(front_range))
    {
        cmd->linear.x = -0.05;
        find_man=none;
        cout << "EM STOP\n";
    }
    if(judge==RIGHT)
    {
        cmd->angular.z = 0.25;
        find_man=none;
        if(cmd->linear.x > 0.2)
            cmd->linear.x = 0.15;
    }
    if(judge==LEFT)
    {
        cmd->angular.z = -0.25;
        find_man=none;
        if(cmd->linear.x > 0.2)
            cmd->linear.x = 0.15;
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

void bounding_boxes_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr & msg)
{
    if(msg->bounding_boxes.size()!=0) 
    {
        box_num=msg->bounding_boxes.size();
        return;
    }

    box_num=0;
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &input_scan)
{
    double right_front_range=0;
    double left_front_range=0;
    front_range=0;
    
    double angle=(input_scan->angle_max-input_scan->angle_min)*180/3.14;
    int one_degree=ceil(input_scan->ranges.size()/angle);

    for(int i=0;i<one_degree;++i)
    {
        
        right_front_range+=input_scan->ranges[0.3*input_scan->ranges.size()+i];
        front_range+=input_scan->ranges[0.5*input_scan->ranges.size()+i];
        left_front_range+=input_scan->ranges[0.7*input_scan->ranges.size()+i];
        
    }
    right_front_range/=one_degree;
    front_range/= one_degree;
    left_front_range/=one_degree;

    if(right_front_range<0.6)
    {
        judge=RIGHT;
    }
    else if(left_front_range<0.6) 
    {
        judge=LEFT;
    }
    else judge=none;
    
}

int main (int argc, char ** argv)
{
    ros::init(argc,argv,"velocity_publisher");
    ros::NodeHandle n;
    cmdpub= n.advertise<geometry_msgs::Twist> ("/ironfish_diff_drive_controller/cmd_vel", 1);
    ros::Subscriber sub_5=n.subscribe("/darknet_ros/bounding_boxes",1,bounding_boxes_callback);
    ros::Subscriber sub_3=n.subscribe("amcl_pose",1,robot_position_callback);
    ros::Subscriber sub_2=n.subscribe("tracker_angles",1,headercallback);
    ros::Subscriber sub_4=n.subscribe("scan",2,scan_callback);
    ros::Subscriber sub_1=n.subscribe("people",5,leg_detector_callback);
    ros::spin();
}
