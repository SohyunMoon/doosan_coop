#include <iostream>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <moveit_msgs/CartesianTrajectory.h>
#include <thread>
#include <chrono>


int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<moveit_msgs::CartesianTrajectory>("/cartesian_trajectory", 100);
    // ros::Publisher mode_pub = nh.advertise<std_msgs::Int32>("robot_mode", 100);

    ros::Rate rate(1);
    int size = 1;
    int traj_count = 0;
    int set = 10;

    while (ros::ok() & (traj_count < set)){

        moveit_msgs::CartesianTrajectory traj_msg;

        // mode_msg.data = 1;  
        traj_msg.points.resize(size); 

        if (traj_count % 2 == 0) { 
            traj_msg.tracked_frame = "Position mode";
        } else { 
            traj_msg.tracked_frame = "Impedance mode";
        }
        
        //point0
        traj_msg.points[0].point.pose.position.x = 559.0-50*traj_count;
        traj_msg.points[0].point.pose.position.y = 34.5-50*traj_count;;
        traj_msg.points[0].point.pose.position.z = 648.42-50*traj_count;
        traj_msg.points[0].point.pose.orientation.x = 0.01;
        traj_msg.points[0].point.pose.orientation.y = -180.0;
        traj_msg.points[0].point.pose.orientation.z = 0.01-5*traj_count;
        traj_msg.points[0].time_from_start = ros::Duration(3.0);

        rate.sleep();
        rate.sleep();
        rate.sleep();
        rate.sleep();
        rate.sleep();

        // traj_pub.publish(traj_msg);
        traj_count ++;
        // ROS_INFO("Trajectory Generated");
        // ros::waitForShutdown();
        // ros::spinOnce();
    }
    return 0;
}
