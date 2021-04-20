//
// Created by alg on 4/19/21.
//

#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "PixelCalculations.h"

//corners
Eigen::Vector3d corner_tl, corner_tr, corner_br, corner_bl;


bool corners_recieved = false;
void cornerTLCallback(const geometry_msgs::PoseStampedConstPtr& msg);
void cornerTRCallback(const geometry_msgs::PoseStampedConstPtr& msg);
void cornerBRCallback(const geometry_msgs::PoseStampedConstPtr& msg);
void cornerBLCallback(const geometry_msgs::PoseStampedConstPtr& msg);


int main(int argc, char** argv){
    ros::init(argc, argv, "aruco_project_border");
    ros::NodeHandle nh;

    //get the corner 3d points
    ros::Subscriber sub_corner_tl = nh.subscribe("/camera_aruco/corner_tl", 1000, cornerTLCallback);
    ros::Subscriber sub_corner_tr = nh.subscribe("/camera_aruco/corner_tr", 1000, cornerTRCallback);
    ros::Subscriber sub_corner_br = nh.subscribe("/camera_aruco/corner_br", 1000, cornerBRCallback);
    ros::Subscriber sub_corner_bl = nh.subscribe("/camera_aruco/corner_bl", 1000, cornerBLCallback);

    ros::Rate rate(20);

    while(!corners_recieved){
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){

        //calculate pixel coordinates of the aruco corners
        //pixels in projectors plane
        cv::Point2d  px_tl, px_tr, px_br, px_bl;
        calculatePixelsInProjectorPlane(corner_tr, corner_tl, corner_br, corner_bl, px_tl, px_tr, px_br, px_bl);

        //draw the corners on the image and show the image
        drawAndShowCorners(px_tl, px_tr, px_br, px_bl);
        ros::spinOnce();
        rate.sleep();

    }
    ROS_INFO("Done. Closing node");
    return 0;
}

void cornerTLCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    corner_tl.x() = msg->pose.position.x;
    corner_tl.y() = msg->pose.position.y;
    corner_tl.z() = msg->pose.position.z;
    corners_recieved = true;
}


void cornerTRCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    corner_tr.x() = msg->pose.position.x;
    corner_tr.y() = msg->pose.position.y;
    corner_tr.z() = msg->pose.position.z;

}


void cornerBRCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    corner_br.x() = msg->pose.position.x;
    corner_br.y() = msg->pose.position.y;
    corner_br.z() = msg->pose.position.z;

}


void cornerBLCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    corner_bl.x() = msg->pose.position.x;
    corner_bl.y() = msg->pose.position.y;
    corner_bl.z() = msg->pose.position.z;

}