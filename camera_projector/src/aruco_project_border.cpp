//
// Created by alg on 4/18/21.
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

std::string image_topic_name = "/camera/color/image_raw";
bool new_image = false;
cv::Mat realsense_image;
void realsenseImageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    realsense_image = cv_ptr->image;

    /*// Update GUI Window
    cv::imshow("RealSenseImage", cv_ptr->image);
    cv::waitKey(3);*/

    new_image = true;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "aruco_project_border");
    ros::NodeHandle nh;

    //get ros image realsenses
    ros::Subscriber sub_image = nh.subscribe(image_topic_name, 1000, realsenseImageCallback);

    ros::Rate rate(20);
    while(!new_image){

        ros::spinOnce();
        rate.sleep();
    }



    //detect aruco corners


    //transform the corners into projection plane


    //draw the corners on the projection plane


    //save the projection image





}