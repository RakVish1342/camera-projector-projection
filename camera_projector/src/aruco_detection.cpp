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

double marker_size = 0.0965;

std::string image_topic_name = "/camera/color/image_raw";
cv::Matx41d aa2quaternion(const cv::Vec3d& aa)
{
    //double angle = cv::norm(aa);
    double angle = aa.val[0]*aa.val[0] + aa.val[1]*aa.val[1] + aa.val[2]*aa.val[2];
    if(angle!=0){
        cv::Matx31d axis(aa(0) / angle, aa(1) / angle, aa(2) / angle);
        double angle_2 = angle / 2;
        //qx, qy, qz, qw
        cv::Matx41d q(axis(0) * sin(angle_2), axis(1) * sin(angle_2), axis(2) * sin(angle_2), cos(angle_2));
        return q;
    }
    else return cv::Matx41d(0, 0, 0, 0);

}

bool new_image = false;
cv::Mat current_image;
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
    current_image = cv_ptr->image;

    /*// Update GUI Window
    cv::imshow("RealSenseImage", cv_ptr->image);
    cv::waitKey(3);*/

    new_image = true;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "realsense_aruco");
	ros::NodeHandle nh;

    //ge    t ros image realsenses
    ros::Subscriber sub_image = nh.subscribe(image_topic_name, 1000, realsenseImageCallback);
    ros::Publisher pub_camera_aruco_pose = nh.advertise<geometry_msgs::PoseStamped>("/camera_aruco/pose", 100);
    ros::Publisher pub_corner_tl = nh.advertise<geometry_msgs::PoseStamped>("/camera_aruco/corner_tl", 100);
    ros::Publisher pub_corner_tr = nh.advertise<geometry_msgs::PoseStamped>("/camera_aruco/corner_tr", 100);
    ros::Publisher pub_corner_br = nh.advertise<geometry_msgs::PoseStamped>("/camera_aruco/corner_br", 100);
    ros::Publisher pub_corner_bl = nh.advertise<geometry_msgs::PoseStamped>("/camera_aruco/corner_bl", 100);
    

     


    ros::Rate rate(20);

    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, 5, 200, markerImage, 1);
    cv::imwrite("/home/alg/marker5.png", markerImage);
    std::cout<<"MarkerWritten"<<std::endl;

    while(!new_image){

        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        //aruco detection
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > marker_corners, rejected_candidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::detectMarkers(current_image, dictionary, marker_corners, markerIds, parameters, rejected_candidates);
        cv::Mat output_image = current_image.clone();
        cv::aruco::drawDetectedMarkers(output_image, marker_corners, markerIds);
        cv::imshow("detected markers", output_image);
        cv::waitKey(3);

        //get pose of markers
        cv::Mat camera_matrix(3,3, CV_32F), distortion_coeffs(1, 5, CV_32F);
        camera_matrix.at<float_t>(0,0) = 614.623;
        camera_matrix.at<float_t >(0,1) = 0;
        camera_matrix.at<float_t >(0,2) = 327.88;

        camera_matrix.at<float_t >(1,0) = 0;
        camera_matrix.at<float_t >(1,1) = 614.458;
        camera_matrix.at<float_t >(1,2) = 238.79;

        camera_matrix.at<float_t >(2,0) = 0;
        camera_matrix.at<float_t >(2,1) = 0;
        camera_matrix.at<float_t >(2,2) = 1;
        //std::cout<<"Camera Matrix: "<<camera_matrix<<std::endl;

        distortion_coeffs.at<float_t > (0,0) = 0;
        distortion_coeffs.at<float_t > (0,1) = 0;
        distortion_coeffs.at<float_t > (0,2) = 0;
        distortion_coeffs.at<float_t > (0,3) = 0;
        distortion_coeffs.at<float_t > (0,4) = 0;


        std::vector<cv::Vec3d> rvecs, tvecs;
        //TODO: CHANGE THE SIZE OF THE MARKERS
        cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size, camera_matrix, distortion_coeffs, rvecs, tvecs);
        if(marker_corners.size() > 0){
            //cv::Matx41d quat_cv = aa2quaternion(rvecs[0]);

            //publish the pose
            geometry_msgs::PoseStamped camera_aruco_pose;
            camera_aruco_pose.header.stamp = ros::Time::now();
            camera_aruco_pose.pose.position.x = tvecs[0].val[0];
            camera_aruco_pose.pose.position.y = tvecs[0].val[1];
            camera_aruco_pose.pose.position.z = tvecs[0].val[2];


            cv::Mat rotation_matrix;
            
            //std::cout<<rvecs[0]<<std::endl;
            cv::Rodrigues(rvecs[0], rotation_matrix);
            std::cout<<"Type: "<<rotation_matrix.type()<<std::endl; 
            std::cout<<rotation_matrix<<std::endl; 

            Eigen::Matrix3d eigen_rotation_matrix;
            for(int i =0; i< 3; i++){
                for(int j=0; j<3; j++)
                    eigen_rotation_matrix(i,j) = rotation_matrix.at<double_t> (i,j);
            }
            Eigen::Quaterniond eigen_quaternion(eigen_rotation_matrix);
            eigen_quaternion.normalize(); 
            camera_aruco_pose.pose.orientation.x = eigen_quaternion.x();
            camera_aruco_pose.pose.orientation.y = eigen_quaternion.y();
            camera_aruco_pose.pose.orientation.z = eigen_quaternion.z();
            camera_aruco_pose.pose.orientation.w = eigen_quaternion.w();
            std::cout<<"ArUco Camera pose: "<<camera_aruco_pose<<std::endl;

            geometry_msgs::PoseStamped corner_tl_pose, corner_tr_pose, corner_br_pose, corner_bl_pose; 
            //Eigen::Matrix4d cHtl, cHtr, cHbr, cHbl; 
            Eigen::Matrix4d cHa; 
            cHa << eigen_rotation_matrix(0,0), eigen_rotation_matrix(0,1), eigen_rotation_matrix(0,2), tvecs[0].val[0],
                      eigen_rotation_matrix(1,0),eigen_rotation_matrix(1,1),eigen_rotation_matrix(1,2), tvecs[0].val[1], 
                      eigen_rotation_matrix(2,0),eigen_rotation_matrix(2,1),eigen_rotation_matrix(2,2), tvecs[0].val[2],
                      0, 0, 0 ,1; 
            
            Eigen::Matrix4d aHtl, aHtr, aHbr, aHbl; 
            aHtl << 1, 0, 0, -marker_size/2, 
                    0, 1, 0, marker_size/2, 
                    0, 0, 1, 0, 
                    0, 0, 0, 1; 

            aHtr << 1, 0, 0, marker_size/2, 
                    0, 1, 0, marker_size/2, 
                    0, 0, 1, 0, 
                    0, 0, 0, 1; 

            aHbr << 1, 0, 0, marker_size/2, 
                    0, 1, 0, -marker_size/2, 
                    0, 0, 1, 0, 
                    0, 0, 0, 1; 

            aHbl << 1, 0, 0, -marker_size/2, 
                    0, 1, 0, -marker_size/2, 
                    0, 0, 1, 0, 
                    0, 0, 0, 1; 

            Eigen::Matrix4d cHtl, cHtr, cHbr, cHbl; 
            cHtl = cHa*aHtl;
            cHtr = cHa*aHtr;
            cHbr = cHa*aHbr; 
            cHbl = cHa*aHbl;

            corner_tl_pose.pose.position.x = cHtl(0,3);
            corner_tl_pose.pose.position.y = cHtl(1,3);
            corner_tl_pose.pose.position.z = cHtl(2,3);

            corner_tr_pose.pose.position.x = cHtr(0,3);
            corner_tr_pose.pose.position.y = cHtr(1,3);
            corner_tr_pose.pose.position.z = cHtr(2,3);

            corner_br_pose.pose.position.x = cHbr(0,3);
            corner_br_pose.pose.position.y = cHbr(1,3);
            corner_br_pose.pose.position.z = cHbr(2,3);

            corner_bl_pose.pose.position.x = cHbl(0,3);
            corner_bl_pose.pose.position.y = cHbl(1,3);
            corner_bl_pose.pose.position.z = cHbl(2,3); 

            pub_corner_tl.publish(corner_tl_pose);
            pub_corner_tr.publish(corner_tr_pose);
            pub_corner_br.publish(corner_br_pose);
            pub_corner_bl.publish(corner_bl_pose); 


        }


        ros::spinOnce();
        rate.sleep();
    }

    


    return 0;
}