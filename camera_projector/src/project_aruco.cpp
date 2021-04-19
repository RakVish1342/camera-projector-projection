/**
 * File to project aruco at different areas on the image
*/
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include "utils_projections.h"

std::string aruco_image_path = "/home/alg/projection_mapping/projection_ws/src/camera-projector-projection/camera_projector/data/images/marker.png";
std::string corner_file_name="/home/alg/projection_mapping/projection_ws/src/camera-projector-projection/camera_projector/data/images/aruco_corners.txt";
const double scale = 1;

int main(){
    //load the checkerboard image
    cv::Mat aruco_img = cv::imread(aruco_image_path);
    //cv::cvtColor(aruco_img, aruco_img, CV_BGR2GRAY);

    //resize image
    //cv::resize(aruco_img, aruco_img, cv::Size(), scale, scale);
    std::cout << "img size: " << aruco_img.rows << " " << aruco_img.cols << std::endl;

    //white background image
    cv::Mat background_white_img = cv::Mat(1080, 1920, CV_8UC1);
    makeImgWhite(background_white_img);
    //cv::cvtColor(background_white_img, background_white_img, CV_GRAY2BGR);
    std::cout<<"bg size: "<<background_white_img.rows<<" "<<background_white_img.cols<<"\n";
    //cv::cvtColor(background_white_img, main_img, CV_BGR2GRAY);

    //add the   checkerboard to the white image
    addImages(background_white_img, aruco_img);

    //detect corners
    cv::Mat corners;
    /*if(cv::findChessboardCorners(background_white_img, cv::Size(8, 6), corners ))
        std::cout<<"corners_written"<<std::endl;
    cv::drawChessboardCorners(background_white_img, cv::Size(8, 6), corners, true);
    cv::imshow("Detected Corners", background_white_img );
    cv::waitKey(0);*/

    //send the corners to a file
    //writeCorners2File(corners, corner_file_name);

    showImgFS("CheckerBoard", background_white_img);


}

