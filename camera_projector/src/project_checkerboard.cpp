//
// Created by alg on 4/9/21.
//
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include "utils_projections.h"

std::string image_path = "/home/alg/projection_mapping/projection_ws/src/camera-projector-projection/camera_projector/data/images/checkerboard.png";
std::string corner_file_name="/home/alg/projection_mapping/projection_ws/src/camera-projector-projection/camera_projector/data/images/corners3.txt";
int main(){
    //load the checkerboard image
    cv::Mat checkerboard_img = cv::imread(image_path);
    cv::cvtColor(checkerboard_img, checkerboard_img, CV_BGR2GRAY);

    //resize image
    cv::resize(checkerboard_img, checkerboard_img, cv::Size(), 0.1, 0.1);
    std::cout<<"img size: "<<checkerboard_img.rows<<" "<<checkerboard_img.cols<<std::endl;

    //white background image
    cv::Mat background_white_img = cv::Mat(1080, 1920, CV_8UC1);
    makeImgWhite(background_white_img);
    //cv::cvtColor(background_white_img, background_white_img, CV_GRAY2BGR);
    std::cout<<"bg size: "<<background_white_img.rows<<" "<<background_white_img.cols<<"\n";
    //cv::cvtColor(background_white_img, main_img, CV_BGR2GRAY);

    //add the   checkerboard to the white image
    addImages(background_white_img, checkerboard_img);

    //detect corners
    cv::Mat corners;
    if(cv::findChessboardCorners(background_white_img, cv::Size(8, 6), corners ))
        std::cout<<"corners_written"<<std::endl;

    //send the corners to a file
    writeCorners2File(corners, corner_file_name);

    //project  the image
    cv::Point2d start_location(100, 100);
    showImgFS("CheckerBoard", background_white_img);


}

