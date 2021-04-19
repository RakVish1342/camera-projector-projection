//
// Created by alg on 4/15/21.
//
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include "utils_projections.h"

std::string aruco_image_path = "/home/alg/Downloads/complex_tree.jpg";
int main(){
    //get the mask
    cv::Mat mask = cv::imread(aruco_image_path);
    cv::cvtColor(mask, mask, CV_BGR2GRAY);

    //black background image
    //cv::Mat pattern_img = cv::Mat(mask.rows, mask.cols, CV_8UC1);
    cv::Mat pattern_img(mask.rows, mask.cols, CV_8UC3, cv::Scalar(0, 0, 0));


    //make image black
    makeImgBlack(pattern_img);

    //pattern height
    int pattern_height = 200;
    //get max and min rows
    int top_row = mask.rows - pattern_height , bottom_row = mask.rows;

    cv::Mat pattern_image;
    //keep running the image
    int iters = 0;

    while(iters < 10000){
        for(int i =top_row; i< bottom_row; i++) {
            for (int j = 0; j < mask.cols; j++)
                if (mask.at<uchar>(i, j) == 255) {
                    cv::Vec3b &color = pattern_img.at<cv::Vec3b>(i, j);
                    color[0] = 250;
                }
        }

        //update the top and bottom rows
        top_row--;
        bottom_row --;
        //reset top and bottom rows if they are 0 and 200;
        if(top_row == 0) {
            top_row = 1;
            //bottom_row = mask.rows;
        }
        if(bottom_row == 0){
            bottom_row = mask.rows;
            top_row = mask.rows - pattern_height;
        }


        iters++;

        //show image
        cv::imshow("Pattern", pattern_img);
        cv::waitKey(10);

        //reset the image
        cv::Mat black_img(mask.rows, mask.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        pattern_img = black_img;

    }

    return 0;
}