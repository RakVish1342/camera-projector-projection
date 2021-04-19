//
// Created by alg on 4/15/21.
//

#include <opencv2/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;


string aruco_image_path = "/home/alg/Downloads/asu_tree4.jpg";

void meanShiftMethod(Mat src, Mat& res ){
    GaussianBlur(src, res, Size(5,5), 2, 2);
    pyrMeanShiftFiltering( res, res, 20, 45, 3);
    imwrite("meanshift.png", res);
    imshow( "Meanshift", res );
    waitKey(0);

}
//segment the image using hsv color separation
void hsvMethod(Mat src, Mat& des){
    int r = 227, g = 190, b = 207;
    cv::Vec3b bgr_pixel(b, g, r);
    Mat3b bgr(bgr_pixel);
    Mat hsv;
    cvtColor(bgr, hsv, COLOR_BGR2HSV);
    Vec3b hsv_pixel(hsv.at<Vec3b>(0,0));
    int thresh_h = 40, thresh_s = 350, thresh_v = 350;
    cv::Scalar minHSV = cv::Scalar(hsv_pixel.val[0] - thresh_h, hsv_pixel.val[1] - thresh_s, hsv_pixel.val[2] - thresh_v);
    cv::Scalar maxHSV = cv::Scalar(hsv_pixel.val[0] + thresh_h, hsv_pixel.val[1] + thresh_s, hsv_pixel.val[2] + thresh_v);
    cv::Mat maskHSV;
    cv::Mat src_hsv;
    cvtColor(src, src_hsv, COLOR_BGR2HSV);
    cv::inRange(src_hsv, minHSV, maxHSV, maskHSV);
    cv::bitwise_and(src_hsv, src_hsv, des, maskHSV);
    Mat mean_shifted_hsv;
    //meanShiftMethod(des, mean_shifted_hsv);
    imshow("HSV Image", des);

    imwrite("hsv_segmentation1.jpg", des);
    waitKey(0);



}


void edgeDetection(Mat src, Mat& des){
    Mat edges;
    //edge detection
    Canny(src, edges, 100, 125, 3, true );
    imshow("Edges", edges);
    waitKey(0);

}
int main(int argc, char** argv)
{
    Mat img, res, element;
    //namedWindow( "Meanshift", 0 );
    img = imread(aruco_image_path );
    cv::resize(img, img, cv::Size(), 0.5, 0.5);
    imshow("OriginalImage", img);
    Mat edge_detected;
    //edgeDetection(img, edge_detected);
    /*Mat mean_shifted_img;
    meanShiftMethod(img, mean_shifted_img);*/

    //second method HSV
    //convert to HSV
    Mat segmented_image;
    hsvMethod(img, segmented_image);
    return 0;
}