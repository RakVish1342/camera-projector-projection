//
// Created by alg on 4/15/21.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <algorithm>
#include <string>
using namespace cv;
using namespace  std;
int main()
{
    string image_path = "/home/alg/Downloads/asu_tree4.jpg";
    Mat img, thresholded, tdilated, tmp, tmp1;
    vector<Mat> channels(3);

    img = imread(image_path);
    split(img, channels);
    threshold( channels[2], thresholded, 149, 255, THRESH_BINARY);                      //prepare ROI - threshold
    dilate( thresholded, tdilated,  getStructuringElement( MORPH_RECT, Size(22,22) ) ); //prepare ROI - dilate
    Canny( channels[2], tmp, 75, 125, 3, true );    //Canny edge detection
    multiply( tmp, tdilated, tmp1 );    // set ROI

    dilate( tmp1, tmp, getStructuringElement( MORPH_RECT, Size(20,16) ) ); // dilate
    erode( tmp, tmp1, getStructuringElement( MORPH_RECT, Size(36,36) ) ); // erode

    vector<vector<Point> > contours, contours1(1);
    vector<Point> convex;
    vector<Vec4i> hierarchy;
    findContours( tmp1, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    //get element of maximum area
    //int bestID = std::max_element( contours.begin(), contours.end(),
    //  []( const vector<Point>& A, const vector<Point>& B ) { return contourArea(A) < contourArea(B); } ) - contours.begin();

    int bestID = 0;
    int bestArea = contourArea( contours[0] );
    for( int i = 1; i < contours.size(); ++i )
    {
        int area = contourArea( contours[i] );
        if( area > bestArea )
        {
            bestArea  = area;
            bestID = i;
        }
    }

    convexHull( contours[bestID], contours1[0] );
    drawContours( img, contours1, 0, Scalar( 100, 100, 255 ), img.rows / 100, 8, hierarchy, 0, Point() );
    namedWindow("image", CV_WINDOW_NORMAL);
    imshow("image", img );
    waitKey(0);



    return 0;
}