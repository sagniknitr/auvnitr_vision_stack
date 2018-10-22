#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(){

	Mat image1,imgHSV, imgHSV1;
    image1 = imread("/home/harsha/Pictures/AUV_2.png");

    cvtColor( image1, imgHSV, COLOR_BGR2HSV );


    Mat hsv[3];
    
    //cvtColor( image1, imgHSV, COLOR_BGR2HSV );
    split(imgHSV,hsv);

    Mat a(hsv[0].rows,hsv[0].cols,CV_8UC1,Scalar(255));
    hsv[1]= a;
    hsv[2]= a;

    addWeighted( hsv[1], 0.5, hsv[2], 0.5, 0.0, hsv[2]);
    addWeighted( hsv[0], 0.5, hsv[2], 0.5, 0.0, hsv[2]);
    //merge(hsv,)
     
    imwrite("/home/harsha/Pictures/H.png",hsv[0]);
    imwrite("/home/harsha/Pictures/S.png",hsv[1]);
    imwrite("/home/harsha/Pictures/V.png",hsv[2]);

    //Mat imgH = imread("/home/harsha/Pictures/H.png");

    cvtColor( hsv[2], imgHSV1, COLOR_HSV2BGR );

    namedWindow("Original Image", CV_WINDOW_AUTOSIZE);
    imshow("s", imgHSV1);

    waitKey(0);
    destroyAllWindows();
    return 0;
}