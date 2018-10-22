#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include<opencv2/photo/photo.hpp>
#include<omp.h>

using namespace std;
using namespace cv;

// -------------------Global Variable------------------------
int front_cam1=1,front_cam2=2,down_cam=3;
VideoCapture cap("/home/shaggy/line_auv/GOPR0033.MP4");
VideoCapture cam1(front_cam1);
VideoCapture cam2(front_cam2);







//-----------------------------------------------------------------







Mat clahe_conversion(Mat input)
{
    vector<Mat> RGB; // Use the STLâ€™s vector structure to store multiple Mat objects
    split(input, RGB); // split the image into separate color planes (R G B)
    ////    Enhance Local Contrast (CLAHE)
    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(4);
    Mat RGB_eq;
    //whiteBalance()

    ////    Equalizes the histogram of a one channel image  (8UC1) using Contrast Limited Adaptive Histogram Equalization.
    clahe->apply(RGB[0],RGB[0]);
    clahe->apply(RGB[1],RGB[1]);
    clahe->apply(RGB[2],RGB[2]);

    merge(RGB,RGB_eq);              // now merge the results back
    return RGB_eq;
}














int main(){
    Mat image ;//= imread("/home/shaggy/line_auv/rippleJPG");

    namedWindow("White Balance", CV_WINDOW_NORMAL);
    //namedWindow("filter_image", CV_WINDOW_NORMAL);
    namedWindow("input", CV_WINDOW_NORMAL);
    namedWindow("h", CV_WINDOW_NORMAL);
    //namedWindow("CLAHE", CV_WINDOW_NORMAL);
    //namedWindow("hue", CV_WINDOW_NORMAL);
    namedWindow("path", CV_WINDOW_NORMAL);
    namedWindow("red buoy", CV_WINDOW_NORMAL);
    namedWindow("green buoy", CV_WINDOW_NORMAL);
    if(!cap.isOpened())
             {
                     cout<<"Check Video"<<"\n";
                     return 0;
                 }
    while(1)
                {

                    if(!cap.read(image))
                          {
                              continue;
                          }



    Mat image_hsv;
    //Mat image_baln=balance_white(image.clone());

    Mat image_CLAHE = clahe_conversion(image.clone());
    cvtColor(image_CLAHE, image_hsv, COLOR_BGR2HSV);

    vector<Mat>hsv;
    split(image_hsv, hsv);


    Mat hsv_image = hsv[0];

    #pragma omp parallel for collapse(2)
    for(int y=0; y<hsv_image.cols; y++){
        for(int x=0; x<hsv_image.rows; x++){
            if(hsv_image.at<uchar>(x,y) >=0 && hsv_image.at<uchar>(x,y) <10 )//|| hsv_image.at<uchar>(x,y) >160 && hsv_image.at<uchar>(x,y)<180){
               {
                hsv_image.at<uchar>(x,y) = 255;
            }
            else
                hsv_image.at<uchar>(x,y) = 0;
        }
    }

   /* #pragma omp parallel
    {
        cout<<"hgfh"<<endl;
    }*/


//----------All imshow here----------------------------------------------
    imshow("path",hsv_image);
    //imshow("hsv",image_hsv);
    imshow("input", image);
    imshow("h",hsv[0]);
    //imshow("saturation", hsv[1]);
   // waitKey(0);
     imshow("White Balance",image_CLAHE);

//-----------------------------------------------------------------------------
    if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
           {
                    cout << "esc key is pressed by user" << endl;
                    break;
           }
}
    return 0;

}
