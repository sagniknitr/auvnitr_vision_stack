
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;

using namespace cv;

float u = 2.5;

Mat colour_correction(Mat img){

    Mat dst = img.clone();

    cv::Scalar tempVal = mean( img );
    float mean1 = tempVal.val[0];

    img = img - mean1;

    int d;
    for(int y=0; y<img.cols; y++){
        for(int x=0; x<img.rows; x++){
            d = img.at<uchar>(x,y); // - mean1;
            d = d*d;
            img.at<uchar>(x,y) = d;
        }
    }

    Scalar tempVal1 = mean( img );
    float umean = tempVal1.val[0];

    cout<<umean<<endl;
    float smin, smax;
    smin = mean1 - u*umean;
    smax = mean1 + u*umean;

    for(int y=0; y<img.cols; y++){
        for(int x=0; x<img.rows; x++){
            d = dst.at<uchar>(x,y);
            if(((d - smin)/(smax - smin))<=1)
            dst.at<uchar>(x,y) = 255*(d - smin)/(smax - smin);
            else
                dst.at<uchar>(x,y)=255;
        }
    }
    return dst;

}

Mat intermediate(Mat img){


    vector<Mat> rgb;
    Mat rgb_wb;
    split(img, rgb);
    rgb[0] = colour_correction(rgb[0]);
    rgb[1] = colour_correction(rgb[1]);
    rgb[2] = colour_correction(rgb[2]);


    merge(rgb, rgb_wb);

    return rgb_wb;
}

int main(){
    VideoCapture cap("/home/shaggy/line_auv/GOPR0033.MP4");
    if(!cap.isOpened()){
        cout<< "Cannot open the video file" << endl;
        return -1;
    }
    namedWindow("Video",CV_WINDOW_NORMAL);
    while(1){
        Mat frame;

        bool bSuccess = cap.read(frame);

         if (!bSuccess){
            cout << "Cannot read the frame from video file" << endl;
            break;
        }

        Mat dst = intermediate(frame);
        namedWindow("Normal", CV_WINDOW_NORMAL);
        namedWindow("colour_balance", CV_WINDOW_NORMAL);
        imshow("Normal", frame);
        imshow("colour_balance", dst);
        imwrite("/home/shaggy/line_auv/color_balance.jpg",dst);

        if(waitKey(30) == 27){
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

}
