#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

void gray_world(Mat src1,float *ml,float *ma, float *mb, int p)
{
        *ma=0;
        *mb=0;
        *ml=0;

        for(int i=0;i<src1.rows;i++)
                {
                        for(int j=0;j<src1.cols;j++)
                        {
                                Vec3b v1=src1.at<Vec3b>(i,j);
                                float lc=pow(v1.val[0],p);
                                float ac=pow(v1.val[1],p);
                float bc=pow(v1.val[2],p);
                *ma=*ma+ac;
                *mb=*mb+bc;
                *ml=*ml+lc;
            }
    }

    *ma=pow((float)*ma/(src1.cols*src1.rows),(float)1/p);
    *mb=pow((float)*mb/(src1.cols*src1.rows),(float)1/p);
    *ml=pow((float)*ml/(src1.cols*src1.rows),(float)1/p);

}

Mat white_balance(Mat src,int p_in,int m_in)
{
    vector<Mat> bgr_planes;
    //Mat src = image;
    Mat dst;
    src.copyTo(dst);
    Mat src1 = src;
    //split(src,bgr_planes);
    split(src,bgr_planes);

    float ma=0,mb=0,ml=0;

           int p=p_in;
           int m=m_in;

           gray_world(src,&ml,&ma,&mb,p);


           float r=(ma+mb+ml)/3;
           if(m==1)
           {
               r=(ma+mb+ml)/3;
               r=max(ma,mb);
               r=max(r,ml);
           }

           MatIterator_<Vec3b> it=src1.begin<Vec3b>();
           MatIterator_<Vec3b> itend=src1.end<Vec3b>();
           MatIterator_<Vec3b> itout=dst.begin<Vec3b>();

           for (;it!=itend;++it,++itout)
           {
               Vec3b v1=*it;

               float l=v1.val[0];
               float a=v1.val[1];
               float b=v1.val[2];


               a=a*(r/ma);
               b=b*(r/mb);
               l=l*(r/ml);

               if(a>255)
                   a=255;
               if(b>255)
                   b=255;
               if(l>255)
                   l=255;
               v1.val[0]=l;
               v1.val[1]=a;
               v1.val[2]=b;
               *itout=v1;
           }
           return dst;
}

Mat clahe_conversion(Mat input)
{
    vector<Mat> RGB; // Use the STLâ€™s vector structure to store multiple Mat objects
    split(input, RGB); // split the image into separate color planes (R G B)
    ////    Enhance Local Contrast (CLAHE)
    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(4);
    Mat RGB_eq;


    ////    Equalizes the histogram of a one channel image  (8UC1) using Contrast Limited Adaptive Histogram Equalization.
    clahe->apply(RGB[0],RGB[0]);
    clahe->apply(RGB[1],RGB[1]);
    clahe->apply(RGB[2],RGB[2]);

    merge(RGB,RGB_eq);              // now merge the results back
    return RGB_eq;
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

        Mat image = frame;
        Mat image_hsv;

        //image = clahe_conversion(image);
        namedWindow("normal",CV_WINDOW_NORMAL);
        imshow("normal", image);

        image = white_balance(image, 1,1);
        namedWindow("white_balance",CV_WINDOW_NORMAL);
        imshow("white_balance", image);

        cvtColor(image, image_hsv, COLOR_BGR2HSV);

        vector<Mat>hsv;
        split(image_hsv, hsv);
        imwrite("/home/harsha/Pictures/clahee.JPG",image);


        namedWindow("hsv",CV_WINDOW_NORMAL);

        namedWindow("h",CV_WINDOW_NORMAL);
        namedWindow("saturation",CV_WINDOW_NORMAL);
        imshow("hsv",image_hsv);

        imshow("h",hsv[0]);
        imshow("saturation", hsv[1]);


        Mat hsv_image = hsv[0];
        Mat hsv_image1 = hsv[1];

        #pragma omp parallel for collapse(2)
        for(int y=0; y<hsv_image.cols; y++){
            for(int x=0; x<hsv_image.rows; x++){
                if(hsv_image.at<uchar>(x,y) >=0 && hsv_image.at<uchar>(x,y) <25 || hsv_image.at<uchar>(x,y) >160 && hsv_image.at<uchar>(x,y)<180){
                    hsv_image.at<uchar>(x,y) = 255;
                }
                else
                    hsv_image.at<uchar>(x,y) = 0;
            }
        }
        #pragma omp parallel for collapse(2)
        for(int y=0; y<hsv_image1.cols; y++){
            for(int x=0; x<hsv_image1.rows; x++){
                if(hsv_image1.at<uchar>(x,y) >=0 && hsv_image1.at<uchar>(x,y) <40 ){
                    hsv_image1.at<uchar>(x,y) = 0;
                }
                else
                    hsv_image1.at<uchar>(x,y);
            }
        }

        for(int y=0; y<hsv_image.cols; y++){
            for(int x=0; x<hsv_image.rows; x++){
                if(hsv_image1.at<uchar>(x,y) == 0){
                    hsv_image.at<uchar>(x,y) = 0;
                }
                else
                    hsv_image.at<uchar>(x,y);
            }
        }

        namedWindow("saturation1",CV_WINDOW_NORMAL);
        namedWindow("path",CV_WINDOW_NORMAL);
        imshow("saturation1",hsv_image1);
        imshow("path",hsv_image);


        //imshow("MyVideo", frame);

        if(waitKey(30) == 27)
       {
                cout << "esc key is pressed by user" << endl;
                break;
       }

    }

}
