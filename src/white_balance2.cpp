#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

Mat clahe_conversion(Mat input)
{
    vector<Mat> RGB; // Use the STL’s vector structure to store multiple Mat objects
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

int main(){
        Mat image = imread("/home/shaggy/line_auv/rippleJPG.jpg");
        Mat image_hsv;
        namedWindow("ColorBalance",CV_WINDOW_NORMAL);
        //image = clahe_conversion(image);



            vector<Mat> bgr_planes;
            Mat src = image;
            Mat dst;
            src.copyTo(dst);
            Mat src1 = src;
            split(src,bgr_planes);

            float ma=0,mb=0,ml=0;

                   int p=1;
                   int m=1;

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
       // cvtColor(dst,dst,CV_Lab2BGR);
        imshow("ColorBalance",dst);


        /*cvtColor(image, image_hsv, COLOR_BGR2HSV);

        /*for(int y=0; y<image_hsv.cols; y++){
                for(int x=0; x<image_hsv.rows; x++){
                        if(image_hsv.at<uchar>(x,y)[1] >=0 && image_hsv.at<uchar>(x,y)[1] <20 ){
                                image_hsv.at<uchar>(x,y)[1] = 255;
                        }
                        else
                                image_hsv.at<uchar>(x,y)[1] = 0;
                }
        }


        vector<Mat>hsv;
        split(image_hsv, hsv);
        imwrite("/home/harsha/Pictures/clahee.JPG",image);

        imshow("hsv",image_hsv);
        imshow("normal", image);
        imshow("h",hsv[0]);
        imshow("saturation", hsv[1]);



        Mat hsv_image = hsv[0];
        Mat hsv_image1 = hsv[1];


        for(int y=0; y<hsv_image.cols; y++){
                for(int x=0; x<hsv_image.rows; x++){
                        if(hsv_image.at<uchar>(x,y) >=0 && hsv_image.at<uchar>(x,y) <25 || hsv_image.at<uchar>(x,y) >160 && hsv_image.at<uchar>(x,y)<180){
                                hsv_image.at<uchar>(x,y) = 255;
                        }
                        else
                                hsv_image.at<uchar>(x,y) = 0;
                }
        }

        for(int y=0; y<hsv_image1.cols; y++){
                for(int x=0; x<hsv_image1.rows; x++){
                        if(hsv_image1.at<uchar>(x,y) >=0 && hsv_image1.at<uchar>(x,y) <20 ){
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


        imshow("saturation1",hsv_image1);
        imshow("path",hsv_image);

        */
        waitKey(0);

        return 0;

}
