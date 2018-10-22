#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>

using namespace std;
using namespace cv;

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
vector<vector<Point> > polygons;
Mat img,img_hsv,img1;
int hl=0,hh=255,sl=0,sh=255,vl=0,vh=255;

char c;
int area,x,y;

int main()
{
    namedWindow("trackbar");
    createTrackbar("hl","trackbar",&hl,255,0);
    createTrackbar("hh","trackbar",&hh,255,0);
    createTrackbar("sl","trackbar",&sl,255,0);
    createTrackbar("sh","trackbar",&sh,255,0);
    createTrackbar("vl","trackbar",&vl,255,0);
    createTrackbar("vh","trackbar",&vh,255,0);
    char c;
    VideoCapture cap(0);
    Mat img;

    while(1)
    {
        if(!cap.isOpened())
 {
            cout<<"gffghc"<<"\n";
            break;
        }
        if(!cap.read(img))
        {
            cout<<"jhf"<<"\n";


        }
            cvtColor(img,img1,CV_BGR2HSV);
 // Canny(img1,img1,100,100,3,false);
        inRange(img_hsv,Scalar(hl,sl,vl),Scalar(hh,sh,vh),img1);
        imshow("befor",img1);

       // erode(img1,img1,Mat(),Point(-1,-1),2,1,1);
        findContours(img_hsv,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
        //findContours(img_bi.clone(), contours,hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        polygons.resize(contours.size());

                  /*  for (int i = 0; i < contours.size(); i++)
                    {
                        approxPolyDP(Mat(contours[i]), polygons[i], arcLength(Mat(contours[i]), true)*0.019, true);
                    }*/
                    for (int i = 0; i < polygons.size(); i++)
                    {
                        Scalar color = Scalar(0, 255, 255);

                        drawContours(img, polygons, i, color, 3, 8, hierarchy, 0, Point());

                   }

                    imshow("Contours",img);
                    c=waitKey(10);
                    if(c==27)
                    break;


    }

    return 0;






}

