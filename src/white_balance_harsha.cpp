#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

/*------------------global variables----------------------------------*/

int hl=14,sl=30,vl=0,hh=155,sh=255,vh=255;
Mat segmented_1,segmented_2,segmented;
Mat inp;
bool imageReceived = false;
/*----------------------------------------------------------------------*/

Mat rotate(Mat image, int method)
{
    if (method <0)
        transpose(image, image);
    //if(method == 1)
    //return image;
    Mat rotated = Mat(image.rows, image.cols, CV_MAKETYPE(CV_8U, image.channels()));
    //Vec3b value;
    //cout << image.size() << " " << rotated.size() <<  endl;
    flip(image, rotated, 1);
    /*for (int i = 0; i<rotated.cols; i++)
    for (int j = 0; j<rotated.rows; j++)
    rotated.at<Vec3b>(Point(i, j)) = image.at<Vec3b>(Point(image.cols - i - 1, j));*/

    if (method >0)
        transpose(rotated, rotated);
    return rotated;
}
void gray_world(Mat src1,float *ml,float *ma, float *mb, float p)
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

Mat white_balance(Mat src,float p_in,int m_in)
{
    vector<Mat> bgr_planes;
    //Mat src = image;
    Mat dst;
    src.copyTo(dst);
    Mat src1 = src;
    //split(src,bgr_planes);
    split(src,bgr_planes);

    float ma=0,mb=0,ml=0;

           float p=p_in;
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
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   try
   {
     inp = (cv_bridge::toCvShare(msg, "bgr8")->image).clone(); // Don't remove this clone call, else it will not finish copying the whole image and overwrite it prematurely
     //imshow("Img",cv_bridge::toCvShare(msg, "bgr8")->image );
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
   imageReceived=true;

}




int main(){
  
 /* VideoCapture cap("/home/shaggy/line_auv/2.avi");
  if(!cap.isOpened()){
    cout<< "Cannot open the video file" << endl; 
        return -1;
  }*/

    ros::init(argc, argv, "SegmentationNode");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub = it.subscribe("auv_cam1",1,imageCallback);
    image_transport::Publisher segPub = it.advertise("frontCameraSegmented", 1);
    //ros::Subscriber hsvData = nh.subscribe("/hsv_data",1,hsvCallback);
    sensor_msgs::ImagePtr msg;
       while(!imageReceived) ros::spinOnce();


  namedWindow("Video",CV_WINDOW_NORMAL);
  int count1 = 0;
  float h=5;
  int templateWindowSize=7;
  int searchWindowSize=21;

  int morph_elem = 2;
  int morph_size = 16;
  int morph_operator = 0;
  int const max_operator = 4;
  int const max_elem = 2;
  int const max_kernel_size = 21;


  while(1){
        Mat frame;

        /*bool bSuccess = cap.read(frame);

         if (!bSuccess){
          cout << "Cannot read the frame from video file" << endl;
            break;
        }*/

        while(!imageReceived) ros::spinOnce();

        Mat image = inp;
        Mat image_hsv;
        Mat image1, image_gray;

        image=rotate(image,-1);
         imshow("Image",image);
        image = white_balance(image,12,1);
        namedWindow("white_balance",CV_WINDOW_NORMAL);
        imshow("white_balance", image);




        cvtColor(image, image_hsv, COLOR_BGR2HSV);
               //cvtColor(imgH, image_hsv_sat, COLOR_BGR2HSV);
        inRange(image_hsv,Scalar(0,sl,vl),Scalar(hl,sh,vh),segmented_1);
        inRange(image_hsv,Scalar(hh,sl,vl),Scalar(180,sh,vh),segmented_2);
        bitwise_or(segmented_2,segmented_1,segmented);
    

        imshow("Seg",segmented);


      /*  fimshow("Image",image);
        imshow("Seg",segmented);
or(int y=0; y<hsv_image.cols; y++){
          for(int x=0; x<hsv_image.rows; x++){
            if(hsv_image.at<uchar>(x,y) >=0 && hsv_image.at<uchar>(x,y) <35 /*|| hsv_image.at<uchar>(x,y) >160 && hsv_image.at<uchar>(x,y)<180*//*{
              hsv_image.at<uchar>(x,y) = 255;
            }
            else
              hsv_image.at<uchar>(x,y) = 0;
            if(hsv_image1.at<uchar>(x,y) >=0 && hsv_image1.at<uchar>(x,y) <65 ){
              hsv_image.at<uchar>(x,y) = 0;
            }
            else
              hsv_image.at<uchar>(x,y);
          }
        }*/ 

  

       /* Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

        morphologyEx( hsv_image, hsv_denoise, 3 /*3 is for closing, element  );



        namedWindow("path",CV_WINDOW_NORMAL);
        imshow("denoise",hsv_denoise);
        imshow("path",hsv_image);*/
        msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", segmented).toImageMsg();
        segPub.publish(msg);
        imageReceived = 0;

            if(waitKey(1) == 27)  
           {
                    cout << "esc key is pressed by user" << endl; 
                    break; 
           }

        }



}
