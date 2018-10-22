#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cmath>
#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include<iostream>

using namespace cv;
using namespace std;

Mat src, src_gray, op,poly,r;//for bottom camera
Mat src_f;
double d, dist, d_min;
Size s;
vector<int> f1, f2;
int flag;
vector<float> point_x,point_y;
double grad=0;
int ma = 0,p=0;
int an = 0;
Point2f p_end_1, p_end_2;
int shift=2;
Mat cartesianRotate(Mat, int);
Point2f pt;
int flg = 1;
double speed_l,speed_r;
//pid parameters
long no_of_frames;
double kp=3;
double sum,pid_p,pid_i,pid_d;
int base_case=12;

int 	c_index_l, c_i2_l, c_index_r, c_i2_r;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

vector<vector<Point> > polygons;

//functions
double dst(Point2f, Point2f);
double imageDetect(Mat);
void speedRight();
void speedLeft();
void speedForward();
void speedBackward();




int main(int argc,char** argv)
{

    ros::init(argc,argv,"line_follow");
    ros::NodeHandle n;
ros::Publisher conf_code=n.advertise<std_msgs::Float16>("/code", 1);
ros::Publisher leftpub=n.advertise<std_msgs::Float16>("sideleftspeed",1);
ros::Publisher rightpub=n.advertise<std_msgs::Float16>("siderightspeed",1);
std_msgs::Float16 msg1,msg2;
    VideoCapture cap(0);
    VideoCapture cap_ball(1);

    if ((!cap.isOpened())||(!cap_ball.isOpened()))  // if not success, exit program
    {

        cout << "Cannot open the camera" << endl;
        return -1;
    }
    namedWindow("input", CV_WINDOW_AUTOSIZE);
    namedWindow("thres", CV_WINDOW_AUTOSIZE);
    nmaedWindow("input front camera",CV_WINDOW_AUTOSIZE);
    //src = imread("E:/AUV/3.jpg");
    //createTrackbar("angle", "Track", &an, 360, NULL);
    while (1)
    {
       ++no_of_frames;
        //Mat img, gr;

        bool bSuccess1 = cap.read(src);
        bool bSuccess2=cap.read(src_f);
        if (!bSuccess1) //if not success, break loop
        {
            cout << "Cannot read the frame from down camera" << endl;
            break;
        }
        if(!bSuccess2)
        {
            cout << "Cannot read the frame from front camera" << endl;
            break;

        }




        //cout << s.width << "  " << s.height;
        //transpose(src, src);
        src = cartesianRotate(src, -1);
        src_f=cartesianRotate(src_f,-1);
        //flip(src,src,1);
        int foundPath=pathDetect(src);


        resize(src, src, Size(640, 480));
        //transpose(src, src);
        op = src(Rect(0, src.rows*0.5, (src.cols), src.rows*0.5));
    //	cout << s.height << "  " << s.width;
        s = op.size();
        //transpose(op, op);
        poly = Mat::zeros(op.size(), CV_8UC3);



        imshow("input", op);

        cvtColor(op, src_gray, CV_BGR2GRAY);
         threshold(src_gray, src_gray, 0, 255, CV_THRESH_OTSU);
        Mat elm = getStructuringElement(MORPH_RECT, Size(17, 17));

        erode(src_gray.clone(), src_gray, elm);
        erode(src_gray, src_gray, elm);
        dilate(src_gray, src_gray, elm);
        erode(src_gray, src_gray, elm);
        dilate(src_gray, src_gray, elm);

        findContours(src_gray.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

        int aHighIndex=0;
            double aHigh = 0;
            for( int i = 0; i< contours.size(); i++ )
            {
                if(contourArea(contours[i]) > aHigh)
                {
                    aHighIndex = i;
                    aHigh = contourArea(contours[i]);
                }
            }
        Mat line = Mat::zeros(src_gray.size(),CV_8UC3);
        drawContours(line,contours,aHighIndex,Scalar::all(255),CV_FILLED);
        /*for(int i=0;i<contours[aHighIndex].size();i++)
        {
            circle(line,contours[aHighIndex][i],2,Scalar(0,0,255));
        }*/
        Point2f lastpt = Point(0,0);
        Point2f cent = Point(0,0);
        int count = 0;
        double noiseDist = line.cols/80;
        vector<double> linecent;
        for(int i = 0;i<line.rows;i++)
        {
            for(int j=0;j<line.cols;j++)
            {
                if(line.at<Vec3b>(Point(j,i))[0])
                {
                    if(count==0)
                    {
                        cent = Point(j,i);
                        count++;
                        continue;
                    }
                //	/*else if(dst(cent,Point(j,i)) < noiseDist)
                //	{
                //		cent.x = (cent.x+j)/(cnt+1);
                //	}
                    else
                    {
                        cent.x = (cent.x*count+j)/(count+1);
                        count++;
                    }
                }
            }
            circle(line,cent,2,Scalar(0,0,255));
            if(count>1)
            linecent.push_back(cent.x);
            cent = Point(0,0);
            count = 0;
        }

        //for(int i = 0;i<)   */
        for (int i = 0; i < (linecent.size() - 2); i++)
        {
            grad += (linecent[i + 1] - linecent[i]);

            //cout << linecent[i] << endl;
        }
        //pid_
        pid_i=pid_i+grad;
        if(no_of_frames==base_case)
        {
        pid_d=grad;
        pid_p=0;
        }
        else
         {   pid_d=pid_d-grad;
            pid_p=grad;
        }
        //cout << "gradient  " << grad << endl;
        if(grad>10)
        {
msg1.data=1300+kp*pid_p;
msg2.data=1000+kp*pid_p;
      leftpub.publish(msg2);
rightpub.publish(msg1);
}
else if(grad<-5)
{
msg1.data=1050+kp*pid_p;
msg2.data=900+kp*pid_p;
leftpub.publish(msg1);
rightpub.publish(msg2);
}
else
{
msg1.data=1050+kp*pid_p;
msg2.data=80+kp*pid_p;
leftpub.publish(msg1);
rightpub.publish(msg2);
}
        grad = 0;
        sum=0;

        imshow("thres", src_gray);
        imshow("line", line);


}

            //	waitKey(0);




            int c = waitKey(10);
            if (c == 27)
                break;
            else if (c == 112)
            {
                while (1)
                {

                    if (waitKey(10) == 32)
                        break;

                }
            }

        }
    }
double dst(Point2f p1, Point2f p2)
{
    double d = (p1.x - p2.x)*(p1.x - p2.x) +(p1.y - p2.y)*(p1.y - p2.y);
    double dist = pow(d, 0.5);
    return dist;
}

Mat cartesianRotate(Mat image, int method)
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

int  pathDetect(Mat img)
{
    vector<Mat> img_base();

    int no_of_bases=2;
    int i=0;
    // 0= only water  1=path
    for(int i=0;i<no_of_bases;i++)
    {
        std::string s;
        std::stringstream out;
        out << i;
        s = out.str();
        Mat base=imread(s+".jpg");
        //cvtColor( base, base, COLOR_BGR2HSV );
        img_base.push_back(base);
    }

    Mat img_test=img.clone();

    /// Using 50 bins for hue and 60 for saturation
       int h_bins = 50; int s_bins = 60;
       int histSize[] = { h_bins, s_bins };

       // hue varies from 0 to 179, saturation from 0 to 255
       float h_ranges[] = { 0, 180 };
       float s_ranges[] = { 0, 256 };

       const float* ranges[] = { h_ranges, s_ranges };

       // Use the o-th and 1-st channels
       int channels[] = { 0, 1 };

       /// Histograms
       MatND histograms_bot;
       vector<MatND> histograms(no_of_bases);

       /// Calculate the histograms for the HSV images
       for(int i=0;i<no_of_bases;i++)
       {
       //calcHist( &hsv_base, 1, channels, Mat(), hist_base, 2, histSize, ranges, true, false );
      // normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );
       calHist(&img_base(i),1,channels,Mat(),histograms(i),2,histSize,ranges,true,false);
       normalize(histograms(i),histograms(i),0,1,NORM_MINMAX,-1,Mat());

 }
       calHist(&img_test,1,channels,Mat(),histograms_bot,2,histSize,ranges,true,Mfalse);
       normalize(histograms_bot,histograms_bot,0,1,NORM_MINMAX,-1,Mat());

       int compare_method=1;double min_value;
       vector<double> compare_values(no_of_bases);

       for(int i=0;i<no_of_bases;i++)
      {
       //calcHist( &hsv_base, 1, channels, Mat(), hist_base, 2, histSize, ranges, true, false );
      // normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );
  compare_values[i] = compareHist(histograms[i], histograms_bot, compare_method );
          if(min_value>compare_values[i])
          {
              min_value=compare_values[i];
              key =i;

          }

 }



       return key;
}

int ballDetect(Mat img2)
{
    vector<Mat> imgf_base();

    int no_of_bases=5;
    //int j=0;
    // 1 = only water  2=small path 3= most path 4= more than 50 percent path 5=maximum path
    for(int i=0;i<no_of_bases;i++)
    {
        std::string s;
        std::stringstream out;
        out << i;
        s = out.str();
        Mat basef=imread(s+"f.jpg");
        //cvtColor( base, base, COLOR_BGR2HSV );
        imgf_base.push_back(basef);
    }

    Mat imgf_test=img2.clone();

    /// Using 50 bins for hue and 60 for saturation
       int h_bins = 50; int s_bins = 60;
       int histSize[] = { h_bins, s_bins };

       // hue varies from 0 to 179, saturation from 0 to 255
       float h_ranges[] = { 0, 180 };
       float s_ranges[] = { 0, 256 };

       const float* ranges[] = { h_ranges, s_ranges };

       // Use the o-th and 1-st channels
       int channels[] = { 0, 1 };

       /// Histograms
       MatND histograms_botf;
       vector<MatND> histograms_f(no_of_bases);

       /// Calculate the histograms for the HSV images
       for(int i=0;i<no_of_bases;i++)
       {
       //calcHist( &hsv_base, 1, channels, Mat(), hist_base, 2, histSize, ranges, true, false );
      // normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );
       calHist(&imgf_base(i),1,channels,Mat(),histograms_f(i),2,histSize,ranges,true,false);
       normalize(histograms(i),histograms(i),0,1,NORM_MINMAX,-1,Mat());

 }
       calHist(&img_test,1,channels,Mat(),histograms_bot,2,histSize,ranges,true,Mfalse);
       normalize(histograms_bot,histograms_bot,0,1,NORM_MINMAX,-1,Mat());

       int compare_method=1;double min_value;
       vector<double> compare_values(no_of_bases);

       for(int i=0;i<no_of_bases;i++)
      {
       //calcHist( &hsv_base, 1, channels, Mat(), hist_base, 2, histSize, ranges, true, false );
      // normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );
  compare_values[i] = compareHist(histograms[i], histograms_bot, compare_method );
          if(min_value>compare_values[i])
          {
              min_value=compare_values[i];
              key =i;

          }

 }



       return key;

}


void speedRight()
{

}
void speedLeft()
{

}
void speedForward()
{

}
void speedBackward()
{

}
void rotate()
{
    msg1.data=60;
    msg2.data=60;
    leftpub.publish(msg1);
    rightpub.publish(msg2);


}
