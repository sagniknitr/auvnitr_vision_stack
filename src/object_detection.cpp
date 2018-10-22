
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tiburon/hsv_data.h>

using namespace cv;
using namespace std;


//---------- All Global  Variables----------------------------------
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

vector<vector<Point> > polygons;
Point2f cent;
Point2f angle_base;
double dist;
double rotate_angle;

Mat image_hsv;
Mat hsv_image;

bool imageReceived = false;
Mat inp, out;
int hl=25,sl=30,vl=0,hh=155,sh=255,vh=255;

int y_hl=25,y_sl=30,y_vl=0,y_hh=155,y_sh=255,y_vh=255;
int r_hl=25,r_sl=30,r_vl=0,r_hh=155,r_sh=255,r_vh=255;
int g_hl=25,g_sl=30,g_vl=0,g_hh=155,g_sh=255,g_vh=255;
float d=0.7;

//--------------------------------------------------------------------------------------------------------

typedef struct t_color_node {
    cv::Mat       mean;       // The mean of this node
    cv::Mat       cov;
    uchar         classid;    // The class ID

    t_color_node  *left;
    t_color_node  *right;
} t_color_node;

/*cv::Mat get_dominant_palette(std::vector<cv::Vec3b> colors) {
    const int tile_size = 64;
    cv::Mat ret = cv::Mat(tile_size, tile_size*colors.size(), CV_8UC3, cv::Scalar(0));
    for(int i=0;i<colors.size();i++) {
        cv::Rect rect(i*tile_size, 0, tile_size, tile_size);
        cv::rectangle(ret, rect, cv::Scalar(colors[i][0], colors[i][1], colors[i][2]), CV_FILLED);
    }
    return ret;
}*/

std::vector<t_color_node*> get_leaves(t_color_node *root) {
    std::vector<t_color_node*> ret;
    std::queue<t_color_node*> queue;
    queue.push(root);

    while(queue.size() > 0) {
        t_color_node *current = queue.front();
        queue.pop();

        if(current->left && current->right) {
            queue.push(current->left);
            queue.push(current->right);
            continue;
        }

        ret.push_back(current);
    }

    return ret;
}

std::vector<cv::Vec3b> get_dominant_colors(t_color_node *root) {
    std::vector<t_color_node*> leaves = get_leaves(root);
    std::vector<cv::Vec3b> ret;

    for(int i=0;i<leaves.size();i++) {
        cv::Mat mean = leaves[i]->mean;
        ret.push_back(cv::Vec3b(mean.at<double>(0)*255.0f,
                                mean.at<double>(1)*255.0f,
                                mean.at<double>(2)*255.0f));
    }

    return ret;
}

int get_next_classid(t_color_node *root) {
    int maxid = 0;
    std::queue<t_color_node*> queue;
    queue.push(root);

    while(queue.size() > 0) {
        t_color_node* current = queue.front();
        queue.pop();

        if(current->classid > maxid)
            maxid = current->classid;

        if(current->left != NULL)
            queue.push(current->left);

        if(current->right)
            queue.push(current->right);
    }

    return maxid + 1;
}


void get_class_mean_cov(cv::Mat img, cv::Mat classes, t_color_node *node) {
    const int width = img.cols;
    const int height = img.rows;
    const uchar classid = node->classid;

    cv::Mat mean = cv::Mat(3, 1, CV_64FC1, cv::Scalar(0));
    cv::Mat cov = cv::Mat(3, 3, CV_64FC1, cv::Scalar(0));

    // We start out with the average color
    double pixcount = 0;
    for(int y=0;y<height;y++) {
        cv::Vec3b* ptr = img.ptr<cv::Vec3b>(y);
        uchar* ptrClass = classes.ptr<uchar>(y);
        for(int x=0;x<width;x++) {
            if(ptrClass[x] != classid)
                continue;

            cv::Vec3b color = ptr[x];
            cv::Mat scaled = cv::Mat(3, 1, CV_64FC1, cv::Scalar(0));
            scaled.at<double>(0) = color[0]/255.0f;
            scaled.at<double>(1) = color[1]/255.0f;
            scaled.at<double>(2) = color[2]/255.0f;

            mean += scaled;
            cov = cov + (scaled * scaled.t());

            pixcount++;
        }
    }

    cov = cov - (mean * mean.t()) / pixcount;
    mean = mean / pixcount;

    // The node mean and covariance
    node->mean = mean.clone();
    node->cov = cov.clone();

    return;
}

void partition_class(cv::Mat img, cv::Mat classes, uchar nextid, t_color_node *node) {
    const int width = img.cols;
    const int height = img.rows;
    const int classid = node->classid;

    const uchar newidleft = nextid;
    const uchar newidright = nextid+1;

    cv::Mat mean = node->mean;
    cv::Mat cov = node->cov;
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(cov, eigenvalues, eigenvectors);

    cv::Mat eig = eigenvectors.row(0);
    cv::Mat comparison_value = eig * mean;

    node->left = new t_color_node();
    node->right = new t_color_node();

    node->left->classid = newidleft;
    node->right->classid = newidright;

    // We start out with the average color
    for(int y=0;y<height;y++) {
        cv::Vec3b* ptr = img.ptr<cv::Vec3b>(y);
        uchar* ptrClass = classes.ptr<uchar>(y);
        for(int x=0;x<width;x++) {
            if(ptrClass[x] != classid)
                continue;

            cv::Vec3b color = ptr[x];
            cv::Mat scaled = cv::Mat(3, 1,
                                  CV_64FC1,
                                  cv::Scalar(0));

            scaled.at<double>(0) = color[0]/255.0f;
            scaled.at<double>(1) = color[1]/255.0f;
            scaled.at<double>(2) = color[2]/255.0f;

            cv::Mat this_value = eig * scaled;

            if(this_value.at<double>(0, 0) <= comparison_value.at<double>(0, 0)) {
                ptrClass[x] = newidleft;
            } else {
                ptrClass[x] = newidright;
            }
        }
    }
    return;
}

cv::Mat get_quantized_image(cv::Mat classes, t_color_node *root){
   std::vector<t_color_node*> leaves = get_leaves(root);

    const int height = classes.rows;
    const int width = classes.cols;
    cv::Mat ret(height, width, CV_8UC3, cv::Scalar(0));

    for(int y=0;y<height;y++) {
        uchar *ptrClass = classes.ptr<uchar>(y);
        cv::Vec3b *ptr = ret.ptr<cv::Vec3b>(y);
        for(int x=0;x<width;x++) {
            uchar pixel_class = ptrClass[x];
            for(int i=0;i<leaves.size();i++) {
                if(leaves[i]->classid == pixel_class) {
                    ptr[x] = cv::Vec3b(leaves[i]->mean.at<double>(0)*255,
                                       leaves[i]->mean.at<double>(1)*255,
                                       leaves[i]->mean.at<double>(2)*255);
                }
            }
        }
    }

    return ret;
}

cv::Mat get_viewable_image(cv::Mat classes) {
    const int height = classes.rows;
    const int width = classes.cols;

    const int max_color_count = 12;
    cv::Vec3b *palette = new cv::Vec3b[max_color_count];
    palette[0]  = cv::Vec3b(  0,   0,   0);
    palette[1]  = cv::Vec3b(255,   0,   0);
    palette[2]  = cv::Vec3b(  0, 0,   0);
    palette[3]  = cv::Vec3b(  255,   255, 255);
    palette[4]  = cv::Vec3b(255, 255,   0);
    palette[5]  = cv::Vec3b(  0, 255, 255);
    palette[6]  = cv::Vec3b(255,   0, 255);
    palette[7]  = cv::Vec3b(128, 128, 128);
    palette[8]  = cv::Vec3b(128, 255, 128);
    palette[9]  = cv::Vec3b( 32,  32,  32);
    palette[10] = cv::Vec3b(255, 128, 128);
    palette[11] = cv::Vec3b(128, 128, 255);

    cv::Mat ret = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    for(int y=0;y<height;y++) {
        cv::Vec3b *ptr = ret.ptr<cv::Vec3b>(y);
        uchar *ptrClass = classes.ptr<uchar>(y);
        for(int x=0;x<width;x++) {
            int color = ptrClass[x];
            if(color >= max_color_count) {
                printf("You should increase the number of predefined colors!\n");
                continue;
            }
            ptr[x] = palette[color];
        }
    }

    return ret;
}

t_color_node* get_max_eigenvalue_node(t_color_node *current) {
    double max_eigen = -1;
    cv::Mat eigenvalues, eigenvectors;

    std::queue<t_color_node*> queue;
    queue.push(current);

    t_color_node *ret = current;
    if(!current->left && !current->right)
        return current;

    while(queue.size() > 0) {
        t_color_node *node = queue.front();
        queue.pop();

        if(node->left && node->right) {
            queue.push(node->left);
            queue.push(node->right);
            continue;
        }

        cv::eigen(node->cov, eigenvalues, eigenvectors);
        double val = eigenvalues.at<double>(0);
        if(val > max_eigen) {
            max_eigen = val;
            ret = node;
        }
    }

    return ret;
}

cv::Mat find_dominant_colors(cv::Mat img, int count) {
    const int width = img.cols;
    const int height = img.rows;

    cv::Mat classes = cv::Mat(height, width, CV_8UC1, cv::Scalar(1));
    t_color_node *root = new t_color_node();

    root->classid = 1;
    root->left = NULL;
    root->right = NULL;

    t_color_node *next = root;
    get_class_mean_cov(img, classes, root);
    for(int i=0;i<count-1;i++) {
        next = get_max_eigenvalue_node(root);
        partition_class(img, classes, get_next_classid(root), next);
        get_class_mean_cov(img, classes, next->left);
        get_class_mean_cov(img, classes, next->right);
    }

    std::vector<cv::Vec3b> colors = get_dominant_colors(root);

    cv::Mat quantized = get_quantized_image(classes, root);
    cv::Mat viewable = get_viewable_image(classes);
  //  cv::Mat dom = get_dominant_palette(colors);

    cv::imwrite("/home/codestation/classification1.png", viewable);
    cv::imwrite("/home/codestation/quantized1.png", quantized);
   // cv::imwrite("./palette.png", dom);

    return quantized;
}

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

//inp=rotate(inp,-1);
}

void hsvCallback(const tiburon::hsv_data::ConstPtr& hsv)
{
    d = hsv->d;
    y_hl = hsv->y_hl;
    y_sl = hsv->y_sl;
    y_vl = hsv->y_vl;
    y_hh = hsv->y_hh;
    y_sh = hsv->y_sh;
    y_vh = hsv->y_vh;
    r_hl = hsv->r_hl;
    r_sl = hsv->r_sl;
    r_vl = hsv->r_vl;
    r_hh = hsv->r_hh;
    r_sh = hsv->r_sh;
    r_vh = hsv->r_vh;
    g_hl = hsv->g_hl;
    g_sl = hsv->g_sl;
    g_vl = hsv->g_vl;
    g_hh = hsv->g_hh;
    g_sh = hsv->g_sh;
    g_vh = hsv->g_vh;
    std::cout << d << std::endl;
}

double contour_area(Mat input){
        findContours(input, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

                int aHighIndex=0;
                    double aHigh = 0;
                    //cout<<contours.size()<<endl;
                    for( int i = 0; i< contours.size(); i++ )
                    {
                        if(contourArea(contours[i]) > aHigh)
                        {
                            aHighIndex = i;
                            aHigh = contourArea(contours[i]);
                        }
                    }
        //cout<<aHigh<<endl;
       /* Mat path = Mat::zeros(hsv_image.size(),CV_8UC3);
         Mat path_apprx = Mat::zeros(hsv_image.size(),CV_8UC3);
        drawContours(path,contours,aHighIndex,Scalar::all(255),2, 8, hierarchy, 0, Point());cout<<"hello"<<endl;
        Moments mc_path;
        Point2f path_centre;
        Point2f path_centre_2;
        Point2f path_centre_approx;
        Point2f path_centre_2_approx;*/

        //circle(path,cent,40,Scalar(0,255,0),5,8,0);
        //circle(path,cent,40,Scalar(0,255,0),5,8,0);

        /*if(contours.size()!=0)
        {
            mc_path=moments(contours[aHighIndex],true);

            path_centre=Point2f(mc_path.m10/mc_path.m00,mc_path.m01/mc_path.m00);
            path_centre_2=findCenter(contours[aHighIndex]);
            circle(path,path_centre,40,Scalar(255,0,0),5,8,0);
            circle(path,path_centre_2,40,Scalar(255,255,0),5,8,0);
            dist=abs_distance(path_centre,cent);
            line(path,path_centre,cent,Scalar(255,255,0),1,8,0);

            rotate_angle= cc_angle(cent,path_centre,angle_base);
            cout<<"angle"<<rotate_angle<<endl;


            vector<vector<Point> > polys;
            vector<Point> approxContour;
            approxPolyDP(contours[aHighIndex],approxContour,15,true);
            polys.push_back(approxContour);
            cout << "approx polygon\n" << approxContour << endl;

             mc_path=moments(approxContour,true);
             path_centre=Point2f(mc_path.m10/mc_path.m00,mc_path.m01/mc_path.m00);
             path_centre_2=findCenter(approxContour);
             circle(path_apprx,path_centre,40,Scalar(255,0,0),5,8,0);
             circle(path_apprx,path_centre_2,40,Scalar(255,255,0),5,8,0);



            drawContours(path_apprx,polys,0,Scalar(0,255,255));
           // drawContours(origdraw,polys,0,Scalar(0,255,255));


        }
        /*vector<vector<Point> > polys;
        vector<Point> approxContour;
        approxPolyDP(contours[aHighIndex],approxContour,10,true);
        polys.push_back(approxContour);
        cout << "approx polygon\n" << approxContour << endl;
        drawContours(newim,polys,0,Scalar(0,255,255));
        drawContours(origdraw,polys,0,Scalar(0,255,255));*/

       // namedWindow("contours",CV_WINDOW_NORMAL);
         //imshow("contours",path);
         //namedWindow("apprx_contours",CV_WINDOW_NORMAL);
          //imshow("apprx_contours",path_apprx);*/
         return aHigh;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "SegmentationNodeFront");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub = it.subscribe("auvLeftCamera",1,imageCallback);
    image_transport::Publisher obj1_seg = it.advertise("obj1_Segmented", 1);
    image_transport::Publisher obj2_seg = it.advertise("obj2_Segmented", 1);
    image_transport::Publisher obj3_seg = it.advertise("obj3_Segmented", 1);
    ros::Publisher object_type = nh.advertise<std_msgs::Float64>("/obj_type",1);
    ros::Subscriber hsvData = nh.subscribe("/hsv_data_2",1,hsvCallback);
    sensor_msgs::ImagePtr msg;
    std_msgs::Float64 f64msg;
    while(!imageReceived) ros::spinOnce();
    Mat Qimg = find_dominant_colors(inp, 1);
    int count1 = 0;
    std::vector<cv::Mat> listOfMatrices;

    Mat image_hsv,image, segmented_1,segmented_2,segmented,object_1,object_2,object_3;


    while(1)
    {
        while(!imageReceived) ros::spinOnce();
        image = inp - Qimg*d;

        //namedWindow("normal",CV_WINDOW_NORMAL);
        //imshow("After Enhancement", image);

        cvtColor(image, image_hsv, COLOR_BGR2HSV);
        inRange(image_hsv,Scalar(y_hl,y_sl,y_vl),Scalar(y_hh,y_sh,y_vh),object_1);
        inRange(image_hsv,Scalar(r_hl,r_sl,r_vl),Scalar(r_hh,r_sh,r_vh),object_2);
        inRange(image_hsv,Scalar(g_hl,g_sl,g_vl),Scalar(g_hh,g_sh,g_vh),object_3);
        double obj1_area,obj2_area,obj3_area;
        obj1_area=contour_area(object_1);
        obj2_area=contour_area(object_2);
        obj3_area=contour_area(object_3);

        //type of object is published here..............

        if((obj1_area>obj2_area)&&(obj1_area>obj2_area))
        {
            f64msg.data= "1.0";
            object_type.publish(f64msg);

        }
        else if((obj2_area>obj1_area)&&(obj2_area>obj3_area))
        {
            f64msg.data= "2.0";
            object_type.publish(f64msg);
        }
        else
        {
            f64msg.data= "3.0";
            object_type.publish(f64msg);
        }

         // all the object image are published here..................

        msg_1 = cv_bridge::CvImage(std_msgs::Header(), "mono8", object_1).toImageMsg();
        msg_2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", object_2).toImageMsg();
        msg_3 = cv_bridge::CvImage(std_msgs::Header(), "mono8",object_3).toImageMsg();
        obj1_seg.publish(msg_1);
        obj2_seg.publish(msg_2);
        obj3_seg.publish(msg_3);
        imageReceived = 0;
        if(char(waitKey(30)) == 27)
       {
                //cout << "esc key is pressed by user" << endl;
                break;
       }
       ros::spinOnce();

    }
    return 0;
}
