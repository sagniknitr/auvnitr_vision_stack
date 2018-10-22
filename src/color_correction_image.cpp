
/*Underwater Object Detection involving color correction algorithm
Author : Sagnik Basu

The MIT License
Copyright (c) 2015 Avi Sagnik Basu
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/








#include<opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include<fstream>
#include<string>
using namespace cv;
using namespace std;

double mul = 0.5;
int iter = 3;  //iterations required for color correction
Point center;
Point critp;
Point lp;
float s1 = 0; float s2 = 0;
int angle;
int scale = 1;
Mat rot_mat(2, 3, CV_32FC1);
//VideoCapture vid("/home/shaggy/line_auv/GOPR0033.MP4");
ofstream myfile;
int k=0; //image counter
int lowcount = 0;
int critpoint = 0;
int gl, rl, bl, bh=255, rh=255, gh=255;
Mat img_bi, poly,dest;                      // stores the binary image
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
vector<vector<Point> > polygons;

/*----------------functions used--------------------*/

Mat correctGamma(Mat& , double);
Mat clahe_conversion(Mat);



/*--------------------------------------------------*/

Mat correctGamma(Mat& img, double gamma) {
    double inverse_gamma = gamma;

    Mat lut_matrix(1, 256, CV_8UC1);
    uchar * ptr = lut_matrix.ptr();
    for (int i = 0; i < 256; i++)
        ptr[i] = (int)(pow((double)i / 255.0, inverse_gamma) * 255.0);

    Mat result;
    LUT(img, lut_matrix, result);

    return result;

}
//moments()
//VideoCapture front(0);

int main()
{
            int p;
            namedWindow("correction", CV_WINDOW_NORMAL);
            namedWindow("filter_image", CV_WINDOW_NORMAL);
            namedWindow("input", CV_WINDOW_NORMAL);
            namedWindow("contour", CV_WINDOW_NORMAL);
            namedWindow("CLAHE", CV_WINDOW_NORMAL);
            // cout<<"Now entering the loop";
            //namedWindow("", CV_WINDOW_NORMAL);
           /* createTrackbar("blue low", "Track", &bl, 255, NULL);
            createTrackbar("blue high", "Track", &bh, 255, NULL);
            createTrackbar("green low ", "Track", &gl, 255, NULL);
            createTrackbar("green high", "Track", &gh, 255, NULL);
            createTrackbar("red low", "Track", &rl, 255, NULL);
            createTrackbar("red high", "Track", &rh, 255, NULL);*/
           // myfile.open ("/home/shaggy/line_auv/example.txt");
            Mat img = imread("/home/shaggy/line_auv/path.jpg");
           // dest=img.clone();
            imshow("input",img);

            //imshow("input",img);
            int i=12;
           // bilateralFilter(img,dest,i, i*2,i/2);
            Mat clahe_conv=clahe_conversion(img);
            imshow("CLAHE",clahe_conv);
            imwrite("/home/shaggy/line_auv/CLAHE.jpg",clahe_conv);
            //cvtColor(img,img,CV_BGR2HSV);
            bilateralFilter(clahe_conv,dest,i, i*2,i/2);
             imshow("filter_image",dest);
            Mat g_corr=correctGamma(img,1);
            Mat imgg = g_corr.clone();
            // cout<<"image is k"<<k<<endl;
             //++k;
            cout<<"Now entering the loop";
            for (int i = 0; i < iter; i++)
            {
               // cout<<"selfie"<<endl;
               //  myfile << "image frame starts..........................................\n";
                for ( p = 0; p < img.rows - img.rows / (64 * mul); p += img.rows / (64 * mul))
                {
                    for (int q = 0; q < img.cols - img.cols / (48 * mul); q += img.cols / (48 * mul))
                    {
                        int bavg = 0;
                        int gavg = 0;
                        int ravg = 0;
                        for (int i = p; i < p + img.rows / (64 * mul); i++)
                        {
                            for (int j = q; j < q + img.cols / (48 * mul); j++)
                            {
                              // Vec3b color = g_corr.at<Vec3b>(i, j);
                                Vec3b color = clahe_conv.at<Vec3b>(i, j);
                                int b = color[0];
                                int g = color[1];
                                int r = color[2];
                                bavg += b;
                                gavg += g;
                                ravg += r;
                            }
                        }
                        bavg = bavg * 64 * 48 * mul*mul / (img.rows*img.cols);
                        gavg = gavg * 64 * 48 * mul*mul / (img.rows*img.cols);
                        ravg = ravg * 64 * 48 * mul*mul / (img.rows*img.cols);
                       // int gg = 2 * gavg - (ravg + bavg);
                       // int rr = 2 * ravg - (gavg + bavg);

                        // myfile << "Writing this to a file.\n";
                       //  myfile.close();
                        //myfile<<"bavg="<<bavg<<"\n"<<"rvg="<<ravg<<"\n"<<"gavg="<<gavg<<"\n";
                       // putText(img,to_string(bavg),Point(0,0));
                        //cout<<"iteration is..."<<p<<"and bavg="<<bavg<<"and red avg="<<ravg<<endl;
                        for (int i = p; i < p + img.rows / (64 * mul); i++)
                        {
                            for (int j = q; j < q + img.cols / (48 * mul); j++)
                            {
                                Vec3b color = g_corr.at<Vec3b>(i, j);
                                /*if (2 * color[1] >= color[2] + color[0] + iter&&gavg >= 45)//&&2*color[1]>color[2]+color[0])
                                {
                                    color[1] = 165;
                                    color[0] = 0;
                                    color[2] = 255;
                                }*/
                                if (2 * color[2] >= color[1] + color[0] + iter + 15 && gavg >=45 /*(color[0]<=bavg) &&( color[1]<=gavg)*/)//&&2*color[2]>color[1]+color[0])
                                {
                                    color[1] = 255;
                                    color[0] =255;
                                    color[2] = 255;
                                }
                                else
                                {
                                    color[1] =0;
                                color[0] =0;
                                color[2] =0;
                                }


                        imgg.at<Vec3b>(i, j) = color;
                    }
                }

            }

        }
               // cout<<p<<endl;

                // myfile << "Writing this to a file ends.....................\n";
    }

        imshow("correction", imgg);

       waitKey(0);
       /* if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
               {
                        cout << "esc key is pressed by user" << endl;
                        break;
               }*/


//    myfile.close();
    return 0;
}


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
