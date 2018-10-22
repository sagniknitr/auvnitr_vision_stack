
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include<sstream>


using namespace std;
using namespace cv;


int main()
{
    VideoCapture cap(0);

     //VideoCapture capf(0);
    Mat src,src_f;
    if ((!cap.isOpened()))  // if not success, exit program
    {
        cout << "Cannot open the camera" << endl;
        return -1;
    }
    int count=0,bases_required=600;
    //int frame_width=600,frame_height=400;
    while (1)
    {

        //Mat img, gr;
        bool bSuccess1 = cap.read(src);
            //bool bSuccess2 = capf.read(src_f);
        if ((!bSuccess1))//||(!bSuccess2)) //if not success, break loop
        {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }
         imshow("down camera",src);
           //imshow("front camera",src_f);
        int start=waitKey(12);
        //int stop=waitKey();
        //++count;
        //++count;

        //VideoWriter out ( "out"+s+".avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);


        if(start>=50)
            {
            ++count;
            std::string s;
            std::stringstream out;
            out << count;
            s = out.str();
             // out.write(src);
                    imwrite(s+"b.jpg",src);
                   // imwrite(s+"f.jpg",src_f);
                    cout<<"image stored"<<"\n";
                    // bool bSuccessr = cap.read(src);
                     //start=waitKey(12);

        }
        /*else if(bases_required==count)
        {
            break;
        }*/

        else if( waitKey(10)==27)
         break;
        else
        continue;

}


    return 0;
}

