
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <stdio.h>
#include <iostream>

using namespace std;


int main(int argc, char *argv[])
 {
  int i=0;
  int frame_no=0;
  char name[1000]={'o','t'};
  cvNamedWindow("mainWin");
  CvVideoWriter *writer = 0;
  bool recording = false;

CvCapture* capture = cvCreateFileCapture( "/home/shaggy/line_auv/1_AUV_REC.avi");
/*if (argv[1]==NULL)
{
    cout<<"Cannot laod video"<<endl;
    return 0;
}*/

IplImage* frame;
cout<<"Press 's' to start recording, 'q' to stop recording 'Esc' to exit"<<endl;

while (1)
{
    cout<<"frame number"<<frame_no++<<endl;
    frame = cvQueryFrame( capture );
    if ( !frame ) break;
    cvShowImage( "mainWin", frame );
    char c = cvWaitKey(33);

    if ((c=='s')&&(recording==false))
    {
        recording=true;
        sprintf(name,"%d.avi",i);
        writer=cvCreateVideoWriter(name,CV_FOURCC('M', 'P', '4', '2'),15,cvSize(640,480),1);
        i++;
    }

    if (recording==true)
    {
        cvWriteFrame(writer,frame);      // add the frame to the file
        cvShowImage( "Output", frame );
    }

    if ((c=='q')&&(recording==true))
    {
        recording=false;
        cvReleaseVideoWriter(&writer);
    }

    if ( c == 27 ) break;
}

cvReleaseCapture( &capture );
if (writer!=NULL) cvReleaseVideoWriter(&writer);
return 0;
}
