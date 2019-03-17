
#define CropSize Rect(175,30,240,380)
Rect myROI = CropSize;//Rect(200,36,250,250);
int erosion_size = 1;
int xmax,ymax;
double alim = 10.0;
int N;
#define GROW 0.95
Mat img;
Mat binary;
Mat dilate_element,erode_element;
Mat drawing,pB,cM;

vector<Point> arr;
vector<vector<int> > arena;
vector<Point> mc;
vector<vector<Point> > contours1;//stores contours
vector<Vec4i> h;//heirarchy
vector<Moments> mu;
Point rotCenter;
bool camstarted = false;  // if start camera is pressed or video is loaded
bool imageloaded = false;
bool videoloaded = false;
bool replayset = false;
bool origEmptyFlag = true; // data exception handling
bool paused = false;       // if paused is pressed
bool botstarted = false;
bool trackingfailed = true;
bool travelled=new bool[100000];
Point findCenter(vector<Point>);
double cc_angle(Point vtx,Point p1,Point p2);
double abs_distance(Point p1,Point p2);
Mat clonemat;
Scalar botColors[2][10];
Point bot_location;
double bot_facing;
//Point target_location;
vector<Point> setPoints;
//int currentLoc = 0;
Mat tempImage;

QSerialPort *serial;

#define GROW 0.95
#define limit1 15
#define limit2 30
#define limit3 70

#define gridsize 5
#define offset 0
#define dilation_size 7
double Kp = -60.0;
double Ki = 0.0;
double Kd = 0.0;
#define baseR 150
#define baseL 150
#define LNorm 200
#define RNorm 200
#define LBang "L100%R150%O2%"
#define RBang "L150%R100%O3%"
#define BANG 30
#define BotStop "O1%L0%R0%"
#define pickCommand "D2000%G125%P125%O6%O4%"
#define dropCommand "D2000%G125%P125%O5%O7%"
#define grabBall "D2000%G125%P125%O6%"
#define leaveBall "D2000%G125%P125%O7%"
#define BotBack "L150%R150%O0%"
#define BotForward "O1%L100%R100%"
#define GrabberStop "P0%G0%"



QImage::Format imgformat;

int cameraNumber = 0;
string loadedFile = "";

bool colorChecked = true;
bool controlChecked = false;
bool dohsv = true;
double gammma = 1;
bool jaaBetaJeeLeApniZindagi = false;
bool capture = false;

uchar ucolor;
Mat original, dilated, stPnts, temp;
vector<vector<Point> > tempcontours;
vector<Moments> momentvec;


Point target_location,botloc,rotcenter;
double botfacing;
//QSerialPort *serial;
int recttype[1000][1000]={0};
//Scalar botColors[3][10];
Mat orig,obstacles,grid,origHSV;
vector<int> traversalorder;
vector<vector<Point> > contours;
vector<Point> wayPoints;
vector<int> type;
vector<Point> allPoints;
uchar color;
bool solved=false,stopbefore=false;
Point stoppedAt;
//bool trackingfailed=true;
int currentLoc = 0;
vector<int> pointOfInterest;
vector<int> task;
Point prev;
int prev_gradient=0,current_gradient=-1;
int pos1;
int reason;
int arrowdirection = 0;
int skip=0;
map<int,map<int,int> > triangledirection;


void AutoColor::loadSaved()
{
    QFile file("D://config.qtiyapa");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    QTextStream in(&file);
    QString line;
    for(int i=0;i<10;i++)
    {
        for(int j=0;j<3;j++)
        {
            line = in.readLine();
            botColors[0][i][j]=line.toInt();
            line = in.readLine();
            botColors[1][i][j]=line.toInt();
        }
    }
}
Point AutoColor::findCenter(vector<Point> contour)
{
    int count  = 0;
    Point2f center = Point2f(0,0);
    for(int i = 0;i < contour.size();i++)
    {
        center.x = (center.x*count+contour[i].x)/(count+1);
        center.y = (center.y*count+contour[i].y)/(count+1);
        count++;
    }
    return center;
}
int AutoColor::triangleDir(vector<Point> contour)
{
  //  Mat tr = Mat::zeros(orig.size(),CV_8UC1);
   // cout << "next" << endl;
    double alim = 15.0;
   // cout << "cs" << contour.size() << endl;
    if(contour.size() == 0)
    {
        cout << "contour  empty" << endl;
        return 1;
    }
     //Moments mu = moments(contour, false );
     //Point mc = Point2i( int(mu.m10/mu.m00) , int(mu.m01/mu.m00) );
    Point mc = findCenter(contour);
    for(int i = 0;i<contour.size();i++)
    {
        //circle(tr,contour[i],1,Scalar::all(255),-1);
       // imshow("Y u no work",tr);
       // waitKey(0);
      //  cout << "point " << contour[i] << endl;
        //cout<<"cs "<<contour.size()<<endl;
        double temp =  cc_angle(mc,Point(orig.cols,mc.y),contour[i])*180.0/3.1415;
       // cout<<"temp "<<temp<<endl;
        if(((temp >= 180.0-alim)&&(temp <= 180+alim)) || (temp >= -180.0-alim)&&(temp <= -180+alim))
            return 4;
        else if( (temp >= 0.0-alim)&&(temp <= 0.0+alim))
            return 2;
        else if( (temp >= 90.0-alim)&&(temp <= 90.0+alim))
            return 1;
        else if( (temp >=-90.0-alim)&&(temp <= -90.0+alim))
            return 3;
    }
    return 3;
}
