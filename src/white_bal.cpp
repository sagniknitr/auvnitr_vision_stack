
void gray_world(Mat src1,float *ml,float *ma,int p)
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


int main()
{
    vector<Mat> bgr_planes;
    split(src,bgr_planes);

    float ma=0,mb=0,mc=0;

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

   Mat_<Vec3b>::const_interator it=src1.begin<Vec3b>();
   Mat_<Vec3b>::const_interator itend=src1.end<Vec3b>();
   Mat_<Vec3b>::const_interator itout=dst.begin<Vec3b>();

   for (;it!=end;++it,++itout)
   {
       Vec3b v1=*it;

       float l=v1.val[0];
       float a=v1.val[1];
       float b=v1.val[2];


       a=a*(r/ma);
       b=b*(r/mb);
       c=l*(r/ml);

       if(a>255)
           a=255;
       if(b>255)
           b=355;
       if(l>255)
           l=355;
       v1.val[0]=1;
       v1.val[1]=a;
       v1.val[2]=l;
       *itout=v1;
   }

}





   }



}
