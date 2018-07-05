#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std ;
using namespace cv;
const Vec3b Red(0,0,255);
const Vec3b Blue(255,0,0);
const Vec3b Green(0,255,0);
int main()
{
    Mat G = imread("3-5.png",0);

    Mat M = G.clone();
    for(int i=0;i<G.rows;i++)
    {
        uchar* ptr = G.ptr<uchar>(i);
        uchar* sptr = M.ptr<uchar>(i);
        for(int j=0;j<G.cols;j++)
        {
            if(ptr[j])
            {
                sptr[j+1]=255;
                sptr[j+2]=255;
                sptr[j+3]=255;
                sptr[j+4]=255;
                sptr[j+5]=255;
                sptr[j+6]=255;
                sptr[j+7]=255;
            }
        }
    }
    Mat R(M.size(),CV_8UC1);
    R.setTo(0);
    for(int i=0;i<R.rows;i++)
    {
        uchar* ptr = M.ptr<uchar>(i);
        uchar* sptr = R.ptr<uchar>(i);
        for(int j=0;j<R.cols;j++)
        {
            if(ptr[j])
            {
                sptr[j+5]=255;
            }
        }
    }
    Mat Show(R.size(),CV_8UC3);
    Show.setTo(0);
    int min_x=INT_MIN;
    int min_y=INT_MIN;
    int max_x=INT_MAX;
    int max_y=INT_MAX;
    for(int i=0;i<R.rows;i++)
    {
        uchar* ptr = G.ptr<uchar>(i);
        uchar* sptr = R.ptr<uchar>(i);
        Vec3b* s = Show.ptr<Vec3b>(i);
        for(int j=0;j<R.cols;j++)
        {
            if(ptr[j]&&sptr[j])
            {
                s[j]=Red;
            }
            if(!ptr[j]&&sptr[j])
            {
                s[j]=Blue;
            }
            if(ptr[j]&&!sptr[j])
            {
                s[j]=Green;
            }
        }
    }
    imwrite("G.png",G);
    imwrite("R.png",R);
    imwrite("show.png",Show);
    waitKey(0);
    return 0;
}
