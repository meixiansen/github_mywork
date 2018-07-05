#include <math.h>
#include <dirent.h>
#include<string>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include<fstream>
#include<iostream>
#include<algorithm>
#include<vector>
#include"opencv/cv.h"
#include<opencv2/opencv.hpp>
using namespace std;
const cv::Vec3b black(0,0,0);
static cv::RNG rng(0xFFFFFFFF);
const char  gt_path[100] = "/home/wangchao/paper/data/2017-6-4-7-37/GT";
const char  gt_path2[100] = "/home/wangchao/paper/data/2017-6-4-7-37/GT2";
//const char  img_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/3_result_png_1";
//const char  gt_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/GT";
//const char  img_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/3_result_png_1";
//const char  gt_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/GT";
//const char  img_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/3_result_png_1";
//const char  gt_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/GT";
//const char  img_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/3_result_png_1";
//const char  gt_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/GT";
//const char  img_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/3_result_png_1";
//const char  gt_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/GT";
//const char  img_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33/3_result_png_1";
struct  OBJID
{
    int ImageID ;
    int ObjectID;
};
struct ColorSize
{
    cv::Vec3b color;
    int sum;
};
bool complare(const OBJID a,const OBJID b)
{
    if(a.ImageID!=b.ImageID)
    {
        return (a.ImageID<b.ImageID);
    }
    else
    {
        return a.ObjectID<b.ObjectID;
    }
}
void ReadGT(const char gt_path[100],vector<OBJID> &objs)
{

    DIR *dir=NULL;
    struct dirent* pDir=NULL;
    dir=opendir(gt_path);
    if(dir == NULL)
    {
        printf("Error! can't open this dir\n");

    }
    else{
        while(1)
        {
            OBJID obj;
            obj.ImageID = 0;
            obj.ObjectID = 0;
            pDir = readdir(dir);
            if (pDir == NULL) break;
            if (pDir->d_type == DT_REG)
            {
                vector<char> imageS;
                vector<char> objectS;
                string d_name = pDir ->d_name;
                bool findx = false;
                bool findd = false;
                for(int i=0;i<d_name.size();i++)
                {
                    if(d_name[i]=='-')
                    {
                        findx=true;
                    }
                    if(d_name[i]=='.')
                    {
                        findd=true;
                    }
                    if(!findx&&!findd)
                    {
                        imageS.push_back(d_name[i]);
                    }
                    if(findx&&!findd)
                    {
                        if(d_name[i]!='-')
                        {
                            objectS.push_back(d_name[i]);
                        }
                    }
                }
                for(int i=0;i<imageS.size();i++)
                {
                    obj.ImageID =obj.ImageID+ (imageS[i]-'0')*pow(10,imageS.size()-i-1);
                }
                for(int i=0;i<objectS.size();i++)
                {
                    obj.ObjectID = obj.ObjectID +(objectS[i] - '0')*pow(10,objectS.size()-i-1);
                }
                objs.push_back(obj);
            }
        }
    }
}
void countcolor(cv::Mat mask ,cv::Mat image,ColorSize &color,int &gt_sum )
{
    vector<ColorSize> colors;
    gt_sum = 0;
    for(int i=0;i<image.rows;i++)
    {
        uchar * p = mask.ptr<uchar>(i);
        cv::Vec3b* ptr = image.ptr<cv::Vec3b>(i);
        for(int j=0;j<image.cols;j++)
        {
            if(int(p[j])==255)
            {
                gt_sum++;
                bool IsTaken = false;
                for(int t=0;t<colors.size();t++)
                {
                    if(colors[t].color  == ptr[j])
                    {
                        IsTaken=true;
                        colors[t].sum++;
                    }
                }
                if(!IsTaken)
                {
                    if(ptr[j]!=black)
                    {
                        ColorSize c;
                        c.color = ptr[j];
                        c.sum=1;
                        colors.push_back(c);
                    }
                }
            }
        }
    }
    int max = 0;
    for(int c=0;c<colors.size();c++)
    {
        if(colors[c].sum>max)
        {
            max = colors[c].sum;
            color = colors[c];
        }
    }
}
void getMask_bycolor(cv::Mat image,cv::Vec3b color,cv::Mat &mask)
{
    for(int i=0;i<image.rows;i++)
    {
        cv::Vec3b* ptr = image.ptr<cv::Vec3b>(i);
        uchar* uptr = mask.ptr<uchar>(i);
        for(int j=0;j<image.cols;j++)
        {
            if(ptr[j]==color)
            {
                uptr[j] = 255;
            }
        }
    }
}
void countOutPint(cv::Mat gt_img,cv::Mat result_mask,int &out_sum)
{
    out_sum = 0;
    for(int i=0;i<gt_img.rows;i++)
    {
        uchar* gptr = gt_img.ptr<uchar>(i);
        uchar* rptr = result_mask.ptr<uchar>(i);
        for(int j=0;j<gt_img.cols;j++)
        {
            if(gptr[j]==0&&rptr[j] ==255)
            {
                out_sum++;
            }
        }
    }
}
int main()
{
    vector<OBJID> objs;
    ReadGT(gt_path,objs);
    vector<cv::Vec3b> colors;
    for(int i= 0;i<20;i++)
    {
        cv::Vec3b color;
        color[0] = rng.uniform(20,230);
        color[1] = rng.uniform(20,230);
        color[2] = rng.uniform(20,230);
        colors.push_back(color);
    }
    if(!objs.empty())
    {
        sort(objs.begin(),objs.end(),complare);
        int preID = 0;
        cv::Mat result(cv::Size(640,480),CV_8UC3);
        for(int i=0;i<objs.size();i++)
        {
            int nowID = objs[i].ImageID;
            if(nowID!=preID)
            {
                preID = nowID ;
                result.setTo(0);
            }
            char rgt_path[100];
            sprintf(rgt_path,"%s%s%d%s%d%s",gt_path,"/",objs[i].ImageID,"-",objs[i].ObjectID,".png");
            char save_path[100];
            sprintf(save_path,"%s%s%d%s",gt_path2,"/",objs[i].ImageID,".png");
            cv::Mat gt_img = cv::imread(rgt_path,-1);
            for(int xi=0;xi<gt_img.rows;xi++)
            {
                uchar* ptr = gt_img.ptr<uchar>(xi);
                cv::Vec3b* sptr = result.ptr<cv::Vec3b>(xi);
                for(int yj=0;yj<gt_img.cols;yj++)
                {
                    if(ptr[yj])
                    {
                        sptr[yj] = colors[objs[i].ObjectID-1];
                    }
                }
            }
            cv::imwrite(save_path,result);
//            cv::waitKey(0);
        }
    }
    return 0;
}


