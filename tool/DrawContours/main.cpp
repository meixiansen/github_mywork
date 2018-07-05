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
const char img_path[100] = "/home/wangchao/paper/data/2017-5-15-9-33";
const cv::Vec3b white(255,255,255);
struct  OBJID
{
    int ImageID ;
    int ObjectID;
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
int main()
{
    int num = 1;
    while(num<31)
    {
        char result_name[100];
        sprintf(result_name,"%s%s%d%s",img_path,"/1_result_png_1/",num,".png");
        char result2_name[100];
        sprintf(result2_name,"%s%s%d%s",img_path,"/1_result2_png/",num,".png");
        char pic_name[100];
        sprintf(pic_name,"%s%s%d%s",img_path,"/pic/",num,".png");
        cv::Mat result1 = cv::imread(result_name);
        cv::Mat pic = cv::imread(pic_name);
        for(int i=1;i<pic.cols-1;i++)
        {
            for(int j =1;j<pic.rows-1;j++)
            {
//                cout<<i<<" "<<j<<endl;
                if(result1.at<cv::Vec3b>(j-1,i)!=result1.at<cv::Vec3b>(j,i)|| result1.at<cv::Vec3b>(j+1,i)!=result1.at<cv::Vec3b>(j,i)||result1.at<cv::Vec3b>(j-1,i-1)!=result1.at<cv::Vec3b>(j,i)||result1.at<cv::Vec3b>(j,i-1)!=result1.at<cv::Vec3b>(j,i)||  result1.at<cv::Vec3b>(j+1,i-1)!=result1.at<cv::Vec3b>(j,i)||result1.at<cv::Vec3b>(j-1,i+1)!=result1.at<cv::Vec3b>(j,i)|| result1.at<cv::Vec3b>(j,i+1)!=result1.at<cv::Vec3b>(j,i)|| result1.at<cv::Vec3b>(j+1,i+1)!=result1.at<cv::Vec3b>(j,i))
                {
                    pic.at<cv::Vec3b>(j,i) = white;
                }
            }
        }
        num++;
        cv::imwrite(result2_name,pic);
    }

    return 0;
}
