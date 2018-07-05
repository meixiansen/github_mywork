
#include"Depth_Gmm.h"
#include<iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/openni_camera/openni_image.h>
//#include <pcl/io/openni_camera/openni_depth_image.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;

cv::Mat bgmodel;
static cv::Rect box(0,0,640,480);
void readBgModel(String filename)
{
    ifstream infile;
    infile.open(filename);
    MixData<float>* mptr = (MixData<float>*)bgmodel.data;
    if(infile.is_open())
    {
        for(int y = box.y; y < box.height; y++ )
        {
            for( int x = box.x; x< box.width; x++, mptr+=5 )
            {
                for( int k = 0; k < 5; k++ )
                {
                    float w,mu,var,sortKey;
                    infile >> w >> mu>>var>>sortKey;
                    mptr[k].weight = w;
                    mptr[k].mean = mu;
                    mptr[k].var = var;
                    mptr[k].sortKey = sortKey;
                }
            }

        }
    }
    else
    {
        cout<<"model path error"<<endl;
    }
    infile.close();
}
int main()
{
    char pic_name[100];
    char save_name1[100];
    char save_name[100];
    char pcd_name[100];
    bgmodel.create(1,640*480*5*5,CV_32F);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int num =1;
    Depth_Gmm mog;
    Mat d_im(Size(640,480),CV_32FC1);
    Mat fg(Size(640,480),CV_8UC1);
     Mat s_fg(Size(640,480),CV_8UC1);
    Mat ori(Size(640,480),CV_8UC3);
    Mat result(Size(640,480),CV_8UC3);
    readBgModel("../../data/bgmodel-5.15.txt");
    while(num<26)
    {
//        sprintf(pic_name,"%s%d%s","../data/2017-5-15-10-37/pic/",num,".png");
//        sprintf(pcd_name,"%s%d%s","../data/2017-5-15-10-37/pcd/",num,".pcd");

        sprintf(pic_name,"%s%d%s","../../data/2017-6-4-7-37/pic/",num,".png");
        sprintf(pcd_name,"%s%d%s","../../data/2017-6-4-7-37/pcd/",num,".pcd");
        sprintf(save_name1,"%s%d%s","../../data/2017-6-4-7-37/nobg/",num,".png");
          sprintf(save_name,"%s%d%s","../../data/2017-6-4-7-37/fgt/",num,".png");

//          sprintf(pic_name,"%s%d%s","../data/2017-5-15-7-52/pic/",num,".png");
//          sprintf(pcd_name,"%s%d%s","../data/2017-5-15-7-52/pcd/",num,".pcd");
//          sprintf(save_name1,"%s%d%s","../data/2017-5-15-7-52/nobg/",num,".png");
//            sprintf(save_name,"%s%d%s","../data/2017-5-15-7-52/nobg_pic/",num,".png");
        ori = imread(pic_name,1);
        result = ori.clone();
        pcl::io::loadPCDFile(pcd_name,*cloud);
        d_im.setTo(0);
        for(int i = box.y;i<box.height;i++)
        {
            float* ptr = d_im.ptr<float>(i);
            for(int j = box.x;j<box.width;j++)
            {
                if(pcl::isFinite(cloud->at(j,i)))
                {
                    ptr[j] = (float)cloud->at(j,i).z*100.0;
                }
            }
        }
//        mog.saveBgmodel(box,d_im,fg,0.7,40,"bgmodel-5.15.txt");
        mog.run(box,d_im,fg,bgmodel);
        s_fg.setTo(0);
        Vec3b black;
        black[0] = 0;
        black[1] = 0;
        black[2] = 0;
        for(int i = box.y;i<box.height;i++)
        {
            Vec3b* ptr = result.ptr<Vec3b>(i);
            uchar* optr = fg.ptr<uchar>(i);
            uchar* sptr = s_fg.ptr<uchar>(i);
            for(int j = box.x+50;j<box.width;j++)
            {
                if(optr[j] == 255)
                {
                    sptr[j] = 255;
                }
            }
        }
        for(int i = box.y;i<box.height;i++)
        {
            Vec3b* ptr = result.ptr<Vec3b>(i);
            uchar* sptr = s_fg.ptr<uchar>(i);
            for(int j = box.x;j<box.width;j++)
            {
                if(sptr[j] == 0)
                {
                    ptr[j] = black;
                }
            }
        }
        cout<<num<<endl;
//        imshow("result",result);
        imwrite(save_name1,s_fg);
        num++;
//        imshow("ori",ori);
//        imshow("s_fg",s_fg);
//        imwrite(save_name,result);
//        imshow("result",result);
//        waitKey(0);
    }
    return 0;
}
