#include"segment.h"
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "GCoptimization.h"
using namespace std;
void readBgModel(String filename,Mat bgmodel)
{
    ifstream infile;
    infile.open(filename);
    MixData<float>* mptr = (MixData<float>*)bgmodel.data;
    if(infile.is_open())
    {
        for(int y = 0; y < 480; y++ )
        {
            for( int x = 0; x< 640; x++, mptr+=5 )
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
    infile.close();
}

int main()
{
    char pic_name[100];
    char pcd_name[100];
    char save_name[100];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Mat bgmodel;
    bgmodel.create( 1, 640*480*5*(2 + 2*3), CV_32F );
    bgmodel = Scalar::all(0);
    readBgModel("../../bgmodel-4.5.txt",bgmodel);
    int num = 1;
    while(num<40)
    {
            cout<<"image:"<<num<<endl;
//        sprintf(pic_name,"%s%d%s","../../../data/2017-4-16-12-55/pic/",num,".png");
//        sprintf(pcd_name,"%s%d%s","../../../data/2017-4-16-12-55/pcd/",num,".pcd");
//        sprintf(save_name,"%s%d%s","../../../data/2017-4-16-12-55/1_result_png/",num,".png");
//        sprintf(pic_name,"%s%d%s","../../../data/2017-4-16-15-14/pic/",num,".png");
//        sprintf(pcd_name,"%s%d%s","../../../data/2017-4-16-15-14/pcd/",num,".pcd");
//        sprintf(save_name,"%s%d%s","../../../data/2017-4-16-15-14/1_result_png/",num,".png");
        sprintf(pic_name,"%s%d%s","../../../data/2017-5-15-9-33/pic/",num,".png");
        sprintf(pcd_name,"%s%d%s","../../../data/2017-5-15-9-33/pcd/",num,".pcd");
        sprintf(save_name,"%s%d%s","../../../data/2017-5-15-9-33/1_result_png_3/",num,".png");
        pcl::io::loadPCDFile(pcd_name,*cloud);
        Mat ori;
        ori = imread(pic_name,1);
        double segment_t1 = getTickCount();
        Segment seg(ori,cloud,bgmodel);
        seg.runSegemnt();
        imshow("result",seg.result);
        waitKey(10);
//        imwrite(save_name,seg.result);
        double segment_t2 = getTickCount();
        cout<<"segment time :"<<(segment_t2-segment_t1)/getTickFrequency() <<" s"<<endl;
        num++;
    }

    return 0;
}
