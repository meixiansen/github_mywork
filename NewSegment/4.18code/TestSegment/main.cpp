#include"segment.h"
#include <fstream>
#include"MeanShift.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
using namespace std;

int main()
{
    char pic_name[100];
    char pcd_name[100];
    char save_name[100];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int num = 3 ;
    while(num<395)
    {
        cout<<num<<endl;
//        sprintf(pic_name,"%s%d%s","../1-7-15:24/pic/",num,".png");
//        sprintf(pcd_name,"%s%d%s","../1-7-15:24/pcd/",num,".pcd");
//        sprintf(pic_name,"%s%d%s","../2017-4-6-17-35/pic/",num,".png");
//        sprintf(pcd_name,"%s%d%s","../2017-4-6-17-35/pcd/",num,".pcd");
//        sprintf(pic_name,"%s%d%s","../../4.16data/2017-4-16-12-55/pic/",num,".png");
//        sprintf(pcd_name,"%s%d%s","../../4.16data/2017-4-16-12-55/pcd/",num,".pcd");
        sprintf(pic_name,"%s%d%s","../../4.16data/2017-4-16-15-14/pic/",num,".png");
        sprintf(pcd_name,"%s%d%s","../../4.16data/2017-4-16-15-14/pcd/",num,".pcd");
        sprintf(save_name,"%s%d%s","../4.26-result/0.7-3-5/",num,".png");
        pcl::io::loadPCDFile(pcd_name,*cloud);
        Mat ori;
        ori = imread(pic_name,1);
        double segment_t1 = getTickCount();
        Segment seg(ori,cloud);
        seg.runSegemnt();
//        imshow("result",seg.result);
//        imwrite("result.png",seg.result);
//        waitKey(0);
//        imwrite(save_name,seg.result);
        double segment_t2 = getTickCount();
        cout<<"segment time :"<<(segment_t2-segment_t1)/getTickFrequency() <<" s"<<endl;

        num++;
    }

    return 0;
}
