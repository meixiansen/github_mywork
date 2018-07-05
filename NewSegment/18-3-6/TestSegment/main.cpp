#include"segment.h"
#include <fstream>
#include"MeanShift.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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
    char depth_name[100];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Mat bgmodel;
    bgmodel.create( 1, 640*480*5*(2 + 2*3), CV_32F );
    bgmodel = Scalar::all(0);
    char sp_name[100]="2017-6-4-7-37";
    readBgModel("../../bgmodel-5.31.txt",bgmodel);
    int num = 1;
    double sum =0;
    while(num<26)
    {
        cout<<"image:"<<num<<endl;
        //        sprintf(pic_name,"%s%d%s","../../../data/2017-4-16-12-55/pic/",num,".png");
        //        sprintf(pcd_name,"%s%d%s","../../../data/2017-4-16-12-55/pcd/",num,".pcd");
        //        sprintf(save_name,"%s%d%s","../../../data/2017-4-16-12-55/1_result_png/",num,".png");
        //        sprintf(pic_name,"%s%d%s","../../../data/2017-4-16-15-14/pic/",num,".png");
        //        sprintf(pcd_name,"%s%d%s","../../../data/2017-4-16-15-14/pcd/",num,".pcd");
        //        sprintf(save_name,"%s%d%s","../../../data/2017-4-16-15-14/1_result_png/",num,".png");
        sprintf(pic_name,"%s%s%s%d%s","../../../data/",sp_name,"/nobg_pic/",num,".png");
        sprintf(depth_name,"%s%s%s%d%s","../../../data/",sp_name,"/nobg_depth/",num,".png");
        sprintf(pcd_name,"%s%s%s%d%s","../../../data/",sp_name,"/pcd/",num,".pcd");
        sprintf(save_name,"%s%s%s%d%s","../../../data/",sp_name,"/crf_result/iter5/",num,".png");

//        cout<<save_name<<endl;
        pcl::io::loadPCDFile(pcd_name,*cloud);
        Mat ori;
        ori = imread(pic_name,1);
        Mat depth_im = imread(depth_name,cv::IMREAD_GRAYSCALE);
//        imshow("ori",ori);
//        waitKey(0);
        double segment_t1 = getTickCount();
        Segment seg(ori,cloud,bgmodel,depth_im);
        seg.runSegemnt();
        double segment_t2 = getTickCount();
        sum+=(segment_t2-segment_t1)/getTickFrequency();
//        imshow("result",seg.result);
//        waitKey(0);
        imwrite(save_name,seg.result);

        num++;
    }
        cout<<"image sum:"<<num-1<<" segment time :"<<sum<<" s"<<endl;
    return 0;
}
