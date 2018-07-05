#include <boost/lexical_cast.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/conversions.h>
using namespace std;
using namespace cv;

int main()
{
    char pcd_name[100];
    char pic_name[100];
    char fg_name[100];
      char depth_name[100];
    char nobg_pcd_name[100];
     char nobg_depth_name[100];
      char nobg_pic_name[100];
    int num=1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    while(num<26)
    {
        sprintf(pcd_name,"%s%d%s","../../data/2017-6-4-7-37/pcd/",num,".pcd");
        sprintf(pic_name,"%s%d%s","../../data/2017-6-4-7-37/pic/",num,".png");
        sprintf(fg_name,"%s%d%s","../../data/2017-6-4-7-37/nobg/",num,".png");
        sprintf(depth_name,"%s%d%s","../../data/2017-6-4-7-37/depth/",num,".png");
        sprintf(nobg_pcd_name,"%s%d%s","../../data/2017-6-4-7-37/nobg_pcd/",num,".pcd");
        sprintf(nobg_depth_name,"%s%d%s","../../data/2017-6-4-7-37/nobg_depth/",num,".png");
         sprintf(nobg_pic_name,"%s%d%s","../../data/2017-6-4-7-37/nobg_pic/",num,".png");
        num++;
//        ****build depth image and save as png
        pcl::io::loadPCDFile(pcd_name,*cloud);
        Mat depth_im(cv::Size(640,480),CV_8UC1);
        Mat depth(cv::Size(640,480),CV_32FC1);
        depth.setTo(0);
        depth_im.setTo(0);
        Mat fg_im=  imread(fg_name,0);
        float min = 10;
        float max = 0;
        for(int i=0;i<depth.rows;i++)
        {
            float *ptr = depth.ptr<float>(i);
            for(int j=30;j<depth.cols-10;j++)
            {
                if(pcl::isFinite(cloud->at(j,i)))
                {
                    ptr[j] =  (float)cloud->at(j,i).z;
                    //                cout<<ptr[j]<<endl;
                    if((float)cloud->at(j,i).z<min)
                    {
                        min = (float)cloud->at(j,i).z;
                    }
                    if((float)cloud->at(j,i).z>max)
                    {
                        max =  (float)cloud->at(j,i).z;
                    }
                }
            }
        }
//        cout<<min<<" "<<max<<endl;
        float t = max-min;
        for(int i=0;i<depth.rows;i++)
        {
            uchar *ptr = depth_im.ptr<uchar>(i);
            float *dptr = depth.ptr<float>(i);
            for(int j=0;j<depth.cols;j++)
            {
                if(dptr[j]>0)
                {
                    ptr[j] =   255- (dptr[j] - min)/t*255.0;
                }
                }
        }
        imwrite(depth_name,depth_im);
//        waitKey(0);
//       Mat ndepth_im(cv::Size(640,480),CV_8UC1);
//       ndepth_im.setTo(0);
//       int num = 0;
//       for(int i=0;i<depth.rows;i++)
//       {
//           uchar *nptr = ndepth_im.ptr<uchar>(i);
//           uchar *ptr = fg_im.ptr<uchar>(i);
//           uchar *dptr = depth_im.ptr<uchar>(i);
//           for(int j=0;j<depth.cols;j++)
//           {
//               if(int(ptr[j])!=0)
//               {
//                   nptr[j] = dptr[j] ;
//                   num++;
//               }
//           }
//       }
//        imwrite(nobg_depth_name,ndepth_im);
//        ***************************
//****save no background image
//       Mat nobg_fg = imread(fg_name,0);
//        Mat pic = imread(pic_name);
//       Mat nobg_pic(cv::Size(640,480),CV_8UC3);
//       nobg_pic.setTo(0);
//       for(int i=0;i<nobg_fg.rows;i++)
//       {
//           uchar *nptr = nobg_fg.ptr<uchar>(i);
//           Vec3b *ptr = pic.ptr<Vec3b>(i);
//           Vec3b *ptr2 = nobg_pic.ptr<Vec3b>(i);
//           for(int j=0;j<nobg_fg.cols;j++)
//           {
//               if(int(nptr[j])!=0)
//               {
//                   ptr2[j] = ptr[j] ;
//               }
//           }
//       }
//       cout<<num<<endl;
//       imwrite(nobg_pic_name,nobg_pic);
//        ***************************

//*** build no background pointcloud and save
//        Mat nobg_fg = imread(fg_name,0);
//        pcl::io::loadPCDFile(pcd_name,*cloud);
//        for(int i=0;i<nobg_fg.rows;i++)
//        {
//            uchar *nptr = nobg_fg.ptr<uchar>(i);
//            for(int j=0;j<nobg_fg.cols;j++)
//            {
//                if(int(nptr[j])==0)
//                {
//                    cloud->at(j,i).r =0;
//                     cloud->at(j,i).g =0;
//                      cloud->at(j,i).b =0;
//                      cloud->at(j,i).z = 0;
//                }
//            }
//        }
//        pcl::io::savePCDFile(nobg_pcd_name,*cloud);
//       waitKey(0);
    }
    return 0;
}
