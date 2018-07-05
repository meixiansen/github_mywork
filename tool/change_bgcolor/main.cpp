#include<opencv/cv.h>
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;
const Vec3b black(0,0,0);
int main()
{
    char result_name[100];
    char nobg_name[100];
    int num=1;
    Mat nobg_im;
    Mat result3_im;
    while (num<69) {
        sprintf(result_name,"%s%d%s","../../data/2017-5-15-7-52/3_result_png_2/",num,".png");
        sprintf(nobg_name,"%s%d%s","../../data/2017-5-15-7-52/nobg/",num,".png");
        nobg_im = imread(nobg_name,1);
        result3_im = imread(result_name,1);
        for(int i = 0;i<nobg_im.rows;i++)
        {
            Vec3b *ptr = nobg_im.ptr<Vec3b>(i);
             Vec3b *ptr1 = result3_im.ptr<Vec3b>(i);
            for(int j = 0;j<nobg_im.cols;j++)
            {
                if(ptr[j] == black)
                {
                    ptr1[j]=black;
                }
            }
        }
        imwrite(result_name,result3_im);
        waitKey(10);
        num++;
    }
return 0;
}
