#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;
const Vec3b black(0,0,0);
const Vec3b white(255,255,255);
int main()
{
    Mat keyregion1 = imread("keyregion1.png");
    Mat keyregion2 = imread("keyregion2.png");
     Mat keyr1 = keyregion1.clone();
     Mat keyr2 =keyregion2.clone();
    Mat result = imread("19.png");
    imshow("test",keyregion1);
    waitKey(0);
    for(int i = 0;i<keyregion1.rows;i++)
    {
//        Vec3b* k1 = keyregion1.ptr<Vec3b>(i);
//        Vec3b *k2 = keyregion2.ptr<Vec3b>(i);
        Vec3b* kr1 = keyr1.ptr<Vec3b>(i);
        Vec3b *kr2 = keyr2.ptr<Vec3b>(i);
          Vec3b *r = result.ptr<Vec3b>(i+256);
        for(int j= 0;j<keyregion1.cols;j++)
        {
            if(kr1[j]==white)
            {
                kr1[j] = r[j+172];
            }
            if(kr2[j]==white)
            {
                kr2[j] = r[j+172];
            }
        }
    }

    imwrite("kr1.png",keyr1);
    imwrite("kr2.png",keyr2);
//    imwrite("bodies.png",bodies);
    waitKey(0);
    return 0;
}
