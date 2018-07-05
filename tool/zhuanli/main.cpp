
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std ;
using namespace cv;
void findcontours(Mat im_contour,vector< vector<Point> >& contours)
{
    Mat im_contour_c = im_contour.clone();
    vector<cv::Vec4i> hier;
    findContours(im_contour_c,contours,hier,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
}
int main()
{
//    Mat img1,img2,img3;
//    img1 = imread("keyregion1.png",0);
//    img2 = imread("keyregion2.png",0);
//    vector< vector<Point> > contours1;
//    findcontours(img1,contours1);
//    vector< vector<Point> > contours2;
//    findcontours(img2,contours2);
//    img3 = imread("roi_mask.png",0);
//    drawContours(img3,contours1,-1,cv::Scalar(0,0,0),1);
//     drawContours(img3,contours2,-1,cv::Scalar(0,0,0),1);
//    imwrite("save.png",img3);
//    waitKey(0);
//    Mat img1,img2,img3,img4;
//    img1 = imread("kr2.png");
//    img2 = imread("keyregion2.png",0);
//    img3.create(img1.size(),CV_8UC1);
//    img3.setTo(0);
////    img4.create(img1.size(),CV_8UC1);
////    img4.setTo(0);
//    Vec3b color;
//    Vec3b black(0,0,0);
//    for(int i=0;i<img1.rows;i++)
//    {
//        Vec3b* ptr1= img1.ptr<Vec3b>(i);
//        for(int j=0;j<img1.cols;j++)
//        {
//            if(ptr1[j]!=black)
//            {
//                color = ptr1[j];
//                break;
//            }
//        }
//    }
//    for(int i=0;i<img1.rows;i++)
//    {
//        Vec3b* ptr1= img1.ptr<Vec3b>(i);
//        uchar* ptr2 = img3.ptr<uchar>(i);
//        for(int j=0;j<img1.cols;j++)
//        {
//            if(ptr1[j]==color)
//            {
//                ptr2[j] =255;
//            }
//        }
//    }
//    vector< vector<Point> > contours;
//    findcontours(img3,contours);
//    drawContours(img2,contours,-1,cv::Scalar(0,0,0),1);
//    imwrite("seg.png",img2);
//    waitKey(0);
    Mat img1;
    img1 = imread("model2.png",0);
    threshold(img1,img1,1,255,THRESH_BINARY);
    imwrite("model.png",img1);
    waitKey(0);
    return 0;
}
