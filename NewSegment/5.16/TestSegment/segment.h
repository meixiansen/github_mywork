#ifndef SEGMENT_H
#define SEGMENT_H
#include"SLIC.h"
#include"SPixels.h"
#include"Depth_Gmm.h"
#include <boost/make_shared.hpp>
#include<vector>
#include <fstream>
#include<iostream>
using namespace std;
using namespace cv;
typedef unsigned int UINT;
const cv::Rect box(50,0,590,480);
const Vec3b black(0,0,0);
const Vec3b red(0,0,255);
const double PI = 3.14159265;
static RNG rng(0xFFFFFFFF);
struct Object
{
    Mat mask;
    vector< vector<Point> > contour;
};
//getrect
static bool cmpx(const cv::Point2f a, const cv::Point2f b)
{
    return a.x<b.x;
}
static bool cmpy(const cv::Point2f a,const cv::Point2f b)
{
    return a.y<b.y;
}
static cv::Rect getrect( std::vector<cv::Point> contours)
{
    sort(contours.begin(),contours.end(),cmpx);
    int x=contours[0].x;
    int width=contours[contours.size()-1].x-x;
    sort(contours.begin(),contours.end(),cmpy);
    int y=contours[0].y;
    int height=contours[contours.size()-1].y-y;
    return cv::Rect(x,y,width,height);
}
//获取自定义核
const int g_nStructElementSize1 = 2;
const Mat element1 = getStructuringElement(MORPH_RECT,
    Size(2*g_nStructElementSize1+1,2*g_nStructElementSize1+1),
    Point( g_nStructElementSize1, g_nStructElementSize1 ));
const int g_nStructElementSize2 = 3;
const Mat element2 = getStructuringElement(MORPH_RECT,
    Size(2*g_nStructElementSize2+1,2*g_nStructElementSize2+1),
    Point( g_nStructElementSize2, g_nStructElementSize2 ));
const int g_nStructElementSize3 = 1;
const Mat element3 = getStructuringElement(MORPH_RECT,
    Size(2*g_nStructElementSize3+1,2*g_nStructElementSize3+1),
    Point( g_nStructElementSize3, g_nStructElementSize3 ));
class Segment{

public:
    Mat origin_image;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    Mat foreground;
    Mat nobg_depth;
    vector<Object> objs;
    Mat result;
    //slic
    Segment();
    Segment(Mat img,pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
    void readBgModel(String filename,Mat bgmodel);
    void findcontours(Mat im_contour,vector< vector<Point>>& contours);
    void setSpixelLabel(vector<int> label,vector<SPixels> &superpixels,int flag);
    void getLabelBycontours(Mat roi,int* truelabels1,vector< vector<Point> > p_set,vector<int> &label );
    void getLabelBycontour(Mat roi,int* truelabels1,vector<Point> p_set,vector<int> &label );
    void getLabelBymask(Mat mask,int* truelabels1,vector<int> &label );
    void getSpByLabel(vector<int> label,vector<SPixels> superpixles, vector<SPixels> &sps );

    void getfg();
    void Depth_canny(cv::Mat mask,cv::Mat Depth_im,cv::Mat &Depth_edge);
    void DoSlic(Mat roi,int* truelabels1,vector<int> &ccontourx1,vector<int> &ccontoury1);
    void buildSuperpixels(Mat roi,Mat roi_mask,Rect rec,int* truelabels1, vector<int> ccontourx1,vector<int> ccontoury1,vector<SPixels> &b_superpixels,vector<SPixels> &t_superpixels);
    void getAdhereEdge(Mat roi_mask,Mat roi_edge,Mat &adhere_im);
    void getEffectEdge(Mat region_mask,vector<Point> contour,Mat adhere_im,Mat roi_mask,int *truelabels1,vector<SPixels> b_superpixels,vector<SPixels> t_superpixels,vector< vector<Point> > &effective_edgec);
    void setSuperpixelsLabels(int *truelabels1,Mat roi_mask,vector< vector<Point> > &effective_edgec,vector<SPixels> &t_superpixels,vector<int> &labels);
    void refinement(const Rect rec, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,vector<int> labels,Mat roi,int *truelabels1,vector<SPixels> &t_superpixels);
    void runSegemnt();

};
#endif // SEGMENT_H
