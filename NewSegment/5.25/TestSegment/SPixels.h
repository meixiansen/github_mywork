#ifndef SPIXELS_H
#define SPIXELS_H
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
struct colorpalette
{
    int r;
    int g;
    int b;
    double depth;
    cv::Point p;

};
struct LAB
{
    double L;
    double A;
    double B;

};
class SPixels
{
public:
    SPixels();
    //    virtual ~SPixels();
    int true_label;
    std::vector<cv::Point> points;
    std::vector<cv::Point> edge_points;
    int label;
    double depth;
    int r;
    int g;
    int b;
    double L;
    double A;
    double B;
    cv::Point middle_point;
    cv::Point vertex;
    void buildFeatures(const cv::Point _vertex,cv::Mat _im,pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
    void giveColorpalette(colorpalette &_c);
    double CaculateSimRGB(const colorpalette &_c);
    double CaculateSimLAB(const SPixels sp);
    void DrawOnOriginimg(cv::Mat &_image,cv::Vec3b _color);
    void DrawOnOriginimg(cv::Mat &_image);
    void DrawPointsOnshow(cv::Mat &_image,cv::Vec3b _color);
    void DrawPointsOnshow(cv::Mat &_image);
    void DrawEdgeOnshow(cv::Mat &_image);
    void CopyTo(SPixels &_sp);
    void RGB2XYZ(double& X,double& Y,double& Z);
    void RGB2LAB();
    void GetXY(int& _x,int& _y);

};

#endif // SPIXELS_H
