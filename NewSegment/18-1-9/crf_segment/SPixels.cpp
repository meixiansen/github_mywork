#include"SPixels.h"


SPixels::SPixels(){
    true_label = -1;
    label = -1;
    depth = -1;
    r = -1;
    g = -1;
    b = -1;
    L = -1;
    A = -1;
    B = -1;
    middle_point = cv::Point(-1,-1);
}
void SPixels::buildFeatures(const cv::Point _vertex,cv::Mat _im, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
{
    vertex = _vertex;
    int size = points.size();
    int sum_x = 0;
    int sum_y = 0;
    int sum_r = 0;
    int sum_g = 0;
    int sum_b = 0;
    double sum_depth = 0;
    int cloud_x,cloud_y;
    cv::Vec3b red ;
    red[0] = 0;
    red[1] = 0;
    red[2] = 255;
    int num =0;
    for(int i= 0;i<size;i++)
    {
        //        im.at<cv::Vec3b>(points[i].y,points[i].x) = red;
        cloud_x = _vertex.x+points[i].x;
        cloud_y = _vertex.y+points[i].y;
        sum_x += points[i].x;
        sum_y += points[i].y;
        cv::Vec3b color = _im.at<cv::Vec3b>(points[i].y,points[i].x);
        sum_r += color[2];
        sum_g += color[1];
        sum_b += color[0];
        if(pcl::isFinite(_cloud->at(cloud_x,cloud_y)))
        {
            num++;
            //            std::cout<<(float)cloud->at(cloud_x,cloud_y).z<<std::endl;
            sum_depth += (float)_cloud->at(cloud_x,cloud_y).z;
        }
    }

    middle_point.x =  sum_x/size;
    middle_point.y =  sum_y/size;
    r = sum_r/size;
    g = sum_g/size;
    b = sum_b/size;
    depth = sum_depth/size;
    RGB2LAB();
    //    std::cout<<size<<" "<<num<<std::endl;
    //    cv::circle(im,middle_point,2,cv::Scalar(0,255,0),1);
    //    std::cout<<"r:"<<r<<" g:"<<g<<" b:"<<b<<" depth:"<<depth<<std::endl;
    //    cv::imshow("im",im);
    //    cv::waitKey(0);

}
void  SPixels::giveColorpalette(colorpalette &_c)
{
    _c.r = r;
    _c.g = g;
    _c.b = b;
    _c.depth = depth;
}
double  SPixels::CaculateSimLAB(SPixels sp)
{
    double depth_dis = fabs(depth-sp.depth)*1000.0;
    double xy_dis = sqrt((middle_point.x-sp.middle_point.x)*(middle_point.x-sp.middle_point.x)+(middle_point.y-sp.middle_point.y)*(middle_point.y-sp.middle_point.y));
    double lab_dis = +sqrt((this->L-sp.L)*(this->L-sp.L)+(this->A-sp.A)*(this->A-sp.A)+(this->B-sp.B)*(this->B-sp.B));
//    std::cout<<"xy_dis:"<<xy_dis<<std::endl;
//    std::cout<<"depth_dis:"<<depth_dis<<std::endl;
 return (4*depth_dis+3*lab_dis+3*xy_dis)/10.0;
}

double  SPixels::CaculateSimRGB(const colorpalette &_c)
{

    double sim = sqrt((r-_c.r)*(r-_c.r)+(g-_c.g)*(g-_c.g)+(b-_c.b)*(b-_c.b));
    return sim;

}
void  SPixels::DrawOnOriginimg(cv::Mat &_image,cv::Vec3b _color)
{

    if(_image.channels()==3)
    {
        for(int i = 0;i<points.size();i++)
        {
            _image.at<cv::Vec3b>(vertex.y+points[i].y,vertex.x+points[i].x) = _color;
        }
    }
}

void  SPixels::DrawOnOriginimg(cv::Mat &_image)
{
//    std::cout<<vertex.y<<" "<<vertex.x<<std::endl;
    if(_image.channels()==1)
    {
        for(int i = 0;i<points.size();i++)
        {
            _image.at<uchar>(vertex.y+points[i].y,vertex.x+points[i].x) = 255;
        }
    }
}

void SPixels::DrawPointsOnshow(cv::Mat &_image,cv::Vec3b _color)
{

    if(_image.channels()==3)
    {
        for(int i = 0;i<points.size();i++)
        {
            _image.at<cv::Vec3b>(points[i].y,points[i].x) = _color;
        }
    }
}

void SPixels::DrawPointsOnshow(cv::Mat &_image)
{
    if(_image.channels()==1)
    {
        for(int i = 0;i<points.size();i++)
        {
            _image.at<uchar>(points[i].y,points[i].x) = 255;
        }
    }
    if(_image.channels()==3)
    {
        cv::Vec3b _color;
        _color[0] =255;
        _color[1] =255;
        _color[2] =255;
        for(int i = 0;i<points.size();i++)
        {
            _image.at<cv::Vec3b>(points[i].y,points[i].x) = _color;
        }
    }
}

void SPixels::DrawEdgeOnshow(cv::Mat &_image)
{
    if(_image.channels()==1)
    {
        for(int i = 0;i<edge_points.size();i++)
        {
            _image.at<uchar>(edge_points[i].y,edge_points[i].x) = 0;
        }
    }
    if(_image.channels()==3)
    {
        cv::Vec3b _color;
        _color[0] =0;
        _color[1] =0;
        _color[2] =0;
        for(int i = 0;i<edge_points.size();i++)
        {
            _image.at<cv::Vec3b>(edge_points[i].y,edge_points[i].x) = _color;
        }
    }
}

void SPixels::CopyTo(SPixels &_sp)
{
    _sp.true_label = this->true_label;
    _sp.label = this->label;
    for(int i=0;i<points.size();i++)
    {
        _sp.points.push_back(points[i]);
    }
//    for(int i=0;i<edge_points.size();i++)
//    {
//        _sp.edge_points.push_back(edge_points[i]);
//    }
}
void SPixels::GetXY(int& _x,int& _y)
{
    _x = middle_point.x;
    _y = middle_point.y;
}
void SPixels::RGB2XYZ(
    double&			X,
    double&			Y,
    double&			Z)
{
    double R = this->r/255.0;
    double G = this->g/255.0;
    double B = this->b/255.0;

    double tr, tg, tb;

    if(R <= 0.04045)	tr = R/12.92;
    else				tr = pow((R+0.055)/1.055,2.4);
    if(G <= 0.04045)	tg = G/12.92;
    else				tg = pow((G+0.055)/1.055,2.4);
    if(B <= 0.04045)	tb = B/12.92;
    else				tb = pow((B+0.055)/1.055,2.4);

    X = tr*0.4124564 + tg*0.3575761 + tb*0.1804375;
    Y = tr*0.2126729 + tg*0.7151522 + tb*0.0721750;
    Z = tr*0.0193339 + tg*0.1191920 + tb*0.9503041;
}
void SPixels::RGB2LAB()
{
    //------------------------
    // sRGB to XYZ conversion
    //------------------------
    double X, Y, Z;
    RGB2XYZ(X, Y, Z);

    //------------------------
    // XYZ to LAB conversion
    //------------------------
    double epsilon = 0.008856;	//actual CIE standard
    double kappa   = 903.3;		//actual CIE standard

    double Xr = 0.950456;	//reference white
    double Yr = 1.0;		//reference white
    double Zr = 1.088754;	//reference white

    double xr = X/Xr;
    double yr = Y/Yr;
    double zr = Z/Zr;

    double fx, fy, fz;
    if(xr > epsilon)	fx = pow(xr, 1.0/3.0);
    else				fx = (kappa*xr + 16.0)/116.0;
    if(yr > epsilon)	fy = pow(yr, 1.0/3.0);
    else				fy = (kappa*yr + 16.0)/116.0;
    if(zr > epsilon)	fz = pow(zr, 1.0/3.0);
    else				fz = (kappa*zr + 16.0)/116.0;

    L = 116.0*fy-16.0;
    A = 500.0*(fx-fy);
    B = 200.0*(fy-fz);

}
