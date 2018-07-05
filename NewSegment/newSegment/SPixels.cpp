#include"SPixels.h"


SPixels::SPixels(){
    flag = -1;
    label = -1;
    depth = -1;
    r = -1;
    g = -1;
    b = -1;
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
double  SPixels::CaculateSimLAB(const colorpalette &_c)
{
    double depth_dis = fabs(depth-_c.depth)*1000.0;
//     std::cout<<"depth:"<<depth<<std::endl;
//      std::cout<<"_c.depth:"<<_c.depth<<std::endl;
//    std::cout<<"depth_dis:"<<depth_dis<<std::endl;
    double lab_l1,lab_a1,lab_b1;
    RGB2LAB(r,g,b,lab_l1,lab_a1,lab_b1);
    double lab_l2,lab_a2,lab_b2;
    RGB2LAB(_c.r,_c.g,_c.b,lab_l2,lab_a2,lab_b2);
//   return sqrt((lab_a1-lab_a2)*(lab_a1-lab_a2)+(lab_l1-lab_l2)*(lab_l1-lab_l2)+(lab_b1-lab_b2)*(lab_b1-lab_b2)+depth_dis*depth_dis);
    return sqrt((lab_a1-lab_a2)*(lab_a1-lab_a2)+(lab_l1-lab_l2)*(lab_l1-lab_l2)+(lab_b1-lab_b2)*(lab_b1-lab_b2));

}

double  SPixels::CaculateSimRGB(const colorpalette &_c)
{

    double sim = sqrt((r-_c.r)*(r-_c.r)+(g-_c.g)*(g-_c.g)+(b-_c.b)*(b-_c.b));
    return sim;

}
void  SPixels::DrawPoints(cv::Mat &_image,cv::Vec3b _color)
{
    for(int i = 0;i<points.size();i++)
    {
        _image.at<cv::Vec3b>(points[i].y,points[i].x) = _color;
    }
}
void SPixels::DrawPointsOnshow(cv::Mat &_image,cv::Vec3b _color)
{

    for(int i = 0;i<points.size();i++)
    {
        _image.at<cv::Vec3b>(vertex.y+points[i].y,vertex.x+points[i].x) = _color;
    }
}

void SPixels::GetXY(int& _x,int& _y)
{
    _x = middle_point.x;
    _y = middle_point.y;
}
void SPixels::RGB2XYZ(
    const int&		sR,
    const int&		sG,
    const int&		sB,
    double&			X,
    double&			Y,
    double&			Z)
{
    double R = sR/255.0;
    double G = sG/255.0;
    double B = sB/255.0;

    double r, g, b;

    if(R <= 0.04045)	r = R/12.92;
    else				r = pow((R+0.055)/1.055,2.4);
    if(G <= 0.04045)	g = G/12.92;
    else				g = pow((G+0.055)/1.055,2.4);
    if(B <= 0.04045)	b = B/12.92;
    else				b = pow((B+0.055)/1.055,2.4);

    X = r*0.4124564 + g*0.3575761 + b*0.1804375;
    Y = r*0.2126729 + g*0.7151522 + b*0.0721750;
    Z = r*0.0193339 + g*0.1191920 + b*0.9503041;
}
void SPixels::RGB2LAB(const int& sR, const int& sG, const int& sB, double& lval, double& aval, double& bval)
{
    //------------------------
    // sRGB to XYZ conversion
    //------------------------
    double X, Y, Z;
    RGB2XYZ(sR, sG, sB, X, Y, Z);

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

    lval = 116.0*fy-16.0;
    aval = 500.0*(fx-fy);
    bval = 200.0*(fy-fz);
}
