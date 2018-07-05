#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
#include<vector>
#include"SLIC.h"
using namespace std;
using namespace cv;

const Vec3b Red(0,0,255);
const Vec3b Black(0,0,0);
const Vec3b White(0,255,0);
typedef unsigned int UINT;
class Label
{
public:
    void mouseClick(int event, int x, int y, int flags, void* param);
    void setImageAndWinName(const Mat& _image, const string& _winName);
    void show();
    void doSlic();
    void reset();
    void save(char name[100]);
    int objId;
private:
    void drawSup(Point _p);
    void udrawSup(Point _p);
    Mat mask;
    Mat all_mask;
    bool leftbutton_down = false;
      bool rightbutton_down = false;
    Mat process_image;
    const string* winName;
    Mat origin_image;
    int* truelabels1 ;
};
void Label::doSlic()
{
    int width=origin_image.cols;
    int height=origin_image.rows;
    int sz = width*height;
    UINT *roi1=new UINT[sz*3];
    for(int c=0;c<3;c++)
    {
        for(int i=0;i<width;i++)
        {
            for(int j=0;j<height;j++)

                roi1[c*(width*height)+i*height+j]=saturate_cast<unsigned int>(origin_image.at<Vec3b>(j,i)[2-c]);
        }
    }
    int* labels1 = new int[sz];
    int spcounta=0;
    int spcount=sz/15;
    double compactness=30.0;
    vector<double> kkseedsx1(0);
    vector<double> kkseedsy1(0);
    SLIC slic;
    vector<int> ccontourx1(0);
    vector<int> ccontoury1(0);
    slic.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(roi1, height, width, labels1, spcounta, spcount, compactness, kkseedsy1,  kkseedsx1);
    slic.DrawContoursAroundSegments(roi1, labels1, height, width, 0, ccontoury1, ccontourx1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            truelabels1[i*width+j]=labels1[j*height+i];
        }
    }
//    Mat show = origin_image.clone();
//    for(int i=0;i<ccontourx1.size();i++)
//    {
//        show.at<Vec3b>(ccontoury1[i],ccontourx1[i]) = Red;
//    }
//    imshow("show",show);
//    waitKey(0);
}
void Label::drawSup(Point _p)
{
    int label = truelabels1[_p.y*origin_image.cols+_p.x];
    for(int i=0;i<origin_image.rows;i++)
    {
        Vec3b* tptr = process_image.ptr<Vec3b>(i);
        uchar* ptr = mask.ptr<uchar>(i);
        uchar* aptr = all_mask.ptr<uchar>(i);
        for(int j=0;j<origin_image.cols;j++)
        {
            int index =i*origin_image.cols + j;
            if(truelabels1[index]==label)
            {
                ptr[j] =255;
                aptr[j] = 255;
                tptr[j] = Red;
            }
        }
    }
}
void Label::udrawSup(Point _p)
{
    int label = truelabels1[_p.y*origin_image.cols+_p.x];
    for(int i=0;i<origin_image.rows;i++)
    {
        Vec3b* tptr = process_image.ptr<Vec3b>(i);
        Vec3b* optr = origin_image.ptr<Vec3b>(i);
        uchar* ptr = mask.ptr<uchar>(i);
        uchar* aptr =all_mask.ptr<uchar>(i);
        for(int j=0;j<origin_image.cols;j++)
        {
            int index =i*origin_image.cols + j;
            if(truelabels1[index]==label)
            {
                aptr[j] = 0;
                ptr[j] =0;
                tptr[j] = optr[j];
            }
        }
    }
}
void Label::mouseClick(int event, int x, int y, int flags, void*)
{
    switch (event)
    {
    case CV_EVENT_LBUTTONDOWN: // set rect or GC_BGD(GC_FGD) labels
    {
        leftbutton_down = true;
    }
        break;

    case CV_EVENT_LBUTTONUP: // set rect or GC_BGD(GC_FGD) labels
    {
        leftbutton_down = false;
    }
        break;
    case CV_EVENT_RBUTTONDOWN: // set GC_PR_BGD(GC_PR_FGD) labels
    {
        rightbutton_down = true;
        udrawSup(Point(x,y));
        show();
    }
        break;

    case CV_EVENT_RBUTTONUP: // set rect or GC_BGD(GC_FGD) labels
    {
        rightbutton_down = false;
    }
        break;
    case CV_EVENT_MOUSEMOVE:
    {

        if(leftbutton_down&&origin_image.at<Vec3b>(y,x)!=White&&all_mask.at<uchar>(y,x)!=255)
        {
            drawSup(Point(x,y));
            show();
        }
        if(rightbutton_down&&origin_image.at<Vec3b>(y,x)!=White)
        {
            udrawSup(Point(x,y));
            show();
        }
    }
        break;
    }
}
void Label::reset()
{
//    process_image = origin_image.clone();
    mask.setTo(0);
}
void Label::setImageAndWinName(const Mat& _image, const string& _winName)
{
    if (_image.empty() || _winName.empty())
        return;
    origin_image = _image.clone();
    winName = &_winName;
    process_image = origin_image.clone();
    mask.create( origin_image.size(),CV_8UC1);
    mask.setTo(0);
    all_mask.create( origin_image.size(),CV_8UC1);
    all_mask.setTo(0);
    truelabels1 = new int[origin_image.cols*origin_image.rows];
    objId = 1;
}
void Label::show()
{
    imshow(*winName,process_image);
}
void Label::save(char name[100])
{
    imwrite(name,mask);
    reset();
    show();
}
Label la;
static void on_mouse(int event, int x, int y, int flags, void* param)
{
    la.mouseClick(event, x, y, flags, param);
}
int main()
{
    bool out = false;
    char pic_name[100];
     char ori_name[100];
    int num =25
            ;
    while(num<26)
    {
//        sprintf(pic_name,"%s%d%s","../../data/2017-6-4-6-34/nobg/",num,".png");
//        Mat image = imread(pic_name,0);
//         Mat eimage (image.size(),CV_8UC1);
//         eimage.setTo(0);
//        threshold(image,eimage,50,255,THRESH_BINARY);
//        vector<cv::Vec4i> hier;
//        vector< vector<Point> > contours;
//        findContours(eimage,contours,hier,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
//        int Id =1;
//        for(int i=0 ;i< contours.size();i++)
//        {
//            if(contours[i].size()>30)
//            {
//            Mat mask(image.size(),CV_8UC1);
//            mask.setTo(0);
//            drawContours(mask,contours,i,cv::Scalar(255,255,255),CV_FILLED);
//            char save_name[100];
//            sprintf(save_name,"%s%d%s%d%s","../../data/2017-6-4-6-34/GT/",num,"-",Id,".png");
//            Id++;
//            imwrite(save_name,mask);
//            }
//        }
//        num++;
        const string winName = "image";
        namedWindow(winName, WINDOW_AUTOSIZE);
        setMouseCallback(winName, on_mouse, 0);

        sprintf(pic_name,"%s%d%s","../../data/2017-6-4-7-37/fgt/",num,".png");
         sprintf(ori_name,"%s%d%s","../../data/2017-6-4-7-37/pic/",num,".png");
        Mat image = imread(pic_name);
        Mat ori = imread(ori_name);
        Mat process_im(image.size(),CV_8UC3);
        process_im.setTo(Vec3b(0,255,0));
        for(int i =0;i<image.rows;i++)
        {
            Vec3b* ptr = image.ptr<Vec3b>(i);
            Vec3b* cptr = process_im.ptr<Vec3b>(i);
            for(int j=0;j<image.cols;j++)
            {
                if(ptr[j]!=Black)
                {
                    cptr[j] = ptr[j];
                }
            }
        }
        la.setImageAndWinName(process_im,winName);
        la.doSlic();
        la.show();
        imshow("ori",ori);
        for (;;)
        {
            int c = waitKey(0);
            switch ((char)c)
            {
            case '\x1b':
                cout << "Exiting ..." << endl;
                out = true;
                goto exit_main;
            case 'r':
                la.reset();
                la.show();
                break;
            case 's':
                char save_name[100];
                sprintf(save_name,"%s%d%s%d%s","../../data/2017-6-4-7-37/GT2/",num,"-",la.objId,".png");
                la.objId++;
                la.save(save_name);
                break;
            case 'n':
                goto next_png;
                break;
            }
        }
next_png:
        num++;
        destroyWindow(winName);
exit_main:
        if(out==true)
        {
            break;
        }
    }
    return 0;
}
