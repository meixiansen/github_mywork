#include <fstream>
#include "SLIC.h"
#include"MeanShift.h"
#include"SPixels.h"
#include"Depth_Gmm.h"
#include<iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/openni_camera/openni_image.h>
//#include <pcl/io/openni_camera/openni_depth_image.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
typedef unsigned int UINT;
const int dilation_size = 5;
const cv::Rect box(0,0,640,480);
cv::Mat bgmodel;
Vec3b black(0,0,0);
Vec3b red(0,0,255);

RNG rng(0xFFFFFFFF);
//getrect
bool cmpx(const cv::Point2f a, const cv::Point2f b)
{
    return a.x<b.x;
}
bool cmpy(const cv::Point2f a,const cv::Point2f b)
{
    return a.y<b.y;
}
cv::Rect getrect( std::vector<cv::Point> contours)
{
    sort(contours.begin(),contours.end(),cmpx);
    int x=contours[0].x;
    int width=contours[contours.size()-1].x-x;
    sort(contours.begin(),contours.end(),cmpy);
    int y=contours[0].y;
    int height=contours[contours.size()-1].y-y;
    return cv::Rect(x,y,width,height);
}
void readBgModel(String filename)
{
    ifstream infile;
    infile.open(filename);
    MixData<float>* mptr = (MixData<float>*)bgmodel.data;
    if(infile.is_open())
    {
        for(int y = box.y; y < box.height; y++ )
        {
            for( int x = box.x; x< box.width; x++, mptr+=5 )
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int num = 8;

    Depth_Gmm mog;

    bgmodel.create( 1, box.width*box.height*defaultNMixtures*(2 + 2*3), CV_32F );
    bgmodel = Scalar::all(0);
    readBgModel("../bgmodel.txt");
    while(num<395)
    {
        cout<<num<<endl;
        sprintf(pic_name,"%s%d%s","../1-7-15:24//pic//",num,".png");
        sprintf(pcd_name,"%s%d%s","../1-7-15:24//pcd//",num,".pcd");
        sprintf(save_name,"%s%d%s","../1-7-15:24//bw6//",num,".png");
        pcl::io::loadPCDFile(pcd_name,*cloud);
        Mat ori;
        ori = imread(pic_name,1);
        Mat corse_result(ori.size(),CV_8UC3);
        corse_result = ori.clone();
        Mat depth_image(ori.size(),CV_32FC1);
        depth_image.setTo(0);
        Mat mi_fg(ori.size(),CV_8UC1);
        Mat fg(ori.size(),CV_8UC1);
        fg.setTo(0);

        for(int i = box.y;i<box.height;i++)
        {
            float* ptr = depth_image.ptr<float>(i);
            for(int j = box.x;j<box.width;j++)
            {
                if(pcl::isFinite(cloud->at(j,i)))
                {
                    ptr[j] = (float)cloud->at(j,i).z*100.0;
                }
            }
        }

        mog.run(box,depth_image,mi_fg,0.5,bgmodel);
        int size = 0;
        vector<vector<SPixels> > result_sp;

        cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                cv::Point(dilation_size, dilation_size));
        cv::erode(mi_fg,mi_fg,element);
        cv::dilate(mi_fg,mi_fg,element);
        vector<vector<Point> > contours;
        vector<cv::Vec4i> hier;
        findContours(mi_fg,contours,hier,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(fg,contours,-1,cv::Scalar(255,255,255),CV_FILLED);
        Mat nobg_depth(ori.size(),CV_32FC1);
        nobg_depth.setTo(0);
        for(int i = box.y;i<box.height;i++)
        {
            float* nptr = nobg_depth.ptr<float>(i);
            Vec3b* ptr = corse_result.ptr<Vec3b>(i);
            uchar* optr = fg.ptr<uchar>(i);
            for(int j = box.x;j<box.width;j++)
            {
                if(optr[j] == 0)
                {
                    ptr[j] = black;
                }
                else
                {
                    nptr[j] = (float)cloud->at(j,i).z*100.0;
                }
            }
        }
        Mat edge_im(ori.size(),CV_32FC1);
        Mat direction_im(ori.size(),CV_8UC1);
        edge_im.setTo(0);
        direction_im.setTo(0);
        for(int i = box.y;i<box.height;i++)
        {
            uchar* optr = fg.ptr<uchar>(i);
            float* eptr = edge_im.ptr<float>(i);
            uchar* dptr = direction_im.ptr<uchar>(i);
            for(int j = box.x;j<box.width;j++)
            {
                if(optr[j] != 0)
                {
                    float d[4];
                    d[0] = ((nobg_depth.at<float>(i-1,j-1)-nobg_depth.at<float>(i-1,j+1))+2.0*(nobg_depth.at<float>(i,j-1)-nobg_depth.at<float>(i,j+1))+(nobg_depth.at<float>(i+1,j-1)-nobg_depth.at<float>(i+1,j+1)))/4.0;
                    d[1] = ((nobg_depth.at<float>(i-1,j-1)-nobg_depth.at<float>(i+1,j+1))+2.0*(nobg_depth.at<float>(i,j-1)-nobg_depth.at<float>(i+1,j))+2.0*(nobg_depth.at<float>(i-1,j)-nobg_depth.at<float>(i,j+1)))/5.0;
                    d[2] = ((nobg_depth.at<float>(i-1,j-1)-nobg_depth.at<float>(i+1,j-1))+2.0*(nobg_depth.at<float>(i-1,j)-nobg_depth.at<float>(i+1,j))+(nobg_depth.at<float>(i-1,j+1)-nobg_depth.at<float>(i+1,j+1)))/4.0;
                    d[3] = (2.0*(nobg_depth.at<float>(i-1,j)-nobg_depth.at<float>(i,j-1))+2.0*(nobg_depth.at<float>(i,j+1)-nobg_depth.at<float>(i+1,j))+(nobg_depth.at<float>(i-1,j+1)-nobg_depth.at<float>(i+1,j-1)))/5.0;
                    float max = 0;
                    int direction = -1;
                    for(int k = 0;k<4;k++)
                    {
                        //                        cout<<d[k]<<" ";
                        if(fabs(d[k])>fabs(max))
                        {

                            max = d[k];
                            direction = k;
                        }
                    }
                    eptr[j] = max;
                    //                    cout<<max<<endl;
                    dptr[j] = direction;
                }
            }
        }
        Mat edge_show(ori.size(),CV_8UC1);
        edge_show.setTo(0);
        for(int i = box.y;i<box.height;i++)
        {
            uchar* optr = fg.ptr<uchar>(i);
            float* eptr = edge_im.ptr<float>(i);
//            uchar* dptr = direction_im.ptr<uchar>(i);
            for(int j = box.x;j<box.width;j++)
            {
                if(optr[j] != 0)
                {

                    if(fabs(eptr[j])>1)
                    {
//                        cout<<float(eptr[j])<<" "<<int(dptr[j])<<endl;
                        edge_show.at<uchar>(i,j) = 255;
                    }
                }
            }
        }
//        imshow("fg",fg);
        imshow("edge_show",edge_show);
        imshow("ori",ori);
        imshow("corse_result.png",corse_result);
//        waitKey(0);
//        cout<<"contours size:"<<contours.size()<<endl;
        vector<vector<Point> > small_object;
        for(int i = 0 ;i < contours.size() ; i++)
        {
            Mat region_mask(ori.size(),CV_8UC1);
            region_mask.setTo(0);
            Mat extract_im(ori.size(),CV_8UC3);
            extract_im.setTo(0);
            vector<vector<Point> > contour;
            contour.push_back(contours[i]);
            cv::drawContours(region_mask,contour,-1,cv::Scalar(255,255,255),CV_FILLED);
            Rect rec = getrect(contours[i]);
            if(rec.width*rec.height<2400)
            {
                small_object.push_back(contours[i]);
                size++;
            }
            else
            {
                for(int i = rec.y -1 ;i<rec.y+rec.height;i++)
                {
                    Vec3b* ptr1 = corse_result.ptr<Vec3b>(i);
                    Vec3b* ptr2 = extract_im.ptr<Vec3b>(i);
                    uchar* optr = region_mask.ptr<uchar>(i);
                    for(int j = rec.x -1 ;j<rec.x+rec.width;j++)
                    {
                        if(optr[j] == 255)
                        {
                            ptr2[j] = ptr1[j];
                        }
                    }
                }
                Mat roi = extract_im(rec);
                //                Rect rec1(rec.x-10,rec.y-10,rec.width+20,rec.height+20);
                //                imshow("roi",roi);
                //                Mat s_edge = edge_show(rec1);
                //                Mat long_edge(cv::Size(rec1.width,rec1.height),CV_8UC1);
                //                long_edge.setTo(0);
                //                vector<vector<Point> > edge_contour;
                //                vector<cv::Vec4i> edge_hier;
                //                findContours(s_edge,edge_contour,edge_hier,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
                //                int max = 0;
                //                int id = 0;
                //                for(int i=0;i<edge_contour.size();i++)
                //                {
                //                    if(edge_contour[i].size()>max)
                //                    {
                //                        max = edge_contour[i].size();
                //                        id = i;
                //                    }
                //                }
                //                cv::drawContours(long_edge,edge_contour,id,cv::Scalar(255,255,255),CV_FILLED);
                //                imshow("s_edge",s_edge);
                //                waitKey(0);
                //                cout<<"roi cols:"<<roi.cols<<endl;
                //                cout<<"roi rows:"<<roi.rows<<endl;
                //                cout<<"roi area:"<<roi.rows*roi.cols<<endl;
                //                imshow("roi",roi);
                //                waitKey(0);
                /**************************************************************/
                //  slic cuda parameters
                //                int diamSpx = 25;
                //                int wc = 50;
                //                int nIteration = 5;
                //                SLIC_cuda::InitType initType = SLIC_cuda::SLIC_SIZE;
                //                auto start = cv::getTickCount();
                //                //start segmentation
                //                SLIC_cuda slic_cuda;
                //                slic_cuda.Initialize(roi,diamSpx,wc,nIteration,initType);
                //                slic_cuda.Segment(roi);
                //                auto end = cv::getTickCount();
                //                cout << "runtime segmentation gpu " << (end - start) / cv::getTickFrequency() <<" s"<<endl;

                //                cv::Mat out = roi.clone();

                //                slic_cuda.displayBound(out, cv::Scalar(0, 0, 255));

                //                cv::imshow("out", out);
                /**************************************************************/

                /**************************************************************/
                //slic
                int width=roi.cols;
                int height=roi.rows;
                int sz = width*height;
                double slic_t1 = getTickCount();

                UINT *roi1=new UINT[sz*3];
                for(int c=0;c<3;c++)
                {
                    for(int i=0;i<width;i++)
                    {
                        for(int j=0;j<height;j++)

                            roi1[c*(width*height)+i*height+j]=saturate_cast<unsigned int>(roi.at<Vec3b>(j,i)[2-c]);
                    }
                }
                int* labels1 = new int[sz];
                int* truelabels1=new int[sz];
                //                vector< vector<int> > a_seed1;
                //                vector< vector<int> > a_label1;
                int spcounta=0;
                int spcount=sz/60;
                cout<<"slic size:"<<spcount<<endl;
                double compactness=30.0;
                vector<double> kkseedsx1(0);
                vector<double> kkseedsy1(0);
                vector<int> ccontourx1;
                vector<int> ccontoury1;
                SLIC slic;
                slic.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(roi1, height, width, labels1, spcounta, spcount, compactness, kkseedsy1,  kkseedsx1);
                slic.DrawContoursAroundSegments(roi1, labels1, height, width, 0, ccontoury1, ccontourx1);

                for(int i=0;i<height;i++)
                {
                    for(int j=0;j<width;j++)
                    {
                        truelabels1[i*width+j]=labels1[j*height+i];
                    }
                }
                //                slic.slicsave(truelabels1, width, height, kkseedsx1, kkseedsy1, ccontourx1, ccontoury1, a_seed1, a_label1);
                //                for(int c=0;c<3;c++)
                //                {
                //                    for(int i=0;i<width;i++)
                //                    {
                //                        for(int j=0;j<height;j++)

                //                            show.at<Vec3b>(j,i)[2-c]= roi1[c*(width*height)+i*height+j];
                //                    }
                //                }
                cout<<"slic over"<<endl;
                double slic_t2 = getTickCount();
                cout<<"slic time :"<<(slic_t2-slic_t1)/getTickFrequency() <<" s"<<endl;
                /**************************************************************/
                int contournum = ccontoury1.size();
                //                    for(int i =0;i<contournum;i++)
                //                    {

                //                        int x = ccontourx1[i];
                //                        int y = ccontoury1[i];
                //                        show.at<Vec3b>(y,x) = red;

                //                    }
                cv::Mat test(roi.size(),CV_8UC1);
                test.setTo(0);
                //                for(int i = 0;i<roi.rows;i++)
                //                {
                //                    Vec3b *ptr = roi.ptr<Vec3b>(i);
                //                    for(int j = 0;j<roi.cols;j++)
                //                    {
                //                         if(ptr[j]!=black){
                //                             test.at<uchar>(i,j) =255;
                //                         }
                //                    }
                //                }
                //                imshow("test",test);
                //                waitKey(0);
                vector<SPixels> superpixels;
                for(int i = 0;i<roi.rows;i++)
                {
                    Vec3b *ptr = roi.ptr<Vec3b>(i);
                    for(int j = 0;j<roi.cols;j++)
                    {
                        if(ptr[j]!=black){
                            int index = i*width + j;
                            int label =  truelabels1[index];
                            bool isTaken = false;
                            for(int k = 0;k<superpixels.size();k++)
                            {
                                if(superpixels[k].label == label)
                                {
                                    superpixels[k].points.push_back(cv::Point(j,i));
                                    isTaken = true;
                                    break;
                                }
                            }
                            if(!isTaken)
                            {
                                SPixels sp;
                                sp.label = label;
                                sp.points.push_back(cv::Point(j,i));
                                superpixels.push_back(sp);
                            }
                        }
                    }
                }
                vector<vector<double> > LABXY_palettes;
                for(int i =0;i<superpixels.size();i++)
                {
                    vector<double> labxy;
                    double l,a,b;
                    int x,y;
                    superpixels[i].buildFeatures(cv::Point(rec.x,rec.y),roi,cloud);
                    superpixels[i].RGB2LAB(superpixels[i].r,superpixels[i].g,superpixels[i].b,l,a,b);
                    labxy.push_back(l);
                    labxy.push_back(a);
                    labxy.push_back(b);
                    labxy.push_back(x);
                    labxy.push_back(y);
                    LABXY_palettes.push_back(labxy);
                }
                // meanshift cluster
                vector<vector<double> > clusters;
                vector<int> group(LABXY_palettes.size(), 0);
                double meanshift_t1 = getTickCount();
                cluster(LABXY_palettes,9, clusters, group);

                for(int i=0;i<group.size();i++)
                {
                    superpixels[i].flag = size + group[i];
                }
                size += clusters.size();
                cout<<"meanshift over size:"<< clusters.size()<<endl;
                double meanshift_t2 = getTickCount();
                cout<<"meanshift   time :"<<(meanshift_t2-meanshift_t1)/getTickFrequency() <<" s"<<endl;
                result_sp.push_back(superpixels);
            }
        }
        Mat result(ori.size(),CV_8UC3);
        result.setTo(0);
         vector<const Vec3b> colors;
        for(int i=0;i<size;i++)
        {
            const Vec3b c(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
//            c[0] = rng.uniform(0,255);
//            c[1] = rng.uniform(0,255);
//            c[2] = rng.uniform(0,255);
            colors.push_back(c);
        }
        for(int i =0;i<result_sp.size();i++)
        {

            for(int j =0;j<result_sp[i].size();j++)
            {
                result_sp[i][j].DrawPointsOnshow(result,colors[result_sp[i][j].flag]);

            }
        }
        for(int i = 0;i<small_object.size();i++)
        {
            drawContours(result,small_object,i,colors[colors.size()-i],CV_FILLED);
        }
        imshow("result",result);
        waitKey(0);
//        imwrite(save_name,result);
        num++;
    }

    return 0;
}
