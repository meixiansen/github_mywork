#include"segment.h"
#include"CrfSegment.h"

Segment::Segment(Mat img,pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,Mat bgmodel,Mat depth_img)
{
    depth_image = depth_img;
    origin_image = img.clone();
    result.create(origin_image.size(),CV_8UC3);
    result.setTo(0);
    foreground.create(origin_image.size(),CV_8UC1);
    foreground.setTo(0);
    nobg_depth.create(origin_image.size(),CV_32FC1);
    nobg_depth.setTo(0);
    cloud = _cloud;
    this->bgmodel = bgmodel.clone();
}
void Segment::findcontours(Mat im_contour,vector< vector<Point>>& contours)
{
    Mat im_contour_c = im_contour.clone();
    vector<cv::Vec4i> hier;
    findContours(im_contour_c,contours,hier,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
}


void Segment::Depth_canny(cv::Mat mask,cv::Mat Depth_im,cv::Mat &Depth_edge)
{
    cv::Mat edge_mask(mask.size(),CV_8UC1);
    cv::Mat edge(mask.size(),CV_8UC1);
    edge.setTo(0);
    cv::Mat dx(mask.size(),CV_32FC1);
    cv::Mat dy(mask.size(),CV_32FC1);
    cv::Mat mag(mask.size(),CV_32FC1);
    cv::Mat angle(mask.size(),CV_32FC1);
    edge_mask.setTo(0);
    dx.setTo(0);
    dy.setTo(0);
    mag.setTo(0);
    angle.setTo(0);
    for(int i = 0;i<mask.rows;i++)
    {
        uchar* optr = mask.ptr<uchar>(i);
        float* mptr = mag.ptr<float>(i);
        float* xptr = dx.ptr<float>(i);
        float* yptr = dy.ptr<float>(i);
        float* aptr = angle.ptr<float>(i);
        for(int j = 0;j<mask.cols;j++)
        {
            if(optr[j] != 0)
            {
                xptr[j] = ((-1.0)*Depth_im.at<float>(i-1,j-1)+Depth_im.at<float>(i-1,j+1)+(-2.0)*Depth_im.at<float>(i,j-1)+2.0*Depth_im.at<float>(i,j+1)+(-1.0)*Depth_im.at<float>(i+1,j-1)+Depth_im.at<float>(i+1,j+1))/4.0;
                yptr[j] = ((-1.0)*Depth_im.at<float>(i-1,j-1)+Depth_im.at<float>(i+1,j-1)+2.0*(Depth_im.at<float>(i+1,j)-Depth_im.at<float>(i-1,j))+Depth_im.at<float>(i+1,j+1)-Depth_im.at<float>(i-1,j+1))/4.0;
                mptr[j] = sqrt(xptr[j]*xptr[j]+yptr[j]*yptr[j]);
                if(xptr[j] == 0)
                {
                    if(yptr[j] > 0)
                    {
                        aptr[j]=90.0;
                    }
                    if(yptr[j] < 0)
                    {
                        aptr[j]=-90.0;
                    }
                }
                else if(yptr[j] == 0)
                {
                    aptr[j]= 0;
                }
                else
                {
                    aptr[j] = (float)((atan(yptr[j]/xptr[j]) * 180)/PI);
                }
            }
        }
    }
    for(int i = 0;i<mask.rows;i++)
    {
        float* mptr = mag.ptr<float>(i);
        uchar* eptr = edge_mask.ptr<uchar>(i);
        float* aptr = angle.ptr<float>(i);
        uchar* ptr = edge.ptr<uchar>(i);
        for(int j = 0;j<mask.cols;j++)
        {
            if(fabs(mptr[j])>0.8)
            {
                ptr[j] =255;
                if(fabs(aptr[j])>-22.5&&fabs(aptr[j])<22.5)
                {
                    if(mptr[j]>mptr[j-1]&&mptr[j]>mptr[j+1])
                    {
                        eptr[j]=255;
                    }
                }
                else if(fabs(aptr[j])>22.5&&fabs(aptr[j])<67.5)
                {
                    if(mptr[j]>mag.at<float>(i-1,j-1)&&mptr[j]>mag.at<float>(i+1,j+1))
                    {
                        eptr[j]=255;
                    }

                }
                else if(fabs(aptr[j])>-67.5&&fabs(aptr[j])<-22.5)
                {
                    if(mptr[j]>mag.at<float>(i-1,j+1)&&mptr[j]>mag.at<float>(i+1,j-1))
                    {
                        eptr[j]=255;
                    }
                }
                else if((fabs(aptr[j])<-67.5&&(fabs(aptr[j])>=-90)||(fabs(aptr[j])>67.5&&fabs(aptr[j])<=90)))
                {
                    if(mptr[j]>mag.at<float>(i-1,j)&&mptr[j]>mag.at<float>(i+1,j))
                    {
                        eptr[j]=255;
                    }
                }
            }
        }
    }
    //        imwrite("edge.png",edge);
    //        imshow("edge_mask",edge_mask);
    //                waitKey(0);
    Depth_edge = edge.clone();
}
void Segment::setSpixelLabel(vector<int> label,vector<SPixels> &superpixels,int flag)
{
    for(int i=0;i<label.size();i++)
    {
        for(int j=0;j<superpixels.size();j++)
        {
            if(label[i]==superpixels[j].label)
            {
                superpixels[j].true_label = flag;
            }
        }
    }
}

void Segment::getLabelBycontours(Mat roi,int* truelabels1,vector< vector<Point> > p_set,vector<int> &label )
{
    Mat stat_mask(roi.size(),CV_8UC1);
    stat_mask.setTo(0);
    drawContours(stat_mask,p_set,-1,cv::Scalar(255,255,255),CV_FILLED);
    for(int i=0;i<roi.rows;i++)
    {
        uchar* ptr = stat_mask.ptr<uchar>(i);
        for(int j=0;j<roi.cols;j++)
        {
            if(ptr[j])
            {
                int index =i*roi.cols + j;
                int la =  truelabels1[index];
                if(label.empty())
                {
                    label.push_back(la);
                }
                else
                {
                    bool IsTaken =false;
                    for(int l=0;l<label.size();l++)
                    {
                        if(la==label[l])
                        {
                            IsTaken = true;
                        }
                    }
                    if(!IsTaken)
                    {
                        label.push_back(la);
                    }
                }
            }
        }
    }
}

void Segment::getLabelBycontour(Mat roi,int* truelabels1,vector<Point> p_set,vector<int> &label )
{
    Mat stat_mask(roi.size(),CV_8UC1);
    stat_mask.setTo(0);
    vector< vector<Point> > stat_contour;
    stat_contour.push_back(p_set);
    drawContours(stat_mask,stat_contour,-1,cv::Scalar(255,255,255),CV_FILLED);
    for(int i=0;i<roi.rows;i++)
    {
        uchar* ptr = stat_mask.ptr<uchar>(i);
        for(int j=0;j<roi.cols;j++)
        {
            if(ptr[j])
            {
                int index =i*roi.cols + j;
                int la =  truelabels1[index];
                if(label.empty())
                {
                    label.push_back(la);
                }
                else
                {
                    bool IsTaken = false;
                    for(int l=0;l<label.size();l++)
                    {
                        if(la==label[l])
                        {
                            IsTaken = true;
                        }
                    }
                    if(!IsTaken)
                    {
                        label.push_back(la);
                    }
                }
            }
        }
    }
}

void Segment::getLabelBymask(Mat mask,int* truelabels1,vector<int> &label )
{
    for(int i=0;i<mask.rows;i++)
    {
        uchar* ptr = mask.ptr<uchar>(i);
        for(int j=0;j<mask.cols;j++)
        {
            if(ptr[j])
            {
                int index =i*mask.cols + j;
                int la =  truelabels1[index];
                if(label.empty())
                {
                    label.push_back(la);
                }
                else
                {
                    bool IsTaken = false;
                    for(int l=0;l<label.size();l++)
                    {
                        if(la==label[l])
                        {
                            IsTaken = true;
                        }
                    }
                    if(!IsTaken)
                    {
                        label.push_back(la);
                    }
                }
            }
        }
    }
}
void Segment::getSpByLabel(vector<int> label,vector<SPixels> superpixles, vector<SPixels> &sps )
{
    for(int i=0;i<label.size();i++)
    {
        for(int j=0;j<superpixles.size();j++)
        {
            if(superpixles[j].label==label[i])
            {
                sps.push_back(superpixles[j]);
            }
        }
    }
}
void Segment::get_foreground()
{
    foreground.setTo(0);
    nobg_depth.setTo(0);
    Depth_Gmm mog;
    Mat corse_result(origin_image.size(),CV_8UC3);
    corse_result = origin_image.clone();
    Mat depth(origin_image.size(),CV_32FC1);
    depth.setTo(0);
    Mat mi_fg(origin_image.size(),CV_8UC1);
    int i,j;
    for(i=0;i<origin_image.rows;i++)
    {
        float* ptr = depth.ptr<float>(i);
        for(j= 0;j<origin_image.cols;j++)
        {
            if(pcl::isFinite(cloud->at(j,i)))
            {
                ptr[j] = (float)cloud->at(j,i).z*100.0;
            }
        }
    }

    mog.run(depth,mi_fg,0.5,bgmodel);
    Mat fg(origin_image.size(),CV_8UC1);
    fg.setTo(0);
    for(i = box.y;i<origin_image.rows;i++)
    {
        uchar* optr = mi_fg.ptr<uchar>(i);
        uchar* ptr = fg.ptr<uchar>(i);
        for( j = box.x;j<origin_image.cols;j++)
        {
            if(optr[j])
            {
                ptr[j] = 255;
            }
        }
    }
    vector<vector<Point> > contours;
    findcontours(fg,contours);
    cv::drawContours(foreground,contours,-1,cv::Scalar(255,255,255),CV_FILLED);
    for(i = 0;i<origin_image.rows;i++)
    {
        float* nptr = nobg_depth.ptr<float>(i);
        Vec3b* ptr = corse_result.ptr<Vec3b>(i);
        uchar* optr = foreground.ptr<uchar>(i);
        for(j = 0;j<origin_image.cols;j++)
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
    //        imshow("foreground",foreground);
    //        imshow("origin_image",origin_image);
    //            waitKey(0);
}
void Segment::DoSlic(Mat roi,int* truelabels1,vector<int> &ccontourx1,vector<int> &ccontoury1)
{
    //slic
    //    cout<<"slic begin"<<endl;
    int width=roi.cols;
    int height=roi.rows;
    int sz = width*height;
    //    double slic_t1 = getTickCount();
    UINT *roi1=new UINT[sz*3+1];
    for(int c=0;c<3;c++)
    {
        for(int i=0;i<width;i++)
        {
            for(int j=0;j<height;j++)

                roi1[c*(width*height)+i*height+j]=saturate_cast<unsigned int>(roi.at<Vec3b>(j,i)[2-c]);
        }
    }
    int* labels1 = new int[sz];
    int spcounta=0;
    int spcount=sz/60;
    double compactness=30.0;
    vector<double> kkseedsx1(0);
    vector<double> kkseedsy1(0);
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
    vector<double> ().swap(kkseedsx1);
    vector<double> ().swap(kkseedsy1);
    delete [] roi1 ;
    roi1 =NULL;
    delete [] labels1 ;
    labels1 =NULL;

    //    double slic_t2 = getTickCount();
    //    cout<<"slic time :"<<(slic_t2-slic_t1)/getTickFrequency() <<" s"<<endl;
}
void Segment::buildSuperpixels(Mat roi,Mat roi_mask,Rect rec,int* truelabels1, vector<int> ccontourx1,vector<int> ccontoury1,vector<SPixels> &b_superpixels,vector<SPixels> &t_superpixels)
{
    for(int i = 0;i<roi_mask.rows;i++)
    {
        uchar *ptr = roi_mask.ptr<uchar>(i);
        for(int j = 0;j<roi_mask.cols;j++)
        {
            if(ptr[j]){
                int index = i*roi.cols + j;
                int label =  truelabels1[index];
                bool isTaken = false;
                for(int k = 0;k<t_superpixels.size();k++)
                {
                    if(t_superpixels[k].label == label)
                    {
                        t_superpixels[k].points.push_back(cv::Point(j,i));
                        isTaken = true;
                        break;
                    }
                }
                if(!isTaken)
                {
                    SPixels sp;
                    sp.label = label;
                    sp.points.push_back(cv::Point(j,i));
                    t_superpixels.push_back(sp);
                }
            }
        }
    }

    for(int i = 0;i<t_superpixels.size();i++)
    {
        t_superpixels[i].buildFeatures(cv::Point(rec.x,rec.y),roi,cloud);
        int threshold = 60*0.4;
        if(t_superpixels[i].points.size()>threshold)
        {
            SPixels sp;
            sp.label = t_superpixels[i].label;
            //            t_superpixels[i].CopyTo(sp);
            //            sp.buildFeatures(cv::Point(rec.x,rec.y),roi,cloud);
            b_superpixels.push_back(sp);
        }
    }
    int contournum = ccontoury1.size();
    Mat slicedge_show(roi.size(),CV_8UC3);
    slicedge_show = roi.clone();
    for(int i =0;i<contournum;i++)
    {
        int x = ccontourx1[i];
        int y = ccontoury1[i];
        int index = y*roi.cols + x;
        int label =  truelabels1[index];
        //        if(roi_mask.at<uchar>(y,x)){
        //            for(int k = 0;k<b_superpixels.size();k++)
        //            {
        //                if(b_superpixels[k].label == label)
        //                {
        //                    b_superpixels[k].edge_points.push_back(Point(x,y));
        //                }
        //            }
        //        }
        if(roi_mask.at<uchar>(y,x)){
            for(int k = 0;k<t_superpixels.size();k++)
            {
                if(t_superpixels[k].label == label)
                {
                    t_superpixels[k].edge_points.push_back(Point(x,y));
                }
            }
        }
        slicedge_show.at<Vec3b>(y,x) = red;
    }
    //        imshow("slicedge_show",slicedge_show);
    //        waitKey(0);
}

void Segment::getAdhereEdge(Mat roi_mask,Mat roi_edge,Mat &adhere_im)
{
    Mat o_contourIm(roi_edge.size(),CV_8UC1);
    o_contourIm.setTo(0);
    vector<vector<Point> > o_contour;
    findcontours(roi_mask,o_contour);
    int max_id = 0;
    int max_points = 0;
    for(int k=0;k<o_contour.size();k++)
    {
        if(max_points<o_contour[k].size())
        {
            max_points = o_contour[k].size();
            max_id = k;
        }
    }
    cv::drawContours(o_contourIm,o_contour,max_id,cv::Scalar(255,255,255),0);
    //    imwrite("o_contourIm1.png",o_contourIm);
    cv::dilate(o_contourIm,o_contourIm,element1);
    for(int i=0;i<roi_mask.rows;i++)
    {
        uchar *rptr = roi_edge.ptr<uchar>(i);
        uchar *optr = o_contourIm.ptr<uchar>(i);
        uchar *aptr = adhere_im.ptr<uchar>(i);
        for(int j=0;j<roi_mask.cols;j++)
        {
            if(rptr[j]==255&&optr[j]==0)
            {
                aptr[j]=255;
            }
        }
    }
    //    imwrite("o_contourIm2.png",o_contourIm);
    //    imwrite("adhere_im.png",adhere_im);
    //    waitKey(0);
    vector<vector<Point> > ().swap(o_contour);
}
int Segment::countNum(Mat mask)
{
    int num = 0;
    if(mask.channels()==1)
    {
        for(int i=0;i<mask.rows;i++)
        {
            uchar* ptr = mask.ptr<uchar>(i);
            for(int j=0;j<mask.cols;j++)
            {
                if(ptr[j])
                {
                    num++;
                }
            }
        }
    }
    return num;
}
void Segment::getStructElementSize(Mat roi_mask,vector<SPixels> b_superpixels,int* truelabels1,Mat &KedgeR,double &possible)
{
    for(int i=0;i<4;i++)
    {
        Mat mask_substract(KedgeR.size(),CV_8UC1);
        mask_substract.setTo(0);
        Mat superpixel_region(KedgeR.size(),CV_8UC1);
        superpixel_region.setTo(0);
        superpixel_region = KedgeR.clone();
        if(i>0)
        {
            int g_nStructElementSize = i;
            Mat element = getStructuringElement(MORPH_RECT,
                                                Size(2*g_nStructElementSize+1,2*g_nStructElementSize+1),Point( g_nStructElementSize, g_nStructElementSize));
            dilate(superpixel_region,superpixel_region,element);
            vector<int> labels;
            getLabelBymask(superpixel_region,truelabels1,labels);

            for(int l=0;l<labels.size();l++){
                for(int j=0;j<b_superpixels.size();j++)
                {
                    if(labels[l]==b_superpixels[j].label)
                    {
                        b_superpixels[j].DrawPointsOnshow(superpixel_region);
                    }
                }
            }
        }
        mask_substract = roi_mask - superpixel_region;
        vector< vector<Point> > s_cons;
        findcontours(mask_substract,s_cons);
        if(s_cons.size()>1)
        {
            int effective_num = 0;
            for(int j=0;j<s_cons.size();j++)
            {
                vector<int> clabel;
                vector<int> true_label;
                getLabelBycontour(KedgeR,truelabels1,s_cons[j],clabel);
                for(int c=0;c<clabel.size();c++)
                {
                    for(int k = 0;k<b_superpixels.size();k++)
                    {
                        if(clabel[c]==b_superpixels[k].label)
                        {
                            true_label.push_back(clabel[c]);
                        }
                    }
                }
                if(true_label.size()>7)
                {
                    effective_num++;
                }
            }
            if(effective_num>1)
            {
                int sps_num = countNum(superpixel_region);
                int roi_num = countNum(roi_mask);
                KedgeR = superpixel_region.clone();
                possible = std::pow((1 - (sps_num*1.0)/(roi_num*1.0)),i+1);
                break;
            }
            else
            {
                KedgeR = superpixel_region.clone();
                possible = 0;
            }
        }
        else
        {
            KedgeR = superpixel_region.clone();
            possible = 0;
        }
        vector< vector<Point> > ().swap(s_cons);
    }
}
void Segment::getEffectEdge(Mat adhere_im,Mat roi_mask,int *truelabels1,vector<SPixels> b_superpixels,vector<SPixels> t_superpixels,vector< vector<Point> > &effective_edgec)
{
    vector< vector<Point> > adhere_cues;
    findcontours(adhere_im,adhere_cues);
    if(!adhere_cues.empty())
    {
        vector<int> adhere_label;
        getLabelBycontours(adhere_im,truelabels1,adhere_cues,adhere_label);
        Mat adere_supers(roi_mask.size(),CV_8UC1);
        adere_supers.setTo(0);
        for(int l=0;l<adhere_label.size();l++)
        {
            for(int i=0;i<t_superpixels.size();i++)
            {
                if(t_superpixels[i].label==adhere_label[l])
                {
                    t_superpixels[i].DrawPointsOnshow(adere_supers);
                }
            }
        }
        vector< vector<Point> > edge_sps;
        findcontours(adere_supers,edge_sps);
        for(int i=0;i<edge_sps.size();i++)
        {
            Mat sps(roi_mask.size(),CV_8UC1);
            sps.setTo(0);
            drawContours(sps,edge_sps,i,cv::Scalar(255,255,255),-1);
            double possible ;
            Mat KedgeR = sps.clone();
            getStructElementSize(roi_mask,b_superpixels,truelabels1,KedgeR,possible);
            if(possible>0.4)
            {
                vector< vector<Point> > effect_edge_sps;
                findcontours(KedgeR,effect_edge_sps);
                effective_edgec.push_back(effect_edge_sps[0]);
            }
        }
    }
    vector< vector<Point> > ().swap(adhere_cues);
}
void Segment::getEffectEdge(Mat adhere_im,Mat roi_mask,Mat objects_c,int *truelabels1,vector<SPixels> b_superpixels,vector<SPixels> t_superpixels,vector< vector<Point> > &effective_edgec)
{
    vector< vector<Point> > adhere_cues;
    findcontours(adhere_im,adhere_cues);
    if(!adhere_cues.empty())
    {
        vector<int> adhere_label;
        getLabelBycontours(adhere_im,truelabels1,adhere_cues,adhere_label);
        Mat adere_supers(roi_mask.size(),CV_8UC1);
        adere_supers.setTo(0);
        for(int l=0;l<adhere_label.size();l++)
        {
            for(int i=0;i<t_superpixels.size();i++)
            {
                if(t_superpixels[i].label==adhere_label[l])
                {
                    t_superpixels[i].DrawPointsOnshow(adere_supers);
                }
            }
        }
        vector< vector<Point> > edge_sps;
        findcontours(adere_supers,edge_sps);
        for(int i=0;i<edge_sps.size();i++)
        {
            Mat sps(roi_mask.size(),CV_8UC1);
            sps.setTo(0);
            drawContours(sps,edge_sps,i,cv::Scalar(255,255,255),-1);
            double possible ;
            Mat KedgeR = sps.clone();
            getStructElementSize(roi_mask,b_superpixels,truelabels1,KedgeR,possible);
            if(possible>0.4)
            {
                vector< vector<Point> > effect_edge_sps;
                findcontours(KedgeR,effect_edge_sps);
                effective_edgec.push_back(effect_edge_sps[0]);
            }
        }
    }
    vector< vector<Point> > ().swap(adhere_cues);
}
int Segment::getObjects_com(int *truelabels1,Mat &objects_c,Mat m_mask,vector<SPixels> &b_superpixels)
{
    int num = 0;
    objects_c.setTo(0);
    erode(m_mask,m_mask,element1);
    vector< vector<Point> > objects_contours;
    findcontours(m_mask,objects_contours);
    vector<Mat> o_m;
    for(int t=0;t<objects_contours.size();t++)
    {
        Mat tobj_mask(m_mask.size(),CV_8UC1);
        tobj_mask.setTo(0);
        drawContours(tobj_mask,objects_contours,t,cv::Scalar(255,255,255),CV_FILLED);

        vector<int> tlabel;
        vector<int> true_label;
        getLabelBycontour(m_mask,truelabels1,objects_contours[t],tlabel);
        for(int c=0;c<tlabel.size();c++)
        {
            for(int k = 0;k<b_superpixels.size();k++)
            {
                if(tlabel[c]==b_superpixels[k].label)
                {
                    true_label.push_back(tlabel[c]);
                }
            }
        }
        if(true_label.size()>10)
        {
            num++;
            Mat obj(m_mask.size(),CV_8UC1);
            obj.setTo(0);
            drawContours(obj,objects_contours,t,cv::Scalar(255,255,255),CV_FILLED);
            o_m.push_back(obj);
        }
    }
    if(!o_m.empty())
    {
        for(int i=0;i<o_m.size();i++)
        {
            dilate(o_m[i],o_m[i],element1);
            for(int y=0;y<m_mask.rows;y++)
            {
                uchar *ptr = o_m[i].ptr<uchar>(y);
                uchar *ptro = objects_c.ptr<uchar>(y);
                for(int x=0;x<m_mask.cols;x++)
                {
                    if(ptr[x]==255&&ptro[x]==255) ptro[x]=0;
                    else if(ptr[x]==255&&ptro[x]==0) ptro[x]=255;
                }
            }
        }
    }
    return num;

}
void Segment::setSuperpixelsLabels(int *truelabels1,Mat roi_mask,vector< vector<Point> > &effective_edgec,vector<SPixels> &t_superpixels,vector<SPixels> &b_superpixels,vector<int> &labels)
{
    if(!effective_edgec.empty())
    {
        Mat effective_edge(roi_mask.size(),CV_8UC1);
        Mat baseObj_mask;
        Mat tobj_mask(roi_mask.size(),CV_8UC1);
        effective_edge.setTo(0);
        baseObj_mask.setTo(0);
        drawContours(effective_edge,effective_edgec,-1,cv::Scalar(255,255,255),CV_FILLED);
        baseObj_mask = roi_mask-effective_edge;
        erode(baseObj_mask,baseObj_mask,element1);
        vector< vector<Point> > tobjmodel_contours;
        findcontours(baseObj_mask,tobjmodel_contours);
        int obj_flag = 1;
        for(int t=0;t<tobjmodel_contours.size();t++)
        {
            tobj_mask.setTo(0);
            drawContours(tobj_mask,tobjmodel_contours,t,cv::Scalar(255,255,255),CV_FILLED);
            vector< vector<Point> > objcontour;
            findcontours(tobj_mask,objcontour);
            vector<int> tlabel;
            vector<int> true_label;
            getLabelBycontour(roi_mask,truelabels1,objcontour[0],tlabel);
            for(int c=0;c<tlabel.size();c++)
            {
                for(int k = 0;k<b_superpixels.size();k++)
                {
                    if(tlabel[c]==b_superpixels[k].label)
                    {
                        true_label.push_back(tlabel[c]);
                    }
                }
            }
            if(true_label.size()>7)
            {
                //                drawContours(save,tobjmodel_contours,t,cv::Scalar(colors[t][0],colors[t][1],colors[t][2]),CV_FILLED);
                //                imshow("effective_edge",effective_edge);
                //                imshow("tobj_mask",tobj_mask);
                //                waitKey(0);
                //                imshow("baseObj_mask",baseObj_mask);
                //                waitKey(0);
                labels.push_back(obj_flag);
                setSpixelLabel(tlabel,t_superpixels,obj_flag);
                obj_flag++;
            }
            else
            {
                effective_edgec.push_back(objcontour[0]);
            }
        }
        for(int i=0;i<effective_edgec.size();i++)
        {
            vector<int> tlabel;
            getLabelBycontour(roi_mask,truelabels1,effective_edgec[i],tlabel);
            setSpixelLabel(tlabel,t_superpixels,0);
        }
        vector< vector<Point> > ().swap(tobjmodel_contours);
    }
}
void Segment::processScattered(Mat isolate_mask,int *truelabels1,Mat &truelabel_mask)
{
    vector<vector<Point> > contour;
    findcontours(isolate_mask,contour);
    if(contour.size()>0)
    {
        vector<vector<Point> > contour;
        findcontours(isolate_mask,contour);
        for(int i=0;i<contour.size();i++)
        {
            Mat is_mask(isolate_mask.size(),CV_8UC1);
            is_mask.setTo(0);
            Mat dil_mask(isolate_mask.size(),CV_8UC1);
            dil_mask.setTo(0);
            Mat ad_mask(isolate_mask.size(),CV_8UC1);
            ad_mask.setTo(0);
            drawContours(is_mask,contour,i,cv::Scalar(255,255,255),CV_FILLED);
            vector<int> is_label;
            getLabelBymask(is_mask,truelabels1,is_label);
            dilate(is_mask,dil_mask,element2);
            ad_mask = dil_mask - is_mask;
            vector<int> true_label;
            vector<int> times;
            for(int t=0;t<ad_mask.rows;t++)
            {
                uchar* ptr = ad_mask.ptr<uchar>(t);
                uchar* tptr = truelabel_mask.ptr<uchar>(t);
                for(int d=0;d<ad_mask.cols;d++)
                {
                    if(ptr[d])
                    {
                        bool IsTaken = false;
                        for(int m=0;m<true_label.size();m++)
                        {
                            if(int(tptr[d])==true_label[m])
                            {
                                IsTaken = true;
                                times[m]++;
                            }
                        }
                        if(!IsTaken)
                        {
                            if(int(tptr[d])>0)
                            {
                                true_label.push_back(int(tptr[d]));
                                times.push_back(1);
                            }
                        }
                    }
                }
            }
            int max = 0;
            int max_id = 0;
            for(int t=0;t<times.size();t++)
            {
                if(times[t]>max)
                {
                    max = times[t];
                    max_id = t;
                }
            }
            for(int t=0;t<ad_mask.rows;t++)
            {
                uchar* ptr = is_mask.ptr<uchar>(t);
                uchar* tptr = truelabel_mask.ptr<uchar>(t);
                for(int d=0;d<ad_mask.cols;d++)
                {
                    if(ptr[d])
                    {
                        tptr[d] = true_label[max_id];
                    }
                }
            }
        }
    }
    //    vector<vector<Point> > ().swap(contour);
}
Mat Segment::refinement(const cv::Rect rec,pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,Mat roi,int *truelabels1,vector<SPixels> &t_superpixels)
{
    vector<SPixels *> uncategorized_edges;
    for(int i=0;i<t_superpixels.size();i++)
    {
        if( t_superpixels[i].true_label==0)
        {
            SPixels * p= &t_superpixels[i];
            uncategorized_edges.push_back(p);
        }
    }
    Mat ue_mask(roi.size(),CV_8UC1);
    ue_mask.setTo(0);
    for(int i=0;i<uncategorized_edges.size();i++)
    {
        uncategorized_edges[i]->DrawPointsOnshow(ue_mask);
    }
    vector< vector<Point> > edge_contours;
    findcontours(ue_mask,edge_contours);

    Mat label_mask(roi.size(),CV_8UC1);
    label_mask.setTo(0);
    for(int c=0;c<edge_contours.size();c++)
    {
        Mat tue_mask(roi.size(),CV_8UC1);
        tue_mask.setTo(0);
        drawContours(tue_mask,edge_contours,c,cv::Scalar(255,255,255),CV_FILLED);
        vector<SPixels> uncategorized_edge;
        vector<int> uca_label;
        getLabelBymask(tue_mask,truelabels1,uca_label);
        for(int i=0;i<uca_label.size();i++)
        {
            for(int j=0;j<t_superpixels.size();j++)
            {
                if(t_superpixels[j].label==uca_label[i])
                {
                    uncategorized_edge.push_back(t_superpixels[j]);
                }
            }
        }
        vector<int>().swap(uca_label);
        Mat uue_mask(roi.size(),CV_8UC1);
        uue_mask.setTo(0);
        //get area_superpixels
        vector<SPixels> area_superpixels;
        vector< vector<SPixels> > t_model_superpixels;
        vector< vector<SPixels> > model_superpixels;
        vector<int> tlabel;
        Mat area_mask(roi.size(),CV_8UC1);
        area_mask.setTo(0);
        dilate(tue_mask,uue_mask,element2);
        area_mask = uue_mask - tue_mask;
        vector<int> area_label;
        getLabelBymask(area_mask,truelabels1,area_label);
        for(int i=0;i<area_label.size();i++)
        {
            for(int j=0;j<t_superpixels.size();j++)
            {
                if(t_superpixels[j].label==area_label[i]&&t_superpixels[j].label>0)
                {
                    area_superpixels.push_back(t_superpixels[j]);
                }
            }
        }
        for(int i=0;i<area_superpixels.size();i++)
        {
            bool isTaken =false;
            for(int j=0;j<tlabel.size();j++)
            {
                if(area_superpixels[i].true_label==tlabel[j]&&area_superpixels[i].true_label!=0&&area_superpixels[i].true_label!=-1)
                {
                    isTaken = true;
                    for(int m=0;m<t_model_superpixels.size();m++)
                    {
                        if(t_model_superpixels[m][0].true_label==tlabel[j])
                        {
                            area_superpixels[i].buildFeatures(Point(rec.x,rec.y),origin_image,_cloud);
                            t_model_superpixels[m].push_back(area_superpixels[i]);
                        }
                    }
                }
            }
            if(!isTaken)
            {
                if(area_superpixels[i].true_label!=0&&area_superpixels[i].true_label!=-1)
                {
                    vector<SPixels> vsp;
                    vsp.push_back(area_superpixels[i]);
                    t_model_superpixels.push_back(vsp);
                    tlabel.push_back(area_superpixels[i].true_label);
                }
            }
        }
        vector<SPixels>().swap(area_superpixels);
        vector<int>().swap(tlabel);
        for(int i=0;i<t_model_superpixels.size();i++)
        {
            Mat t_model(roi.size(),CV_8UC1);
            t_model.setTo(0);
            Mat model(roi.size(),CV_8UC1);
            model.setTo(0);
            for(int j=0;j<t_model_superpixels[i].size();j++)
            {
                t_model_superpixels[i][j].DrawPointsOnshow(t_model);
            }
            vector<vector<Point> > contour;
            findcontours(t_model,contour);
            int max_contour = 0;
            int max_id = 0;
            for(int c=0;c<contour.size();c++)
            {
                if(max_contour<contour[c].size())
                {
                    max_contour = contour[c].size();
                    max_id = c;
                }
            }
            drawContours(model,contour,max_id,cv::Scalar(255,255,255),CV_FILLED);
            vector<int> model_label;
            getLabelBymask(model,truelabels1,model_label);
            vector<SPixels> tsp;
            for(int j=0;j<t_model_superpixels[i].size();j++)
            {
                for(int m=0;m<model_label.size();m++)
                {
                    if(t_model_superpixels[i][j].label == model_label[m])
                    {
                        tsp.push_back(t_model_superpixels[i][j]);
                    }
                }
            }
            model_superpixels.push_back(tsp);
        }
        for(int i=0;i<tue_mask.rows;i++)
        {
            uchar* ptr = tue_mask.ptr<uchar>(i);
            uchar* lptr = label_mask.ptr<uchar>(i);
            for(int j=0;j<tue_mask.cols;j++)
            {
                if(ptr[j])
                {
                    Point p(rec.x+j,rec.y+i);
                    Vec3b color = origin_image.at<Vec3b>(p.y,p.x);
                    SPixels s;
                    s.r = color[2];
                    s.g = color[1];
                    s.b = color[0];
                    s.middle_point = Point(j,i);
                    s.RGB2LAB();
                    if(pcl::isFinite(cloud->at(p.x,p.y)))
                    {
                        s.depth =  (float)cloud->at(p.x,p.y).z;
                    }
                    else
                    {
                        s.depth = 0;
                    }
                    double min_dis = 1000000;
                    int id = -1;
                    int tl = -1;
                    for(int t=0;t<model_superpixels.size();t++)
                    {
                        for(int m=0;m<model_superpixels[t].size();m++)
                        {
                            double dis = model_superpixels[t][m].CaculateSimLAB(s);
                            if(min_dis>dis)
                            {
                                min_dis = dis;
                                id = t;
                                tl = m;
                            }
                        }
                    }
                    lptr[j] = model_superpixels[id][tl].true_label;
                }
            }
        }
    }
    return label_mask;
}
vector<Object> Segment::processSingle(vector<int> labels,Mat roi_mask,Rect rec,Mat roi,Mat label_mask,int *truelabels1,vector<SPixels> &t_superpixels)
{
     vector<Object> result_objs;
    Mat truelabel_mask(roi.size(),CV_8UC1);
    truelabel_mask.setTo(0);
    Mat isolate_mask(roi.size(),CV_8UC1);
    isolate_mask.setTo(0);
    Mat assigned_mask(origin_image.size(),CV_8UC1);
    assigned_mask.setTo(0);
    for(int i=0;i<label_mask.rows;i++)
    {
        uchar* ptr = label_mask.ptr<uchar>(i);
        uchar* uptr = truelabel_mask.ptr<uchar>(i);
        uchar* aptr = assigned_mask.ptr<uchar>(rec.y+i);
        for(int j=0;j<label_mask.cols;j++)
        {
            if(ptr[j])
            {
                aptr[rec.x+j] =255;
                uptr[j] = ptr[j];
            }
        }
    }
    for(int i =0;i<t_superpixels.size();i++)
    {
        for(int j=0;j<t_superpixels[i].points.size();j++)
        {
            if(t_superpixels[i].true_label>0)
            {
                truelabel_mask.at<uchar>(t_superpixels[i].points[j].y,t_superpixels[i].points[j].x) = t_superpixels[i].true_label;
                t_superpixels[i].DrawOnOriginimg(assigned_mask);
            }
        }
    }
    Mat roi_assigned_mask = assigned_mask(rec);
    isolate_mask = roi_mask - roi_assigned_mask;
    for(int l=0;l<labels.size();l++)
    {
        Mat r_mask(roi.size(),CV_8UC1);
        r_mask.setTo(0);
        for(int i=0;i<truelabel_mask.rows;i++)
        {
            uchar* ptr = truelabel_mask.ptr<uchar>(i);
            uchar* uptr = r_mask.ptr<uchar>(i);
            for(int j=0;j<truelabel_mask.cols;j++)
            {
                if(ptr[j]== labels[l])
                {
                    uptr[j] = 255;
                }
            }
        }
        vector<vector<Point> > contour;
        findcontours(r_mask,contour);
        int max_contour = 0;
        int max_id = 0;
        for(int c=0;c<contour.size();c++)
        {
            Mat cp(r_mask.size(),CV_8UC1);
            cp.setTo(0);
            drawContours(cp,contour,c,cv::Scalar(255,255,255),CV_FILLED);
            if(max_contour<CountPoint(cp))
            {
                max_contour = CountPoint(cp);
                max_id = c;
            }
        }
        for(int c=0;c<contour.size();c++)
        {
            if(c!=max_id)
            {
                drawContours(isolate_mask,contour,c,cv::Scalar(255,255,255),CV_FILLED);
            }
        }
    }
    Mat check_objs = roi_mask - isolate_mask;
    vector<vector<Point> > check_contour;
    findcontours(check_objs,check_contour);
    if(!check_contour.empty())
    {
        processScattered(isolate_mask,truelabels1,truelabel_mask);
        for(int l=0;l<labels.size();l++)
        {
            Mat r_mask(origin_image.size(),CV_8UC1);
            r_mask.setTo(0);
            for(int i=0;i<truelabel_mask.rows;i++)
            {
                uchar* ptr = truelabel_mask.ptr<uchar>(i);
                uchar* uptr = r_mask.ptr<uchar>(i+rec.y);
                for(int j=0;j<truelabel_mask.cols;j++)
                {
                    if(ptr[j]== labels[l])
                    {
                        uptr[j+rec.x] = 255;
                    }
                }
            }
            Object obj;
            obj.mask = r_mask.clone();
            result_objs.push_back(obj);
        }
    }
    else
    {
        Mat r_mask(origin_image.size(),CV_8UC1);
        r_mask.setTo(0);
        for(int i=0;i<roi_mask.rows;i++)
        {
            uchar* ptr = roi_mask.ptr<uchar>(i);
            uchar* uptr = r_mask.ptr<uchar>(i+rec.y);
            for(int j=0;j<roi_mask.cols;j++)
            {
                if(ptr[j]==255)
                {
                    uptr[j+rec.x] = 255;
                }
            }
        }
        Object obj;
        obj.mask = r_mask.clone();
        result_objs.push_back(obj);
    }
    return result_objs;
}
int Segment::CountPoint(Mat input)
{
    int num=0;
    for(int i=0;i<input.rows;i++)
    {
        uchar* ptr = input.ptr<uchar>(i);
        for(int j=0;j<input.cols;j++)
        {
            if(ptr[j]) num++;
        }
    }
    return num;
}
 void*  Segment::thread_process(void * _this)
{

     Segment * this_ = (  Segment *) _this;
    pthread_mutex_lock(&this_->mutex);
    cv::Size ori_size = this_->origin_image.size();
    Mat ori_image =  this_->origin_image.clone();
    Mat ori_depth = this_->nobg_depth.clone();
    vector<Point> pv = this_->q_contours.front();
    this_->q_contours.pop();
    pthread_mutex_unlock(&this_->mutex);
    vector< vector<Point> > s_contours;
    s_contours.push_back(pv);

    Mat region_mask(ori_size,CV_8UC1);
    region_mask.setTo(0);
    Mat extract_im(ori_size,CV_8UC3);
    extract_im.setTo(0);
    cv::drawContours(region_mask,s_contours,0,cv::Scalar(255,255,255),CV_FILLED);
    Rect rec = getrect(pv);
    Rect reca(rec.x-5,rec.y-5,rec.width+10,rec.height+10);
    if(rec.width*rec.height<=2400&&rec.width*rec.height>600)
    {
        Object obj;
        obj.mask = region_mask.clone();
        obj.contour.push_back(pv);
        pthread_mutex_lock(&this_->mutex);
        this_->objs.push_back(obj);
        pthread_mutex_unlock(&this_->mutex);
    }
    else if(rec.width*rec.height>2400)
    {
        Mat Dedge(ori_size,CV_8UC1);
        Dedge.setTo(0);
        this_->Depth_canny(region_mask,ori_depth,Dedge);
        for(int i = rec.y -1 ;i<rec.y+rec.height;i++)
        {
            Vec3b* ptr1 = ori_image.ptr<Vec3b>(i);
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
        Mat roi = extract_im(reca);
        Mat roi_mask = region_mask(reca);
        Mat roi_edge = Dedge(reca);
        Mat adhere_im(roi.size(),CV_8UC1);
        adhere_im.setTo(0);
        this_->getAdhereEdge(roi_mask,roi_edge,adhere_im);
        int* truelabels1 = new int[roi.cols*roi.rows];
        vector<int> ccontourx1;
        vector<int> ccontoury1;
        vector<SPixels> b_superpixels;    //big
        vector<SPixels> t_superpixels;    //all
        this_->DoSlic(roi,truelabels1,ccontourx1,ccontoury1);
        this_->buildSuperpixels(roi,roi_mask,reca,truelabels1,ccontourx1,ccontoury1,b_superpixels,t_superpixels);
        vector< vector<Point> > effective_edgec;
        this_->getEffectEdge(adhere_im,roi_mask,truelabels1,b_superpixels,t_superpixels,effective_edgec);
        if(effective_edgec.size()>0)
        {
            int label_num = 0;
            Mat m_mask;
            Mat am_regions(roi.size(),CV_8UC1);
            am_regions.setTo(0);
            drawContours(am_regions,effective_edgec,-1,Scalar(255,255,255),-1);
            m_mask = roi_mask - am_regions;

            Mat obj_masks;
            obj_masks.create(roi.size(),CV_8UC1);
            label_num= this_->getObjects_com(truelabels1,obj_masks,m_mask,b_superpixels);
            if(label_num<2) {
                Mat r_mask(ori_size,CV_8UC1);
                r_mask.setTo(0);
                for(int i=0;i<roi_mask.rows;i++)
                {
                    uchar* ptr = roi_mask.ptr<uchar>(i);
                    uchar* uptr = r_mask.ptr<uchar>(i+reca.y);
                    for(int j=0;j<roi_mask.cols;j++)
                    {
                        if(ptr[j]==255)
                        {
                            uptr[j+reca.x] = 255;
                        }
                    }
                }
                Object obj;
                obj.mask = r_mask.clone();
                this_->objs.push_back(obj);
            }
            else
            {
                am_regions =  roi_mask - obj_masks;
                dilate(am_regions,am_regions,element2);
                obj_masks = roi_mask- am_regions;

                Mat roi_depth = this_->depth_image(reca);
                Mat roi_4u(roi.size(),CV_8UC4);
                roi_4u.setTo(0);
                for(int i=0;i<roi.rows;i++)
                {
                    Vec4b* pi = roi_4u.ptr<Vec4b>(i);
                    Vec3b* pu = roi.ptr<Vec3b>(i);
                    uchar* pd = roi_depth.ptr<uchar>(i);
                    for(int j=0;j<roi.cols;j++)
                    {
                        pi[j][0] = pu[j][0];
                        pi[j][1] =  pu[j][1];
                        pi[j][2] =  pu[j][2];
                        pi[j][3] = pd[j];
                    }
                }
                CrfSegment cs(roi_4u,obj_masks,am_regions,label_num+1,6, 10, 2, 20, 33, 3, 41);
                int sum_label = 0;
                Mat crf_result=cs.run(sum_label);
                if(sum_label==1)
                {
                    Mat r_mask(ori_size,CV_8UC1);
                    r_mask.setTo(0);
                    for(int i=0;i<roi_mask.rows;i++)
                    {
                        uchar* ptr = roi_mask.ptr<uchar>(i);
                        uchar* uptr = r_mask.ptr<uchar>(i+reca.y);
                        for(int j=0;j<roi_mask.cols;j++)
                        {
                            if(ptr[j]==255)
                            {
                                uptr[j+reca.x] = 255;
                            }
                        }
                    }
                    pthread_mutex_lock(&this_->mutex);
                    Object obj;
                    obj.mask = r_mask.clone();
                    this_->objs.push_back(obj);
                    pthread_mutex_unlock(&this_->mutex);
                }
                else
                {
                    vector<int> labels;
                    for (int l=1;l<=label_num;l++)
                    {
                        labels.push_back(l);
                    }
                    vector<Object> r_objs;
                    r_objs = this_->processSingle(labels,roi_mask,reca,roi,crf_result,truelabels1,t_superpixels);
                    pthread_mutex_lock(&this_->mutex);
                    for(int i=0;i<r_objs.size();i++)
                        this_->objs.push_back(r_objs[i]);
                    pthread_mutex_unlock(&this_->mutex);
                }
            }
        }
        else
        {
            Object obj;
            obj.mask = region_mask.clone();
            obj.contour.push_back(pv);
            pthread_mutex_lock(&this_->mutex);
            this_->objs.push_back(obj);
            pthread_mutex_unlock(&this_->mutex);
        }
        delete [] truelabels1;
        truelabels1=NULL;
        vector<SPixels>().swap(b_superpixels);
        vector<SPixels>().swap(t_superpixels);
    }
    return NULL;
}
void Segment::runSegemnt(){
    get_foreground();
    findcontours(foreground,contours);
    int i;
     for (i = 0; i < contours.size(); i++)
     {
         q_contours.push(contours[i]);
     }
    pthread_t* thread_id = new pthread_t[contours.size()];
    pthread_mutex_init(&mutex, NULL); //创建互斥量
    for (i = 0; i < contours.size(); i++)
        pthread_create(&thread_id[i], NULL, thread_process, (void*)this);
    for (i = 0; i < contours.size(); i++)
        pthread_join(thread_id[i], NULL);

    Vec3b* colors = new Vec3b[objs.size()+1];
    for(int i=0;i<objs.size();i++)
    {
        colors[i][0] = rng.uniform(0,255);
        colors[i][1] = rng.uniform(0,255);
        colors[i][2] = rng.uniform(0,255);
    }
    for(int s=0;s<objs.size();s++)
    {
        Mat color_image(origin_image.size(),CV_8UC3);
        color_image.setTo(colors[s]);
        color_image.copyTo(result,objs[s].mask);
    }

    pthread_mutex_destroy(&mutex);
    delete [] thread_id;
    thread_id = NULL;
    delete []  colors;
    colors=NULL;
}
