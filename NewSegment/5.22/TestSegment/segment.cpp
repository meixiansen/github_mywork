#include"segment.h"

Segment::Segment()
{
    cloud =  boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >();

}
Segment::Segment(Mat img,pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
{
    origin_image = img.clone();
    result.create(origin_image.size(),CV_8UC3);
    result.setTo(0);
    foreground.create(origin_image.size(),CV_8UC1);
    foreground.setTo(0);
    nobg_depth.create(origin_image.size(),CV_32FC1);
    nobg_depth.setTo(0);
    cloud = _cloud;
}
void Segment::findcontours(Mat im_contour,vector< vector<Point>>& contours)
{
    Mat im_contour_c = im_contour.clone();
    vector<cv::Vec4i> hier;
    findContours(im_contour_c,contours,hier,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
}

void Segment::readBgModel(String filename,Mat bgmodel)
{
    ifstream infile;
    infile.open(filename);
    MixData<float>* mptr = (MixData<float>*)bgmodel.data;
    if(infile.is_open())
    {
        for(int y = 0; y < origin_image.rows; y++ )
        {
            for( int x = 0; x< origin_image.cols; x++, mptr+=5 )
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
            if(fabs(mptr[j])>0.68)
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
//        imshow("edge",edge);
    //    imshow("edge_mask",edge_mask);
//        waitKey(0);
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
    //    imshow("stat_mask",stat_mask);
    //    waitKey(0);
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
void Segment::getfg()
{
    foreground.setTo(0);
    nobg_depth.setTo(0);
    Depth_Gmm mog;
    Mat bgmodel;
    bgmodel.create( 1, origin_image.rows*origin_image.cols*5*(2 + 2*3), CV_32F );
    bgmodel = Scalar::all(0);
    readBgModel("../../bgmodel-5.31.txt",bgmodel);
    Mat corse_result(origin_image.size(),CV_8UC3);
    corse_result = origin_image.clone();
    Mat depth(origin_image.size(),CV_32FC1);
    depth.setTo(0);
    Mat mi_fg(origin_image.size(),CV_8UC1);
    for(int i = 0;i<origin_image.rows;i++)
    {
        float* ptr = depth.ptr<float>(i);
        for(int j = 0;j<origin_image.cols;j++)
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
    for(int i = box.y;i<origin_image.rows;i++)
    {
        uchar* optr = mi_fg.ptr<uchar>(i);
        uchar* ptr = fg.ptr<uchar>(i);
        for(int j = box.x;j<origin_image.cols;j++)
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
    for(int i = 0;i<origin_image.rows;i++)
    {
        float* nptr = nobg_depth.ptr<float>(i);
        Vec3b* ptr = corse_result.ptr<Vec3b>(i);
        uchar* optr = foreground.ptr<uchar>(i);
        for(int j = 0;j<origin_image.cols;j++)
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
//    imshow("foreground",foreground);
//    imshow("origin_image",origin_image);
    //    waitKey(0);
}
void Segment::DoSlic(Mat roi,int* truelabels1,vector<int> &ccontourx1,vector<int> &ccontoury1)
{
    //slic
    cout<<"slic begin"<<endl;
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
    double slic_t2 = getTickCount();
    cout<<"slic time :"<<(slic_t2-slic_t1)/getTickFrequency() <<" s"<<endl;
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
    //    imshow("slicedge_show",slicedge_show);
    //    waitKey(0);
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
}
void Segment::getEffectEdge(Mat adhere_im,Mat roi_mask,int *truelabels1,vector<SPixels> b_superpixels,vector<SPixels> t_superpixels,vector< vector<Point> > &effective_edgec)
{
    vector< vector<Point> > adhere_cues;
    findcontours(adhere_im,adhere_cues);
    dilate(adhere_im,adhere_im,element2);
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
        //        imshow("adere_supers",adere_supers);
//                imwrite("adere_supers.png",adere_supers);
        //        waitKey(0);
        vector< vector<Point> > edge_sps;
        vector<int> edge_label;
        findcontours(adere_supers,edge_sps);
        for(int i=0;i<edge_sps.size();i++)
        {
            Mat sps(roi_mask.size(),CV_8UC1);
            Mat mask_substract(roi_mask.size(),CV_8UC1);
            sps.setTo(0);
            mask_substract.setTo(0);
            drawContours(sps,edge_sps,i,cv::Scalar(255,255,255),-1);
            mask_substract = roi_mask - sps;
//            erode(mask_substract,mask_substract,element1);
            vector< vector<Point> > substract_cons;
            findcontours(mask_substract,substract_cons);
            if(substract_cons.size()==1)
            {
                edge_label.push_back(0);
            }
            else
            {
                int effective_num = 0;
//                imshow("mask_substract",mask_substract);
//                waitKey(0);
                for(int j=0;j<substract_cons.size();j++)
                {
                    vector<int> clabel;
                    vector<int> true_label;
                    getLabelBycontour(roi_mask,truelabels1,substract_cons[j],clabel);
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
                    if(true_label.size()>4)
                    {
                        effective_num++;
                    }
                }
                if(effective_num>1)
                {
                    edge_label.push_back(1);
                }
                else
                {
                    edge_label.push_back(0);
                }
            }
//            imshow("mask_substract",mask_substract);
//            waitKey(0);
        }
        for(int i=0;i<edge_sps.size();i++)
        {
            if(edge_label[i]==1)
            {
                effective_edgec.push_back(edge_sps[i]);
            }
        }
    }
}

void Segment::setSuperpixelsLabels(int *truelabels1,Mat roi_mask,vector< vector<Point> > &effective_edgec,vector<SPixels> &t_superpixels,vector<int> &labels)
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
            dilate(tobj_mask,tobj_mask,element1);
            //            imshow("tobj_mask",tobj_mask);
            //             waitKey(0);
            vector< vector<Point> > objcontour;
            findcontours(tobj_mask,objcontour);
            vector<int> tlabel;
            getLabelBycontour(roi_mask,truelabels1,objcontour[0],tlabel);
//            imshow("effective_edge",effective_edge);
//            imshow("tobj_mask",tobj_mask);
//            waitKey(0);
            if(tlabel.size()>7)
            {

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
        //        imshow("effective_edge",effective_edge);
        //        imshow("baseObj_mask",baseObj_mask);
        //        waitKey(0);
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
            dilate(is_mask,dil_mask,element3);
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
}
void Segment::refinement(const cv::Rect rec,Mat roi_mask ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,vector<int> labels,Mat roi,int *truelabels1,vector<SPixels> &t_superpixels)
{
    vector<SPixels> uncategorized_edges;
    for(int i=0;i<t_superpixels.size();i++)
    {
        if( t_superpixels[i].true_label==0)
        {
            uncategorized_edges.push_back(t_superpixels[i]);
        }
    }
    Mat ue_mask(roi.size(),CV_8UC1);
    ue_mask.setTo(0);
    for(int i=0;i<uncategorized_edges.size();i++)
    {
        uncategorized_edges[i].DrawPointsOnshow(ue_mask);
    }
    //    imshow("ue_mask",ue_mask);
    //    waitKey(0);
    vector< vector<Point> > edge_contours;
    findcontours(ue_mask,edge_contours);
    Mat truelabel_mask(roi.size(),CV_8UC1);
    truelabel_mask.setTo(0);
    Mat label_mask(roi.size(),CV_8UC1);
    label_mask.setTo(0);
    for(int c=0;c<edge_contours.size();c++)
    {
        Mat tue_mask(roi.size(),CV_8UC1);
        tue_mask.setTo(0);
        drawContours(tue_mask,edge_contours,c,cv::Scalar(255,255,255),CV_FILLED);
        //        imshow("tue_mask",tue_mask);
        //        waitKey(0);
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
        Mat uue_mask(roi.size(),CV_8UC1);
        uue_mask.setTo(0);
        //get area_superpixels
        vector<SPixels> area_superpixels;
        vector< vector<SPixels> > t_model_superpixels;
        vector< vector<SPixels> > model_superpixels;
        vector<int> tlabel;
        Mat due_mask(roi.size(),CV_8UC1);
        due_mask.setTo(0);
        Mat area_mask(roi.size(),CV_8UC1);
        area_mask.setTo(0);
        dilate(tue_mask,uue_mask,element1);
        dilate(tue_mask,due_mask,element2);
        area_mask = due_mask - uue_mask;
        vector<int> area_label;
        getLabelBymask(area_mask,truelabels1,area_label);
        for(int i=0;i<area_label.size();i++)
        {
            for(int j=0;j<t_superpixels.size();j++)
            {
                if(t_superpixels[j].label==area_label[i])
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
//        for(int i=0;i<model_superpixels.size();i++)
//        {
//            Mat save(roi.size(),CV_8UC1);
//            save.setTo(0);
//            for(int j=0;j<model_superpixels[i].size();j++)
//            {
//                model_superpixels[i][j].DrawPointsOnshow(save);
//                model_superpixels[i][j].DrawEdgeOnshow(save);
//            }
//            }
//            imwrite("model3.png",save);
//            waitKey(0);
//        }
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
    for(int i =0;i<t_superpixels.size();i++)
    {
            if(t_superpixels[i].true_label==-1)
            {
                 t_superpixels[i].DrawOnOriginimg(isolate_mask);
            }
    }
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
        vector<vector<Point> > contour;
        findcontours(r_mask,contour);
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
        for(int c=0;c<contour.size();c++)
        {
            if(c!=max_id)
            {
             drawContours(assigned_mask,contour,c,cv::Scalar(0,0,0),CV_FILLED);
            }
        }
    }
    Mat roi_assigned_mask = assigned_mask(rec);
    isolate_mask = roi_mask - roi_assigned_mask;
//    imshow("isolate_mask",isolate_mask);
//    waitKey(0);
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
        objs.push_back(obj);
    }
//    imwrite("sp_result.png",p_mask);
//    waitKey(0);
}

void Segment::runSegemnt(){
    getfg();
    vector<vector<Point> > contours;
    findcontours(foreground,contours);
    for(int i = 0 ;i < contours.size() ; i++)
    {
        Mat region_mask(origin_image.size(),CV_8UC1);
        region_mask.setTo(0);
        Mat extract_im(origin_image.size(),CV_8UC3);
        extract_im.setTo(0);
        cv::drawContours(region_mask,contours,i,cv::Scalar(255,255,255),CV_FILLED);
        Rect rec = getrect(contours[i]);
        Rect reca(rec.x-5,rec.y-5,rec.width+10,rec.height+10);
        if(rec.width*rec.height<2400)
        {
            Object obj;
            obj.mask = region_mask.clone();
            obj.contour.push_back(contours[i]);
            objs.push_back(obj);
        }
        else
        {
            Mat Dedge(origin_image.size(),CV_8UC1);
            Dedge.setTo(0);
            Depth_canny(region_mask,nobg_depth,Dedge);
            for(int i = rec.y -1 ;i<rec.y+rec.height;i++)
            {
                Vec3b* ptr1 = origin_image.ptr<Vec3b>(i);
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

//            imshow("roi",roi);
//            imshow("roi_mask",roi_mask);
//            imshow("roi_edge",roi_edge);

//            imwrite("roi.png",roi);
//            imwrite("roi_mask.png",roi_mask);
//            imwrite("roi_edge.png",roi_edge);
//            waitKey(0);
            Mat adhere_im(roi.size(),CV_8UC1);
            adhere_im.setTo(0);
            getAdhereEdge(roi_mask,roi_edge,adhere_im);

//            imshow("adhere_im",adhere_im);
            //                        waitKey(0);
            int* truelabels1 = new int[roi.cols*roi.rows];
            vector<int> ccontourx1;
            vector<int> ccontoury1;
            vector<SPixels> b_superpixels;    //big
            vector<SPixels> t_superpixels;    //all

            DoSlic(roi,truelabels1,ccontourx1,ccontoury1);
            buildSuperpixels(roi,roi_mask,reca,truelabels1,ccontourx1,ccontoury1,b_superpixels,t_superpixels);
//            Mat save = roi.clone();
//            for(int i=0;i<ccontourx1.size();i++)
//            {
//                save.at<Vec3b>(ccontoury1[i],ccontourx1[i]) = Vec3b(0,0,255);
//            }
//            imwrite("slic.png",save);
//            waitKey(0);
            vector< vector<Point> > effective_edgec;
            getEffectEdge(adhere_im,roi_mask,truelabels1,b_superpixels,t_superpixels,effective_edgec);
            if(effective_edgec.size()>0)
            {
                vector<int> labels;
                setSuperpixelsLabels(truelabels1,roi_mask,effective_edgec,t_superpixels,labels);
                //                refinement(Point(reca.x,reca.y),cloud,labels,roi,truelabels1,t_superpixels);
                refinement(reca,roi_mask,cloud,labels,roi,truelabels1,t_superpixels);
            }
            else
            {
                Object obj;
                obj.mask = region_mask.clone();
                obj.contour.push_back(contours[i]);
                objs.push_back(obj);
            }

            //            Mat show(roi.size(),CV_8UC3);
            //            show.setTo(0);
            //            int size = labels.size();
            //            vector<Vec3b> colors;
            //            for(int i=0;i<size;i++)
            //            {
            //                Vec3b c;
            //                c[0] = rng.uniform(0,255);
            //                c[1] = rng.uniform(0,255);
            //                c[2] = rng.uniform(0,255);
            //                colors.push_back(c);
            //            }
            //            for(int i=0;i<t_superpixels.size();i++)
            //            {
            //                if( t_superpixels[i].true_label==0)
            //                {
            //                    t_superpixels[i].DrawPointsOnshow(show);
            //                }
            //                for(int l=0;l<labels.size();l++)
            //                {
            //                    if( t_superpixels[i].true_label==labels[l])
            //                    {
            //                        t_superpixels[i].DrawPointsOnshow(show,colors[l]);
            //                    }
            //                }
            //            }
            //            imshow("show",show);
            //            waitKey(0);
        }
    }

    vector<Vec3b> colors;
    for(int i=0;i<objs.size();i++)
    {
        Vec3b c;
        c[0] = rng.uniform(0,255);
        c[1] = rng.uniform(0,255);
        c[2] = rng.uniform(0,255);
        colors.push_back(c);
    }
    for(int s=0;s<objs.size();s++)
    {
        Mat color_image(origin_image.size(),CV_8UC3);
        color_image.setTo(colors[s]);
        color_image.copyTo(result,objs[s].mask);
        //        for(int i=0;i<objs[s].mask.rows;i++)
        //        {
        //            uchar *ptr = objs[s].mask.ptr<uchar>(i);
        //            Vec3b *optr = origin_image.ptr<Vec3b>(i);
        //            Vec3b *rptr = result.ptr<Vec3b>(i);
        //            for(int j=0;j<objs[s].mask.cols;j++)
        //            {
        //                if(ptr[j])
        //                {
        //                    rptr[j] = optr[j];
        //                }
        //            }
        //        }
        //        imshow("result",result);
        //        waitKey(0);
        //           imshow("mask",objs[s].mask);
        //           waitKey(0);
    }
}
