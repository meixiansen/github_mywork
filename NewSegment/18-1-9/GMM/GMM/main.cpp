
#include"CmGMM.h"
void Mfindcontours(Mat im_contour,vector< vector<Point>>& contours)
{
    Mat im_contour_c = im_contour.clone();
    vector<cv::Vec4i> hier;
    findContours(im_contour_c,contours,hier,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
}
int main()
{
    Mat roi_3u = imread("roi.png");
     Mat depth_1u = imread("depth.png");
     cvtColor(depth_1u,depth_1u,CV_BGR2GRAY);
     Rect rec(165,113,175 ,182);
      Mat roidepth_1u = depth_1u(rec);
    Mat roi_4u(roi_3u.size(),CV_8UC4);
    roi_4u.setTo(0);
    for(int i=0;i<roi_3u.rows;i++)
    {
        Vec4b* pi = roi_4u.ptr<Vec4b>(i);
         Vec3b* pu = roi_3u.ptr<Vec3b>(i);
         uchar* pd = roidepth_1u.ptr<uchar>(i);
        for(int j=0;j<roi_3u.cols;j++)
        {
            pi[j][0] = pu[j][0];
            pi[j][1] =  pu[j][1];
             pi[j][2] =  pu[j][2];
            pi[j][3] = pd[j];
        }
    }
    Mat roi_4f;
    roi_4u.convertTo(roi_4f,CV_32FC4,1/255.0);
//    imshow("input",input);
//    imshow("roi_u",roi_u);
//    imshow("roi_depth",roi_depth);
//    waitKey(0);
//    for(int i=0;i<roi_u.rows;i++)
//    {
//        Vec4b* pi = input.ptr<Vec4b>(i);
//         Vec3b* pu = roi_u.ptr<Vec3b>(i);
//         uchar* pd = roi_depth.ptr<uchar>(i);
//        for(int j=0;j<roi_u.cols;j++)
//        {
//           cout<<int(pi[j][0])<<" "<<int(pi[j][1]) <<" "<< int(pi[j][2]) <<" "<<int(pi[j][3]) <<endl;
//        }
//    }

    Mat m_mask = imread("objs.png");
    cvtColor(m_mask,m_mask,CV_RGB2GRAY);
    vector< vector<Point> > contours;
    Mfindcontours(m_mask,contours);
    int num = contours.size();
    MGMM *GMM_models = new MGMM[num] ;
    Mat *obj_model =  new Mat[num];
    Mat *masks = new Mat[num];
    int i=0;
    while(i<num)
    {
        Mat obj_mask(roi_3u.size(),CV_8UC1);
        obj_mask.setTo(0);
        drawContours(obj_mask,contours,i,Scalar(255,255,255),-1);
        masks[i] = Mat::zeros(roi_3u.size(), CV_32F);
        masks[i].setTo(1,obj_mask);
        GMM_models[i].BuildGMMs(roi_4f,obj_model[i], masks[i]);
         GMM_models[i].RefineGMMs(roi_4f,obj_model[i], masks[i]);
        imshow("model_mask",obj_mask);
        waitKey(0);
         i++;
    }
    return 0;
}
