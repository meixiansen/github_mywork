#include"DenseCRF.h"

#include"CrfSegment.h"

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
    Mat am_regions =imread("effective_edge.png",cv::IMREAD_GRAYSCALE);
    Mat m_mask = imread("objs.png");
//    imshow("roi_4u",roi_4u);
//     imshow("roidepth_1u",roidepth_1u);
//    imshow("am_regions",am_regions);
//    imshow("m_mask",m_mask);
//    waitKey(0);
    CrfSegment cs(roi_4u,m_mask,am_regions,4,6, 10, 2, 20, 33, 3, 41);
    cs.run();
    cout<<"back to main"<<endl;

//    int j,t;
//    for(j=0;j<roi_4u.rows;j++)
//    {
//        Vec4b *ptr = roi_4u.ptr<Vec4b>(j);
////        uchar *ptru = am_regions.ptr<uchar>(j);
//        for(t=0;t<roi_4u.cols;t++)
//        {
//            if(ptr[t][0]!=0&&ptr[t][1]!=0&&ptr[t][2]!=0)
//            {
//                cout<<int(ptr[t][3])<<endl;
//            }
//        }
//    }
    return 0;
}

