#ifndef CRFSEGMENT_H
#define CRFSEGMENT_H
#include"DenseCRF.h"
#include"CmGMM.h"
typedef Vec<float, 4> Color_Depth;
static void Mfindcontours(Mat im_contour,vector< vector<Point>>& contours)
{
    Mat im_contour_c = im_contour.clone();
    vector<cv::Vec4i> hier;
    findContours(im_contour_c,contours,hier,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
}
class CrfSegment
{
public:
    CrfSegment(Mat &img4u,Mat objmasks,Mat am_r,int Nlabels,float w1, float w2, float w3, float alpha, float beta, float gama, float mu);

    void test();
    void run();
    Mat result;
    //private:
    int _h;
    int _w;
    int labels;
    Mat img_4u;
    Mat img_4f;
    float *_unary; // Unary energies for segmentation
    Mat obj_masks;
    Mat labelled_masks;
    Mat ambigous_regions;
    Mat *obj_models;
    MGMM *GMM_models;
    int N_labels;
    DenseCRFMulL _crf;

    void build_UnaryEnergy();
    void buildGMM_Models();

};
#endif // CRFSEGMENT_H
