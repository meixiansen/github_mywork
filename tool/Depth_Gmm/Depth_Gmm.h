#ifndef DEPTH_GMM_H
#define DEPTH_GMM_H
#include<string.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include<opencv/cv.h>
static const int defaultNMixtures = 5;
static const int defaultHistory = 200;
static const double defaultBackgroundRatio = 0.7;
static const double defaultVarThreshold = 100;
//static const double defaultVarThreshold = 1;
static const double defaultNoiseSigma = 0.1;
static const double defaultInitialWeight = 0.05;

using namespace cv;

template<typename VT> struct MixData
{
    float sortKey;
    float weight;
    VT mean;
    VT var;
};
class Depth_Gmm{
public:
    Depth_Gmm()
    {
        frameSize = Size(0,0);
        frameType = 0;

        nframes = 0;
        nmixtures = defaultNMixtures;
        history = defaultHistory;
        varThreshold = defaultVarThreshold;
        backgroundRatio = defaultBackgroundRatio;
        noiseSigma = defaultNoiseSigma;

    }
    void init(Rect _box,Size _frameSize, int _frameType);
    void processdepth(const Mat& image, Mat& fgmask, double learningRate,
                 Mat& bgmodel, int nmixtures, double backgroundRatio,
                 double varThreshold, double noiseSigma);
    void run(Rect _box,Mat _image, OutputArray _fgmask, double learningRate);

    void run(Rect _box,Mat _image, OutputArray _fgmask,Mat _bgmodel);
    void saveBgmodel(Rect _box,Mat _image, OutputArray _fgmask,double learningRate,int num,String save_name);
protected:
    Size frameSize;
    int frameType;
    cv::Rect box;
    cv::Mat bgmodel;
    int nframes;
    int history;
    int nmixtures;
    double varThreshold;
    double backgroundRatio;
    double noiseSigma;
};

#endif // DEPTH_GMM_H
