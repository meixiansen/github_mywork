
#include"CrfSegment.h"
#include"math.h"
CrfSegment::CrfSegment(Mat &img4u, Mat objmasks,Mat am_r,int Nlabels,float w1, float w2, float w3, float alpha, float beta, float gama, float mu): _w(objmasks.cols), _h(objmasks.rows),N_labels(Nlabels),_crf(_w, _h, N_labels)
{
    CV_Assert(img4u.type() == CV_8UC4);
    img_4u = img4u;
    obj_masks = objmasks;
    ambigous_regions = am_r;
    GMM_models = new MGMM[Nlabels-1] ;
    obj_models =  new Mat[Nlabels-1];
    img_4u.convertTo(img_4f,CV_32FC4,1/255.0);
    labelled_masks.create(_h, _w, CV_8SC1);
    labelled_masks.setTo(-1,ambigous_regions);
    result.create(_h, _w, CV_8UC1);
    _unary = new float[N_labels*_w*_h];
    if (w1 != 0)
        _crf.addPairwiseBilateral(alpha, alpha, beta, beta, beta,beta, img_4u.data, w1);
    if (w2 != 0)
        _crf.addPairwiseGaussian(gama, gama, w2);
    if (w3 != 0)
        _crf.addPairwiseColorGaussian(mu, mu, mu, mu,img_4u.data, w3);
}
void CrfSegment::test()
{

    int i,j;
    for(i=0;i<labelled_masks.rows;i++)
    {
        schar *ptr = labelled_masks.ptr<schar>(i);
        for(j=0;j<labelled_masks.cols;j++)
        {
            //            if(int(ptr[j])==4) cout<<int(ptr[j])<<endl;
        }
    }
}
void CrfSegment::buildGMM_Models()
{
    cvtColor(obj_masks,obj_masks,CV_RGB2GRAY);
    vector< vector<Point> > contours;
    Mfindcontours(obj_masks,contours);
    int objs_sum = contours.size();
    Mat *masks = new Mat[objs_sum];
    int i=0;
    while(i<objs_sum)
    {
        Mat obj_mask(obj_masks.size(),CV_8UC1);
        obj_mask.setTo(0);
        drawContours(obj_mask,contours,i,Scalar(255,255,255),-1);
        //        imshow("obj_mask",obj_mask);
        //        waitKey(0);
        drawContours(labelled_masks,contours,i,Scalar(i+1,i+1,i+1),-1);
        masks[i] = Mat::zeros(obj_masks.size(), CV_32F);
        masks[i].setTo(1,obj_mask);
        GMM_models[i].BuildGMMs(img_4f,obj_models[i], masks[i]);
        GMM_models[i].RefineGMMs(img_4f,obj_models[i], masks[i]);
        //        cout<<GMM_models[i].K()<<endl;
        i++;
    }
}
void CrfSegment::build_UnaryEnergy()
{
#pragma omp parallel for
    int index;
    Mat show(result.size(),CV_8UC1);
    show.setTo(0);
    for (int y = 0; y < _h; y++){
        schar* triV = labelled_masks.ptr<schar>(y);
        Vec4f* img = img_4f.ptr<Vec4f>(y);
          uchar* ptr = show.ptr<uchar >(y);
        for (int x = 0; x < _w; x++){
            index = (y*_w+x)*N_labels;
            float prb; // User Back
            if(triV[x]==-1)
            {
                _unary[index] = 1;
                float sum = 0;
                for(int i=1;i<N_labels;i++ )
                {
                    sum+=(GMM_models[i-1].P(img[x]));
                }
                for(int i=1;i<N_labels;i++ )
                {
                    prb = (GMM_models[i-1].P(img[x]))/sum;
                    _unary[index+i] =  0.5*(1- prb);
                }
            }
            else
            {
                for(int i=0;i<N_labels;i++ )
                {
                    if(triV[x] == i) _unary[index+i] = 0;
                    else _unary[index+i] =1;
                }
            }
            if(triV[x]==1)
            {
                ptr[x]=255;
            }
        }
    }

//    for (int y = 0; y < _h; y++){
//        schar* triV = labelled_masks.ptr<schar >(y);
//        uchar* ptr = show.ptr<uchar >(y);
//        for (int x = 0; x < _w; x++){
//            //                if(triV[x]==-1)
//            //                {
//            //                    for(int l=0;l<N_labels;l++)
//            //                    {
//            //                        cout<<(_w*y+x)*N_labels+l<<":"<<float(_unary[(_w*y+x)*N_labels+l])<<" ";
//            //                    }
//            //                    cout<<endl;
//            //                }
//            if(triV[x]==0)
//            {
//                ptr[x]=255;
//            }
//        }
//    }
    imshow("show",show);
    waitKey(0);
}
void CrfSegment::run()
{
    buildGMM_Models();
    build_UnaryEnergy();
    _crf.setUnaryEnergy(_unary);
    float* prob=new float[_w*_h*N_labels] ;
    int iter = 0;
    _crf.inference(iter, prob,1.f);
    // for(int i=0; i<_w * _h * N_labels ; i++)
    // {
    //     cout<<float(prob[i])<<" ";
    // }
    Mat show(result.size(),CV_8UC1);
    show.setTo(0);
    for(int y=0; y<_h; y++)
    {
        uchar* triV = show.ptr<uchar >(y);
        for(int x=0; x<_w; x++)
        {
            float max = 0;
            int j_max = 0;
            for(int j=0;j<N_labels;j++ )
            {
                if(std::isnan(prob[(y*_w+x)*N_labels+j])) break;
                else
                {
                    triV[x] =255;
                    if(max<prob[(y*_w+x)*N_labels+j])
                    {
                        max = prob[(y*_w+x)*N_labels+j];
                        j_max = j;
                    }
                }
            }
            triV[x] = j_max;
        }
    }
    Mat test(result.size(),CV_8UC3);
    test.setTo(0);
    for(int y=0; y<_h; y++)
    {
        uchar* triV = show.ptr<uchar >(y);
        uchar* ptra = ambigous_regions.ptr<uchar >(y);
        Vec3b* ptr = test.ptr<Vec3b >(y);
        for(int x=0; x<_w; x++)
        {
            //            if(ptra[x]==255){
            ptr[x][0]=triV[x]*70;
            ptr[x][1]=triV[x]*80;
            ptr[x][2]=triV[x]*90;}
        //           if(triV[x]==3) ptr[x][0]=255;
        //        }
    }
    imshow("img_4u",img_4u);
    imshow("test",test);
    waitKey(0);
}
