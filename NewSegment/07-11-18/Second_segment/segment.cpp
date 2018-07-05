/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#include <cstdio>
#include <cstdlib>
#include <image.h>
#include <misc.h>
#include <pnmfile.h>
#include "segment-image.h"
#include<opencv2/opencv.hpp>

#include<iostream>
using namespace std;
using namespace cv;
int main() {
    //  if (argc != 6) {
    //    fprintf(stderr, "usage: %s sigma k min input(ppm) output(ppm)\n", argv[0]);
    //    return 1;
    //  }

    float sigma  = 0.5;
    float k = 400;
    int min_size = 300;
    int num = 1;
    char png_name[100];
    char depth_name[100];
    char save_name[100];
    //  printf("loading input image.\n");
    char c_name[200]="/home/wangchao/paper/data/3-9/2017-5-15-7-52";
    double sum = 0;
    while(num<69)
    {
        sprintf(png_name,"%s%s%d%s",c_name,"/pic/",num,".png");
        sprintf(depth_name,"%s%s%d%s",c_name,"/depth/",num,".png");
        sprintf(save_name,"%s%s%d%s",c_name,"/rgbd_result/",num,".png");
//        sprintf(png_name,"%s%s%d%s",c_name,"/nobg_pic/",num,".png");
//        sprintf(depth_name,"%s%s%d%s",c_name,"/nobg_depth/",num,".png");
//        sprintf(save_name,"%s%s%d%s",c_name,"/nobg_rgbd_result/",num,".png");
        image<rgbd> *input = loadPNG_DEPTH(png_name,depth_name);
//        image<uchar> *input = loadDEPTH(depth_name);
//        image<rgb> *input = loadPNG(png_name);
//        printf("processing\n");
        int num_ccs;
        double t1 = getTickCount();
        image<rgb> *seg = segment_image(input, sigma, k, min_size, &num_ccs);
        savePNG(seg,save_name);

//        printf("got %d components\n", num_ccs);
//        printf("done! uff...thats hard work.\n");
        double t2 = getTickCount();
        sum+=(t2-t1)/getTickFrequency();
//        cout<<"segment time :"<<(t2-t1)/getTickFrequency() <<" s"<<endl;
        num++;
    }
cout<<"segment sum time :"<<sum <<" s"<<endl;
    return 0;
}

