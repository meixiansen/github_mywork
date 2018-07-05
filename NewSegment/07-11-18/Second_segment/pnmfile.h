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

/* basic image I/O */

#ifndef PNM_FILE_H
#define PNM_FILE_H
#include <cstdlib>
#include <climits>
#include <cstring>
#include <fstream>
#include "image.h"
#include "misc.h"
#include<opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O

#include<iostream>
#include<string>
#define BUF_SIZE 256

class pnm_error { };

static void read_packed(unsigned char *data, int size, std::ifstream &f) {
    unsigned char c = 0;

    int bitshift = -1;
    for (int pos = 0; pos < size; pos++) {
        if (bitshift == -1) {
            c = f.get();
            bitshift = 7;
        }
        data[pos] = (c >> bitshift) & 1;
        bitshift--;
    }
}

static void write_packed(unsigned char *data, int size, std::ofstream &f) {
    unsigned char c = 0;

    int bitshift = 7;
    for (int pos = 0; pos < size; pos++) {
        c = c | (data[pos] << bitshift);
        bitshift--;
        if ((bitshift == -1) || (pos == size-1)) {
            f.put(c);
            bitshift = 7;
            c = 0;
        }
    }
}

/* read PNM field, skipping comments */ 
static void pnm_read(std::ifstream &file, char *buf) {
    char doc[BUF_SIZE];
    char c;

    file >> c;
    while (c == '#') {
        file.getline(doc, BUF_SIZE);
        file >> c;
    }
    file.putback(c);

    file.width(BUF_SIZE);
    file >> buf;
    file.ignore();
}

static image<uchar> *loadPBM(const char *name) {
    char buf[BUF_SIZE];

    /* read header */
    std::ifstream file(name, std::ios::in | std::ios::binary);
    pnm_read(file, buf);
    if (strncmp(buf, "P4", 2))
        throw pnm_error();
    
    pnm_read(file, buf);
    int width = atoi(buf);
    pnm_read(file, buf);
    int height = atoi(buf);

    /* read data */
    image<uchar> *im = new image<uchar>(width, height);
    for (int i = 0; i < height; i++)
        read_packed(imPtr(im, 0, i), width, file);

    return im;
}

static void savePBM(image<uchar> *im, const char *name) {
    int width = im->width();
    int height = im->height();
    std::ofstream file(name, std::ios::out | std::ios::binary);

    file << "P4\n" << width << " " << height << "\n";
    for (int i = 0; i < height; i++)
        write_packed(imPtr(im, 0, i), width, file);
}

static image<uchar> *loadPGM(const char *name) {
    char buf[BUF_SIZE];

    /* read header */
    std::ifstream file(name, std::ios::in | std::ios::binary);
    pnm_read(file, buf);
    if (strncmp(buf, "P5", 2))
        throw pnm_error();

    pnm_read(file, buf);
    int width = atoi(buf);
    pnm_read(file, buf);
    int height = atoi(buf);

    pnm_read(file, buf);
    if (atoi(buf) > UCHAR_MAX)
        throw pnm_error();

    /* read data */
    image<uchar> *im = new image<uchar>(width, height);
    file.read((char *)imPtr(im, 0, 0), width * height * sizeof(uchar));

    return im;
}

static void savePGM(image<uchar> *im, const char *name) {
    int width = im->width();
    int height = im->height();
    std::ofstream file(name, std::ios::out | std::ios::binary);

    file << "P5\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
    file.write((char *)imPtr(im, 0, 0), width * height * sizeof(uchar));
}
static image<rgb> *loadPNG(const char* name) {
    cv::Mat img = cv::imread(name,CV_LOAD_IMAGE_COLOR);
    cv::imshow("show",img);
    image<rgb> *im = new image<rgb>(img.cols, img.rows);
    for (int y = 0; y < img.rows; y++) {
        cv::Vec3b *ptr = img.ptr<cv::Vec3b>(y);
        for (int x = 0; x < img.cols; x++) {
            imRef(im, x, y).r =int( ptr[x][2]);
            imRef(im, x, y).g = int(ptr[x][1]);
            imRef(im, x, y).b = int(ptr[x][0]);
        }
    }
    return im;
}

static image<rgbd> *loadPNG_DEPTH(const char* name1,const char* name2) {
    cv::Mat img = cv::imread(name1,CV_LOAD_IMAGE_COLOR);
    cv::Mat depth = cv::imread(name2,CV_LOAD_IMAGE_GRAYSCALE);

    image<rgbd> *im = new image<rgbd>(img.cols, img.rows);
    for (int y = 0; y < img.rows; y++) {
        cv::Vec3b *ptr = img.ptr<cv::Vec3b>(y);
       uchar *dptr = depth.ptr<uchar>(y);
        for (int x = 0; x < img.cols; x++) {
            imRef(im, x, y).r =int( ptr[x][2]);
            imRef(im, x, y).g = int(ptr[x][1]);
            imRef(im, x, y).b = int(ptr[x][0]);
            imRef(im, x, y).d = int(dptr[x]);
        }
    }
    return im;
}

static image<rgb> *loadPPM(const char *name) {
    char buf[BUF_SIZE], doc[BUF_SIZE];

    /* read header */
    std::ifstream file(name, std::ios::in | std::ios::binary);
    pnm_read(file, buf);
    if (strncmp(buf, "P6", 2))
        throw pnm_error();

    pnm_read(file, buf);
    int width = atoi(buf);
    pnm_read(file, buf);
    int height = atoi(buf);

    pnm_read(file, buf);
    if (atoi(buf) > UCHAR_MAX)
        throw pnm_error();

    /* read data */
    image<rgb> *im = new image<rgb>(width, height);
    file.read((char *)imPtr(im, 0, 0), width * height * sizeof(rgb));
    //    for (int y = 0; y < height; y++) {
    //        for (int x = 0; x < width; x++) {
    //            std::cout<<int(imRef(im, x, y).r)<<" ";
    //            std::cout<< int(imRef(im, x, y).g)<<" ";
    //            std::cout<<int(imRef(im, x, y).b)<<std::endl;
    //        }
    //    }
    return im;
}
static image<uchar> *loadDEPTH(const char* name) {
    cv::Mat depth = cv::imread(name,CV_LOAD_IMAGE_GRAYSCALE);
    image<uchar> *im = new image<uchar>(depth.cols, depth.rows);
    for (int y = 0; y < depth.rows; y++) {
       uchar *dptr = depth.ptr<uchar>(y);
        for (int x = 0; x < depth.cols; x++) {
            imRef(im, x, y) = dptr[x];
        }
    }
    return im;
}
static void savePPM(image<rgb> *im, const char *name) {
    int width = im->width();
    int height = im->height();
    std::ofstream file(name, std::ios::out | std::ios::binary);

    file << "P6\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
    file.write((char *)imPtr(im, 0, 0), width * height * sizeof(rgb));
}
static void savePNG(image<rgb> *im, const char *name) {
    int width = im->width();
    int height = im->height();
    cv::Mat save(cv::Size(width,height),CV_8UC3);
    save.setTo(0);
    for (int y = 0; y < height; y++) {
        cv::Vec3b *ptr = save.ptr<cv::Vec3b>(y);
        for (int x = 0; x < width; x++) {
            ptr[x][2]=int(imRef(im, x, y).r);
            ptr[x][1]=int(imRef(im, x, y).g);
            ptr[x][0]=int(imRef(im, x, y).b) ;

        }
    }
//    std::cout<<"over"<<std::endl;
    cv::imwrite(name,save);
}

static void showPNG(image<rgb> *im) {
    int width = im->width();
    int height = im->height();
    cv::Mat save(cv::Size(width,height),CV_8UC3);
    save.setTo(0);
    for (int y = 0; y < height; y++) {
        cv::Vec3b *ptr = save.ptr<cv::Vec3b>(y);
        for (int x = 0; x < width; x++) {
            ptr[x][2]=int(imRef(im, x, y).r);
            ptr[x][1]=int(imRef(im, x, y).g);
            ptr[x][0]=int(imRef(im, x, y).b) ;

        }
    }
    std::cout<<"over"<<std::endl;
    cv::imshow("result",save);
    cv::waitKey(0);
}
template <class T>
void load_image(image<T> **im, const char *name) {
    char buf[BUF_SIZE];

    /* read header */
    std::ifstream file(name, std::ios::in | std::ios::binary);
    pnm_read(file, buf);
    if (strncmp(buf, "VLIB", 9))
        throw pnm_error();

    pnm_read(file, buf);
    int width = atoi(buf);
    pnm_read(file, buf);
    int height = atoi(buf);

    /* read data */
    *im = new image<T>(width, height);
    file.read((char *)imPtr((*im), 0, 0), width * height * sizeof(T));
}

template <class T>
void save_image(image<T> *im, const char *name) {
    int width = im->width();
    int height = im->height();
    std::ofstream file(name, std::ios::out | std::ios::binary);

    file << "VLIB\n" << width << " " << height << "\n";
    file.write((char *)imPtr(im, 0, 0), width * height * sizeof(T));
}

#endif
