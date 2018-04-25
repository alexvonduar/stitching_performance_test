#pragma once

#include <vector>
#include <cstdint>

#include <opencv2/opencv.hpp>

#include "arm_compute/core/Types.h"
#include "arm_compute/runtime/Tensor.h"
#include "arm_compute/runtime/Pyramid.h"

#include "fastcv.h"

#include "pislam/Orb.h"
#include "pislam/Fast.h"
#include "pislam/Util.h"

static const int MAX_LEVEL = 8;

static const int pyramidWidths[MAX_LEVEL] = {
    640,
    533,
    444,
    370,
    309,
    257,
    214,
    179
};

static const int MAX_WIDTH = 640;

static const int pyramidHeights[MAX_LEVEL] = {
    480,
    400,
    333,
    278,
    231,
    193,
    161,
    134
};

static const int MAX_HEIGHT = 480 + 400 + 333 + 278 + 231 + 193 + 161 + 134;
static const int ALIGNMENT = 128;
static const int ALLOCATE_SIZE = MAX_WIDTH * MAX_HEIGHT + ALIGNMENT;

class FAST_NEON_ORB_640x480_8
{
    public:
    FAST_NEON_ORB_640x480_8() {
        std::uintptr_t i = (std::uintptr_t)_in;
        i %= ALIGNMENT;
        img = _in + (ALIGNMENT - i);
        i = (std::uintptr_t)_out;
        i %= ALIGNMENT;
        out = _out + (ALIGNMENT - i);
    };
    ~FAST_NEON_ORB_640x480_8() {};

    uint8_t * img;
    uint8_t * out;

    private:
    uint8_t _in[ALLOCATE_SIZE];
    uint8_t _out[ALLOCATE_SIZE];
};

static inline int fast_orb_640x480_downscale(FAST_NEON_ORB_640x480_8& input, int level)
{
    //
    level = level > MAX_LEVEL ? MAX_LEVEL : level;
    cv::Mat tmp(pyramidHeights[0], MAX_WIDTH, CV_8UC1);
    uint8_t * src = input.img;
    for (int i = 0; i < (level - 1); ++i) {
        //
        //std::cout << "filter " << i << " src " << (std::uintptr_t)src << pyramidWidths[i] << pyramidHeights[i] << (std::uintptr_t)tmp.ptr<uint8_t>() << std::endl;
        fcvFilterGaussian5x5u8_v2(src, pyramidWidths[i], pyramidHeights[i], MAX_WIDTH,
                                  tmp.ptr<uint8_t>(), MAX_WIDTH, 1);
        src += pyramidHeights[i] * MAX_WIDTH;
        uint8_t * dst = src;
        fcvScaleDownMNu8(tmp.ptr<uint8_t>(), pyramidWidths[i], pyramidHeights[i], MAX_WIDTH,
                        dst, pyramidWidths[i + 1], pyramidHeights[i + 1], MAX_WIDTH);
        //std::cout << "scale " << i << std::endl;
    }
}

static inline int fast_orb_640x480_save(FAST_NEON_ORB_640x480_8& input, std::string name)
{
    cv::Mat tmp(MAX_HEIGHT, MAX_WIDTH, CV_8UC1, input.img);
    cv::imwrite(name, tmp);
}

template<int vstep>
static inline void paintPoint(uint8_t img[][vstep], int x, int y) {
  img[y-5][x] = 0;
  img[y-4][x] = 0;
  img[y+4][x] = 0;
  img[y+5][x] = 0;

  img[y][x-5] = 0;
  img[y][x-4] = 0;
  img[y][x+4] = 0;
  img[y][x+5] = 0;
}

static inline int fast_orb_640x480(FAST_NEON_ORB_640x480_8& input, int level, std::vector<cv::KeyPoint>& keypoints, std::vector<uint32_t>& descriptors) {
    std::vector<uint32_t> points;
    //std::vector<uint32_t> descriptors;

    uint32_t pyramidRow = 0;
    uint8_t * img_ptr = input.img;
    uint8_t * out_ptr = input.out;
    for (size_t i = 0; i < level; i++) {
        uint32_t levelWidth = pyramidWidths[i];
        uint32_t levelHeight = pyramidHeights[i];
        uint8_t (*imgPtr)[MAX_WIDTH] = (uint8_t (*)[MAX_WIDTH])img_ptr;
        uint8_t (*outPtr)[MAX_WIDTH] = (uint8_t (*)[MAX_WIDTH])out_ptr;

        pislam::fastDetect<MAX_WIDTH, 16>(levelWidth, levelHeight, imgPtr, outPtr, 10);
        pislam::fastScoreHarris<MAX_WIDTH, 16>(levelWidth, levelHeight, imgPtr, 1 << 12, outPtr);

        size_t oldSize = points.size();
        pislam::fastExtract<MAX_WIDTH, 16>(levelWidth, levelHeight, outPtr, points);

        // Adjust y coordinate to match position in image pyramid.
        //if (i > 0) {
        float yscale = (float)pyramidHeights[0] / pyramidHeights[i];
        float xscale = (float)pyramidWidths[0] / pyramidWidths[i];
        //}
        for (auto p = points.begin() + oldSize; p < points.end(); ++ p) {
            uint32_t x = pislam::decodeFastX(*p);
            uint32_t y = pislam::decodeFastY(*p);
            keypoints.push_back(cv::KeyPoint(x * xscale, y * yscale, 16));
            y += pyramidRow;
            uint32_t score = pislam::decodeFastScore(*p);
            *p = pislam::encodeFast(score, x, y);
        }

        pyramidRow += levelHeight;
        img_ptr += levelHeight * MAX_WIDTH;
        out_ptr += levelHeight * MAX_WIDTH;
    }
    pislam::orbCompute<MAX_WIDTH, 8>((uint8_t (*)[MAX_WIDTH])input.img, points, descriptors);
    std::cout << points.size() << " points" << std::endl;

#if 0
    uint8_t (*img)[MAX_WIDTH] = (uint8_t (*)[MAX_WIDTH])input.img;
    for (uint32_t point: points) {
        uint32_t x = pislam::decodeFastX(point);
        uint32_t y = pislam::decodeFastY(point);
        paintPoint<MAX_WIDTH>(img, x, y);
    }
#endif
}
