#include <vector>
#include <time.h>

#include <opencv2/opencv.hpp>

//#include "arm_compute/runtime/NEON/NEFunctions.h"
#include "arm_compute/runtime/NEON/NEScheduler.h"
#include "arm_compute/runtime/NEON/functions/NEGaussianPyramid.h"

#include "arm_compute/core/Types.h"
#include "utils/Utils.h"

#include "arm_compute_showinfo.hpp"

#include "neonorb.hpp"

template <typename T>
static inline T get_msecs(struct timespec &start, struct timespec &stop)
{
    long long seconds = stop.tv_sec - start.tv_sec;
    long long nseconds = stop.tv_nsec - start.tv_nsec;
    nseconds += (seconds * 1000000000);
    double mseconds = (double)nseconds / 1000000;
    return (T)(mseconds);
}

static const int W = 640;
static const int H = 480;

#define USE_FASE_ORB 1

int main(int argc, char *argv[])
{
    //cv::TickMeter tm;
    struct timespec s;
    struct timespec t;
    cv::Mat descriptor1;
    cv::Mat descriptor2;
    cv::Ptr<cv::FlannBasedMatcher> flann = cv::FlannBasedMatcher::create();
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    std::vector<cv::DMatch> matches;
    cv::Mat matches_draw;
    //cv::Mat img1;
    //cv::Mat img2;

    clock_gettime(CLOCK_MONOTONIC, &s);
    //tm.start();
    cv::Mat src_img1 = cv::imread(argv[1]);
    cv::Mat src_img2 = cv::imread(argv[2]);
    clock_gettime(CLOCK_MONOTONIC, &t);
    //tm.stop();
    std::cout << "take " << get_msecs<double>(s, t) << " ms to read image" << std::endl;

    FAST_NEON_ORB_640x480_8 data1;
    FAST_NEON_ORB_640x480_8 data2;

    cv::Mat img1(pyramidHeights[0], MAX_WIDTH, CV_8UC1, data1.img);
    cv::Mat img2(pyramidHeights[0], MAX_WIDTH, CV_8UC1, data2.img);

    clock_gettime(CLOCK_MONOTONIC, &s);

    if (src_img1.channels() == 3)
    {
        cv::resize(src_img1, src_img1, cv::Size(W, H));
        cv::cvtColor(src_img1, img1, cv::COLOR_BGR2GRAY);
    }
    else if (src_img1.channels() == 4)
    {
        cv::resize(src_img1, src_img1, cv::Size(W, H));
        cv::cvtColor(src_img1, img1, cv::COLOR_BGRA2GRAY);
    }
    else if (src_img1.channels() == 1)
    {
        cv::resize(src_img1, img1, cv::Size(W, H));
    }

    if (src_img2.channels() == 3)
    {
        cv::resize(src_img2, src_img2, cv::Size(W, H));
        cv::cvtColor(src_img2, img2, cv::COLOR_BGR2GRAY);
    }
    else if (src_img2.channels() == 4)
    {
        cv::resize(src_img2, src_img2, cv::Size(W, H));
        cv::cvtColor(src_img2, img2, cv::COLOR_BGRA2GRAY);
    }
    else if (src_img2.channels() == 1)
    {
        cv::resize(src_img2, img2, cv::Size(W, H));
    }
    clock_gettime(CLOCK_MONOTONIC, &t);
    std::cout << "take " << get_msecs<double>(s, t) << " ms to convert image to gray" << std::endl;

    clock_gettime(CLOCK_MONOTONIC, &s);
    fast_orb_640x480_downscale(data1, 8);
    fast_orb_640x480_downscale(data2, 8);

    clock_gettime(CLOCK_MONOTONIC, &t);
    std::cout << "take " << get_msecs<double>(s, t) << " ms to downscale" << std::endl;

    clock_gettime(CLOCK_MONOTONIC, &s);
    std::vector<uint32_t> desc1;
    std::vector<uint32_t> desc2;
    fast_orb_640x480(data1, 8, keypoints1, desc1);
    fast_orb_640x480(data2, 8, keypoints2, desc2);
    clock_gettime(CLOCK_MONOTONIC, &t);
    std::cout << "take " << get_msecs<double>(s, t) << " ms to get orb" << std::endl;
    std::cout << "keypoints " << keypoints1.size() << " " << keypoints2.size() << std::endl;
    std::cout << "desc1 size " << desc1.size() << " desc2 size " << desc2.size() << std::endl;

    descriptor1 = cv::Mat(keypoints1.size(), 32, CV_8UC1, desc1.data());
    descriptor2 = cv::Mat(keypoints2.size(), 32, CV_8UC1, desc2.data());

    fast_orb_640x480_save(data1, "s1.jpg");
    fast_orb_640x480_save(data2, "s2.jpg");

    //return 0;

    //tm.start();
    clock_gettime(CLOCK_MONOTONIC, &s);
#if 0
    matcher->match(descriptor1, descriptor2, matches);
#else
    if (descriptor1.type() != CV_32F)
    {
        descriptor1.convertTo(descriptor1, CV_32F);
    }

    if (descriptor2.type() != CV_32F)
    {
        descriptor2.convertTo(descriptor2, CV_32F);
    }
    flann->match(descriptor1, descriptor2, matches);
#endif
    clock_gettime(CLOCK_MONOTONIC, &t);
    //tm.stop();
    std::cout << "take " << get_msecs<double>(s, t) << " ms to do matching " << matches.size() << std::endl;

    //tm.start();
    clock_gettime(CLOCK_MONOTONIC, &s);
    std::vector<cv::Point2f> X;
    std::vector<cv::Point2f> Y;

    for (int i = 0; i < matches.size(); ++i)
    {
        X.push_back(keypoints1[matches[i].queryIdx].pt); //Point2(matchings[i].first.x, matchings[i].first.y));
        Y.push_back(keypoints2[matches[i].trainIdx].pt); //(Point2(matchings[i].second.x, matchings[i].second.y));
    }

    std::vector<char> final_mask(matches.size(), 0);
    cv::Mat H = cv::findHomography(X, Y, cv::RANSAC, 3.0, final_mask);
    clock_gettime(CLOCK_MONOTONIC, &t);
    //tm.stop();
    int num_valid = 0;
    for (int i = 0; i < final_mask.size(); ++i)
    {
        if (final_mask[i] > 0)
        {
            num_valid++;
        }
    }
    std::cout << "take " << get_msecs<double>(s, t) << " ms to find homography:\n"
              << H << "\n valid matching " << num_valid << std::endl;

    cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, matches_draw, cv::Scalar::all(-1), cv::Scalar::all(-1), final_mask);
    if (argc > 3)
    {
        cv::imwrite(argv[3], matches_draw);
    }
#if !defined(__ANDROID__)
    else
    {
        cv::imshow("matches", matches_draw);
        cv::waitKey();
    }
#endif
}
