#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

//using namespace cv;
//using namespace std;

static const int WIDTH = 320;
static const int HEIGHT = 240;

static cv::Mat H = cv::Mat::eye(3, 3, CV_32FC1);

cv::Mat img;

static const int MAX_SX = 2000;
static const int MAX_SHEER = 1000;
static const int SX_NORM = 1000;
static const int SHEER_NORM = 1000;

static int sx = SX_NORM;
static int sy = SX_NORM;
static int sheer = 0;

/// Function header
void sx_callback(int, void *);
void sy_callback(int, void *);
void sheer_callback(int, void *);
void gen_output(cv::Mat &dst, const cv::Mat &H, const int &w, const int &h);

/** @function main */
int main(int argc, char **argv)
{
    /// Load source image and convert it to gray
    gen_output(img, H, WIDTH, HEIGHT);

    /// Create Window
    char *source_window = "Source";
    cv::namedWindow(source_window, CV_WINDOW_AUTOSIZE);
    cv::imshow(source_window, img);

    cv::createTrackbar(" sx:", "Source", &sx, MAX_SX, sx_callback);
    cv::createTrackbar(" sy:", "Source", &sy, MAX_SX, sy_callback);
    cv::createTrackbar(" sheer:", "Source", &sheer, MAX_SHEER, sheer_callback);
    sx_callback(0, 0);

    cv::waitKey(0);
    return (0);
}

/** @function thresh_callback */
void sx_callback(int, void *)
{
    float _sx = (float)sx / SX_NORM;
    H.at<float>(0, 0) = _sx;
    //std::cout << " set sx to " << _sx << std::endl;

    /// Draw contours
    gen_output(img, H, WIDTH, HEIGHT);

    /// Show in a window
    cv::namedWindow("Source", CV_WINDOW_AUTOSIZE);
    cv::imshow("Source", img);
}

/** @function thresh_callback */
void sy_callback(int, void *)
{
    float _sy = (float)sy / SX_NORM;
    H.at<float>(1, 1) = _sy;
    //std::cout << " set sx to " << _sx << std::endl;

    /// Draw contours
    gen_output(img, H, WIDTH, HEIGHT);

    /// Show in a window
    cv::namedWindow("Source", CV_WINDOW_AUTOSIZE);
    cv::imshow("Source", img);
}

/** @function thresh_callback */
void sheer_callback(int, void *)
{
    float _sheer = (float)sheer / SHEER_NORM;
    H.at<float>(0, 1) = _sheer;

    /// Draw contours
    gen_output(img, H, WIDTH, HEIGHT);

    /// Show in a window
    cv::namedWindow("Source", CV_WINDOW_AUTOSIZE);
    cv::imshow("Source", img);
}

void gen_rectangle(cv::Point2f &tl, cv::Point2f &br, const cv::Mat &H, const int &w, const int &h)
{
    std::vector<cv::Point2f> src(4);
    src[0] = cv::Point2f(0, 0);
    src[1] = cv::Point2f(w, 0);
    src[2] = cv::Point2f(w, h);
    src[3] = cv::Point2f(0, h);
    std::vector<cv::Point2f> dst(4);

    cv::perspectiveTransform(src, dst, H);
    //
    tl = cv::Point2f(w, h);
    br = cv::Point2f(0, 0);

    for (int i = 0; i < 4; ++i)
    {
        tl.x = tl.x > dst[i].x ? dst[i].x : tl.x;
        br.x = br.x < dst[i].x ? dst[i].x : br.x;
        tl.y = tl.y > dst[i].y ? dst[i].y : tl.y;
        br.y = br.y < dst[i].y ? dst[i].y : br.y;
    }
}

void gen_output(cv::Mat &dst, const cv::Mat &H, const int &w, const int &h)
{
    cv::Point2f tl, br;
    gen_rectangle(tl, br, H, w, h);
    int off_x = std::floor(tl.x);
    int off_y = std::floor(tl.y);
    int size_x = std::ceil(br.x) - std::floor(tl.x);
    int size_y = std::ceil(br.y) - std::floor(tl.y);

    float shift_x = tl.x - std::floor(tl.x) + w;
    float shift_y = tl.y - std::floor(tl.y);

    cv::Mat src = cv::Mat(cv::Size(w, h), CV_8UC3, cv::Scalar(0, 255, 0)); //  = dst(roi);
    cv::Mat T = cv::Mat::eye(3, 3, CV_32FC1);
    H.copyTo(T);
    T.at<float>(0, 2) = shift_x;
    T.at<float>(1, 2) = shift_y;
    cv::warpPerspective(src, dst, T, cv::Size(size_x + w, h > size_y ? h : size_y));
    cv::Rect roi(0, 0, w, h);
    cv::Mat left = dst(roi);
    left.setTo(cv::Scalar(0, 0, 255));
}
