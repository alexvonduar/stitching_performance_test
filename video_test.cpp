#include <vector>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

template<typename T>
static inline T get_msecs(struct timespec& start, struct timespec& stop)
{
    long long seconds = stop.tv_sec - start.tv_sec;
    long long nseconds = stop.tv_nsec - start.tv_nsec;
    nseconds += (seconds * 1000000000);
    double mseconds = (double)nseconds / 1000000;
    return (T)(mseconds);
}

static const int WIDTH = 320;
static const int HEIGHT = 180;

#define USE_FASE_ORB 1

static inline void gen_poly(const int& width, const int& height, const cv::Mat& H, std::vector<cv::Point2f>& poly)
{
    std::vector<cv::Point2f> src(4);
    std::vector<cv::Point2f> dst(4);
    for (int i = 0; i < 4; ++i) {
        src[i] = cv::Point2f(width * (((i + 1) / 2) & 0x1), height * (i / 2));
    }
    cv::perspectiveTransform(src, dst, H);
    cv::intersectConvexConvex(src, dst, poly);
}

int main(int argc, char * argv[])
{
    struct timespec s;
    struct timespec t;
    cv::Mat desc_ref;
    cv::Mat desc_cur;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    cv::Ptr<cv::FeatureDetector> feature = cv::ORB::create(1000, 1.2, 4, 8, 0, 2, 0, 8, 0);
    std::vector<cv::KeyPoint> kp_ref;
    std::vector<cv::KeyPoint> kp_cur;
    std::vector<cv::DMatch> matches;
    cv::Mat matches_draw;
    //cv::Mat img1;
    //cv::Mat img2;
    cv::VideoCapture vin;
    cv::VideoWriter vout;

    clock_gettime(CLOCK_MONOTONIC, &s);
    vin.open(argv[1]);
    vout.open(argv[2], CV_FOURCC('a', 'v', 'c', '1'), 30, cv::Size(WIDTH * 2, HEIGHT));
    clock_gettime(CLOCK_MONOTONIC, &t);
    std::cout << "take " << get_msecs<double>(s, t) << " ms to read image" << std::endl;

    cv::Mat img;
    cv::Mat ref;

    if (!vin.grab()) {
        std::cout << "fatal error: grab image error " << argv[1] << std::endl;
        return 0;
    }

    if (!vin.retrieve(img)) {
        std::cout << "fatal error: read image error " << argv[1] << std::endl;
        return 0;
    }

    cv::resize(img, ref, cv::Size(WIDTH, HEIGHT));

    feature->detectAndCompute(ref, cv::Mat(), kp_ref, desc_ref);
    //if(desc_ref.type()!=CV_32F) {
    //    desc_ref.convertTo(desc_ref, CV_32F);
    //}

    cv::Mat cur;
    while (vin.grab()) {
        clock_gettime(CLOCK_MONOTONIC, &s);
        vin.retrieve(img);
        cv::resize(img, cur, cv::Size(WIDTH, HEIGHT));
        feature->detectAndCompute(cur, cv::Mat(), kp_cur, desc_cur);
        //if(desc_cur.type()!=CV_32F) {
        //    desc_cur.convertTo(desc_cur, CV_32F);
        //}
        matcher->match(desc_ref, desc_cur, matches);
        clock_gettime(CLOCK_MONOTONIC, &t);
        std::cout << "take " << get_msecs<double>(s, t) << " ms to convert image to gray" << std::endl;

        //clock_gettime(CLOCK_MONOTONIC, &s);
        //std::vector<cv::DMatch> gms_matches;
        //cv::xfeatures2d::matchGMS(ref.size(), cur.size(), kp_ref, kp_cur, matches, gms_matches, true, true);
        //clock_gettime(CLOCK_MONOTONIC, &t);
        //std::cout << "take " << get_msecs<double>(s, t) << " ms to do gms" << std::endl;

        std::vector<cv::Point2f> X;
        std::vector<cv::Point2f> Y;
        cv::Mat H;
        std::vector<char> final_mask;
        if (matches.size() > 20) {
        clock_gettime(CLOCK_MONOTONIC, &t);
        for (int i = 0; i < matches.size(); ++i)
        {
           X.push_back(kp_ref[matches[i].queryIdx].pt); //Point2(matchings[i].first.x, matchings[i].first.y));
           Y.push_back(kp_cur[matches[i].trainIdx].pt); //(Point2(matchings[i].second.x, matchings[i].second.y));
        }

        final_mask.resize(matches.size(), 0);
        H = cv::findHomography(X, Y, CV_RANSAC, 1.0, final_mask);
        //tm.stop();
        int num_valid = 0;
        for (int i = 0; i < final_mask.size(); ++i) {
            if (final_mask[i] > 0) {
               num_valid++;
            }
        }
        std::cout << "take " << get_msecs<double>(s, t) << " ms to find homography:\n" << H << "\n valid matching " << num_valid << std::endl;
        }

        cv::drawMatches(ref, kp_ref, cur, kp_cur, matches, matches_draw, cv::Scalar::all(-1), cv::Scalar::all(-1), final_mask);
        clock_gettime(CLOCK_MONOTONIC, &s);
        std::vector<cv::Point2f> polyf;
        if (!H.empty()) {
            cv::Rect roi(0, 0, 2, 2);
            cv::Mat first2 = H(roi);
            if (cv::determinant(first2) > 0) {
                gen_poly(WIDTH, HEIGHT, H, polyf);
            } else {
                std::cout << "bad homography " << std::endl;
            }
        }
        clock_gettime(CLOCK_MONOTONIC, &t);
        std::cout << "take " << get_msecs<double>(s, t) << " ms to find poly" << std::endl;
        if (polyf.size()) {
        cv::Mat weights_img = cv::Mat::ones(matches_draw.size(), CV_32FC1);
        cv::Mat weights_mask = cv::Mat::zeros(matches_draw.size(), CV_32FC1);
        cv::Mat mask = cv::Mat::zeros(matches_draw.size(), CV_8UC3);
        std::vector<cv::Point2i> poly(polyf.size());
        for (int i = 0; i < polyf.size(); ++i) {
            poly[i] = cv::Point2i((int)polyf[i].x + WIDTH, (int)polyf[i].y);
        }
        cv::fillConvexPoly(weights_img, poly, cv::Scalar::all(0.7));
        cv::fillConvexPoly(weights_mask, poly, cv::Scalar(0.3));
        cv::fillConvexPoly(mask, poly, cv::Scalar(0, 255, 0));
        cv::blendLinear(matches_draw, mask, weights_img, weights_mask, matches_draw);
        }
        vout.write(matches_draw);
    }
}
