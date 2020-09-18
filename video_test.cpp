#include <vector>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <ORBextractor.h>

template <typename T>
static inline T get_msecs(struct timespec &start, struct timespec &stop)
{
    long long seconds = stop.tv_sec - start.tv_sec;
    long long nseconds = stop.tv_nsec - start.tv_nsec;
    nseconds += (seconds * 1000000000);
    double mseconds = (double)nseconds / 1000000;
    return (T)(mseconds);
}

static const int WIDTH = 640;
static const int HEIGHT = 360;

#define USE_FASE_ORB 1
#define USE_OPTF 1
#define USE_ORBSLAM 1

static inline void gen_poly(const int &width, const int &height, const cv::Mat &H, std::vector<cv::Point2f> &poly)
{
    std::vector<cv::Point2f> src(4);
    std::vector<cv::Point2f> dst(4);
    for (int i = 0; i < 4; ++i)
    {
        src[i] = cv::Point2f(width * (((i + 1) / 2) & 0x1), height * (i / 2));
    }
    cv::perspectiveTransform(src, dst, H);
    cv::intersectConvexConvex(src, dst, poly);
}

static inline bool good_hom(const cv::Mat &H)
{
    cv::Rect roi(0, 0, 2, 2);
    cv::Mat first2 = H(roi);
    if (cv::determinant(first2) <= 0)
    {
        return false;
    }
    float a = H.at<float>(0, 0);
    float b = H.at<float>(1, 1);
    float c = H.at<float>(0, 1);
    if (a > 0 && b > 0)
    {
        //
        if (a < b)
        {
            float tmp = a;
            a = b;
            b = tmp;
        }
        if (a > 1.15 || b < 0.85 || c > 0.15)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

#if defined(USE_OPTF)
void drawPoints(const cv::Mat &ref, const std::vector<cv::KeyPoint> &kp, cv::Mat &output)
{
    output = cv::Mat(ref.size(), ref.type());
    ref.copyTo(output);
    for (int i = 0; i < kp.size(); ++i)
    {
        cv::Scalar clr = cv::Scalar(std::rand() % 255, std::rand() % 255, std::rand() % 255);
        cv::circle(output, kp[i].pt, 3, clr);
    }
}

template <typename T>
void drawOpticalFlowPoints(const cv::Mat &ref, const std::vector<cv::Point2f> &kp1,
                           const cv::Mat &cur, const std::vector<cv::Point2f> &kp2,
                           const std::vector<T> &status, cv::Mat &output)
{
    //
    int width = ref.cols;
    assert(kp1.size() == kp2.size());
    assert(kp2.size() == status.size());
    output = cv::Mat::zeros(ref.rows > cur.rows ? ref.rows : cur.rows, ref.cols + cur.cols, ref.type());
    cv::Mat lines = cv::Mat::zeros(output.size(), output.type());
    cv::Mat lines_mask = cv::Mat::zeros(output.size(), CV_32F);
    cv::Mat bg_mask = cv::Mat::ones(output.size(), CV_32F);

    cv::Rect roi(0, 0, ref.cols, ref.rows);
    cv::Mat left = output(roi);
    roi = cv::Rect(ref.cols, 0, cur.cols, cur.rows);
    cv::Mat right = output(roi);

    ref.copyTo(left);
    cur.copyTo(right);

    float alpha = 0.5;

    for (int i = 0; i < kp2.size(); ++i)
    {
        cv::Scalar clr = cv::Scalar::all(255);
        if (status[i])
        {
            clr = cv::Scalar(std::rand() % 255, std::rand() % 255, std::rand() % 255);
            cv::line(lines, kp1[i], cv::Point((int)(kp2[i].x + width), (int)kp2[i].y), cv::Scalar(255, 0, 0));
            cv::line(lines_mask, kp1[i], cv::Point((int)(kp2[i].x + width), (int)kp2[i].y), cv::Scalar::all(alpha));
            cv::line(bg_mask, kp1[i], cv::Point((int)(kp2[i].x + width), (int)kp2[i].y), cv::Scalar::all(1.0 - alpha));
        }
        cv::circle(output, cv::Point((int)(kp2[i].x + width), (int)kp2[i].y), 3, clr);
    }
    cv::blendLinear(lines, output, lines_mask, bg_mask, output);
}
#endif

int main(int argc, char *argv[])
{
    struct timespec s;
    struct timespec t;
    cv::Mat desc_ref;
    cv::Mat desc_cur;
#if !defined(USE_OPTF)
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
#endif
#if defined(USE_ORBSLAM)
    ORB_SLAM2::ORBextractor feature(1000, 1.2, 8, 20, 7);
#else
    cv::Ptr<cv::FeatureDetector> feature = cv::ORB::create(1000, 1.2, 4, 8, 0, 2, 0, 8, 0);
#endif
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
    vout.open(argv[2], cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, cv::Size(WIDTH * 2, HEIGHT));
    clock_gettime(CLOCK_MONOTONIC, &t);
    std::cout << "take " << get_msecs<double>(s, t) << " ms to read image" << std::endl;

    cv::Mat img;
    cv::Mat ref;
#if defined(USE_OPTF)
    cv::Mat ref_gray;
#endif

    if (!vin.grab())
    {
        std::cout << "fatal error: grab image error " << argv[1] << std::endl;
        return 0;
    }

    if (!vin.retrieve(img))
    {
        std::cout << "fatal error: read image error " << argv[1] << std::endl;
        return 0;
    }

#if defined(USE_OPTF)
    //cv::Mat img_gray;
    cv::cvtColor(img, ref_gray, cv::ColorConversionCodes::COLOR_BGR2GRAY);
    cv::resize(ref_gray, ref_gray, cv::Size(WIDTH, HEIGHT));
#endif
    cv::resize(img, ref, cv::Size(WIDTH, HEIGHT));

#if defined(USE_ORBSLAM)
    feature(ref_gray, cv::Mat(), kp_ref, desc_ref);
#else
    feature->detectAndCompute(ref, cv::Mat(), kp_ref, desc_ref);
#endif
    cv::Mat ref_points;
    drawPoints(ref, kp_ref, ref_points);

    //if(desc_ref.type()!=CV_32F) {
    //    desc_ref.convertTo(desc_ref, CV_32F);
    //}

    cv::Mat cur;
#if defined(USE_OPTF)
    cv::Mat cur_gray;
    std::vector<cv::Point2f> p_ref;
    std::vector<cv::Point2f> p_cur;
    for (int i = 0; i < kp_ref.size(); ++i)
    {
        p_ref.push_back(kp_ref[i].pt);
        p_cur.push_back(kp_ref[i].pt);
    }
#endif
    while (vin.grab())
    {
        clock_gettime(CLOCK_MONOTONIC, &s);
        vin.retrieve(img);
        cv::resize(img, cur, cv::Size(WIDTH, HEIGHT));
        cv::Mat H;
        std::vector<char> final_mask;

#if defined(USE_OPTF)
        cv::cvtColor(img, cur_gray, cv::COLOR_BGR2GRAY);
        cv::resize(cur_gray, cur_gray, cv::Size(WIDTH, HEIGHT));
        //kp_cur.clear();
        std::vector<uchar> status;
        std::vector<float> err;

        cv::calcOpticalFlowPyrLK(ref_gray, cur_gray, p_ref, p_cur, status, err, cv::Size(10, 10), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 0, 1e-4);
        assert(p_ref.size() == p_cur.size());

        std::vector<cv::Point2f> vp_ref;
        std::vector<cv::Point2f> vp_cur;
        {
            int i = 0;
            for (i = 0; i < p_cur.size(); ++i)
            {
                if (status[i])
                {
                    vp_ref.push_back(p_ref[i]);
                    vp_cur.push_back(p_cur[i]);
                }
            }
        }
        final_mask.resize(vp_cur.size(), 0);
        if (vp_ref.size() > 20)
        {
            H = cv::findHomography(vp_ref, vp_cur, cv::RANSAC, 3.0, final_mask, 2000, 0.9999);
        }
        H.convertTo(H, CV_32FC1);

        //drawOpticalFlowPoints(matches_draw, p_ref, p_cur, status, ref.cols, ref.rows);
        drawOpticalFlowPoints<char>(ref_points, vp_ref, cur, vp_cur, final_mask, matches_draw);
#else
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
        if (matches.size() > 20)
        {
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
            for (int i = 0; i < final_mask.size(); ++i)
            {
                if (final_mask[i] > 0)
                {
                    num_valid++;
                }
            }
            std::cout << "take " << get_msecs<double>(s, t) << " ms to find homography:\n"
                      << H << "\n valid matching " << num_valid << std::endl;
        }
        cv::drawMatches(ref, kp_ref, cur, kp_cur, matches, matches_draw, cv::Scalar::all(-1), cv::Scalar::all(-1), final_mask);
#endif

        clock_gettime(CLOCK_MONOTONIC, &s);
        std::vector<cv::Point2f> polyf;
        if (!H.empty())
        {
            if (good_hom(H))
            {
                gen_poly(WIDTH, HEIGHT, H, polyf);
            }
            else
            {
                std::cout << "bad homography " << std::endl;
            }
        }
        clock_gettime(CLOCK_MONOTONIC, &t);
        std::cout << "take " << get_msecs<double>(s, t) << " ms to find poly" << std::endl;

        if (polyf.size())
        {
            cv::Mat weights_img = cv::Mat::ones(matches_draw.size(), CV_32FC1);
            cv::Mat weights_mask = cv::Mat::zeros(matches_draw.size(), CV_32FC1);
            cv::Mat mask = cv::Mat::zeros(matches_draw.size(), CV_8UC3);
            std::vector<cv::Point2i> poly(polyf.size());
            for (int i = 0; i < polyf.size(); ++i)
            {
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
