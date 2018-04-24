#include <opencv2/opencv.hpp>

#include <vector>
#include <time.h>

template<typename T>
static inline T get_msecs(struct timespec& start, struct timespec& stop)
{
    long long seconds = stop.tv_sec - start.tv_sec;
    long long nseconds = stop.tv_nsec - start.tv_nsec;
    nseconds += (seconds * 1000000000);
    double mseconds = (double)nseconds / 1000000;
    return (T)(mseconds);
}

int main(int argc, char * argv[])
{
    //cv::TickMeter tm;
    struct timespec s;
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &s);
    //tm.start();
    cv::Mat img1 = cv::imread(argv[1]);
    cv::Mat img2 = cv::imread(argv[2]);
    clock_gettime(CLOCK_REALTIME, &t);
    //tm.stop();
    std::cout << "take " << get_msecs<double>(s, t) << " ms to read image" << std::endl;

    cv::Ptr<cv::ORB> orb = cv::ORB::create(700);
    cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create();
    cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    cv::Ptr<cv::FlannBasedMatcher> flann = cv::FlannBasedMatcher::create();
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptor1;
    cv::Mat descriptor2;
    std::vector<cv::DMatch> matches;
    cv::Mat matches_draw;
    //std::vector<char> valid_matching;
    //tm.start();
    clock_gettime(CLOCK_REALTIME, &s);
    orb->detectAndCompute(img1, cv::Mat(), keypoints1, descriptor1);
    orb->detectAndCompute(img2, cv::Mat(), keypoints2, descriptor2);
    clock_gettime(CLOCK_REALTIME, &t);
    //tm.stop();
    std::cout << "take " << get_msecs<double>(s, t) << " ms to get key points " << keypoints1.size() << " " << keypoints2.size() << std::endl;
    std::cout << keypoints1.size() << " key points " << descriptor1.type() << " " << descriptor1.rows << " " << descriptor1.cols << std::endl;
    //tm.start();
    clock_gettime(CLOCK_REALTIME, &s);
#if 0
    matcher->match(descriptor1, descriptor2, matches);
#else
    if(descriptor1.type()!=CV_32F) {
        descriptor1.convertTo(descriptor1, CV_32F);
    }

    if(descriptor2.type()!=CV_32F) {
        descriptor2.convertTo(descriptor2, CV_32F);
    }
    flann->match(descriptor1, descriptor2, matches);
#endif
    clock_gettime(CLOCK_REALTIME, &t);
    //tm.stop();
    std::cout << "take " << get_msecs<double>(s, t) << " ms to do matching " << matches.size() << std::endl;

    //tm.start();
    clock_gettime(CLOCK_REALTIME, &s);
    std::vector<cv::Point2f> X;
    std::vector<cv::Point2f> Y;

    for (int i = 0; i < matches.size(); ++i)
    {
        X.push_back(keypoints1[matches[i].queryIdx].pt); //Point2(matchings[i].first.x, matchings[i].first.y));
        Y.push_back(keypoints2[matches[i].trainIdx].pt); //(Point2(matchings[i].second.x, matchings[i].second.y));
    }

    std::vector<char> final_mask(matches.size(), 0);
    cv::Mat H = cv::findHomography(X, Y, CV_RANSAC, 3.0, final_mask);
    clock_gettime(CLOCK_REALTIME, &t);
    //tm.stop();
    int num_valid = 0;
    for (int i = 0; i < final_mask.size(); ++i) {
        if (final_mask[i] > 0) {
            num_valid++;
        }
    }
    std::cout << "take " << get_msecs<double>(s, t) << " ms to find homography:\n" << H << "\n valid matching " << num_valid << std::endl;

    cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, matches_draw, cv::Scalar::all(-1), cv::Scalar::all(-1), final_mask);
    if (argc > 3) {
        cv::imwrite(argv[3], matches_draw); 
    } else {
        cv::imshow("matches", matches_draw);
        cv::waitKey();
    }
}
