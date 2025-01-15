#include <omni.hpp>

Omni::Omni()
{
    image = cv::Mat(cv::Size(640, 640), CV_8UC3);
    originalImage = cv::Mat(cv::Size(640, 640), CV_8UC3);

    cv::Mat idxImage = cv::Mat(cv::Size(640, 640), CV_8UC3);
    cv::Mat segImage = cv::Mat(cv::Size(640, 640), CV_8UC3);
    cv::Mat realImage = cv::Mat(cv::Size(640, 640), CV_8UC3);
    cv::Mat resolutionImage = cv::Mat(cv::Size(640, 640), CV_8UC3);

    cv::Point robotCenter(640 / 2, 640 / 2);
    cv::Rect whiteRect(640 - 100, 0, 50, 50);

    cv::namedWindow("tes");
    ScanLines linesRad(idxImage, UAV_RADIAL, robotCenter, 450, 10, 10, 1);

    lut = new Lut(config, linesRad);
    
}