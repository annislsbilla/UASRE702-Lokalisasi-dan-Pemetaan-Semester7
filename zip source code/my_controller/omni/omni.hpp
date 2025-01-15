#ifndef OMNI_HPP
#define OMNI_HPP

#include <opencv2/opencv.hpp>
#include <UAVision.h>
#include <Config.h>
#include <Camera.h>
#include <Lut.h>
#include <RLE.h>
#include <Blob.h>
#include <libCalib.h>
#include <CameraOpenCV.h>
#include <ScanLines.h>
#include <vector>
#include <Vec.h>
#include <CameraCalib.h>

class Omni
{
public:
Omni();

private:
cv::Mat image, originalImage;
Config config;
Lut *lut;
Camera *cam;


};
#endif