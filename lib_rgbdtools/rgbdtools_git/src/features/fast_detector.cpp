#include "rgbdtools/features/fast_detector.h"

namespace rgbdtools {

FastDetector::FastDetector(double threshold):FeatureDetector()
{
    fast_detector_.reset(new cv::FastFeatureDetector(threshold));
}

FastDetector::~FastDetector()
{

}

void FastDetector::findFeatures(RGBDFrame& frame, const cv::Mat& input_img)
{
  cv::Mat mask(frame.depth_img.size(), CV_8UC1);
  frame.depth_img.convertTo(mask, CV_8U);

  fast_detector_->detect(input_img,frame.keypoints,mask);
}

void FastDetector::setThreshold(double threshold)
{
    threshold_ = threshold;
    fast_detector_.reset(new cv::FastFeatureDetector(threshold_));
}

} // end namespace rgbdtools
