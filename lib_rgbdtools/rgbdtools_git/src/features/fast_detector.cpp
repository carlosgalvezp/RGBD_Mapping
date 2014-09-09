#include "rgbdtools/features/fast_detector.h"

namespace rgbdtools {

FastDetector::FastDetector(int n_features, double threshold):FeatureDetector()
{
    fast_detector_.reset(new cv::FastFeatureDetector(threshold));
    n_features_ = n_features;
}

FastDetector::~FastDetector()
{

}

void FastDetector::findFeatures(RGBDFrame& frame, const cv::Mat& input_img)
{
  cv::Mat mask(frame.depth_img.size(), CV_8UC1);
  frame.depth_img.convertTo(mask, CV_8U);

  // **** Detect keypoints
  std::vector<cv::KeyPoint> keypoints;
  fast_detector_->detect(input_img,keypoints,mask);

  // **** Retain only a number of keypoints
  if (keypoints.size() > n_features_)
      reduceKeypoints(keypoints, n_features_, frame.keypoints);

  // **** Compute SIFT descriptor at the extracted keypoints
  //Debug
  cv::Mat img_kpts;
  cv::drawKeypoints(input_img, frame.keypoints, img_kpts);
  cv::imshow("Depth mask", mask);
  cv::imshow("Keypoints ", img_kpts);
  cv::waitKey(1);

  compute_descriptors_ = true;
  if(compute_descriptors_)
  {
      sift_descriptor_.compute(
        input_img, frame.keypoints, frame.descriptors);
  }

          // **** DEBUG
          std::cout<<"FAST keypoints: "<<frame.keypoints.size()<<
                    " SIFT Descriptors: "<<frame.descriptors.rows<<"x"<<
                    frame.descriptors.cols<<std::endl;
}

void FastDetector::setThreshold(double threshold)
{
    threshold_ = threshold;
    fast_detector_.reset(new cv::FastFeatureDetector(threshold_));
}

} // end namespace rgbdtools
