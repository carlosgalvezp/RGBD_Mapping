#ifndef FAST_DETECTOR_H
#define FAST_DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>


#include "rgbdtools/features/feature_detector.h"


namespace rgbdtools {

/**
 * @brief The FastDetector class
 */
class FastDetector: public FeatureDetector
{
public:
    /**
     * @brief Default constructor
     */
    FastDetector(int n_features= 200,
                 double threshold = 10);

    /**
     * @brief Default destructor
     */
    ~FastDetector();

    /** @brief Implementation of the feature detector.
     * @param frame the input frame
     * @param input_img the image for feature detection, derived from the
     *        RGB image of the frame after (optional) blurring
     */
    void findFeatures(RGBDFrame& frame, const cv::Mat& input_img);

    /**
     * @brief Set the detection threshold
     * @param threshold
     */
    void setThreshold(double threshold);

private:

    double threshold_; // Threshold for the detector
    int n_features_; // Number of features to extract

    boost::shared_ptr<cv::FastFeatureDetector> fast_detector_; ///< OpenCV FAST detector object    
    cv::SiftDescriptorExtractor sift_descriptor_; ///< OpenCV SIFT descriptor extractor object
};

typedef boost::shared_ptr<FastDetector> FastDetectorPtr;

} // namespace rgbdtools
#endif // FAST_DETECTOR_H

