//
//

#ifndef DYNAMIC_ORB_SLAM2_DYNAMICEXTRACTOR_H
#define DYNAMIC_ORB_SLAM2_DYNAMICEXTRACTOR_H

#include <fstream>
#include <sstream>
#include <iostream>
#include <string.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <unordered_set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/dnn.hpp>



namespace ORB_SLAM2 {
    class DynamicExtractor {

    public:
        // need model location to load the model
        // only supports mask-rcnn for now
        DynamicExtractor(const std::string &strModelPath,
                         float confThreshold = 0.6, float maskThreshold = 0.3);

        // compute dynamic mask for given frame
        void extractMask(const cv::Mat &frame, std::vector<cv::Mat>& objectMasks, std::vector<int>& maskClassId);
        // void extractMask(const cv::Mat &frame, std::vector<cv::Mat>& objectMasks, std::vector<char*>& objectLabel);

    private:
        // return non-zero if the corresponding class is dynamic
        bool is_dynamic(int classId) {
            return dynamicClasses.count(classes[classId]);
        }

        float confThreshold; // Confidence threshold
        float maskThreshold; // Mask threshold

        std::vector<std::string> classes; // classId --> className
        std::unordered_set<std::string> dynamicClasses; // name of dynamic classes

        // std::vector<int> maskClassId; // class id for each mask
        cv::dnn::Net net; // mask-rcnn model
    };

}
#endif //DYNAMIC_ORB_SLAM2_DYNAMICEXTRACTOR_H
