#include "DynamicExtractor.h"

using namespace std;
using namespace cv;
using namespace dnn;

bool comp(const cv::Mat &mask1, const cv::Mat &mask2){
    int size_mask1 = mask1.cols*mask1.rows-countNonZero(mask1);
    int size_mask2 = mask2.cols*mask2.rows-countNonZero(mask2);
    return size_mask2>size_mask1;
}

namespace ORB_SLAM2 {
    DynamicExtractor::DynamicExtractor(const string &strModelPath,
                                       float confThreshold, float maskThreshold) : confThreshold(confThreshold),
                                                                                   maskThreshold(maskThreshold) {
        string textGraph = strModelPath + "mask_rcnn_inception_v2_coco_2018_01_28.pbtxt";
        string modelWeights = strModelPath +  "frozen_inference_graph.pb";
        string classesFile = strModelPath + "mscoco_labels.names";
        // string dynamicClassFile = strModelPath + "dynamic.txt";

        // Load names of classes
        ifstream ifs(classesFile.c_str());
        string line;
        while (getline(ifs, line)) classes.push_back(line);

        // std::cout<<"number of classes:"<<classes.size()<<std::endl;

        // load names of dynamic classes
        // ifstream ifs2(dynamicClassFile.c_str());
        // while (getline(ifs2, line)) dynamicClasses.insert(line);

        // Load the network
        net = readNetFromTensorflow(modelWeights, textGraph);
        net.setPreferableBackend(DNN_BACKEND_OPENCV);
        // should be able to use intel GPU
        // net.setPreferableTarget(DNN_TARGET_OPENCL);

    }


    void DynamicExtractor::extractMask(const Mat &frame, std::vector<cv::Mat>& objectMasks, std::vector<int>& maskClassId) {
        Mat blob;

        // Create 4D blob from a frame as input
        blobFromImage(frame, blob, 1.0, Size(frame.cols, frame.rows), Scalar(), true, false);
        net.setInput(blob);

        // Runs the forward pass to get output from the output layers
        std::vector<String> outNames{"detection_out_final", "detection_masks"};
        vector<Mat> outs;
        net.forward(outs, outNames);
        Mat outDetections = outs[0];
        Mat outMasks = outs[1];

        // Output size of masks is NxCxHxW where
        // N - number of detected boxes
        // C - number of classes (excluding background)
        // HxW - segmentation shape
        // const int numClasses = outMasks.size[1];

        // Output size of Detection size is 1 * 1 * numDetections * 7
        // 7 refers to classId, score, and detection box information
        const int numDetections = outDetections.size[2];
        // reshape to channel = 1, row = num of detections
        // now outDetection size is numDetections * 7
        outDetections = outDetections.reshape(1, outDetections.total() / 7);

        // aggregate binary mask of dynamic objects into dynamic_mask
        // dynamic part should be zero
        // dynamic_mask = Mat(frame.size(), CV_8U, Scalar(255));
        Mat mat_zeros = Mat::zeros(frame.size(), CV_8U);
        std::map<int, int> classMaskMatch;
        std::vector<int> maskSize;
        maskSize.reserve(20);
        maskClassId.reserve(20);
        std::cout<<"----objects----"<<std::endl;
        for (int i = 0; i < numDetections; ++i) {
            float score = outDetections.at<float>(i, 2);
            if (score > confThreshold) {
                // Extract class id
                int classId = static_cast<int>(outDetections.at<float>(i, 1));
                
                // Extract bounding box
                int left = static_cast<int>(frame.cols * outDetections.at<float>(i, 3));
                int top = static_cast<int>(frame.rows * outDetections.at<float>(i, 4));
                int right = static_cast<int>(frame.cols * outDetections.at<float>(i, 5));
                int bottom = static_cast<int>(frame.rows * outDetections.at<float>(i, 6));

                left = max(0, min(left, frame.cols - 1));
                top = max(0, min(top, frame.rows - 1));
                right = max(0, min(right, frame.cols - 1));
                bottom = max(0, min(bottom, frame.rows - 1));
                Rect box = Rect(left, top, right - left + 1, bottom - top + 1);

                // Extract the mask for the object
                Mat objectMask(outMasks.size[2], outMasks.size[3], CV_32F, outMasks.ptr<float>(i, classId));
                // Resize the mask, threshold, color and apply it on the image
                resize(objectMask, objectMask, Size(box.width, box.height));
                // threshold mask into binary 255/0 mask
                Mat mask = (objectMask > maskThreshold);
                mask.convertTo(mask, CV_8U);
                Mat dynamic_mask = Mat(frame.size(), CV_8U, Scalar(255));
                mat_zeros(box).copyTo(dynamic_mask(box), mask);
                int mSize = dynamic_mask.cols*dynamic_mask.rows-countNonZero(dynamic_mask);
                
                //Filter out small objects
                if(mSize>3100){
                    std::cout<<classId<<":"<<classes[classId]<<":"<<mSize<<std::endl;
                    maskSize.push_back(mSize);
                    classMaskMatch.insert(pair<int,int>(mSize,classId));
                    objectMasks.push_back(dynamic_mask);
                }

            }
        }

        // sort the dynamic_mask by the size of area
        sort(maskSize.begin(), maskSize.end());
        for(size_t s = 0; s<maskSize.size();s++){
            maskClassId.push_back(classMaskMatch[maskSize[s]]);
        }
        
        sort(objectMasks.begin(), objectMasks.end(), comp);
    }
}