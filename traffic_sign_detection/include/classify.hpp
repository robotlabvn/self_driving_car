/*** MIT License
 * Author: Tri.B. Minh
 * Reference: Miguel Maestre Trueba
 * File: classify.hpp
 * header filer with definitions for class classifier.
 * */

#pragma once //The use of #pragma once can reduce build times as the compiler will not open and read the file after the first #include of the file in the translation unit.

#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/ml/ml.hpp"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"


// It is contain the function of the classifier

class classify {
    private: 
    //DATASET Loading
    // Load all the image from the dataset include sign and labels
    //@parameter trainImgs is an output vector that contains all the training images
    //@parameter trainLabels is an output vector that contains all the labels of the training image
    void loadTrainingImgs(std::vector<cv::Mat> &trainImgs,
                        std::vector<int> &trainLabels);

    /*****************************************************/
    //TRAIN SVM
    /**
     * Set all the parameters of the SVM object and feed it with the HOG features of the labeled training image.
     * @paramter svm is the support vector object
     * @parameter trainHOG is the cv::Mat with the HOG features from the training images
     * @parameter trainLabels is the vector with all labels for the training images.
     * @return none
     **/
    void SVMTraining(cv::Ptr<cv::ml::SVM> &svm,
            cv::Mat trainHOG, std::vector<int> trainLabels);

    public:
        cv::Mat imagen; // OpenCV Imgae from the camera by the subscriber
        std::vector<cv::Rect> boxes; // Bounding boxes in the current frame
        float traffic_sign; // the label of the detection, outputed by the SVM

        // IMAGE CALLBACK
        /**
         *  Callback used in the subcriber for the camera topic. Get the ROS Imgs to the OpenCV Img 
         * @parameter msg is the ROS Image msg
         */ 
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        //*******************************************/
        /** Apply Gaussian Filter to the imput image denoise it
         * @param inputImage is the current image to be filtered
         * @return Blurred and denoised image
         * */
        cv::Mat deNoise(cv::Mat inputImage);

        //*******************************************/
        /**MSER Feature 
        * Find the MSER feature in the image. First normalize image and binarize, then look the regions and resize each regions image.
        * @parameter img is the current image where MSER features are detected
        * @parameter area is an output variable that stores the area of each region's bounding box
        * @return Vector of all images of detections
        * */
       std::vector<cv::Mat> MSER_Feature(cv::Mat img, double &area);

       //*******************************************/
       //HOG Feature 
       /* Computer the HOG feature in every image of the vector
       * @parameter hog is the HOG Descriptor object with all the parameters setups                                                                                                                                                                        
       * @parameter imgs is the vector of images to get theHOG from
       * @return Matrix with the HOG Descriptor for all images.
       */
      cv::Mat HOG_Feature(cv::HOGDescriptor hog, std::vector<cv::Mat> imgs);
    
      //*******************************************/
      //TRAININGS
      /*
      * Function that runs loadTrainings(), HOG_Features() and SVM Training
      * @param hog is the HOG Descriptor object with all parameters set up
      * @param svm is the Support Vector Machine object
      * @param trainImgs is an output vector that contains all the training images
      * @param trainLabels is an output vector that contains all the labels of the training imgs
      * @return 1 if success
      * */
     int trainStage(cv::HOGDescriptor &hog,cv::Ptr<cv::ml::SVM> &svm,
        std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLables);

    //***********************************************/
    //CLASSIFICATION
    /**
     * Feed SVM with the HOG Matrix of the current image output its label.
     * @param testHOG is the HOG Descriptor object with all parameters setup
     * @param svm is the Support Vector Machine object
     * @return Labels of the tested set of features. Label of sign being recognized
     * */
    float SVMTesting(cv::Ptr<cv::ml::SVM> &svm, cv::Mat testHOG);

    //***********************************************/
    //VISUALIZATION
    /**
     * Open a Windowns with the Robot's view of the workspace
     * @param none
     * @return 1 if success
     * */
    int  visualization();
};