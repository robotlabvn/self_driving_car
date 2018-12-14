/*** MIT licence
 * The traffic_sign_detection Package using SVM (Support Vector Machine)
 * Author: Tri.B.M 2018
 * Reference: Miguel Maestre Trueba 2017
 * Method: receiving images and detecting and classifying signs in the images.
 *
*/
#include <cv_bridge/cv_bridge.h> // package connect Open CV and ROS
#include <cstdlib> //Define general purpose function, Dynamic memmory management,..
#include <string>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "classify.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>



void classify::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr; 
    //Convert from ROS image msg to OpenCV image
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // is simply a constant for "bgr8"
        //cv::imshow("View", cv_ptr -> image);
        imagen = cv_ptr ->image; 
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
            msg -> encoding.c_str());
    }

}
cv::Mat classify::deNoise(cv::Mat inputImage){
    cv::Mat output;
    //Apply Gausian filter to denoise image
    cv::GaussianBlur(inputImage, output, cv::Size(3,3),0,0 );
    return output;
}
std::vector<cv::Mat> classify::MSER_Feature(cv::Mat img, double &area){
    cv::Mat bgr[3];
    cv::Mat red_blue;
    cv::Mat rb_binary;
    cv::Mat detection;
    cv::Size size(64,64);

    std::vector<cv::Mat> detections;

    //Normalize images with respect to Red and Blue and binarize
    split(img, bgr);

    cv::Mat red_norm =255*(bgr[2]/(bgr[0]+bgr[1]+bgr[2]));
    cv::Mat red;
    red_norm.convertTo(red, CV_8UC1);
    cv::Mat blue_norm = 255*(bgr[0]/(bgr[0]+bgr[1]+bgr[2]));
    cv::Mat blue;
    blue_norm.convertTo(blue, CV_8UC1);

    max(red, blue, red_blue);

    threshold(red_blue, rb_binary, 200, 255, cv::THRESH_BINARY);
   
    //Show image for debugging
    
    imshow("Blue", blue);
    imshow("Red", red);
    imshow("Blue_norm", blue_norm);
    imshow("Red_norm", red_norm);
    imshow("Red_blue", red_blue);
    imshow("Rb_binary",rb_binary);
    imshow("Image", img);

    //MSER region detection
    cv::Ptr<cv::MSER> ms = cv::MSER::create(5, 60,14400, 0.25, 0.1, 200, 1.01, 0.1, 1);
    std::vector<std::vector<cv::Point> > regions;
    std::vector<cv::Rect> mser_bbox;
    ms -> detectRegions(rb_binary, regions, mser_bbox);
  
    //For every bounding box in the image
    for (cv::Rect i : mser_bbox){
        //Ratio filter of detected regions

        double ratio = (static_cast<double>(i.height) /static_cast<double>(i.width));

        if (ratio > 0.8 && ratio <1.2){
            //Crop bounding boxes to get new images
            detection =img(i);
            area = static_cast<double>(i.height)*static_cast<double>(i.width);

            //Resize image to fit the trained data
            cv:resize(detection, detection, size);

            //Output the vector of images
            detections.push_back(detection);
            this ->boxes.push_back(i);
        }
    }
    return detections;
    
}
cv::Mat classify::HOG_Feature(cv::HOGDescriptor hog,
    std::vector<cv::Mat> imgs) {
    std::vector<std::vector<float> > HOG;

    //For all of the images of the vector, compute HOG (Histograms of Oriented Gradients (HOG)) features 
    for (cv::Mat i :imgs){
        std::vector<float> descriptor;
        hog.compute(i, descriptor);
        HOG.push_back(descriptor);
    }

    //Convert HOG features vector
    cv::Mat signMat(HOG.size(), HOG[0].size(), CV_32FC1);
    auto i=0;
    while (i < HOG.size()){
        auto j =0;
        while (j <HOG[0].size()){
            signMat.at<float>(i,j) = HOG[i][j];
            j++;
        }
        i++;
    }
    cv::namedWindow("HOG Feature");
    imshow("HogFeature", signMat);
    return signMat; 
}
void classify::loadTrainingImgs(std::vector<cv::Mat> &trainImgs,
    std::vector<int> &trainLabels){
        //Load all the forward sign images from dataset and label them
        //cv::String path name "./Training_Image/1"
        cv::String pathname ="/home/robotlab/catkin_ws/src/traffic_sign_detection/Train_Imgs/1";
        std::vector<cv::String> filenames; //The elements are stored contiguously, which means that elements can be accessed not only through iterators, but also using offsets to regular pointers to elements
        cv::glob (pathname, filenames); //Generate a list of all files that match the globbing pattern
        cv::Size size(64,64); //Declare image with same size (WxD =64x64)

        for (cv::String i : filenames){
            cv::Mat src = imread(i);

            cv::resize(src, src, size); //Resize image with same size (WxD =64x64)
            trainImgs.push_back(src); //Adds a new element at the end of the vector
            trainLabels.push_back(1);
        }
        // Load all the turn signs images from dataset and label them
        cv::String pathname2 = "/home/robotlab/catkin_ws/src/traffic_sign_detection/Train_Imgs/2";
        std::vector<cv::String> filenames2;
        cv::glob(pathname2, filenames2);
        cv::Size size2(64, 64);
     

        for (cv::String i : filenames2) {
            cv::Mat src2 = imread(i);

            cv::resize(src2, src2, size2);
            trainImgs.push_back(src2);
            trainLabels.push_back(2);
        }
        // Load all the stop signs images from dataset and label them
        cv::String pathname3 = "/home/robotlab/catkin_ws/src/traffic_sign_detection/Train_Imgs/3";
        std::vector<cv::String> filenames3;
        cv::glob(pathname3, filenames3);
        cv::Size size3(64, 64);

        for (cv::String i : filenames3) {
            cv::Mat src3 = imread(i);

            cv::resize(src3, src3, size3);
            trainImgs.push_back(src3);
            trainLabels.push_back(3);
        }
    }
void classify::SVMTraining(cv::Ptr<cv::ml::SVM> &svm, cv::Mat trainHOG, 
        std::vector<int> trainLabels){ //cv::Ptr -Smart pointer to dynamically allocated objects.
        //Set parameters of SVM
        svm -> setGamma(0.50625);
        svm -> setC(12.5);
        svm -> setKernel(cv::ml::SVM::RBF);
        svm -> setType(cv::ml::SVM::C_SVC);

        //Feed SVM with all the labeled data and train it
        //cv::ml::TrainData::create(sample(input array), layout(ROW_SAMPLE), responses(input array) )
        cv::Ptr<cv::ml::TrainData> td =cv::ml::TrainData::create(trainHOG,
            cv::ml::ROW_SAMPLE, trainLabels);
        svm -> train(td);
        }

/********************************************************
This program is training the SVM following steps:
            Step1: Load Training Image and resize image
                            ||
                            \/
            Step2: HOG Feature of the Images
                            ||
                            \/
            Step3: SVM Training       
*/
int classify::trainStage(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &svm,
    std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels){
    ROS_INFO_STREAM("SVM Training State started..");
    //Load training image and resize
    this -> loadTrainingImgs(trainImgs, trainLabels);

    // HOG features of training images
    cv::Mat trainHOG = this->HOG_Feature(hog, trainImgs);

    //Train SVM and save model
    this -> SVMTraining(svm,trainHOG, trainLabels);
    ROS_INFO_STREAM("SVM Training Stage Completed");
    ros::Duration(2).sleep();

    //Return 1 as success
    return 1;
    }
//***************************************************8

// This code feed SVM with HOG Feature and return label it
float classify::SVMTesting(cv::Ptr<cv::ml::SVM> &svm,cv::Mat testHOG){
    cv::Mat answer;

    //Feed SVM with HOG features from detection and label it
    svm -> predict(testHOG, answer);

    //Return the label if the direction
    auto i=0;
    while (i< answer.rows){
        this ->traffic_sign = answer.at<float>(i,0);
        i++;
        return this-> traffic_sign;
    }
}
// Visualization, show the bounding box around the detection objects
int classify::visualization(){
    cv::Mat viz;
    this->imagen.copyTo(viz);
    cv::putText(viz, "Robot View", cv::Point(5, 10),
        cv::FONT_HERSHEY_COMPLEX, 1, CV_RGB(0,0, 255));
    
    // For all the signs in the image. print the bounding box amd type of sign
    for (cv::Rect i : this->boxes){
        cv::rectangle(viz, i, CV_RGB(50,200,0),2);
        if (this ->traffic_sign ==1 ){
            cv::Point org(i.x, i.y +5);
            cv::putText(viz,"Object",org,
            cv::FONT_HERSHEY_COMPLEX,1, CV_RGB(0,0,255));
        }
    
         if (this->traffic_sign == 2) {
            cv::Point org(i.x, i.y + 5);
            cv::putText(viz, "Turn_Right", org,
                cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
        }
        if (this->traffic_sign == 3) {
            cv::Point org(i.x, i.y + 5);
            cv::putText(viz, "Turn_Left", org,
                cv::FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 255));
        }
    }
    this->boxes.clear();
    cv::namedWindow("View");
    imshow("view", viz);

    return 1;
}