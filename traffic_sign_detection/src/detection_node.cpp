#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.h"
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "classify.hpp"
#include "traffic_sign_detection/sign.h"

/****************
 * This function runs the main algorithms of the traffic_sign_detection
 * Subscribe: ROS_image from camera
 * Algorithm: Using the SVM trained and classify the new sign features 
 * */
int main (int argc, char **argv){
    //Node creation
    ros::init(argc, argv, "classification");
    ros::NodeHandle n;

    //Initializations
    classify visual;
    cv::HOGDescriptor hog(cv::Size(64, 64),
                    cv::Size(32, 32),
                    cv::Size(16,16),
                    cv::Size(32, 32),
                    9, 1, -1, 0, 0.2,
                    1, 64, 1);
    ROS_INFO_STREAM("HOG Descriptor created");

    visual.imagen = cv::Mat::zeros(640, 480, CV_8UC3);
    std::vector<cv::Mat> trainImgs;
    std::vector<int> trainLabels;
    cv::Mat trainHOG;

    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
    ROS_INFO_STREAM("Support Vector Machine Created");
    double area;

    cv::Mat img_denoised;
    std::vector<cv::Mat> imgs_mser(100);
    cv::Mat testHOG;

    //Image Subcriber
    cv::startWindowThread();
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub =it.subscribe("/Team1_image", 1, &classify::imageCallback, &visual);
    
    //Custom message Publisher
    ros::Publisher signPub = n.advertise<traffic_sign_detection::sign>(
        "traffic", 1);
    traffic_sign_detection::sign msg;

    ////////TRAINING/////////////////////////
    int trained = visual.trainStage(hog, svm, trainImgs, trainLabels);

    ///////CLASSIFICATION////////////////////
    ROS_INFO_STREAM("Dectection and Classification started.....");
    while(ros::ok()){
        if(!visual.imagen.empty()){
            // Denoised image with gaussian blur
            img_denoised =visual.deNoise(visual.imagen);
            //Show image denoised
            imshow("image_denoised", visual.imagen);

            //Get the detections using MSER
            imgs_mser = visual.MSER_Feature(visual.imagen, area);

            // If there are detection in the frame:
            if (imgs_mser.size() != 0) {
                // HOG features of detections
                testHOG = visual.HOG_Feature(hog, imgs_mser);
                imshow("testHOG", testHOG);
        
                // Evaluate using the SVM
                visual.traffic_sign = visual.SVMTesting(svm, testHOG);

                // Publish the type of sign through message
                msg.area = area;
                msg.sign_type = visual.traffic_sign;
                signPub.publish(msg);

                imgs_mser.clear();
            }
              
            //Visualization of the robot view with all detections
            int flagviz = visual.visualization();
        }
        ros::spinOnce();
    }
    return 0;
}