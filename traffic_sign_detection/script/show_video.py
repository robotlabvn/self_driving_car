#!/usr/bin/env python
import rospy
import roslib
import sys
import cv2
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, CameraInfo
import numpy as np

scale =1
delta = 0
ddepth = cv2.CV_16S

class show_image():

    def __init__(self, node_name):
        ## Initial ROS node 
        self.node_name = node_name
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        ## Intitial ROS publisher
        self.image_pub = rospy.Publisher("output/image_raw/compressed", CompressedImage)
        
        ## Initial ROS Subcriber
        self.cv_window_name =self.node_name
        self.image_sub = rospy.Subscriber("Team1_image/compressed", CompressedImage, self.callback, queue_size =1)

        ## Log information
        rospy.loginfo("Warning for image topic ....")
        rospy.wait_for_message("input_rgb_image", CompressedImage)
        rospy.loginfo("Ready.")

       

    def callback(self, ros_image):
        
         ## Display Image
        ros_image_data=ros_image.data
        frame = cv2.imdecode(np.fromstring(ros_image_data, dtype=np.uint8), -1)
        cv2.imshow(self.node_name, frame)
        cv.waitKey(5)

         ## Display Process Image
        display_process_image = self.process_image(frame)
        cv2.imshow("image_process", display_process_image)

    def process_image(self, frame):
        #Convert BRG to HSL
        grey = cv2.cvtColor(frame, cv.COLOR_BGR2HSV_FULL)
        #grey_2 = cv2.cvtColor(grey, cv.COLOR_BGR2GRAY)
        
        ########### Sobel Operator
        #Gradient - X
        grad_x = cv2.Sobel(grey, ddepth, 1, 0, ksize =3, scale =scale, delta = delta, borderType = cv2.BORDER_DEFAULT)
        # Gradient -Y
        grad_y = cv2.Sobel(grey, ddepth, 0, 1, ksize =3, scale =scale, delta = delta, borderType = cv2.BORDER_DEFAULT) 
        # Converting back to unit8
        abs_grad_x = cv2.convertScaleAbs(grad_x)
        abs_grad_y = cv2.convertScaleAbs(grad_y)
        sobel = cv2.addWeighted(abs_grad_x, 0.8, abs_grad_y, 0.8,0)

        # Threshhold Operation in Range
        kernel = np.ones((1,1), np.float32)/1
        img_filter = cv2.filter2D(sobel, -1, kernel)
        frame_threshold =cv.inRange(img_filter,(50,50,50),(150,150,150))

        # Compute edges using the Canny edge filter
        blur = cv2.blur(grey, (3,3))
        edges = cv2.Canny(blur,30.0, 50.0)
        #edges = cv2.Canny(blur, 30.0, 50.0)

        return sobel
    

    
    
    def cleanup(self):
        print "Shutting down image"
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        node_name = "show_image"
        show_image(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
        cv2.destroyAllWindows()

