#!/usr/bin/env python
# coding=UTF-8

""" camshift_node.py - Version 1.0 2011-04-19

    Modification of the ROS OpenCV Camshift example using cv_bridge and publishing the ROI
    coordinates to the /roi topic.   
"""
# rosrun video_stream_opencv video_stream _fps:=30 _camera_name:=videofile _video_stream_provider:=/mnt/data/workspace/ros/src/blob-tracking/blob-tracking/src/media/2018-06-14-16.35.22.mp4
# ./camshift.py input_rgb_image:=/camera

import roslib
import rospy
import numpy as np
import sys
import cv2 as cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class ROS2OpenCV2(object):
    def __init__(self, node_name):
        self.node_name = node_name

        rospy.init_node(node_name)
        rospy.loginfo("Starting node " + str(node_name))

        rospy.on_shutdown(self.cleanup)

        # A number of parameters to determine what gets displayed on the
        # screen. These can be overridden the appropriate launch file
        self.show_text = rospy.get_param("~show_text", True)
        self.show_features = rospy.get_param("~show_features", True)
        self.show_boxes = rospy.get_param("~show_boxes", True)
        self.flip_image = rospy.get_param("~flip_image", False)
        self.feature_size = rospy.get_param("~feature_size", False)

        # Initialize the Region of Interest and its publisher
        self.ROI = RegionOfInterest()
        self.roi_pub = rospy.Publisher("/roi", RegionOfInterest, queue_size = 1)

        # Initialize a number of global variables
        self.frame = None
        self.frame_size = None
        self.frame_width = None
        self.frame_height = None
        self.depth_image = None
        self.marker_image = None
        self.display_image = None
        self.grey = None
        self.prev_grey = None
        self.selected_point = None
        self.selection = None
        self.drag_start = None
        self.keystroke = None
        self.detect_box = None
        self.track_box = None
        self.display_box = None
        self.keep_marker_history = False
        self.night_mode = False
        self.auto_face_tracking = False
        self.cps = 0 # Cycles per second = number of processing loops per second.
        self.cps_values = list()
        self.cps_n_values = 20
        self.busy = False
        self.resize_window_width = 0
        self.resize_window_height = 0
        self.face_tracking = False

        # Create the main display window
        self.cv_window_name = self.node_name
        #cv2.namedWindow(self.cv_window_name, cv2.WINDOW_NORMAL)
        #if self.resize_window_height > 0 and self.resize_window_width > 0:
        #    cv2.resizeWindow(self.cv_window_name, self.resize_window_width, self.resize_window_height)
        #else:
        #    cv2.resizeWindow(self.cv_window_name, 640, 480)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Set a call back on mouse clicks on the image window
        #cv2.setMouseCallback (self.node_name, self.on_mouse_click, None)

        # Subscribe to the image and depth topics and set the appropriate callbacks
        # The image topic names can be remapped in the appropriate launch file
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback)
        #self.hsv_sub   = rospy.Subscriber("adaptive_hsv_thresholds", HSV_history, self.hsv_history_callback)

    def on_mouse_click(self, event, x, y, flags, param):
        # This function allows the user to selection a ROI using the mouse
        if self.frame is None:
            return

        if event == cv2.EVENT_LBUTTONDOWN and not self.drag_start:
            self.features = []
            self.track_box = None
            self.detect_box = None
            self.selected_point = (x, y)
            self.drag_start = (x, y)

        if event == cv2.EVENT_LBUTTONUP:
            self.drag_start = None
            self.classifier_initialized = False
            self.detect_box = self.selection

        if self.drag_start:
            xmin = max(0, min(x, self.drag_start[0]))
            ymin = max(0, min(y, self.drag_start[1]))
            xmax = min(self.frame_width, max(x, self.drag_start[0]))
            ymax = min(self.frame_height, max(y, self.drag_start[1]))
            self.selection = (xmin, ymin, xmax - xmin, ymax - ymin)

    def image_callback(self, data):
        print("Image received")
        cv2.namedWindow(self.cv_window_name, cv2.WINDOW_NORMAL)
        # Set a call back on mouse clicks on the image window
        cv2.setMouseCallback (self.node_name, self.on_mouse_click, None)
        if self.resize_window_height > 0 and self.resize_window_width > 0:
            cv2.resizeWindow(self.cv_window_name, self.resize_window_width, self.resize_window_height)
        else:
            cv2.resizeWindow(self.cv_window_name, 640, 480)

        # Store the image header in a global variable
        self.image_header = data.header

        # Time this loop to get cycles per second
        start = time.time()

        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        frame = self.convert_image(data)

        # Some webcams invert the image
        if self.flip_image:
            frame = cv2.flip(frame, 0)

        # Store the frame width and height in a pair of global variables
        if self.frame_width is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size

        # Create the marker image we will use for display purposes
        if self.marker_image is None:
            self.marker_image = np.zeros_like(frame)

        # Copy the current frame to the global image in case we need it elsewhere
        self.frame = frame.copy()

        # Reset the marker image if we're not displaying the history
        if not self.keep_marker_history:
            self.marker_image = np.zeros_like(self.marker_image)

        # Process the image to detect and track objects or features
        processed_image = self.process_image(frame)

        # If the result is a greyscale image, convert to 3-channel for display purposes """
        #if processed_image.channels == 1:
            #cv2.CvtColor(processed_image, self.processed_image, cv2.CV_GRAY2BGR)
        #else:

        # Make a global copy
        self.processed_image = processed_image.copy()

        # Display the user-selection rectangle or point
        self.display_selection()

        # Night mode: only display the markers
        if self.night_mode:
            self.processed_image = np.zeros_like(self.processed_image)

        # Merge the processed image and the marker image
        self.display_image = cv2.bitwise_or(self.processed_image, self.marker_image)

        # If we have a track box, then display it.  The track box can be either a regular
        # cvRect (x,y,w,h) or a rotated Rect (center, size, angle).
        if self.show_boxes:
            if self.track_box is not None and self.is_rect_nonzero(self.track_box):
                if len(self.track_box) == 4:
                    x,y,w,h = self.track_box
                    size = (w, h)
                    center = (x + w / 2, y + h / 2)
                    angle = 0
                    self.track_box = (center, size, angle)
                else:
                    (center, size, angle) = self.track_box

                # For face tracking, an upright rectangle looks best
                if self.face_tracking:
                    pt1 = (int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
                    pt2 = (int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
                    cv2.rectangle(self.display_image, pt1, pt2, (50, 255, 50), self.feature_size, 8, 0)
                else:
                    # Otherwise, display a rotated rectangle
                    vertices = np.int0(cv2.boxPoints(self.track_box))
                    cv2.drawContours(self.display_image, [vertices], 0, (50, 255, 50), self.feature_size)

            # If we don't yet have a track box, display the detect box if present
            elif self.detect_box is not None and self.is_rect_nonzero(self.detect_box):
                (pt1_x, pt1_y, w, h) = self.detect_box
                if self.show_boxes:
                    cv2.rectangle(self.display_image, (pt1_x, pt1_y), (pt1_x + w, pt1_y + h), (50, 255, 50), self.feature_size, 8, 0)

        # Publish the ROI
        self.publish_roi()

        # Handle keyboard events
        self.keystroke = cv2.waitKey(5)

        # Compute the time for this loop and estimate CPS as a running average
        end = time.time()
        duration = end - start
        fps = int(1.0 / duration)
        self.cps_values.append(fps)
        if len(self.cps_values) > self.cps_n_values:
            self.cps_values.pop(0)
        self.cps = int(sum(self.cps_values) / len(self.cps_values))

        # Display CPS and image resolution if asked to
        if self.show_text:
            font_face = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5

            """ Print cycles per second (CPS) and resolution (RES) at top of the image """
            if self.frame_size[0] >= 640:
                vstart = 25
                voffset = int(50 + self.frame_size[1] / 120.)
            elif self.frame_size[0] == 320:
                vstart = 15
                voffset = int(35 + self.frame_size[1] / 120.)
            else:
                vstart = 10
                voffset = int(20 + self.frame_size[1] / 120.)
            cv2.putText(self.display_image, "CPS: " + str(self.cps), (10, vstart), font_face, font_scale, (255, 255, 0))
            cv2.putText(self.display_image, "RES: " + str(self.frame_size[0]) + "X" + str(self.frame_size[1]), (10, voffset), font_face, font_scale, (255, 255, 0))

	      #print("Show image")
        # Update the image display
        cv2.imshow(self.node_name, self.display_image)

        # Process any keyboard commands
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'n':
                self.night_mode = not self.night_mode
            elif cc == 'f':
                self.show_features = not self.show_features
            elif cc == 'b':
                self.show_boxes = not self.show_boxes
            elif cc == 't':
                self.show_text = not self.show_text
            elif cc == 'q':
                # The has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

    def depth_callback(self, data):
        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        depth_image = self.convert_depth_image(data)

        # Some webcams invert the image
        if self.flip_image:
            depth_image = cv2.flip(depth_image, 0)

        # Process the depth image
        processed_depth_image = self.process_depth_image(depth_image)

        # Make global copies
        self.depth_image = depth_image.copy()
        self.processed_depth_image = processed_depth_image.copy()

    def convert_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

    def convert_depth_image(self, ros_image):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "32FC1")
            # Convert to a numpy array since this is what OpenCV 2.3 uses
            depth_image = np.array(depth_image, dtype=np.float32)
            return depth_image

        except CvBridgeError, e:
            print e

    def publish_roi(self):
        if not self.drag_start:
            if self.track_box is not None:
                roi_box = self.track_box
            elif self.detect_box is not None:
                roi_box = self.detect_box
            else:
                return
        try:
            roi_box = self.cvBox2D_to_cvRect(roi_box)
        except:
            return

        # Watch out for negative offsets
        roi_box[0] = max(0, roi_box[0])
        roi_box[1] = max(0, roi_box[1])

        try:
            ROI = RegionOfInterest()
            ROI.x_offset = int(roi_box[0])
            ROI.y_offset = int(roi_box[1])
            ROI.width = int(roi_box[2])
            ROI.height = int(roi_box[3])
            self.roi_pub.publish(ROI)
        except:
            rospy.loginfo("Publishing ROI failed")

    def process_image(self, frame):
        return frame

    def process_depth_image(self, frame):
        return frame

    def display_selection(self):
        # If the user is selecting a region with the mouse, display the corresponding rectangle for feedback.
        if self.drag_start and self.is_rect_nonzero(self.selection):
            x,y,w,h = self.selection
            cv2.rectangle(self.marker_image, (x, y), (x + w, y + h), (0, 255, 255), self.feature_size)
            self.selected_point = None

        # Else if the user has clicked on a point on the image, display it as a small circle.
        elif not self.selected_point is None:
            x = self.selected_point[0]
            y = self.selected_point[1]
            cv2.circle(self.marker_image, (x, y), self.feature_size, (0, 255, 255), self.feature_size)

    def is_rect_nonzero(self, rect):
        # First assume a simple CvRect type
        try:
            (_,_,w,h) = rect
            return (w > 0) and (h > 0)
        except:
            try:
                # Otherwise, assume a CvBox2D type
                ((_,_),(w,h),a) = rect
                return (w > 0) and (h > 0)
            except:
                return False

    def cvBox2D_to_cvRect(self, roi):
        try:
            if len(roi) == 3:
                (center, size, angle) = roi
                pt1 = (int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
                pt2 = (int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
                rect = [pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1]]
            else:
                rect = list(roi)
        except:
            return [0, 0, 0, 0]

        return rect

    def cvRect_to_cvBox2D(self, roi):
        try:
            if len(roi) == 3:
                box2d = roi
            else:
                (p1_x, p1_y, width, height) = roi
                center = (int(p1_x + width / 2), int(p1_y + height / 2))
                size = (width, height)
                angle = 0
                box2d = (center, size, angle)
        except:
            return None

        return list(box2d)

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()


class CamShiftNode(ROS2OpenCV2):
    def __init__(self, node_name):

        self.node_name = node_name
        
        # The minimum saturation of the tracked color in HSV space,
        # as well as the min and max value (the V in HSV) and a 
        # threshold on the backprojection probability image.
        self.smin = rospy.get_param("~smin", 85)
        self.vmin = rospy.get_param("~vmin", 50)
        self.vmax = rospy.get_param("~vmax", 254)
        self.smax = rospy.get_param("~smax", 255)
        self.threshold = rospy.get_param("~threshold", 50)
                       
        #self.bin_count = 16
        #self.bin_w     = 24
        self.bin_count = 180
        self.bin_w     = 2

        # Initialize a number of variables
        self.hist = None
        self.track_window = None
        self.show_backproj = False
        ROS2OpenCV2.__init__(self, node_name)
  
        # self.hsv_history_sub = rospy.Subscriber("adaptive_hsv_thresholds", HSV_history, self.hsv_history_callback)
        print("Init done...")
    
    # These are the callbacks for the slider controls
    def set_smin(self, pos):
        self.smin = pos
        print("set_smin()", self.smin, self.smax, self.vmin, self.vmax)
        
    def set_vmin(self, pos):
        self.vmin = pos
        print("set_vmin()", self.smin, self.smax, self.vmin, self.vmax)
            
    def set_vmax(self, pos):
        self.vmax = pos
        print("set_vmax()", self.smin, self.smax, self.vmin, self.vmax)
       
    def set_threshold(self, pos):
        self.threshold = pos


    def hsv_history_callback(self, hsv_history):
        history_len = len(hsv_history.history)
        #print(history_len)
        #print(hsv_history.history[history_len-1])
        self.last_hsv = hsv_history.history[history_len-1]
        self.smin = self.last_hsv.sat.left
        self.smax = self.last_hsv.sat.right
        self.vmin = self.last_hsv.val.left
        self.vmax = self.last_hsv.val.right
        #self.hist
        tr = 1.0 * self.bin_count / 180.0
        hue_min = self.last_hsv.val.left  * tr
        hue_med = self.last_hsv.val.ch    * tr
        hue_max = self.last_hsv.val.right * tr
        mu, sigma = hue_med, 1.0*(hue_med-hue_min)/4 # mean and standard deviation
        print(mu, sigma)
        s = np.random.normal(mu, sigma, 10000)
        self.hist = np.histogram(s, bins=180, range=(1,180), normed=True)[0]
        #plt.plot(self.hist)
        
          
        print("hsv_history_callback()", self.smin, self.smax, self.vmin, self.vmax)
        '''
                      #> rosmsg show blob_tracking/HSV_history 
                      std_msgs/Header header
                        uint32 seq
                        time stamp
                        string frame_id
                      blob_tracking/HSV[] history
                        blob_tracking/HSVth hue
                          uint8 ch
                          uint8 left
                          uint8 right
                          uint8 width
                        blob_tracking/HSVth sat
                          uint8 ch
                          uint8 left
                          uint8 right
                          uint8 width
                        blob_tracking/HSVth val
                          uint8 ch
                          uint8 left
                          uint8 right
                          uint8 width
        '''


    # The main processing function computes the histogram and backprojection
    def process_image(self, cv_image):
        print("Processing...")

        # Moving these namedWindow() calls in the callback because of this bug:
        # http://answers.opencv.org/question/160607/imshow-call-never-returns-opencv-320-dev/

        # Create a number of windows for displaying the histogram,
        # parameters controls, and backprojection image
        cv2.namedWindow("Histogram", cv2.WINDOW_NORMAL)
        cv2.moveWindow("Histogram", 700, 50)
        cv2.namedWindow("Parameters", 0)
        cv2.moveWindow("Parameters", 700, 325)
        cv2.namedWindow("Backproject", 0)
        cv2.moveWindow("Backproject", 700, 600)
        cv2.namedWindow("Mask", 0)
        cv2.moveWindow("Mask", 900, 500)
        cv2.resizeWindow("Mask", 640, 480)
        
        # Create the slider controls for saturation, value and threshold
        cv2.createTrackbar("Saturation", "Parameters", self.smin, 255, self.set_smin)
        cv2.createTrackbar("Min Value", "Parameters", self.vmin, 255, self.set_vmin)
        cv2.createTrackbar("Max Value", "Parameters", self.vmax, 255, self.set_vmax)
        cv2.createTrackbar("Threshold", "Parameters", self.threshold, 255, self.set_threshold)
        
        # First blue the image
        frame = cv2.blur(cv_image, (5, 5))
        
        # Convert from RGB to HSV spave
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask using the current saturation and value parameters
        print("cv2.inRange()", self.smin, self.smax, self.vmin, self.vmax)
        mask = cv2.inRange(hsv, np.array((0., self.smin, self.vmin)), np.array((180., 255., self.vmax)))
        #print("mask:", mask)
        cv2.imshow("Mask", mask)
        
        # If the user is making a selection with the mouse, 
        # calculate a new histogram to track
        if self.selection is not None:
            x0, y0, w, h = self.selection
            x1 = x0 + w
            y1 = y0 + h
            self.track_window = (x0, y0, x1, y1)
            hsv_roi = hsv[y0:y1, x0:x1]
            mask_roi = mask[y0:y1, x0:x1]
            #self.hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
            self.hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [self.bin_count], [0, 180] )
            #print(self.hist)
            cv2.normalize(self.hist, self.hist, 0, 255, cv2.NORM_MINMAX);
            #print(self.hist)
            self.hist = self.hist.reshape(-1)
            self.show_hist()

        if self.detect_box is not None:
            self.selection = None
        
        # If we have a histogram, tracking it with CamShift
        if self.hist is not None:
            # Compute the backprojection from the histogram
            backproject = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
            
            # Mask the backprojection with the mask created earlier
            backproject &= mask

            # Threshold the backprojection
            ret, backproject = cv2.threshold(backproject, self.threshold, 255, cv2.THRESH_TOZERO)

            if self.track_window is not None:
              x, y, w, h = self.track_window
            if self.track_window is None or w <= 0 or h <=0:
                self.track_window = 0, 0, self.frame_width - 1, self.frame_height - 1
            
            # Set the criteria for the CamShift algorithm
            term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
            
            # Run the CamShift algorithm
            self.track_box, self.track_window = cv2.CamShift(backproject, self.track_window, term_crit)

            # Display the resulting backprojection
            cv2.imshow("Backproject", backproject)

        return cv_image
        
    def show_hist(self):
        #bin_count = self.hist.shape[0]
        #bin_w = 24
        img = np.zeros((256, self.bin_count*self.bin_w, 3), np.uint8)
        for i in xrange(self.bin_count):
            h = int(self.hist[i])
            top, left     = (i*self.bin_w+2, 255)
            bottom, right = ((i+1)*self.bin_w-2, 255-h)
            hue           = int(180.0*i/self.bin_count)
            #cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
            cv2.rectangle(img, (top, left), (bottom, right), (int(180.0*i/self.bin_count), 255, 255), -1)
            if h > 20:
                print("hue: ", hue, "- h:", h, "-", top, left, bottom, right)
                print(self.bin_count)
            img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
            cv2.imshow('Histogram', img)
        

    def hue_histogram_as_image(self, hist):
            """ Returns a nice representation of a hue histogram """
            histimg_hsv = cv.CreateImage((320, 200), 8, 3)

            mybins = cv.CloneMatND(hist.bins)
            cv.Log(mybins, mybins)
            (_, hi, _, _) = cv.MinMaxLoc(mybins)
            cv.ConvertScale(mybins, mybins, 255. / hi)
    
            w,h = cv.GetSize(histimg_hsv)
            hdims = cv.GetDims(mybins)[0]
            for x in range(w):
                xh = (180 * x) / (w - 1)  # hue sweeps from 0-180 across the image
                val = int(mybins[int(hdims * x / w)] * h / 255)
                cv2.rectangle(histimg_hsv, (x, 0), (x, h-val), (xh,255,64), -1)
                cv2.rectangle(histimg_hsv, (x, h-val), (x, h), (xh,255,255), -1)
    
            histimg = cv2.cvtColor(histimg_hsv, cv.CV_HSV2BGR)
            
            return histimg
         

if __name__ == '__main__':
    try:
        node_name = "camshift"
        CamShiftNode(node_name)
        try:
            rospy.init_node(node_name)
        except:
            pass
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()
