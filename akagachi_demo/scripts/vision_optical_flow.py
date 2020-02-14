#!/usr/bin/env python
import numpy as np
import cv2
from skimage import measure
from scipy import ndimage
import rospy
import rospkg
from std_msgs.msg import String
import json
from threading import Thread
import tf
"""Detect motion and publish point of interest to tf
"""

SET_DRAW_DENSITY = 1
SET_THINNING = 6

class OptFlowDetector:
    def __init__(self, compress=0.3):
        self.compress = compress
        self.prev = None

    def detect(self, image):
        '''
        detect movement.
        parameters:
        - input: input image. You obviously need at least 2 images to detect movement, so nothing is calculated on the very first call.
        - doodle: image to overwrite results on. If none, will overwrite results on the input image. Should have same aspect ratio as input image, but can be of different size. optional.

        returns:
        - doodle: overlays optical flow in red, translucent colored squares in areas where there was movement, and red circle for the focus point.
        - point to focus on (None if there are no interesting points)
        '''
        overlayedImage = image.copy()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.resize(image, (0, 0), fx=self.compress,
                        fy=self.compress, interpolation=cv2.INTER_AREA)
        if self.prev is None:
            # on the first call to the function. self.prev saves previous image for comparison.
            self.prev = image
            return overlayedImage, None
        # Parameters based on http://funvision.blogspot.com/2016/02/opencv-31-tutorial-optical-flow.html
        flow = cv2.calcOpticalFlowFarneback(
            self.prev, image, None, 0.4, 1, 12, 2, 5, 1.1, 0)
        # flow is of shape [height, width, 2]
        self.prev = image

        # get the norm for each pixel (=how much movement there was)
        flow_norm = np.linalg.norm(flow, axis=(2))

        # convert to an image that you can process in opencv. 40 seems fine, but tuning may be necessary.
        as_image = (flow_norm*40).astype(np.uint8)

        # cf. Pyimagesearch- how to detect bright spots in an image
        # https://www.pyimagesearch.com/2016/10/31/detecting-multiple-bright-spots-in-an-image-with-python-and-opencv/
        # 0 for all pixels below 50, 255 for all above it.
        as_image = cv2.threshold(as_image, 40, 255, cv2.THRESH_BINARY)[1]
        as_image = cv2.erode(as_image, None, iterations=2)
        as_image = cv2.dilate(as_image, None, iterations=4)

        labels = measure.label(as_image, neighbors=4, background=0)
        # saves masks for each label (labelmasks[i] corresponds to label i+1, because background label is 0 and we don't care about the background label
        labelmasks = []

        # variables to find the label with largest area
        max_numPixels = 0
        max_label = 0
        max_label_com = None
        for label in np.unique(labels):
            if label == 0:
                # this is the background. Ignore.
                continue
            labelmask = np.zeros(as_image.shape, dtype='uint8')
            labelmask[labels == label] = 255
            labelmasks.append(labelmask)
            numPixels = cv2.countNonZero(labelmask)

            if numPixels > max_numPixels:
                max_numPixels = numPixels
                max_label = label

        if max_numPixels < self.prev.size*0.01:
            return self.draw(overlayedImage, flow_norm, labelmasks=labelmasks), None

        if max_label != 0:
            # the "center of mass" should be the center of movement
            max_label_com = ndimage.measurements.center_of_mass(labelmasks[max_label-1])
            max_label_com_remapped = np.array([max_label_com[1]/self.compress,max_label_com[0]/self.compress])
            overlayedImage = self.draw(overlayedImage, flow_norm,
                            max_index=max_label_com, labelmasks=labelmasks)
            return overlayedImage, max_label_com_remapped
        return self.draw(overlayedImage, flow_norm), None

    def draw(self, overlayedImage, flow_norm, max_index=None, keypoints=None, labelmasks=None):
        overlayedImage_h, overlayedImage_w = overlayedImage.shape[:2]
        h, w = flow_norm.shape
        ratio = overlayedImage_h/h

        # keypoints are what you get from blob detection.
        if keypoints is not None:
            for keypoint in keypoints:
                cv2.circle(overlayedImage, (int(
                    keypoint.pt[0]*ratio), int(keypoint.pt[0]*ratio)), int(keypoint.size*ratio), (255, 0, 0), 10)

        # color each area with movement.
        # cf: https://www.pyimagesearch.com/2016/03/07/transparent-overlays-with-opencv/
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
                  (160, 160, 0), (160, 0, 160), (0, 160, 160)]
        if labelmasks is not None:
            overlay = overlayedImage.copy()
            # this process was found to take a lot of time (because pixelwise operations?)
            for i in range(len(labelmasks)):
                X, Y = labelmasks[i].shape
                for x in range(0, X//SET_THINNING, SET_DRAW_DENSITY):
                    for y in range(0, Y//SET_THINNING, SET_DRAW_DENSITY):
                        if labelmasks[i][x*SET_THINNING, y*SET_THINNING] == 255:
                            overlay[int(x*SET_THINNING/self.compress):int((x+1)*SET_THINNING/self.compress), int(
                                y*SET_THINNING/self.compress):int((y+1)*SET_THINNING/self.compress)] = colors[i % len(colors)]
                            continue
            cv2.addWeighted(overlay, 0.5, overlayedImage, 0.5, 0, overlayedImage)

        # draw center of focus point
        if max_index is not None:
            cv2.circle(overlayedImage, (int(
                max_index[1]*ratio), int(max_index[0] * ratio)), 50, (0, 0, 255), -1)

        return overlayedImage

def imgPoint_to_space_point(imgPoint, inv_proj, dist=1):
    '''
    convert point in 2d image to point in 3d space
    '''
    imgPoint = np.array([imgPoint[0], imgPoint[1], 1.0])
    space_point = np.dot(inv_proj, imgPoint)
    space_point *= dist
    return (space_point[2], -space_point[0], -space_point[1])

def cameraLoop(width, height):
    '''
    thread to process frame retrieval.
    By executing the I/O process on another thread, the whole process can be sped up.
    cf: https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/
    '''
    # newFlag: a flag to be set to True when a new frame is saved to captured_frame.
    global captured_frame, newFlag
    newFlag = False
    print("starting video stream...")
    print("press \"q\" in camera feed window to quit program")
    cap = cv2.VideoCapture(0)
    cap.set(3, width)
    cap.set(4, height)
    while not rospy.is_shutdown():
        ret, captured_frame = cap.read()
        newFlag = True
    cap.release()

firstFlag = True
if __name__ == "__main__":
    rospy.init_node("vision", anonymous=True)
    rospack = rospkg.RosPack()
    try:
        camera_name = rospy.get_param("/camera_info/name")
        width = rospy.get_param("/camera_info/width")
        height = rospy.get_param("/camera_info/height")
    except KeyError:
        print("ROS param for vision.py not found, so using default values....")
        camera_name = "ELP_170"
        width = 480
        height = 640
    print("camera name:{}\theight:{}\twidth:{}".format(camera_name, height, width))
    with open("{}/calibration_param/{}.json".format(rospack.get_path("akagachi_demo"), camera_name, ".json")) as fp:
        params = json.load(fp)
    proj = np.array(params['camera_matrix'])
    print("camera projection matrix\n{}".format(proj))
    inv_proj = np.linalg.inv(proj)
    print("Inverse of camera projection matrix\n{}".format(inv_proj))

    cameraThread = Thread(target=cameraLoop, args=(width, height))
    cameraThread.daemon = True
    cameraThread.start()
    
    tfBroadcaster = tf.TransformBroadcaster()
    opt_flow_detector = OptFlowDetector(compress=0.2)

    while not rospy.is_shutdown():
        while not newFlag:
            # wait until a new frame is available.
            # the main thread's process should probably always be slower than the camera thread, so in theory this should be bypassed every time.
            if rospy.is_shutdown():
                break
            rospy.sleep(0.05)
        frame = captured_frame.copy()
        newFlag = False

        overlayedImage, center = opt_flow_detector.detect(frame)
        if center is not None or firstFlag:
            if firstFlag:
                center = np.array([width/2, height/2])
            point3D = imgPoint_to_space_point(center, inv_proj)
            tfBroadcaster.sendTransform(point3D, tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(), "/focus", "/camera")

        cv2.flip(overlayedImage, 0)
        cv2.imshow('detection_results', overlayedImage)
        input = cv2.waitKey(1) & 0xFF
        if input == ord('q'):
            break
        firstFlag = False
    cv2.destroyAllWindows()