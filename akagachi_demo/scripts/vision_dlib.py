#!/usr/bin/env python
import json
from threading import Thread
from imutils import face_utils
import numpy as np
import cv2
from skimage import measure
from scipy import ndimage
import dlib
import rospkg
import rospy
import roslib.packages
from std_msgs.msg import String
import tf

"""Detect motion and publish point of interest to tf
"""

SET_DRAW_DENSITY = 1
SET_THINNING = 6
SET_EXPAND = 300
ROS_PATH = roslib.packages.get_pkg_dir('akagachi_demo')
CV2_PATH = "/usr/local/lib/python3.6/dist-packages/cv2/data/"
FACE_CASCADE_PATH = CV2_PATH + '/haarcascade_frontalface_alt2.xml'
PROFILE_FACE_CASCADE_PATH = CV2_PATH + '/haarcascade_profileface.xml'
EYE_CASCADE_PATH = CV2_PATH + '/haarcascade_eye.xml'
DLIB_FACE_LMS_PREDICTER_PATH = ROS_PATH + '/../face_model/shape_predictor_68_face_landmarks.dat'

try:
    dlib_face_landmarks_predictor = dlib.shape_predictor(DLIB_FACE_LMS_PREDICTER_PATH)
    face_cascade = cv2.CascadeClassifier(FACE_CASCADE_PATH)
    PROFILE_FACE_CASCADE = cv2.CascadeClassifier(PROFILE_FACE_CASCADE_PATH)
    eye_cascade = cv2.CascadeClassifier(EYE_CASCADE_PATH)

except IOError:
    print('dace landmark data is nothing! lets go facemodel folder and run shell!')

class DlibFaceDetector:
    '''
    Ditect bu using dlib
    '''
    def __init__(self, compress=0.3):
        self.compress = compress
        self.prev = None
        self.pre_pos = np.zeros(2)

    def dlib_detect(self, image):
        '''
        detect movement.
        parameters:
        - input: input image. You obviously need at least 2 images to detect movement, so nothing is calculated on the very first call.
        - doodle: image to overwrite results on. If none, will overwrite results on the input image. Should have same aspect ratio as input image, but can be of different size. optional.

        returns:
        - overlayedImage: overlays optical flow in red, translucent colored squares in areas where there was movement, and red circle for the focus point.
        - point to focus on (None if there are no interesting points)
        '''
        detector = dlib.get_frontal_face_detector()
        center_pos = (0,0)
        target_face = dlib.rectangle(300, 300, 1000, 1000)
        
        #Image handling
        overlayedImage = image.copy()
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #gray_image = cv2.resize(gray_image, (0, 0), fx=self.compress,
        #                fy=self.compress, interpolation=cv2.INTER_AREA)
        if self.prev is None:
            # on the first call to the function. self.prev saves previous image for comparison.
            self.prev = image
            return overlayedImage, None

        #For speeding up, cascade detect faces.
        faces = face_cascade.detectMultiScale(gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(10, 10))

        # target face is just one
        if len(faces) == 1:
            x, y, w, h = faces[0, :]
            cv2.rectangle(overlayedImage, (x, y), (x + w, y + h), (255, 0, 0), 2)

            # trim the face the cascade detedted 
            face = dlib.rectangle(x, y, x + w, y + h)
            face_img = image[y: y + h, x: x + w]
            face_parts = dlib_face_landmarks_predictor(image, face).parts()

            center_pos = self.get_middle_eyes(image, face_parts)

            # if you need the LMS, use here
            #for i in face_parts:
            #    cv2.circle(overlayedImage, (i.x, i.y), 3, (255, 0, 0), -1)

            if center_pos is not None: 
                cv2.circle(overlayedImage, center_pos, 5, (36, 74, 247), -1)
                self.pre_pos = center_pos
            else:
                center_pos = self.pre_pos
                
        return overlayedImage, center_pos

    def eye_point(self, img, parts, left=True):
        '''
        calicurate the center of eye images
        parameters:
        - img : camera raw image
        - parts : the face landmarks from Dlib
        - L or R : Left is True, Right is False
        return
        - eye_point_center : center point of the eye image
        '''

        if left:
            eyes = [
                parts[36],
                min(parts[37], parts[38], key=lambda x: x.y),
                max(parts[40], parts[41], key=lambda x: x.y),
                parts[39],
                ]
        else:
            eyes = [
                parts[42],
                min(parts[43], parts[44], key=lambda x: x.y),
                max(parts[46], parts[47], key=lambda x: x.y),
                parts[45],
                ]
        org_x = eyes[0].x
        org_y = eyes[1].y

        # for blinking, under development
        #if self.is_close(org_y, eyes[2].y):
        #    return None

        eye = img[org_y:eyes[2].y, org_x:eyes[-1].x]
        _, eye = cv2.threshold(cv2.cvtColor(eye, cv2.COLOR_RGB2GRAY), 30, 255, cv2.THRESH_BINARY_INV)
     
        eye_point_center = self.get_center(eye)

        if eye_point_center is not None:
            return eye_point_center[0] + org_x, eye_point_center[1] + org_y
        return eye_point_center

    def get_middle_eyes(self, img, parts):
        '''
        cariculate the point between two eyes.
        '''
        center_posl = self.eye_point(img, parts, True) 
        center_posr = self.eye_point(img, parts, False)

        if (center_posl is None) or (center_posr is None):
            return None
        
        center_pos = ((center_posl[0] + center_posr[0])/2, (center_posl[1] + center_posr[1])/2)
        return center_pos

    def get_center(self, gray_img):
        '''
        cariculate the center of the input image.
        '''
        moments = cv2.moments(gray_img, False)
        print(moments)
        try:
            return int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])
        except:
            return None

    def is_close(self, y0, y1):
        '''
        detect the closing eye
        '''
        if abs(y0 - y1) < 10:
            return True
        return False

def imgPoint_to_space_point(imgPoint, inv_proj, dist=1):
    '''
    convert point in 2d image to point in 3d space
    '''
    imgPoint = np.array([imgPoint[0], imgPoint[1], 1.0])
    space_point = np.dot(inv_proj, imgPoint)
    space_point *= dist
    return (space_point[2], -space_point[0], -space_point[1])

def cameraLoop(c_width, c_height):
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
    cap.set(3, c_width)
    cap.set(4, c_height)
    print("resolution is {} x {}".format(cap.get(3), cap.get(4)))
    print("setresolution is {} x {}".format(c_width, c_height))

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
        camera_name = "BSW20KM11BK"
        width = 1280
        height = 720
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
    dlib_detector = DlibFaceDetector(compress=0.2)

    while not rospy.is_shutdown():
        while not newFlag:
            # wait until a new frame is available.
            # the main thread's process should probably always be slower than the camera thread, so in theory this should be bypassed every time.
            if rospy.is_shutdown():
                break
            rospy.sleep(0.05)
        frame = captured_frame.copy()
        newFlag = False

        overlayedImage, center = dlib_detector.dlib_detect(frame)
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