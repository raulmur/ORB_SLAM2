import time, sys, os, math
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2


import ImageFile

model = 'plumb_bob'

D = [0.09588004493901636, -0.2751577208590459, 0.0007265931787498115, -0.007979244347949572, 0.0]
K = [627.4842111637764, 0.0, 301.42612443545056, 0.0, 626.5832675966146, 249.65402637073538, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [629.0787353515625, 0.0, 296.5435491975986, 0.0, 0.0, 632.731201171875, 249.43851519734744, 0.0, 0.0, 0.0, 1.0, 0.0]

def GetImageFromFile(im_path, id ,rate = 10.):
	im = cv2.imread(im_path,0)
	bridge = CvBridge()
	im_stamp =  id*(1/rate)
	Stamp = rospy.rostime.Time.from_sec(im_stamp)
	#Img = Image()
	#Img.header.stamp = Stamp
	#Img.width = im.shape[1]
	#Img.height = im.shape[0]
	if len(im.shape) < 3:
		Img = bridge.cv2_to_imgmsg(im, 'mono8')
	elif im.shape[2] == 3:
                Img = bridge.cv2_to_imgmsg(im, 'bgr8')
#	Img.step=Img.width #some nodes may complains ...
	Img.header.frame_id = "camera"
#	Img_data = list(im) #works for mono channels images (grayscale)
	#Img_data = [pix for pix in im.getdata()]
	#  Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
#	Img.data = Img_data
	return (im_stamp, Stamp, Img)

def GetCameraInfo(width, height):
	cam_info = CameraInfo()
	cam_info.width = width
	cam_info.height = height
	cam_info.distortion_model = model
	#cam_info.D = [0.0]*5
	#cam_info.K = [0.0]*9
	#cam_info.R = [0.0]*9
	#cam_info.P = [0.0]*12
	cam_info.D = D
        cam_info.K = K
        cam_info.R = R
        cam_info.P = P
	cam_info.binning_x = 0
	cam_info.binning_y = 0
	return cam_info

def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
    all = []
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            files.sort()
            for f in files:
                if os.path.splitext(f)[1] in ['.png', 'jpg', 'pgm']:
                    all.append( os.path.join( path, f ) )
    print 'found '+str(len(all))+' images'
    return all

def CreateMonoBag(imgs,bagname, rate = 10):
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')
    line = "%"
    try:
	print 'Complete 0% \r',
        for i in range(len(imgs)):
            #print("Adding %s" % imgs[i])
            (im_stamp, Stamp, Img) = GetImageFromFile(imgs[i], i, rate )
            print 'Complete '+str((float(i)/float(len(imgs)))*100.0)+'% \r' ,
            cam_info = GetCameraInfo(Img.width, Img.height)
            bag.write('camera/camera_info', cam_info, Stamp)
            bag.write('camera/image_raw', Img, Stamp)
        print 'Complete 100% \n\r'
    finally:
        bag.close()


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    all_imgs = GetFilesFromDir(args[0])
    if len(all_imgs) <= 0:
        print("No images found in %s" % args[0])
        exit()
    CreateMonoBag(all_imgs, args[2], float(args[1]))

if __name__ == "__main__":
    if len( sys.argv ) == 4:
        CreateBag(sys.argv[1:])
    else:
        print( "Usage: img2bag imagedir image_rate bagfilename")
