#!/usr/bin/env python3

###############################################################################
#JPAlves 2023 
#Fiducial Marker Detection and Tracking with Deep Learning and OpenCV for ROS2
#Baseado no trabalho de Yongtao Hu 
###############################################################################
#DeepTag: A General Framework for Fiducial Marker Design and Detection 
#IEEE TRANSACTIONS ON PATTERN ANALYSIS AND MACHINE INTELLIGENCE
#March 2023, pp. 2931-2944, vol. 45
###############################################################################

#import sys
import rclpy
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import message_filters
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster
from deeptag_model_setting import load_deeptag_models
from marker_dict_setting import load_marker_codebook
from stag_decode.detection_engine import DetectionEngine 
import json
import os
import argparse
import math
from ament_index_python.packages import get_package_share_directory
 
class DeepTag:	
	def __init__(self,args=None):
		qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=5)
                                          
		rclpy.init(args=args)
		self.node = rclpy.create_node('deep_tag')
		deeptag_ros_dir = get_package_share_directory('deeptag_ros')
		self.node.declare_parameter('config',os.path.join(deeptag_ros_dir, "config", "config.json"))
		config_filename = self.node.get_parameter('config').get_parameter_value().string_value
		load_config_flag = False
		try:
			config_dict = json.load(open(config_filename, 'r'))
			self.cameraMatrix = []
			self.distCoeffs   = []
			self.tag_real_size_in_meter = config_dict['marker_size']
			self.is_video = config_dict['is_video']!=0
			self.filename = config_dict['filepath']
			self.tag_family = config_dict['family']
			self.codebook_filename  = config_dict['codebook'] if len(config_dict['codebook']) else os.path.join(deeptag_ros_dir, 'codebook', self.tag_family + '_codebook.txt')
			self.hamming_dist = config_dict['hamming_dist']
			camera_info_topic = config_dict['camera_info_topic']
			camera_topic = config_dict['camera_topic']

			load_config_flag = True
		except:
			print('Cannot load config: %s'% config_filename)

		self.image_sub   =  message_filters.Subscriber(self.node,CompressedImage,camera_topic + '/compressed', qos_profile=qos_profile)
		self.image_info  =  message_filters.Subscriber(self.node,CameraInfo,camera_info_topic, qos_profile=qos_profile)
		self.ts = message_filters.Cache(self.image_sub, 10)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.image_info],100,0.001)
		self.ts.registerCallback(self.callback)
		self.tf_broadcaster = TransformBroadcaster(self.node)
		
		self.device = None
		load_model_flag = False
		try:
			self.model_detector, self.model_decoder, self.device, self.tag_type, self.grid_size_cand_list = load_deeptag_models(self.tag_family, self.device) 
			load_model_flag = True
		except:
			print('Cannot load models.')
		
		load_codebook_flag = False
		try:
			self.codebook = load_marker_codebook(self.codebook_filename, self.tag_type)
			load_codebook_flag = True
		except:
			print('Cannot load codebook: %s'% self.codebook_filename)
		
		self.inicia = False
		if load_config_flag and load_codebook_flag and load_model_flag:
			# initialize detection engine
			self.inicia = True

	def callback(self,rgb_msg,camera_info):
		rgb_image         = np.frombuffer(bytes(rgb_msg.data), np.uint8)
		rgb_image         = cv2.imdecode(rgb_image,cv2.IMREAD_COLOR)
		
		camera_info_K_l   = np.array(camera_info.k).reshape([3, 3])
		camera_info_D     = np.array(camera_info.d)
		rgb_undist1       = cv2.undistort(rgb_image, camera_info_K_l, camera_info_D)
		self.cameraMatrix = camera_info_K_l
		self.distCoeffs   = camera_info_D
		
		if self.inicia: 		
			self.stag_image_processor = DetectionEngine(self.model_detector, self.model_decoder, self.device, self.tag_type, self.grid_size_cand_list, 
						stg2_iter_num    = 2, # 1 or 2
						min_center_score = 0.2, min_corner_score = 0.2, # 0.1 or 0.2 or 0.3
						batch_size_stg2  = 4, # 1 or 2 or 4
						hamming_dist     = self.hamming_dist, # 0, 2, 4
						cameraMatrix     = self.cameraMatrix, distCoeffs=  self.distCoeffs, codebook = self.codebook,
						tag_real_size_in_meter_dict = {-1:self.tag_real_size_in_meter})
			self.inicia = False
                    	
		decoded_tags = self.stag_image_processor.process(rgb_undist1, detect_scale=None)
		self.stag_image_processor.print_timming()
		for decoded_tag in decoded_tags:
			if not decoded_tag['is_valid']: continue
			
			Rr,_ = cv2.Rodrigues(decoded_tag['rvecs'])
			T         = np.eye(4)
			T[:3, :3] = Rr
			for i in range(3):
				T[i,3] = decoded_tag['tvecs'][i]
			
			r = R.from_matrix(Rr)
			q = r.as_quat()
			
			tf_ = TransformStamped()
			tf_.header = rgb_msg.header
			
			tf_.transform.translation.x = T[0, 3]
			tf_.transform.translation.y = T[1, 3]
			tf_.transform.translation.z = T[2, 3]
			
			tf_.transform.rotation.x = q[0]
			tf_.transform.rotation.y = q[1]
			tf_.transform.rotation.z = q[2]
			tf_.transform.rotation.w = q[3]
			tf_.child_frame_id = 'tag_' + str(decoded_tag['tag_id'])
			self.tf_broadcaster.sendTransform(tf_)			
			
		c = self.stag_image_processor.visualize(is_pause= not self.is_video)
		if cv2.waitKey(1) and c == ord(' '):
      			print('asneira')

if __name__ == '__main__':
	ic = DeepTag()
	try:
		while rclpy.ok():
			rclpy.spin_once(ic.node)

		ic.node.destroy_node()
		rclpy.shutdown()
	except KeyboardInterrupt:
		print("Shutting down")
        
