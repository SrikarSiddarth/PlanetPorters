#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, String

class detectProbe():
	def __init__(self):
		rospy.init_node('detect_probe')#,
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/uav_camera_down/image_raw", Image, self.img_callback)
		self.pos_pub = rospy.Publisher("/probe_pose", Point, queue_size = 10)
		self.area_pub = rospy.Publisher("/probe_area", Float64, queue_size = 10)
		self.probe_pose = Point()
		self.area = 0						# area of the probe
		self.probe_pose.z = 1			#using z as a flag for the visbility of target: z=0 means object in sight, else lost target
		self.rec = None
		

	def img_callback(self, data):
		try:
			self.rec = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)


	def run(self):
		self.area = 0
		self.probe_pose.x = 0
		self.probe_pose.y = 0
		self.probe_pose.z = 1
		if self.rec is not None:
			# print(self.rec.shape)				= (480, 640, 3)
			hsv = cv2.cvtColor(self.rec, cv2.COLOR_BGR2HSV)
			# im = np.copy(hsv)
			
			low_red1 = np.array([160,100,100])
			up_red1 = np.array([179,255,255])
			maskRed1 = cv2.inRange(hsv, low_red1, up_red1)
			low_red2 = np.array([0,100,100])
			up_red2 = np.array([10,255,255])
			maskRed2 = cv2.inRange(hsv, low_red2, up_red2)
			mask = cv2.addWeighted(maskRed1, 1, maskRed2, 1, 0)
			# else:
			# 	low = np.array([0,0,0])
			# 	up = np.array([255, 255, 10])
			# 	mask = cv2.inRange(hsv, low, up)
			_, Cnt, _ = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_SIMPLE)
			# print('length of contour : ' + str(len(Cnt)), self.flag)
			area = []
			for i in range(len(Cnt)):
				area.append(cv2.contourArea(Cnt[i]))
				# if self.flag ==1:
				
				# 	print('contour area : ' + str(cv2.contourArea(Cnt[i])))
				# 	cv2.drawContours(im, [Cnt[i]], 0, (0,0,0), 3)
				# 	cv2.imshow('t', im)
				# 	cv2.waitKey(4)
			if len(area)>0:
				i = np.argmax(area)
				self.area = cv2.contourArea(Cnt[i])
				Mr= cv2.moments(Cnt[i])
				if Mr['m00']!=0:
					xr = (Mr['m10']/Mr['m00'])
					yr = (Mr['m01']/Mr['m00'])
					self.probe_pose.x = xr
					self.probe_pose.y = yr
					self.probe_pose.z = 0
					# if self.flag == 1:

					# 	cv2.circle(self.rec, (int(xr), int(yr)), 3, (200,255,255), -1)
					# 	self.rec = cv2.putText(self.rec, str((xr,yr)), (int(xr),int(yr)),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 255,255), 1, cv2.LINE_AA )
					# 	cv2.imshow('t', self.rec)
					# 	cv2.waitKey(4)
					
					
				else:
					# target out of sight
					pass
			print(self.area)
			self.area_pub.publish(self.area)	
			self.pos_pub.publish(self.probe_pose)
				

if __name__ == '__main__':
	p = detectProbe()
	r = rospy.Rate(30)
	while not rospy.is_shutdown():
		p.run()
		r.sleep()
