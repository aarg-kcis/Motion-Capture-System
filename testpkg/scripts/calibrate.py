#!/usr/bin/env python
import tf
import sys
import rospy
import numpy as np

class Calibrate:
	def __init__(self, cam_i, cam_j, filename):
		self.cam_i = cam_i
		self.cam_j = cam_j
		self.R = None
		self.T = None
		self.counter = 0
		self.to_exit = False

	def exit(self):
		rospy.loginfo(self.error)
		self.to_exit = True

	def valid_poses(self):
		self.poses_i = self.cam_i.purana_pose
		self.poses_j = self.cam_j.purana_pose
		self.detected_i = self.poses_i.keys()
		self.detected_j = self.poses_j.keys()
		self.counter += 1
		self.detected = set(self.detected_i) & set(self.detected_j)
		if len(self.detected) < 3:
			self.error = "Less than 3 common caliberation points."
			self.error = "for CAM_PAIR:[{:d}, {:d}]".format(self.cam_i.id, self.cam_j.id)
			self.error += "\nCan\'t caliberate"
			return False
		self.detected = list(self.detected)
		return True

	def get_vector(self, pose1, pose2):
		print 'get_vector'
		y = pose1['x']-pose2['x']
		x = pose1['y']-pose2['y']
		print pose1['x'], pose1['y'], pose2['x'], pose2['y']
		print [x, y]
		return [x, y]

	def get_point(self, point):
		print point
		return [point['x'], point['y']]

	def get_corresponding_vectors(self, index):
		print 'get_corresponding_vectors'
		cam_id_1 = self.detected[index]
		cam_id_2 = self.detected[index + 1]
		vec_i = self.get_vector(self.poses_i[cam_id_1], self.poses_i[cam_id_2])
		vec_j = self.get_vector(self.poses_j[cam_id_1], self.poses_j[cam_id_2])
		return (vec_i, vec_j)

	def gen_vectors(self):
		print 'gen_vectors'
		while (not self.valid_poses()) and self.counter < 20000000:
			pass
		if not self.valid_poses():
			self.exit()
			return
		print '>--|--|--|--|--|--<'
		print self.detected_i, self.detected_j, self.detected
		self.vectors = []
		for i in range(2):
			self.vectors.append(self.get_corresponding_vectors(i))
		print self.vectors

	def generate_rotation_mat(self):
		print 'generate_rotation_mat'
		cam_i_mat = [self.vectors[0][0], self.vectors[1][0]]
		cam_j_mat = [self.vectors[0][1], self.vectors[1][1]]
		print cam_i_mat
		print '--------------'
		print cam_j_mat
		print '-------------+'
		cam_i_mat = np.matrix(cam_i_mat).transpose()
		cam_j_mat = np.matrix(cam_j_mat).transpose()
		print cam_i_mat
		print '--------------'
		print cam_j_mat
		print '-------------+'
		self.R = cam_i_mat * np.linalg.inv(cam_j_mat)


	def get_relative_rotation(self):
		print 'get_relative_rotation {:d}, {:d}'.format(self.cam_i.id, self.cam_j.id)
		self.generate_rotation_mat()
		costh = (self.R[0,0] + self.R[1,1])/2.
		sinth = (self.R[1,0] - self.R[0,1])/2.
		self.rotation = np.arctan2(sinth, costh)
		print self.rotation*180/np.pi, '---------------->rot'

	def get_relative_translation(self):
		print 'get_relative_translation'
		permutation_mat = np.matrix([[0, 1], [1, 0]])
		print self.detected, self.poses_i, self.poses_j
		self.T = np.matrix(self.get_point(self.poses_i[self.detected[0]])).T
		self.T = permutation_mat * self.T
		print self.T, '1'
		t = np.matrix(self.get_point(self.poses_j[self.detected[0]])).T
		t = permutation_mat * t
		print t, '2'
		self.T = self.T - self.R * t
		# self.T = permutation_mat * self.T
		print self.T, '<<<<<<<<<<<<<<<<<<<<'

	def gen_transformation(self):
		print 'gen_transformation'
		self.get_relative_rotation()
		self.get_relative_translation()

	def calibrate(self):
		rospy.loginfo('Starting calibration '\
			'for CAM_PAIR:[{:d}, {:d}]'.format(self.cam_i.id, self.cam_j.id))
		self.gen_vectors()
		if self.to_exit:
			return
		self.gen_transformation()
		print self.R
		print self.T
		self.error = 'Calibrated for '\
		'CAM_PAIR:[{:d}, {:d}]'.format(self.cam_i.id, self.cam_j.id)
		self.generate_tf()
		# return self.tf

	def generate_tf(self):
		R = (0, 0, self.rotation)
		T = [self.T[0,0], self.T[1,0], 0]
		self.tf = tf.transformations.compose_matrix(angles=R, translate=T)
		print self.tf

