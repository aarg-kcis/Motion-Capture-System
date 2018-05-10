#!/usr/bin/env python
import json
import rospy
import os
import numpy as np
from collections import defaultdict

class TFWrapper:
	def __init__(self, filename, mother_cam_id, cam_ids):
		self.filename = filename
		self.cam_ids = cam_ids
		self.tfs = {}
		self.graph = defaultdict(list)
		self.mother = mother_cam_id

	def get_data_from_file(self):
		dir = os.path.dirname(os.path.realpath(__file__)) + '../config/'
		cal_file_abs = dir + self.filename
		try:
			f = open(cal_file_abs, 'r')
		except:
			rospy.loginfo('Error opening calib file.')
		self.content = json.loads(f.read())
		for i in self.content:
			i['tf'] = np.matrix(i['tf'])
		self.vertices()
		self.create_graph()

	def vertices(self):
		self.vertices = set([])
		for i in self.content:
			u, v = map(int, i['from_to'])
			self.vertices.add(u)
			self.vertices.add(v)
		self.V = len(self.vertices)

	def create_graph(self):
		for i in self.content:
			u, v = map(int, i['from_to'])		
			self.graph[u].append(v)
			self.graph[v].append(u)
		
	def get_tf_from_path(self, path):
		print 'path', path
		tf = np.matrix(np.identity(4))
		error = False
		for i in range(len(path)-1):
			from_ = str(path[i])
			to_ = str(path[i+1])
			print from_, to_
			i_tf = next((j['tf'] for j in self.content 	if j['from_to'] == [to_, from_]), None)
			print '---'
			print i_tf
			if  type(i_tf) is type(None):
				i_tf = next((j['tf'] for j in self.content if j['from_to'] == [from_, to_]), None)
				i_tf = np.linalg.inv(i_tf)
			if  type(i_tf) is type(None):
				error = True
				break
			tf = np.matrix(i_tf) * tf
		if  error:
			rospy.loginfo("Can\'t find transformatio"\
				" form CAM{:d} to CAM{:d}".format(from_, to_))
		print tf
		return tf


	def create_tfs(self):
		for i in self.cam_ids:
			if i == self.mother:
				continue
			path = self.get_path_from_s2d(i, self.mother)
			self.tfs[i] = self.get_tf_from_path(path)

	def get_path_to_dest(self, u, d, visited, path):
		print path, u
		visited[u]= True
		path.append(u)
		if u == d:
			return path
		else:
			for i in self.graph[u]:
				if visited[i]==False:
					return self.get_path_to_dest(i, d, visited, path)
		path.pop()
		visited[u]= False

	def get_path_from_s2d(self,s, d):
		print s, 'to', d
		visited = {}
		for i in self.vertices:
			visited[i] = False
		path = []
		path = self.get_path_to_dest(s, d, visited, path)
		if not path:
			return None
		return path

if __name__ == '__main__':
	tfw = TFWrapper('somefilename', 0, [3])
	tfw.get_data_from_file()
	print tfw.graph
	tfw.create_tfs()
	print tfw.tfs


