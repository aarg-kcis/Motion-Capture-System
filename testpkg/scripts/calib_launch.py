#!/usr/bin/env python
import sys
import json
import os
import rospy
import tf
from calibrate import Calibrate
from cam_tracker import CamTracker

def get_cam_to_be_calibrated(cam_pairs):
	cams_to_be_calib = {}
	for i, j in cam_pairs:
		if int(i) not in cams_to_be_calib:
			cams_to_be_calib[int(i)] = len(cams_to_be_calib)
		if int(j) not in cams_to_be_calib:
			cams_to_be_calib[int(j)] = len(cams_to_be_calib)
	return cams_to_be_calib

def close_trackers(trackers):
	rospy.loginfo("Closing Trackers")
	for i in trackers.keys():
		trackers[i].stop()


def calibrate(numcam, num_bots, tcp_ips, tcp_ports, buffer_size, cam_pairs):
	CAM_IDS = [x%10 for x in tcp_ports]
	cams_to_be_calib = get_cam_to_be_calibrated(cam_pairs)
	print 'cams_to_be_calib', cams_to_be_calib
	trackers = {}
	for y, x in cams_to_be_calib.items():
		print x, y
		idx = CAM_IDS.index(y)
		z = CamTracker(y, num_bots, tcp_ips[idx], tcp_ports[idx], buffer_size)
		trackers[y] = z

	for i in trackers.keys():
		trackers[i].thread.start()
	print cam_pairs
	tfs = []
	for i, j in cam_pairs:
		print "======>", i,j
		tf = {}
		tf['from_to'] = (i, j)
		id_i = int(i)
		id_j = int(j)
		print id_j, id_i
		calib_obj = Calibrate(trackers[id_i], trackers[id_j], 'somefilename')
		calib_obj.calibrate()
		try:
			tf['tf'] = calib_obj.tf
		except:
			continue
		print calib_obj.R
		tfs.append(tf)
	close_trackers(trackers)
	return tfs

def generate_calibration_file(cal_file_abs, data, cam_pairs):
	print 'inside generate_calibration_file'
	if not os.path.exists(os.path.dirname(cal_file_abs)):
		try:
			os.makedirs(os.path.dirname(cal_file_abs))
		except OSError as exc:
			if exc.errno != errno.EEXIST:
				raise
	f = open(cal_file_abs, 'w')
	print data
	for i in data:
		i['tf'] = i['tf'].tolist()
	f.write(json.dumps(data))
	f.close()
	msg = 'Calibration file created for CAM_PAIRS:\n'
	for i,j in cam_pairs:
		msg += 'CAM{:s} : CAM{:s}\n'.format(i, j)
	msg += 'at ' + cal_file_abs
	rospy.loginfo(msg)


if __name__ == '__main__':
	rospy.init_node('custom_talker', anonymous=True)

	NUM_CAM = rospy.get_param("~numcam", 3)
	NUM_BOTS = rospy.get_param("~numbot", 6)
	TCP_PORT = rospy.get_param("~ports", "")
	TCP_IP = rospy.get_param("~ips", "")
	CAM_PAIRS = rospy.get_param("~cam_pairs")
	CALIB_FILE = rospy.get_param("~calib_file")

	dir = os.path.dirname(os.path.realpath(__file__)) + '../config/'
	cal_file_abs = dir+CALIB_FILE

	TCP_PORT = map(int, TCP_PORT.split(","))
	 
	TCP_IP = [x.strip() for x in TCP_IP.split(',')]
	CAM_PAIRS = [tuple(x.strip().split(':')) \
		for x in CAM_PAIRS.split(',')]
	BUFFER_SIZE = 1024

	data = calibrate(NUM_CAM, NUM_BOTS, TCP_IP, TCP_PORT, BUFFER_SIZE, CAM_PAIRS)
	generate_calibration_file(cal_file_abs, data, CAM_PAIRS)
