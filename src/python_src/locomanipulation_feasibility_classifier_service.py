#!/usr/bin/env python

from avatar_locomanipulation.srv import *
import rospy

import numpy as np
import tensorflow as tf

class LocomanipulationFeasibilityClassifier:
	def __init__(self):
		self.x = []

	def perform_binary_classification(self, req):
		print "Input Vector:", np.array(req.x)
		res = BinaryClassifierQueryResponse()
		res.y = 0.75
		print "Returning:", res.y
		return res

	def start_classifier_server(self):
	    rospy.init_node('locomanipulation_feasibility_classifier_server')
	    s = rospy.Service('locomanipulation_feasibility_classifier', BinaryClassifierQuery, self.perform_binary_classification)
	    print "Locomanipulation Feasibility Classifier Server Started."
	    rospy.spin()

if __name__ == "__main__":
	classifier = LocomanipulationFeasibilityClassifier()
	classifier.start_classifier_server()