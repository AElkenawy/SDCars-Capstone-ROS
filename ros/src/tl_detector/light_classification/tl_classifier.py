# This code is based on guidelines from repo 
# https://github.com/DruidKuma/Self-Driving-Car-ND-Capstone/tree/master/ros/src/tl_detector

from styx_msgs.msg import TrafficLight
import tensorflow as tf
import os
import rospy
import numpy as np


TL_THREHOLD = 0.5
NUM_CLASSES = 4

# Importing Single Shot MultiBox Detector frozen graph
MODEL_PATH = '/SSD_model/frozen_inference_graph.pb'

class TLClassifier(object):

    def __init__(self):
        curr_dir = os.path.dirname(os.path.realpath(__file__))

        model_path = curr_dir + MODEL_PATH
        
        # TL labels dictionary
        green = {'id': 1, 'name': 'Green'}
        red = {'id': 2, 'name': 'Red'}
        yellow = {'id': 3, 'name': 'Yellow'}
        self.label_dict = {1: green, 2: red, 3: yellow}
        
        # Calling build model function
        self.build_model_graph(model_path)
    
    # Function to build the SSD model from frozen model (.pb)
    def build_model_graph(self, model_path):
        self.model_graph = tf.Graph()
        with self.model_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                saved_graph = fid.read()
                od_graph_def.ParseFromString(saved_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.sess = tf.Session(graph=self.model_graph)
        
        # Tensors declaration
        self.image_tensor = self.model_graph.get_tensor_by_name('image_tensor:0')
        self.scores = self.model_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.model_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.model_graph.get_tensor_by_name('num_detections:0')    
        self.boxes = self.model_graph.get_tensor_by_name('detection_boxes:0')

    # Function to detect traffic lights based on scores thresholding
    def detect_traffic_light(self, scores, biggest_score_idx, classes, detected_light):
        
        # Logging messages to terminal with the detected light color
        if scores[biggest_score_idx] > TL_THREHOLD:
            rospy.logwarn("Traffic light is: {}".format(self.label_dict[classes[biggest_score_idx]]['name']))
            
            if classes[biggest_score_idx] == 1:
                detected_light = TrafficLight.GREEN
                
            elif classes[biggest_score_idx] == 2:
                detected_light = TrafficLight.RED
                
            elif classes[biggest_score_idx] == 3:
                detected_light = TrafficLight.YELLOW
        
        # Logged message in case of no light/not classified light
        else:
            rospy.logwarn("Not classified")
            
        return detected_light

    # Function to determine light states due to classes scores 
    def get_classification(self, image):

        detected_light = TrafficLight.UNKNOWN
        
        # Expand dimensions by adding a dimension to match the placeholder
        image_expanded = np.expand_dims(image, axis=0)
        
        
        with self.model_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run([self.boxes, self.scores, self.classes, self.num_detections], 
                                                          feed_dict={self.image_tensor: image_expanded})
            
        scores = np.squeeze(scores)

        return self.detect_traffic_light(scores, scores.argmax(), np.squeeze(classes).astype(np.int32), detected_light)