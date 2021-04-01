#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from perception_pipeline.srv import PredictLabels, PredictLabelsRequest, PredictLabelsResponse

import random
import yaml
import numpy as np


def callback(data):

    print("In callback")

    rospy.wait_for_service("prediction_service")

    prediction_service = rospy.ServiceProxy("prediction_service", PredictLabels)

    request = PredictLabelsRequest()
    request.input.data = "charith mung"

    try:
        response = prediction_service(request)
        print("I got response:" + response.output.data)

    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))


if __name__ == "__main__":

    point_pub = rospy.Publisher("/pointcloud_segmented", PointCloud2, queue_size=1)
    subscriber = rospy.Subscriber("/pointcloud_input", PointCloud2, callback, queue_size=1)

    rospy.init_node("pointcloud_segmenter", anonymous=False)
    rospy.spin()
