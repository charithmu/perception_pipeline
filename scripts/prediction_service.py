#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from perception_pipeline.srv import PredictLabels, PredictLabelsRequest, PredictLabelsResponse
import time
import numpy as np
from semseg.semseg import SemsegInferencer

instance_id = 1
ss = None


def roscloud_to_mldata(ros_cloud):
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(point_cloud2.read_points(ros_cloud, skip_nans=False, field_names=field_names))

    # input has x,y,z,intensity but intensity is not used
    xyz = [(x, y, z) for x, y, z, i in cloud_data]

    points = np.asarray(xyz, dtype=np.float32)
    feat = None
    labels = None

    data = {"point": points, "feat": feat, "label": labels}

    return points, data


def ros_create_pointcloud2(points, labels, stamp=None, frame_id="world_frame"):
    """Create a sensor_msgs.PointCloud2 from an array of points and class labels"""

    pointcloud = np.array(np.column_stack([points, labels]))

    header = Header()
    if stamp is None:
        header.stamp = rospy.Time().now()
    else:
        header.stamp = stamp

    header.frame_id = frame_id

    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("label", 12, PointField.FLOAT32, 1),
    ]

    pc2 = point_cloud2.create_cloud(header, fields, pointcloud)
    return pc2


def prediction_service_call(req):

    points, mldata = roscloud_to_mldata(req.input)

    st = time.perf_counter()
    results = ss.run_inference_pipeline(mldata)
    et = time.perf_counter()

    du = et - st
    # rospy.loginfo(f"Prediction took {du:0.6f} seconds.")

    pred_labels = (results["predict_labels"]).astype(np.int32)

    rospc2 = ros_create_pointcloud2(points, pred_labels)

    resp = PredictLabelsResponse()
    resp.output = rospc2
    return resp


def prediction_service():
    rospy.init_node("prediction_service_node", anonymous=True)

    nodename = rospy.get_name()
    rospy.loginfo("%s Started." % nodename)

    model_name = rospy.get_param("~model", "randlanet")
    rospy.loginfo("Using Deep Neural Network: %s" % model_name)

    global instance_id
    instance_id = rospy.get_param("~instance_id", 1)

    service_name = "prediction_service_" + str(instance_id)

    global ss
    ss = SemsegInferencer(multiGPU=False, thread_id=int(instance_id), num_gpus=1, model=model_name)

    s = rospy.Service(service_name, PredictLabels, prediction_service_call)

    rospy.loginfo("Prediction Service Ready. Name: %s" % service_name)
    rospy.spin()


if __name__ == "__main__":
    prediction_service()