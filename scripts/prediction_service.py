#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from perception_pipeline.srv import PredictLabels, PredictLabelsRequest, PredictLabelsResponse
import random
import yaml
import numpy as np
import os
import pprint
import time
from pathlib import Path

# set paths
home_path = str(Path.home())
base_path = home_path + "/dev/Open3D-ML"
dateset_path = home_path + "/datasets/SmartLab"
ckpt_base_path = base_path + "/mytests/logs"

# import custom open3d.ml
os.environ["OPEN3D_ML_ROOT"] = base_path
import open3d.ml as _ml3d

# smart lab dataset config
randlanet_smartlab_cfg = base_path + "/ml3d/configs/randlanet_smartlab.yml"
kpconv_smartlab_cfg = base_path + "/ml3d/configs/kpconv_smartlab.yml"

# checkpoints
ckpt_path = ckpt_base_path + "/RandLANet_SmartLab_tf/checkpoint/ckpt-6"

kwargs = {
    "framework": "tf",
    "device": "cuda",
    "dataset_path": dateset_path,
    "split": "test",
    "ckpt_path": ckpt_path,
    "cfg_file": randlanet_smartlab_cfg,
}

# on-the-fly object using kwargs
args = type("args", (object,), kwargs)()
pprint.pprint(kwargs)

# import tensorflow or pytorch
if args.framework == "torch":
    import open3d.ml.torch as ml3d
else:
    import open3d.ml.tf as ml3d
    import tensorflow as tf

gpus = tf.config.experimental.list_physical_devices("GPU")
pprint.pprint(gpus)

# TF GPU settings
if gpus is not None and args.framework == "tf":
    print("Setting up GPUs for TF")
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        if args.device == "cpu":
            tf.config.set_visible_devices([], "GPU")
        elif args.device == "cuda":
            tf.config.set_visible_devices(gpus[0], "GPU")
        else:
            idx = args.device.split(":")[1]
            tf.config.set_visible_devices(gpus[int(idx)], "GPU")
    except RuntimeError as e:
        print(e)

# merge args into config file
def merge_cfg_file(cfg, args, extra_dict):
    if args.device is not None:
        cfg.pipeline.device = args.device
        cfg.model.device = args.device
    if args.split is not None:
        cfg.pipeline.split = args.split
    if args.dataset_path is not None:
        cfg.dataset.dataset_path = args.dataset_path
    if args.ckpt_path is not None:
        cfg.model.ckpt_path = args.ckpt_path

    return cfg.dataset, cfg.pipeline, cfg.model


cfg = _ml3d.utils.Config.load_from_file(args.cfg_file)
cfg_dataset, cfg_pipeline, cfg_model = merge_cfg_file(cfg, args, None)

Pipeline = _ml3d.utils.get_module("pipeline", cfg.pipeline.name, args.framework)
Model = _ml3d.utils.get_module("model", cfg.model.name, args.framework)
Dataset = _ml3d.utils.get_module("dataset", cfg.dataset.name)

dataset = Dataset(**cfg_dataset)
model = Model(**cfg_model)
pipeline = Pipeline(model, dataset, **cfg_pipeline)

pipeline.load_ckpt(ckpt_path=args.ckpt_path)


def run_inferences_online(data):
    print("Running Inferences...")
    results = pipeline.run_inference(data)
    pred = (results["predict_labels"]).astype(np.int32)
    # pred = np.zeros((len(data["point"])))
    return pred


def ros2mldata(ros_cloud):
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(point_cloud2.read_points(ros_cloud, skip_nans=False, field_names=field_names))

    # input has x,y,z,intensity but intensity is not used
    xyz = [(x, y, z) for x, y, z, i in cloud_data]

    points = np.asarray(xyz, dtype=np.float32)
    feat = None
    labels = None

    data = {"point": points, "feat": feat, "label": labels}

    return points, data


def create_pointcloud2(points, labels, stamp=None, frame_id=None):
    """Create a sensor_msgs.PointCloud2 from an array of points and class labels"""

    pointcloud = np.array(np.column_stack([points, labels]))

    header = Header()
    if stamp is None:
        header.stamp = rospy.Time().now()
    else:
        header.stamp = stamp

    if frame_id is None:
        header.frame_id = "world_frame"
    else:
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

    points, mldata = ros2mldata(req.input)
    pred_labels = run_inferences_online(mldata)
    rospc2 = create_pointcloud2(points, pred_labels)

    resp = PredictLabelsResponse()
    resp.output = rospc2
    return resp


def prediction_service():
    rospy.init_node("prediction_service_node", anonymous=True)

    nodename = rospy.get_name()
    rospy.loginfo("%s started..." % nodename)

    instance_id = rospy.get_param("~instance_id", 0)

    service_name = "prediction_service_" + str(instance_id)

    s = rospy.Service(service_name, PredictLabels, prediction_service_call)

    rospy.loginfo("Prediction Service Ready. Name: %s" % service_name)
    rospy.spin()


if __name__ == "__main__":
    prediction_service()