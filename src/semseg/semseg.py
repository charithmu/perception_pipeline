import os
import pprint
from pathlib import Path

import numpy as np
import tensorflow as tf


class SemsegInferencer:
    def __init__(self, multiGPU=False, thread_id=1, num_gpus=1, model="randlanet"):
        # set paths
        self.home_path = str(Path.home())
        self.base_path = self.home_path + "/dev/Open3D-ML"
        self.dateset_path = self.home_path + "/datasets/SmartLab"
        self.ckpt_base_path = self.base_path + "/mytests/logs"
        # smart lab dataset config
        randlanet_smartlab_cfg = self.base_path + "/ml3d/configs/randlanet_smartlab.yml"
        kpconv_smartlab_cfg = self.base_path + "/ml3d/configs/kpconv_smartlab.yml"
        if model == "randlanet":
            self.cfg_file = randlanet_smartlab_cfg
        elif model == "kpconv":
            self.cfg_file = kpconv_smartlab_cfg
        # checkpoints
        self.ckpt_path = self.ckpt_base_path + "/RandLANet_SmartLab_tf/checkpoint/ckpt-6"

        # import custom open3d.ml
        os.environ["OPEN3D_ML_ROOT"] = self.base_path
        global ml3d, _ml3d
        import open3d.ml as _ml3d
        import open3d.ml.tf as ml3d

        # kwargs
        kwargs = {
            "framework": "tf",
            "device": "cuda",
            "dataset_path": self.dateset_path,
            "split": "test",
            "ckpt_path": self.ckpt_path,
            "cfg_file": self.cfg_file,
        }
        self.args = type("args", (object,), kwargs)()
        # pprint.pprint(kwargs)
        self.setupGPUs(multiGPU, thread_id, num_gpus)
        self.dataset, self.model, self.pipeline = self.init_pipeline()

    # merge args into config file
    @staticmethod
    def merge_cfg_file(cfg, args):
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

    # setup GPUs
    def setupGPUs(self, multiGPU=False, thread_id=1, num_gpus=1):
        gpus = tf.config.experimental.list_physical_devices("GPU")
        pprint.pprint(gpus)
        # TF GPU settings
        if gpus is not None:
            print("Setting up GPUs for TF")
            try:
                for gpu in gpus:
                    tf.config.experimental.set_memory_growth(gpu, True)
                if multiGPU:
                    gpu_id = thread_id % num_gpus
                    print("GPU %s selected" % gpu_id)
                    tf.config.set_visible_devices(gpus[gpu_id], "GPU")
                else:
                    tf.config.set_visible_devices(gpus[0], "GPU")
            except RuntimeError as e:
                print(e)

    def init_pipeline(self):
        cfg = _ml3d.utils.Config.load_from_file(self.args.cfg_file)
        cfg_dataset, cfg_pipeline, cfg_model = SemsegInferencer.merge_cfg_file(cfg, self.args)

        Pipeline = _ml3d.utils.get_module("pipeline", cfg.pipeline.name, self.args.framework)
        Model = _ml3d.utils.get_module("model", cfg.model.name, self.args.framework)
        Dataset = _ml3d.utils.get_module("dataset", cfg.dataset.name)

        dataset = Dataset(**cfg_dataset)
        model = Model(**cfg_model)
        pipeline = Pipeline(model, dataset, **cfg_pipeline)

        return dataset, model, pipeline

    def run_inference_pipeline(self, data):
        self.pipeline.load_ckpt(ckpt_path=self.args.ckpt_path)
        results = self.pipeline.run_inference(data, tqdm_disable=True)
        return results

    def get_dataset(self):
        return self.dataset