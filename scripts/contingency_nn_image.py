import warnings
warnings.filterwarnings("ignore")
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'  # or any {'0', '1', '2'}

from segments import SegmentsClient
from datasets import load_dataset
from huggingface_hub import hf_hub_download
from torchvision.transforms import ColorJitter
from transformers import SegformerFeatureExtractor, SegformerForSemanticSegmentation, TrainingArguments, Trainer
from torch import nn
import torch
import sys
import time
import signal
import evaluate
from datasets import Dataset
import json
import cv2
from git import Repo
from cv_bridge import CvBridge
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
from spot_calib.spot_calib import SpotCameraCalibration
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
from copy import deepcopy
import math
import argparse


class SegFormerFineTuned:
    def __init__(self,
                 cuda: str = "0",
                 continue_training: bool = False,
                 segments_dataset_name: str = "None",
                 segments_release_name: str = "None",
                 segments_username: str = "None",
                 hf_username: str = "None",
                 segformer_model_id: int = 0,
                 hf_model_ver: int = 0,
                 epochs: int = 1000,
                 lr: float = 5e-5,
                 lambd: float = 0.0,
                 lr_decay: str = "linear",
                 dataloader_num_workers: int = 2,
                 batch_size: int = 16,
                 eval_steps: int = 40):
        self.api_keys = {}
        api_key_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), ".api_keys")
        with open(api_key_path, "r") as file:
            for line in file:
                key, value = line.strip().split(": ")
                self.api_keys[key] = value
        self.segments_key = self.api_keys["segments_key"]
        self.hf_key = self.api_keys["hf_key"]
        self.cuda = cuda
        self.continue_training = continue_training
        self.segments_dataset_name = segments_dataset_name
        self.segments_release_name = segments_release_name
        self.segments_username = segments_username
        self.hf_username = hf_username
        self.segformer_model_id = segformer_model_id
        self.hf_model_ver = hf_model_ver
        self.epochs = epochs
        self.lr = lr
        self.lambd = lambd
        self.lr_decay = lr_decay
        self.dataloader_num_workers = dataloader_num_workers
        self.batch_size = batch_size
        self.eval_steps = eval_steps
        os.environ["CUDA_VISIBLE_DEVICES"] = self.cuda
        model_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "models")
        os.makedirs(model_path, exist_ok=True)
        if torch.cuda.is_available():
            self.device_batch_size = self.batch_size // torch.cuda.device_count()
        else:
            self.device_batch_size = self.batch_size
        self.train_percentd = 0.60  # DO NOT CHANGE THIS TO HAVE SAME test_ds
        self.val_percentd = 0.25  # DO NOT CHANGE THIS TO HAVE SAME test_ds
        self.id2label_filename = "id2label.json"
        self.global_strategy = "steps"

        self.segments_dataset_identifier = f"{self.segments_username}/{self.segments_dataset_name}"
        self.segments_client = SegmentsClient(self.segments_key)
        self.hf_dataset_identifier = f"{self.hf_username}/{self.segments_dataset_name}_{self.segments_release_name}"
        self.pretrained_model_name = f"nvidia/mit-b{self.segformer_model_id}"
        self.hub_model_id = f"segformer-b{self.segformer_model_id}-finetuned-{self.segments_dataset_name}-{self.segments_release_name}-v{self.hf_model_ver}"

        self.id2label = json.load(open(hf_hub_download(repo_id=self.hf_dataset_identifier, filename=self.id2label_filename, repo_type="dataset"), "r"))
        self.id2label = {int(k): v for k, v in self.id2label.items()}
        self.label2id = {v: k for k, v in self.id2label.items()}
        self.num_labels = len(self.id2label)

        self.jitter = ColorJitter(brightness=0.25, contrast=0.25, saturation=0.25, hue=0.1)
        self.metric = evaluate.load("mean_iou")
        self.output_dir = f"{model_path}/{self.hub_model_id}"

        # signal.signal(signal.SIGINT, self.signal_handler)

    def prepare_dataset(self, hf_dataset_identifier: str = None):
        if hf_dataset_identifier is None:
            hf_dataset_identifier = self.hf_dataset_identifier
        self.ds = load_dataset(hf_dataset_identifier)
        self.train_ds, self.val_ds, self.test_ds = SegFormerFineTuned.split_dataset(self.ds, self.train_percentd, self.val_percentd)

    def apply_transforms(self):
        self.train_ds.set_transform(self.train_transforms)
        self.val_ds.set_transform(self.val_transforms)

    def prepare_model(self):
        self.feature_extractor = SegformerFeatureExtractor()
        self.model = SegformerForSemanticSegmentation.from_pretrained(
            self.pretrained_model_name,
            id2label=self.id2label,
            label2id=self.label2id
        )

    def load_model_inference(self):
        self.feature_extractor = SegformerFeatureExtractor.from_pretrained(f"{self.hf_username}/{self.hub_model_id}", use_safetensors=True)
        self.model = SegformerForSemanticSegmentation.from_pretrained(f"{self.hf_username}/{self.hub_model_id}")

    def predict_one(self, pred_ds, idx):
        assert 0 <= idx < len(pred_ds), f"idx must be in range [0, {len(pred_ds)})"
        image = pred_ds[idx]['pixel_values']
        gt_seg = pred_ds[idx]['labels']
        inputs = self.feature_extractor(images=image, return_tensors="pt")
        outputs = self.model(**inputs)
        logits = outputs.logits  # shape (batch_size, num_labels, height/4, width/4)

        # First, rescale logits to original image size
        upsampled_logits = nn.functional.interpolate(
            logits,
            size=image.size[::-1],  # (height, width)
            mode='bilinear',
            align_corners=False
        )
        # Second, apply argmax on the class dimension
        pred_seg = upsampled_logits.argmax(dim=1)[0]

        pred_img = SegFormerFineTuned.get_seg_overlay(image, pred_seg)
        gt_img = SegFormerFineTuned.get_seg_overlay(image, np.array(gt_seg))
        return pred_img, gt_img, pred_seg, gt_seg

    def predict_new(self, image):
        inputs = self.feature_extractor(images=image, return_tensors="pt")
        outputs = self.model(**inputs)
        logits = outputs.logits  # shape (batch_size, num_labels, height/4, width/4)

        # First, rescale logits to original image size
        upsampled_logits = nn.functional.interpolate(
            logits,
            size=image.size[::-1],  # (height, width)
            mode='bilinear',
            align_corners=False
        )
        # Second, apply argmax on the class dimension
        pred_seg = upsampled_logits.argmax(dim=1)[0]

        pred_img = SegFormerFineTuned.get_seg_overlay(image, pred_seg)
        return pred_img, pred_seg

    def predict_ds_metrics(self, pred_ds):
        pred_ds = {
            'pixel_values': np.stack([np.array(img) for img in pred_ds['pixel_values']]),
            'labels': np.stack([np.array(img) for img in pred_ds['labels']])
        }
        images = pred_ds['pixel_values']
        gt_segs = pred_ds['labels']
        inputs = self.feature_extractor(images=images, return_tensors="pt")
        outputs = self.model(**inputs)
        logits = outputs.logits  # shape (batch_size, num_labels, height/4, width/4)

        upsampled_logits = nn.functional.interpolate(
            logits,
            size=images[0].shape[:-1],   # (height, width)
            mode='bilinear',
            align_corners=False
        )

        pred_segs = np.array(upsampled_logits.argmax(dim=1))
        gt_segs_array = np.stack([np.array(img) for img in gt_segs])

        metrics = self.metric._compute(
            predictions=pred_segs,
            references=gt_segs_array,
            num_labels=len(self.id2label),
            ignore_index=0,
            reduce_labels=self.feature_extractor.do_reduce_labels,
        )
        per_category_accuracy = metrics.pop("per_category_accuracy").tolist()
        per_category_iou = metrics.pop("per_category_iou").tolist()
        metrics.update({f"accuracy_{self.id2label[i]}": v for i, v in enumerate(per_category_accuracy)})
        metrics.update({f"iou_{self.id2label[i]}": v for i, v in enumerate(per_category_iou)})
        return metrics

    def predict_ds_metrics_wrapper(self, pred_ds, batch_size=32):
        # Create indices for batches
        num_samples = len(pred_ds['pixel_values'])
        indices = list(range(num_samples))
        np.random.shuffle(indices)  # shuffling may provide a more robust estimate of mean metrics
        batch_indices = [indices[i:i + batch_size] for i in range(0, num_samples, batch_size)]

        metrics_list = []
        batch_sizes = []

        pred_ds_array = {
            'pixel_values': np.stack([np.array(img) for img in pred_ds['pixel_values']]),
            'labels': np.stack([np.array(img) for img in pred_ds['labels']])
        }
        def get_batch(idx): return (pred_ds_array['pixel_values'][idx], pred_ds_array['labels'][idx])

        # Loop over all batches
        for bidx, batch in enumerate(batch_indices):
            print(f"Processing batch {bidx + 1}/{len(batch_indices)}...")
            pixel_values_batch, labels_batch = get_batch(batch)
            batch_ds = {
                'pixel_values': pixel_values_batch,
                'labels': labels_batch
            }
            print(f"Computing metrics for batch {bidx + 1}/{len(batch_indices)}...")
            metrics = self.predict_ds_metrics(batch_ds)
            metrics_list.append(metrics)
            batch_sizes.append(len(batch_ds['pixel_values']))

        # Compute the weighted mean metrics
        mean_metrics = {}
        for key in metrics_list[0].keys():
            mean_metrics[key] = np.sum([metrics[key] * size for metrics, size in zip(metrics_list, batch_sizes)]) / np.sum(batch_sizes)
        return mean_metrics

    @staticmethod
    def labels_color_palette():
        return [
            [0, 0, 0],
            [0, 113, 188],
            [236, 176, 31],
        ]

    @staticmethod
    def get_seg_overlay(image, seg):
        color_seg = np.zeros((seg.shape[0], seg.shape[1], 3), dtype=np.uint8)  # height, width, 3
        palette = np.array(SegFormerFineTuned.labels_color_palette())
        for label, color in enumerate(palette):
            color_seg[seg == label, :] = color

        # Show image + mask (i.e., basically transparency of overlay)
        img = np.array(image) * 0.5 + color_seg * 0.5
        img = img.astype(np.uint8)
        return img

    def train_transforms(self, example_batch):
        images = [self.jitter(x) for x in example_batch['pixel_values']]
        labels = [x for x in example_batch['labels']]
        inputs = self.feature_extractor(images, labels)
        return inputs

    def val_transforms(self, example_batch):
        images = [x for x in example_batch['pixel_values']]
        labels = [x for x in example_batch['labels']]
        inputs = self.feature_extractor(images, labels)
        return inputs

    @staticmethod
    def split_dataset(ds: Dataset, train_percent: float, val_percent: float):
        assert train_percent + val_percent < 1.0, "Train and validation percentages must sum to less than 1."
        ds = ds.shuffle(seed=42)
        train_temp_ds = ds["train"].train_test_split(test_size=1.0 - train_percent, seed=42)
        val_prop = val_percent / (1.0 - train_percent)
        val_test_ds = train_temp_ds["test"].train_test_split(test_size=1.0 - val_prop, seed=42)
        train_ds = train_temp_ds["train"]
        val_ds = val_test_ds["train"]
        test_ds = val_test_ds["test"]
        return train_ds, val_ds, test_ds

    def compute_metrics(self, eval_pred):
        with torch.no_grad():
            logits, labels = eval_pred
            logits_tensor = torch.from_numpy(logits)
            # scale the logits to the size of the label
            logits_tensor = nn.functional.interpolate(
                logits_tensor,
                size=labels.shape[-2:],
                mode="bilinear",
                align_corners=False,
            ).argmax(dim=1)
            pred_labels = logits_tensor.detach().cpu().numpy()
            metrics = self.metric._compute(
                predictions=pred_labels,
                references=labels,
                num_labels=len(self.id2label),
                ignore_index=0,
                reduce_labels=self.feature_extractor.do_reduce_labels,
            )
            # add per category metrics as individual key-value pairs
            per_category_accuracy = metrics.pop("per_category_accuracy").tolist()
            per_category_iou = metrics.pop("per_category_iou").tolist()
            metrics.update({f"accuracy_{self.id2label[i]}": v for i, v in enumerate(per_category_accuracy)})
            metrics.update({f"iou_{self.id2label[i]}": v for i, v in enumerate(per_category_iou)})
            return metrics

    def prepare_trainer(self):
        self.training_args = TrainingArguments(
            output_dir=self.output_dir,
            learning_rate=self.lr,
            lr_scheduler_type=self.lr_decay,
            weight_decay=self.lambd,
            dataloader_num_workers=self.dataloader_num_workers,
            overwrite_output_dir=False,
            evaluation_strategy=self.global_strategy,
            eval_steps=self.eval_steps,
            report_to="tensorboard",  # new version reports to all (including wandb)
            eval_accumulation_steps=5,
            per_device_train_batch_size=self.device_batch_size,
            per_device_eval_batch_size=self.device_batch_size,
            num_train_epochs=self.epochs,
            warmup_ratio=0.05,
            log_level="warning",
            log_level_replica="warning",
            logging_strategy=self.global_strategy,
            logging_first_step=True,
            logging_steps=1,
            save_strategy=self.global_strategy,
            save_steps=4 * self.eval_steps,
            save_total_limit=4,
            load_best_model_at_end=True,
            metric_for_best_model="eval_loss",
            greater_is_better=False,  # metric_for_best_model's behaviour
            ignore_data_skip=False,  # for resuming training from the exact same condition
            optim="adamw_hf",
            push_to_hub=True,
            hub_model_id=self.hub_model_id,
            hub_strategy="all_checkpoints",
            hub_token=self.hf_key,
            hub_private_repo=False,
            auto_find_batch_size=False,
        )

        self.trainer = Trainer(
            model=self.model,
            args=self.training_args,
            train_dataset=self.train_ds,
            eval_dataset=self.val_ds,
            compute_metrics=self.compute_metrics,
        )

    @staticmethod
    def git_pull(repo_path):
        repo = Repo(repo_path)
        if not repo.bare:
            print('Pulling latest changes for repository: {0}'.format(repo_path))
            repo.remotes.origin.pull()
        else:
            print('Cannot pull changes, the repository is bare.')

    def signal_handler(self, sig, frame):
        print("Ctrl+C detected! Stopping training and saving model and pushing to hub...")
        time.sleep(2)
        try:
            self.end_training_cleanly()
        except:
            print("LOGINFO: Error while saving model and pushing to hub!")
            pass
        time.sleep(6)
        sys.exit(0)

    def end_training_cleanly(self):
        self.trainer.save_model()  # so that the best model will get now loaded when you do from_pretrained()
        self.feature_extractor.push_to_hub(self.hub_model_id, use_auth_token=self.hf_key)
        SegFormerFineTuned.git_pull(self.output_dir)
        kwargs = {
            "tags": ["vision", "image-segmentation"],
            "finetuned_from": self.pretrained_model_name,
            "dataset": self.hf_dataset_identifier,
        }
        self.trainer.push_to_hub(**kwargs)
        print("Training terminated cleanly! Model saved and pushed to hub!")

    def train(self):
        if self.continue_training:
            print("Continuing training from the last checkpoint...")
            self.trainer.train(resume_from_checkpoint=True)
        else:
            print("Starting training from scratch...")
            self.trainer.train()
        self.end_training_cleanly()


class PublishSafeSpot:
    def __init__(self, res):
        self.latest_image_msg = None
        self.resolution = res
        self.c = SpotCameraCalibration(resolution=self.resolution)
        self.s = SegFormerFineTuned(cuda="0",
                                    segments_dataset_name="neurosymbolic-contingency-bag1",
                                    segments_release_name="v0.1",
                                    segments_username="smodak",
                                    hf_username="sam1120",
                                    segformer_model_id=0,
                                    hf_model_ver=0)
        self.s.load_model_inference()
        self.cv_bridge = CvBridge()

        self.OUT_FPS = 1.6
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1, buff_size=2**32)
        self.loc_pub = rospy.Publisher("/contingency/safe_pose", Float64MultiArray, queue_size=1)
        self.overlay_pub = rospy.Publisher("/contingency/overlay/compressed", CompressedImage, queue_size=1)
        rospy.Timer(rospy.Duration(1 / self.OUT_FPS), self.processing_callback)
        print("Ready to publish safe spot!")

    def image_callback(self, msg):
        self.latest_image_msg = msg

    def processing_callback(self, event):
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(self.latest_image_msg, desired_encoding="passthrough")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(np.array(cv_image))
        pred_img, pred_seg = self.s.predict_new(pil_image)
        pred_mask = np.where(np.array(pred_seg) == 1, 1, 0)  # 1 for safe now, rest 0
        b1, pixel_x, pixel_y = PublishSafeSpot.deduce_safe_loc(pred_mask)
        b2, orient_1_x, orient_1_y, orient_2_x, orient_2_y = PublishSafeSpot.deduce_safe_angle(pred_mask, pixel_x, pixel_y)

        if (b1 and b2):
            pcs_coords = np.array([
                [pixel_x, pixel_y],
                [orient_1_x, orient_1_y],
                [orient_2_x, orient_2_y]
            ])
            wcs_coords, bool_mask = self.c.projectPCStoWCSground(pcs_coords)
            if np.all(bool_mask):
                loc_w = wcs_coords[0].squeeze()
                orient1_w = wcs_coords[1].squeeze()
                orient2_w = wcs_coords[2].squeeze()
                rel_ang = math.atan((orient2_w[0] - orient1_w[0]) / (orient2_w[1] - orient1_w[1]))
            else:
                loc_w = np.array([0, 0])
                rel_ang = 0
        else:
            loc_w = np.array([0, 0])
            rel_ang = 0

        # publishing 6 floats:
        #       0 -> bool for gotSafePose (1 means true)
        #       1 -> bool for loc (0 -> local, 1 -> ground)
        #       2 -> x
        #       3 -> y
        #       4 -> bool for theta (0 -> local, 1 -> ground)
        #       5 -> theta

        msg = Float64MultiArray()
        msg.data.append(float(b1 and b2))
        msg.data.append(0)
        msg.data.append(loc_w[0])
        msg.data.append(loc_w[1])
        msg.data.append(0)
        msg.data.append(rel_ang)
        self.loc_pub.publish(msg)

        pred_img_cv = cv2.cvtColor(np.array(pred_img), cv2.COLOR_RGB2BGR)
        if b1:
            cv2.circle(pred_img_cv, (pixel_x, pixel_y), 8, (0, 0, 255), -1)
        pub_msg = self.cv_bridge.cv2_to_compressed_imgmsg(pred_img_cv)
        self.overlay_pub.publish(pub_msg)

    @staticmethod
    def deduce_safe_loc(pred_mask):
        """
        Outputs pixel locs of safe loc. Scans from bottom to top to find a safe pixel with padding at all 4 dirns
        Return False and garbage values if not possible
        TOIMPLEMENT
        """
        # DUMMY
        bmask = pred_mask.squeeze()
        for i in range(bmask.shape[0] - 1, -1, -1):  # y
            for j in range(bmask.shape[1] - 1, -1, -1):  # x
                if bmask[i, j] == 1:
                    return True, j, i

        return False, -1, -1

    @staticmethod
    def deduce_safe_angle(bmask, x, y, min_=50):
        """
        For now (later, do a more sophisticated approach), just scans 8 possible dirns and sees where longest length of safe pixels, and returns the pair of opposite dirns with highest sum (min_ reqd)
        Convention, #1 has lower y (ie, upwards)
        Return False and garbage values if not possible
        TOIMPLEMENT
        """
        # DUMMY
        return True, x, y - 1, x, y + 1  # just go forward


if __name__ == '__main__':
    rospy.init_node("contingency_nn_image", anonymous=False)
    parser = argparse.ArgumentParser()
    parser.add_argument("--res", default=1440, type=int, help="Camera resolution 1440 or 1536")
    args = parser.parse_args()
    p = PublishSafeSpot(res=args.res)
    rospy.spin()
