#!/bin/bash

set -e

export AM_I_DOCKER=False
export BUILD_WITH_CUDA=True
export CUDA_HOME=/usr/local/cuda/

echo "Install torch manually"
pip3 install -r requirements.txt
pip3 install --ignore-installed open3d

cd gsam/
python3 -m pip install -e segment_anything
pip3 install --no-build-isolation -e GroundingDINO
mkdir weights/
cd weights/
wget https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth -O groundingdino_swint_ogc.pth
wget https://huggingface.co/lkeab/hq-sam/resolve/main/sam_hq_vit_h.pth -O sam_hq_vit_h.pth
cd ../..

cd depthany2/metric_depth/
mkdir checkpoints/
cd checkpoints/
wget https://huggingface.co/depth-anything/Depth-Anything-V2-Large/resolve/main/depth_anything_v2_vitl.pth -O depth_anything_v2_vitl.pth
wget https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-VKITTI-Large/resolve/main/depth_anything_v2_metric_vkitti_vitl.pth -O depth_anything_v2_metric_vkitti_vitl.pth
wget https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Large/resolve/main/depth_anything_v2_metric_hypersim_vitl.pth -O depth_anything_v2_metric_hypersim_vitl.pth
cd ../../..

pip3 uninstall numpy && pip3 install "numpy<1.26.4" && pip3 install supervision==0.6.0