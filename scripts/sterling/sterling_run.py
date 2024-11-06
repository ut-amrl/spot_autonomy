import numpy as np
import torch
import torch.nn as nn
from termcolor import cprint
import cv2
import torch.nn.functional as F
import argparse
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class CostNet(nn.Module):
    def __init__(self, latent_size=64):
        super(CostNet, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(latent_size, latent_size // 2), nn.BatchNorm1d(latent_size // 2), nn.ReLU(),
            nn.Linear(latent_size // 2, 1), nn.Sigmoid(),  # nn.ReLU(), #nn.Softplus(),
        )

    def forward(self, x):
        return self.fc(x)


class VisualEncoderModel(nn.Module):
    def __init__(self, latent_size=64):
        super(VisualEncoderModel, self).__init__()

        self.block1 = nn.Sequential(
            nn.Conv2d(3, 8, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(8),  # output shape : (batch_size, 8, 64, 64),
            nn.MaxPool2d(kernel_size=2, stride=2),  # output shape : (batch_size, 8, 32, 32),
        )

        self.skipblock = nn.Sequential(
            nn.Conv2d(8, 8, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(8),  # output shape : (batch_size, 8, 32, 32),
            nn.Conv2d(8, 8, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(8),  # output shape : (batch_size, 8, 32, 32),
        )

        self.block2 = nn.Sequential(
            nn.Conv2d(8, 16, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(16),  # output shape : (batch_size, 16, 32, 32),
            nn.MaxPool2d(kernel_size=2, stride=2),  # output shape : (batch_size, 16, 16, 16),
        )

        self.skipblock2 = nn.Sequential(
            nn.Conv2d(16, 16, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(16),  # output shape : (batch_size, 16, 16, 16),
            nn.Conv2d(16, 16, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(16),  # output shape : (batch_size, 16, 16, 16),
        )

        self.block3 = nn.Sequential(
            nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(32),  # output shape : (batch_size, 32, 16, 16),
            nn.AvgPool2d(kernel_size=2, stride=2),  # output shape : (batch_size, 32, 8, 8),
        )

        self.skipblock3 = nn.Sequential(
            nn.Conv2d(32, 32, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(32),  # output shape : (batch_size, 32, 8, 8),
            nn.Conv2d(32, 32, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(32),  # output shape : (batch_size, 32, 8, 8),
        )

        self.block4 = nn.Sequential(
            nn.Conv2d(32, 64, kernel_size=5, stride=3, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(64),  # output shape : (batch_size, 64, 2, 2),
        )

        self.skipblock4 = nn.Sequential(
            nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(64),  # output shape : (batch_size, 64, 2, 2),
            nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=1, bias=False), nn.ReLU(), nn.BatchNorm2d(64),  # output shape : (batch_size, 64, 2, 2),
        )

        self.fc = nn.Linear(256, latent_size)

    def forward(self, x):
        x = self.block1(x)
        x = self.skipblock(x) + x
        x = self.block2(x)
        x = self.skipblock2(x) + x
        x = self.block3(x)
        x = self.skipblock3(x) + x
        x = self.block4(x)
        x = self.skipblock4(x) + x
        x = x.view(x.size(0), -1)  # flattened to (batch_size, 256)

        x = self.fc(x)

        # normalize
        x = F.normalize(x, dim=-1)

        return x


class CostVisualizer:
    def __init__(self, model_path):
        self.model_path = model_path
        visual_encoder = VisualEncoderModel()
        cost_net = CostNet()
        self.model = nn.Sequential(visual_encoder, cost_net)

        # load weights of model
        model_state_dict = torch.load(self.model_path)
        self.model.load_state_dict(model_state_dict)
        self.model.eval()
        cprint('Model loaded', 'green')

    @torch.inference_mode()
    def forward(self, bevimage: torch.Tensor, stride: int = 1):
        """ 
        Args:
            bevimage: [C, H, W]
            stride: stride of the sliding window
        """
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        with torch.device(device):
            patches = bevimage.unfold(0, 3, 3).unfold(1, 64, stride).unfold(2, 64, stride)
            patches = patches.contiguous().view(-1, 3, 64, 64)

            # resize patches to 64x64
            # patches = F.interpolate(patches, size=(64, 64), mode='bilinear', align_corners=True)
            cost = self.model(patches)

            # find patches with sum of pixels == 0 and set their cost to 0
            idx = torch.sum(patches, dim=(1, 2, 3)) == 0
            cost[idx] = 0

            # costm = cost.view(704//stride, 1408//stride)
            costm = cost.view((704 - 64) // stride + 1, (1408 - 64) // stride + 1)

            cost = F.interpolate(costm.unsqueeze(0).unsqueeze(0), size=(704, 1408), mode='nearest')
            # cost = F.interpolate(costm.unsqueeze(0).unsqueeze(0), size=(704, 1408), mode='bilinear', align_corners=True)
        return cost


class ImageProcessor:
    def __init__(self, model_path):
        self.model_path = model_path
        self.max_val = 6.0
        self.costviz = CostVisualizer(self.model_path)
        self.bridge = CvBridge()

        # Publisher
        self.pub_cost = rospy.Publisher('/sterling/costmap/compressed', CompressedImage, queue_size=1)
        self.pub_stacked = rospy.Publisher('/sterling/stacked/compressed', CompressedImage, queue_size=1)
        # Subscriber
        rospy.Subscriber('/bev/single/compressed', CompressedImage, self.callback)

    def callback(self, msg):
        try:
            # Convert compressed image to cv2 image using cv_bridge
            curr_bev_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Convert BGR to RGB
            curr_bev_img = cv2.cvtColor(curr_bev_img, cv2.COLOR_BGR2RGB)  # img size is (749, 1457, 3)

            # remove the bottom and right part of the image to get a size of (704, 1408, 3)
            curr_bev_img = curr_bev_img[:704, :1408, :]   # (64*11, 64*22, 3)
            bevimage = curr_bev_img.copy()  # Copy for stacking later

            # Preprocess image for model input
            curr_bev_img = curr_bev_img.transpose(2, 0, 1).astype(np.float32) / 255.0
            curr_bev_img = torch.from_numpy(curr_bev_img)

            # Forward pass through the model
            cost = self.costviz.forward(curr_bev_img, stride=64).squeeze(0).squeeze(0)
            cost = cost.detach().cpu().numpy()
            cost = (cost * 255.0 / self.max_val).astype(np.uint8)
            cost = cv2.cvtColor(cost, cv2.COLOR_GRAY2RGB)
            cost = cv2.resize(cost, (1408, 704))

            # Stack original and cost images side by side
            stacked_img = np.hstack((bevimage, cost))
            stacked_img = cv2.cvtColor(stacked_img, cv2.COLOR_RGB2BGR)  # Convert back to BGR for encoding

            ret, buffer = cv2.imencode('.jpg', cost)
            if ret:
                comp_msg = CompressedImage()
                comp_msg.header = msg.header
                comp_msg.format = 'jpeg'
                comp_msg.data = buffer.tobytes()
                self.pub_cost.publish(comp_msg)
            ret, buffer = cv2.imencode('.jpg', stacked_img)
            if ret:
                comp_msg = CompressedImage()
                comp_msg.header = msg.header
                comp_msg.format = 'jpeg'
                comp_msg.data = buffer.tobytes()
                self.pub_stacked.publish(comp_msg)
        except Exception as e:
            rospy.logerr("Error processing image: {}".format(e))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_path', type=str, default='./scripts/sterling/models/cost_model.pt')
    args = parser.parse_args()

    rospy.init_node('sterling', anonymous=True)
    processor = ImageProcessor(model_path=args.model_path)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
