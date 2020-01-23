import os
import cv2
import torch
import torchvision
import torch.nn.functional as F
import rospy
import numpy as np
from rospkg import RosPack
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray


class DetectCollision(object):
    def __init__(self, model, image_topic_in, classification_topic_out, width=224, height=224):
        self.__device = torch.device('cuda')
        self.__model = model.to(self.__device)
        mean = 255.0 * np.array([0.485, 0.456, 0.406])
        stdev = 255.0 * np.array([0.229, 0.224, 0.225])
        self.__normalize = torchvision.transforms.Normalize(mean, stdev)
        rospy.Subscriber(image_topic_in, Image,
                         self.__image_callback, queue_size=1)
        self.__pub_classification = rospy.Publisher(
            classification_topic_out, Float32MultiArray, queue_size=1)
        self.__width = width
        self.__height = height
        self.__bridge = CvBridge()

    def __preprocess(self, camera_value):
        x = cv2.resize(camera_value, (self.__width, self.__height))
        x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
        x = x.transpose((2, 0, 1))
        x = torch.from_numpy(x).float()
        x = self.__normalize(x)
        x = x.to(self.__device)
        x = x[None, ...]
        return x

    def __image_callback(self, data):
        try:
            cv_image = self.__bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = self.__preprocess(cv_image)
            y = self.__model(cv_image)
            y = F.softmax(y, dim=1)
            probs = y.flatten()
            rospy.logdebug(probs)
            msg = Float32MultiArray()
            msg.data = probs
            self.__pub_classification.publish(msg)
        except Exception as e:
            rospy.logerr(e)

    def spin(self):
        pass


def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)
    model_default = RosPack().get_path('oit_navigation_microbot_01') + \
        '/models/collision_avoidance/best_model.pth'
    image_topic_in = "/jetbot_camera_image_modifier/mod_image"
    classification_topic_out = "~classification"
    process_rate = rospy.get_param("~process_rate", 20.0)
    width = rospy.get_param("~width", 224)
    height = rospy.get_param("~height", 224)
    model_path = rospy.get_param("~model_path", model_default)
    try:
        rospy.loginfo("%s:Loading DNN model from %s..." %
                      (node_name, model_path))
        model = torchvision.models.alexnet(pretrained=False)
        model.classifier[6] = torch.nn.Linear(
            model.classifier[6].in_features, 2)
        model.load_state_dict(torch.load(model_path))
        rospy.loginfo("%s:Done." % (node_name))
    except Exception as e:
        rospy.logerr(e)
        return
    node = DetectCollision(model, image_topic_in,
                           classification_topic_out, width, height)
    rate = rospy.Rate(process_rate)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()

