from __future__ import division
import time
import torch
import torch.nn as nn
from torch.autograd import Variable
import numpy as np
import cv2
from gpu_yolo.util import *
from gpu_yolo.darknet import Darknet
import random, os
import pickle as pkl


class Detector:
    def __init__(self, resolution=416, nms_thresh=0.5, conf_thresh=0.5, debug=False):
        self.debug = debug
        self.cfgfile = "cfg/yolov3.cfg"
        self.weightsfile = "yolov3.weights"
        self.num_classes = 80

        self.conf_thresh = conf_thresh
        self.nms_thresh = nms_thresh
        self.CUDA = torch.cuda.is_available()

        path = os.path.dirname(os.path.abspath(__file__))
        self.classes = open(os.path.join(path, 'data', 'coco.names')).read().split('\n')[:-1]
        self.colors = np.random.randint(0, 255, size=(len(self.classes), 3), dtype='uint8')

        self.model = Darknet(os.path.join(path, 'cfg', 'yolov3.cfg'))
        self.model.load_weights(os.path.join(path, 'yolov3.weights'))

        self.model.net_info["height"] = resolution
        self.inp_dim = int(self.model.net_info["height"])

        if self.CUDA:
            self.model.cuda()

        self.model.eval()


    def detectObjects(self, image):
        org_img = image
        t0 = time.time()
        image, dim = self.prep_image(image)
        t1 = time.time()
        im_dim = torch.FloatTensor(dim).repeat(1,2)

        if self.CUDA:
            im_dim = im_dim.cuda()
            image = image.cuda()

        output = self.model(Variable(image), self.CUDA)
        t2 = time.time()
        output = write_results(output,self.conf_thresh,
                               self.num_classes, nms = True, nms_conf = self.nms_thresh)
        t3 = time.time()


        output[:,1:5] = torch.clamp(output[:,1:5], 0.0, float(self.inp_dim))/self.inp_dim

        output[:,[1,3]] *= org_img.shape[1]
        output[:,[2,4]] *= org_img.shape[0]

        list(map(lambda x: self.draw(x, org_img), output))
        if self.debug:
            print('-----------------------------')
            print('Preprocessing:        {0:.3f} s'.format(t1-t0))
            print('Computing prediction: {0:.3f} s'.format(t2-t1))
            print('Filtering:            {0:.3f} s'.format(t3-t2))
            print(torch.cuda.get_device_name(torch.cuda.current_device()), 'is cuda available: ', torch.cuda.is_available())
            print('Framerate: {0:.2f}'.format(1/(t3-t0)))

        return output, org_img

    def prep_image(self, img):
        """
        Prepare image for inputting to the neural network.

        Returns a Variable
        """
        dim = img.shape[1], img.shape[0]
        img = cv2.resize(img, (self.inp_dim, self.inp_dim))
        img_ = img[:,:,::-1].transpose((2,0,1)).copy()
        img_ = torch.from_numpy(img_).float().div(255.0).unsqueeze(0)
        return img_ , dim

    def draw(self, x, img):
        c1 = x[1:3].int()
        c2 = x[3:5].int()
        cls = int(x[-1])
        label = "{0}".format(self.classes[cls])
        color = self.colors[cls].tolist()
        cv2.rectangle(img, tuple(c1), tuple(c2),color, 1)
        t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 1 , 1)[0]
        c2 = c1[0] + t_size[0] + 3, c1[1] + t_size[1] + 4
        cv2.putText(img, label, (c1[0], c1[1] + t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 1, [225,255,255], 1);
        return img


if __name__ == '__main__':
    det = Detector()
    img = cv2.imread('./det_messi.jpg')
    res = det.detectObjects(img)
    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.killAllWindows()





