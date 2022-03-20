from raf import node
import rospy
import time
import numpy as np
import cv2
import torch
from torchvision import models
from torch import nn
import torchvision

device = torch.device('cuda:0')

print(torch.__version__)
print(torchvision.__version__)
print(device)



# node消息处理器配置
model = models.alexnet()


model.classifier._modules['7'] = nn.Sequential(nn.ReLU(),nn.Dropout(0.5),nn.Linear(1000,1))

model.load_state_dict(torch.load('best8.pt'))
model.eval()
model = model.to(device)

@node.img_in.set_reader
def f1(data):
    # get img
    cv_img = data['pic']
    #img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    img = cv2.cvtColor(cv_img[160::],cv2.COLOR_BGR2RGB)

    cv2.imshow('demo',img)
    cv2.waitKey(1)
    img = torch.from_numpy(img).permute(2,0,1)/255
    img = torch.Tensor(img)
    #img = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)


    #cv2.destroyAllWindows()

    img = torch.unsqueeze(img,dim=0)
    img = img.to(device)
    output = model(img)


    
    data = {
        'linear_x': 0.60,
        'angular_z': float(output[0][0]),
        'time': time.time()
    }
    print(data)
    node.speed_out.write(data)


