import cv2
import torch
from PIL import Image



model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
im = Image.open('ko.jpg')
# im =  'https://ultralytics.com/images/zidane.jpg'

results = model(im)
results.show()
results.print()
