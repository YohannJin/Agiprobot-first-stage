import torch
from PIL import Image
import cv2


model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
im = Image.open('brushless.jpg')
# im =  'https://ultralytics.com/images/zidane.jpg'

results = model(im)
results.show()
results.print()
