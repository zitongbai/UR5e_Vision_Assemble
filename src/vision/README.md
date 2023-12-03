This package is used for object detection and segmentation

# Dependences

* yolov5 and its dependences
* vision_msgs

# Usage

## Train

first, go to the path of yolov5

remember to change to the proper python env which contains `torch`, etc.

```shell
python segment/train.py --img 640 --batch 8 --epochs 100 --data gazebo_seg.yaml --weights yolov5s-seg.pt --cache
```