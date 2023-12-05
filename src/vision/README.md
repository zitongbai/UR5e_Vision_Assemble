This package is used for object detection and segmentation

# Dependences

* yolov5 and its dependences
* vision_msgs

Make sure you have installed all the dependences of yolov5 in your python env (e.g. conda env containing torch).
```shell
# in yolov5 path
pip install -r requirements.txt
```

Install `vision_msgs`: 
```shell
sudo apt update
sudo apt install ros-humble-vision-msgs
```

In order to use both python modules in ROS2 and those in conda env, you should change the conda env path mannully in the file `obj_seg.py`, `obj_detect.py`, etc. 
```python
sys.path.append(str(Path.home()) + '/anaconda3/envs/torch/lib/python3.10/site-packages')
```

# Usage

## Train

Go to the path of yolov5

remember to change to the proper python env which contains `torch`, etc.

```shell
conda activate torch
```

Then train the model.

```shell
python segment/train.py --img 640 --batch 8 --epochs 100 --data gazebo_seg.yaml --weights yolov5s-seg.pt --cache
```

the datasets and weights will be downloaded automatically. 

* datasets in `vision/datasets`
* weights in `yolov5/yolov5s-seg.pt`

copy the training result (weights) from `yolov5/runs/exp*/best.pt` to `yolov5`

now if the gazebo simulation is running, then you can run the segmentation script: 
```shell
ros2 run vision obj_seg
```

<img src="./detect%20result.png" width = "500" alt="detect"/>