# REPLAB: A Reproducible Low-Cost Arm Benchmark for Robotic Learning
![Imgur](https://i.imgur.com/rf2tucH.jpg)
This package contains the scripts used to operate a REPLAB cell. More details and the full paper can be found [here](https://sites.google.com/view/replab/).

## Setup
Since the package is only meant to be run inside of the provided Docker container, no additional installation setup is required aside from running the Docker image (instructions for setting up a Docker container can be found on the website). This package can be found in `/root/widowx_arm/src/` inside of the Docker container.

*Note: certain graphical components for scripts (e.g. plotting, calibration window) may not be displayed without using nvidia-docker. However, most scripts (including data collection and evaluation) do not require the use of a graphical interface.*

### Initializing the Camera / MoveIt!
Whenever operating the cell, the camera nodelet and MoveIt! stack need to be initialized before use. A convenience script `/root/widowx_arm/start.sh` is provided for this purpose.
```
sh /root/widowx_arm/src/start.sh
```
This script will run in the background during the operation of the cell. Once this script is running, we recommend entering the container in another terminal using `docker exec`.
```
docker exec -it [container ID] bash
```
The container ID can be found using `docker container ls`.

### Collecting a point cloud base
The grasping routine is reliant on blob detection to identify objects in the point cloud. This requires a base of point cloud points to perform background subtraction and filter out non-object points. To collect the base point cloud, clear the arena of any extraneous objects and use `src/store_base.py` to store the base point cloud. The script should move the arm to neutral position upon initialization.
```
rosrun replab store_base.py
```
This script will take a few minutes to run. The point cloud is stored in `pc_base.npy`. 

*Note: the base point cloud is sensitive to camera/arena peturbations, so this process may need to be repeated every so often to recollect the point cloud base.*


### Robot-Camera Calibration
If the camera is aligned to our provided reference image and the cell is built to specification, then calibration is not required since our provided calibration matrix (which is already stored in `src/config.py`) should work out of the box. Otherwise, use `src/calibrate.py` to compute a new calibration matrix.
```
rosrun replab calibrate.py
```
This will launch a GUI showing the input from the camera. The script works by collecting correspondences between the position of the end-effector in robot coordinates and the position in camera coordinates, which are used to compute a calibration matrix. To use, simply click a point in the GUI to save the camera coordinate of the clicked point. Then, move the arm to the clicked point and record the position of the end-effector. Note that the script requires the end-effector to be perpendicular to the arena floor for each correspondence. We recommend collecting at least 15 correspondences around the arena. Once finished (ctrl-C to exit), the script will output the computed calibration matrix that can be copied into `src/config.py`.

### Configuring Settings
Various parameters and settings can be found in `src/config.py`.

## Operating the Cell
### Data Collection
To run data collection, use `src/collect_data.py`. Before starting data collection, make sure there are objects in the arena.
```
rosrun replab collect_data.py --samples [# of samples] --datapath [path for saving samples]
```
The default settings in `config.py` should be fine for running data collection out of the box.

### Evaluation
To run an evaluation, use `src/evaluation.py`. Before running an evaluation, make sure there are test objects in the arena.
```
rosrun replab evaluation.py --method [method name]
```

### Controller Interface
To operate the arm by directly issuing commands, use `src/controller.py`.
```
rosrun replab controller.py
```
This will launch a `pdb` console where the user can freely use predefined motion routines to control the arm. Example usage includes:
```
widowx.move_to_neutral()  # Moves the arm to neutral position

widowx.get_joint_values() # Returns the servo positions in joint angles

widowx.move_to_drop()     # Moves the arm to object-dropping position
widowx.move_to_drop(-.5)  # Moves the arm to object-dropping position with the first servo rotated -.5 radians from neutral

widowx.open_gripper()     # Opens the gripper
widowx.close_gripper()    # Closes the gripper

widowx.sweep_arena()      # Sweep the arena

NEUTRAL_VALUES[0] += .5   # Modify the position of the first servo in the neutral position by rotating it .5 radians
widowx.move_to_neutral()  # Move to the new neutral position
```
