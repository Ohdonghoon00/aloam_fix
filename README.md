# aloam_fix

## Build

```
mkdir -p catkin/src
cd catkin/src
git clone https://github.com/Ohdonghoon00/aloam_fix.git
cd ../..
catkin_make --only-pkg-with-deps fix_aloam
source devel/setup.bash
```

## Run

```
roscore
roslaunch fix_aloam fix_aloam.launch
```

## Data Structure Ex
```
 DATA_DIR
    ├─ lidar_timestamp.csv // lidar timestamp and fidx			
    ├─ lidar2	// Undistortion lidar points
    	├─ 00000.bin
    	├─ 00001.bin
    	└─ ...						
    └─ VIOLidarPoses_lidarframes.txt // VIO pose
```

## Output File
```
Result_DIR
    └─ aloam_mapping_pose.txt // LaserMapping Pose Result 
```

## Data 경로 수정

**DATA_DIR**
- launch 폴더내 fix_aloam.launch 파일 data_dir value값 수정

**Result_DIR**
- launch 폴더내 fix_aloam.launch 파일 result_dir value값 수정

이미지를 받아와서 2d structure from motion을 통해 이미지의 포즈를 추정하는 패키지 개발

* subscribe node
  * command
  ```
  $ ros2 run visual_odometry visualize
  ```
  * input
    * raw_image
  * publish
    * raw_image
    * estimated pose   

* publish node
  * command 
  ```
  $ ros2 run visual_odometry estimate_pose
  ```

  * return
    * current raw image
    * ```
      [INFO] [1678693120.469923139] [visualize_node]:  Timestamp : 1678693120 sec
      [INFO] [1678693120.469993015] [visualize_node]:  Camera Pose : 0.006333 -0.057185 -0.001671 -0.706567 -0.177956 16.975463
      ```
