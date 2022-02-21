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
