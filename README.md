# vision2019

## 环境依赖
- ubuntu + ros(opencv)

## 规范
- 代码规范参考 google
- 日志统一用ROS_INFO \ ROS_WARN \ ROS_ERROR
- 参数统一用ROS package + opencv + .xml（不要使用ros param）
- 如果配环境的水平有所提高，未来要改 glog + google protobuf

## roborts_bringup
所有的脚本、启动文件都在这里，但是.xml参数配置文件不在这里，而是和代码放在一起。

launch/包含所有机器人的启动文件，scripts/是所有的脚本文件，udev/创建接口规则，upstart/是自启动文件。

## roborts_camera
一个ros相机节点，用于测试相机、标定相机、快速调试；比赛代码不含有它，因为ROS订阅有延迟。

## roborts_msgs
所有的ROS消息统一在这里，目前的代码是双ROS节点（serial + vision ），所以msg只用于这两个ROS节点之间互传。延迟大约0.3ms。

## roborts_serial
这是一个统一所有机器人的串口的实现+协议，不要修改它，只给它发数据，以及读它发来的数据即可。

## roborts_vision
- camera / 相机统一接口
  - mercure / 大恒水星
  - realsense / 深度相机
  - stereo / 双目相机
  - uvc / 免驱相机
- detect_factory / 目标检测统一接口
  - armor / 装甲板
  - box / 弹药箱
  - qrcode / 二维码
  - rune / 大符
  - tags / AprilTags码
- executor / 硬件层控制统一接口
  - chassis / 底盘麦轮解算
  - gimbal / 云台角度解算
- node / 
- GXnode / 

## Todo
- 工业相机分辨率还需要测试，分辨率太高会导致电脑处理不过来。目前推荐1280x720。
- 工业相机有很多硬件参数可调，可以替代不少图像预处理。
- 相机缓存到读图，读图缓存到送入识别，中间的缓存设计还不完善，这部分可以降低5ms左右的延时。
- 统一接口设计还没有完成camera、detect_factory。
- 良好的设计roborts_vision主节点还没有架构还没有定型。
- 串口部分可能会有多线程bug，需要测试。
- 坐标角度相关的部分，考虑使用ros::tf重写。
- 录制视频这一块还没移植过来。
- svm这一块还没移植过来。
- gimbal_control需要多几种解算模型，根据相机安装位置而改变。
- chassis_control目前没能加进去，但是条件允许，哨兵要加，未来icra也要加。
- serial_sdk部分可能需要重写。
- serial_proto部分可能需要拓展，例如电机编码器与陀螺仪都能提供角度，未来还要读底盘电机数据（哨兵）。