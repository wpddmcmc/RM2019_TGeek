# roborts_vision
除了串口节点，就是这个节点，我们一共就２个ROS节点。

## camera
不久的将来要覆盖所有类型的相机，而且接口统一，实时性第一，祖传性并列第一，RoboRTS实时性不太行。camera不能跨节点订阅，就在同一个代码段内写，当然解耦，解耦，解耦。
mercure 
uvc
opencv
realsence
未来可能会上高级操作、但目前不需要。

## detect_factory
这里是所有的检测算法，pnp_solver是公开出来的，其他各种类型的东西，接口统一，所有的检测不同对象的算法，全部统一，甚至装甲片识别的算法也全部统一，不要加奇奇怪怪的操作，解耦！只含有图像逻辑！输入image，输出rect/roataedrect.
armor
rune
box
tags
未来可能会上高级操作，但目前不需要。

## executor
执行器分为gimbal\chassis，但是目前只有云台，只做云台控制、解耦后的逻辑，这里只有角度结算，也就是计算yaw pitch，而pnp是图像到xyz。

这里需要开坑，参考串口协议，控制模式，1 绝对角度 2 相对角度 3 绝对速度 4 相对速度。

## node / GXnode
node是uvc相机的图像处理节点，单节点，双线程，GXnode是工业相机的图像处理节点，两者几乎一样。

## filter
不要把预测和检测放在一起、不要把滤波和串口放在一起、不要把图像与结算放到一起、如果要加新的功能，再开新的文件夹

## CMake
为了方便移植，也是正常逻辑解耦，也是出于对新手的保护，所有文件夹都是生成库，不生成exe，生成库的名字叫
- camera_LIBS
- detect_factory_LIBS
- executor_LIBS
....
