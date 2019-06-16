# Camera

## mercure
先确定Linux内核版本>4.８.0, 然后配置环境，解压Galaxy_Camra.tar.gz, run一下。

大恒水星MER-U3系列，代码已经写好，不要乱改它。


## UVC
这是RM2017大疆开源的uvc相机打开方式，比opencv要高效很多。


## Intel RealSense
### install
先确定Linux内核版本>4.4.0, 然后配置环境
```shell
uname -r
sudo apt-get install libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev
git clone https://github.com/IntelRealSense/librealsense
cd librealsense/
mkdir build 
cd build
cmake ../
cmake ../ -D BUILD_EXAMPLES=true
make && sudo make install
```
在 librealsense 文件夹下安装V4L，注意不要插上RealSense 摄像头。在librealsense的路径下执行：
```shell
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```
安装Openssl库：
```shell
sudo apt-get install libssl-dev
```
编译配置文件：
```shell
./scripts/patch-realsense-ubuntu-xenial.sh
```
提示完成后，插上RealSense，再执行：
```shell
sudo dmesg | tail -n 50 
```
可无失败信息则可验证安装驱动成功。 
此时可进入`/usr/local/lib`中查看或者`librealsense`下的`example`文件夹下，执行：
```shell
./cpp-capture.cpp
```
出现RealSense拍摄的图像即成功！ 