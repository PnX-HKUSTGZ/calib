## 运行指南

1. 克隆仓库：
   ```bash
   git clone https://github.com/PnX-HKUSTGZ/calib.git
2. 编译：
   ```bash
   cd ./utils/CameraCalibration
   mkdir ./build
   cd ./build
   cmake ..
   make
3. 参数文件
   主要修改[default.xml](./default.xml)和[CameraParam.yaml](./utils/CameraCalibration/CameraParam.yaml)。
   将[default.xml](./default.xml)复制到build文件中使用。
4. 运行：
   要在nomachine中进行，ssh没有转发的话运行不了。可执行文件生成在build文件夹中。
   
