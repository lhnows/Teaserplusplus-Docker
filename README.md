# Teaserplusplus-Docker

基于仓库 ```https://github.com/MIT-SPARK/TEASER-plusplus``` 打包docker镜像
## docker镜像下载
- 镜像下载:
  - **链接:** https://pan.baidu.com/s/15HxOJatiNzBDkiGMMN7Zbw?pwd=ad5a
  - **提取码:** `ad5a`
## 使用方法
1. clone 代码
   ```git clone https://github.com/lhnows/Teaserplusplus-Docker.git```
2. 下载docker镜像，并导入docker镜像
   ```docker load < teaserpp_dockerimage-aptpcl.tar```
3. 启动容器并挂载代码目录到app: 
- linux `docker run -it -v ./:/app teaserpp:latest /bin/bash`
- windows  `docker run -it -v %cd%:/app teaserpp:latest /bin/bash`

5. 进入容器内的挂载目录: `cd /app/teaser_cpp_ply`
6. 创建 build 目录: `mkdir build`
7. 进入 build 目录: `cd build`
8. 配置: `cmake ..`
9. 编译: `make`
10. 执行: `./teaser_cpp_fpfh`
