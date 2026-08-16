# mas_nav_2027

# 开发环境配置
1. docker 安装
```bash
// docker 安装脚本
export DOWNLOAD_URL="https://mirrors.tuna.tsinghua.edu.cn/docker-ce"
wget -O- https://raw.githubusercontent.com/docker/docker-install/master/install.sh | sh
// 更换为国内docker镜像源
bash <(wget -qO- https://xuanyuan.cloud/docker.sh)
// 权限处理
sudo usermod -aG docker mas
```
2. 构建容器
```bash
sudo docker compose up -d --build
```
3. 编译
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 4
```

4. 
```bash
sudo xhost + 
ros2 run nav2_map_server map_saver_cli -f MAS 
```