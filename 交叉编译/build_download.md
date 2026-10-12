# 编译
```bash
colcon build --build-base build_aarch64 --install-base install_aarch64 --cmake-args -DCMAKE_TOOLCHAIN_FILE=$(pwd)/toolchains/aarch64-toolchain.cmake
```

# 下载编译产物
```bash
# 跳板传输
rsync -avz -J -e "ssh -J JumpHostname@[ip]]" \
  folder_path/ \
  hostname@[ip]:folder_path

# 直接传输
rsync -avz \
  folder_path/ \
  hostname@[ip]:folder_path
```

## dshanpi (Boat1)
```bash
# navigation
rsync -avz -J -e "ssh -J Administrator@192.168.1.40" \
  install_aarch64/ \
  root@[240e:3bb:3464:15d0:b7c6:31f5:edc2:5c3d]:/root/workspace/pool-surface-robot_ws/install_aarch64

# vls_h5_lidar
rsync -avz -J -e "ssh -J Administrator@192.168.1.40" \
  install_aarch64/ \
  root@[240e:3bb:3464:15d0:b7c6:31f5:edc2:5c3d]:/root/workspace/vls_h5_lidar_ws/install_aarch64
```

## kickpi (Boat2)
```bash
# navigation
rsync -avz -J -e "ssh -J Administrator@192.168.1.40" \
  install_aarch64/ \
  kickpi@[192.168.1.177]:/home/kickpi/workspace/robot_navigation_ws/install_aarch64

# lslidar
rsync -avz -J -e "ssh -J Administrator@192.168.1.40" \
  install_aarch64/ \
  kickpi@[192.168.1.177]:/home/kickpi/workspace/lslidar_ws/install_aarch64
```

