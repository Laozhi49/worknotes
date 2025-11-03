Pangolin源码：https://github.com/stevenlovegrove/Pangolin

安装步骤

```bash
# Get Pangolin
cd ~/your_fav_code_directory
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin

# Install dependencies (as described above, or your preferred method)
./scripts/install_prerequisites.sh recommended

# Configure and build
cmake -B build
cmake --build build

# 安装到系统 
sudo cmake --install build

```