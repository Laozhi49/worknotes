
---

# **WSL 常用操作汇总**

## 一、WSL 版本与发行版管理

| 命令                                         | 说明                        |
| ------------------------------------------ | ------------------------- |
| `wsl --list --verbose` 或 `wsl -l -v`       | 查看已安装的 WSL 发行版及 WSL1/2 版本 |
| `wsl --set-default <DistroName>`           | 设置默认启动发行版                 |
| `wsl --set-version <DistroName> <Version>` | 切换 WSL 版本（1 或 2）          |
| `wsl --install -d <DistroName>`            | 安装指定发行版（如 Ubuntu-24.04）   |
| `wsl --unregister <DistroName>`            | 卸载某个发行版（删除所有数据）           |
| `wsl --shutdown`                           | 停止所有 WSL 实例（重启内核、释放资源）    |
| `wsl -d <DistroName>`                      | 启动指定发行版并进入终端              |

---

## 二、Linux 文件操作

| 命令                         | 说明                                 |
| -------------------------- | ---------------------------------- |
| `cd ~` 或 `cd /home/<user>` | 进入家目录（`~` = 当前用户家目录）               |
| `ls`                       | 查看当前目录文件                           |
| `ls -a`                    | 查看隐藏文件                             |
| `pwd`                      | 查看当前路径                             |
| `/mnt/c/...`               | 访问 Windows C 盘文件系统（`/mnt/d` → D 盘） |
| `cp/mv/rm`                 | 文件复制/移动/删除（标准 Linux 命令）            |
| `mkdir`                    | 创建目录                               |

**注意**：

* 开发项目尽量放在 `/home/<user>`，不要直接放在 `/mnt/c`，否则性能和权限可能不稳定。

---

## 三、包管理与软件安装

### Ubuntu / Debian 系

```bash
sudo apt update          # 更新软件源
sudo apt upgrade -y      # 升级所有包
sudo apt install <pkg>   # 安装软件
sudo apt remove <pkg>    # 卸载软件
```

**示例：安装 Python、pip、Git**

```bash
sudo apt install python3 python3-pip git -y
```

---

## 四、Python & 虚拟环境

```bash
python3 --version        # 查看 Python 版本
pip3 --version           # 查看 pip 版本
python3 -m venv venv     # 创建虚拟环境
source venv/bin/activate # 激活虚拟环境
deactivate               # 退出虚拟环境
```

---

## 五、网络与端口

```bash
ip -4 a          # 查看 WSL IPv4 地址
ping <host>      # 测试网络连通性
curl http://...  # 网络请求测试
```

**注意**：

* WSL2 使用 NAT 网络，外部局域网默认无法直接访问 WSL 内服务
* 如果需要暴露端口，可以使用 Windows 端口转发：

```powershell
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8000 connectaddress=<WSL_IP> connectport=8000
```

---

## 六、Docker 相关（WSL2 配合 Docker Desktop）

```bash
docker version        # 查看 Docker 版本
docker run hello-world # 测试 Docker 是否可用
docker ps -a          # 查看容器列表
```

**提示**：

* Docker Desktop 可直接与 WSL2 集成
* 建议在 **Ubuntu-24.04** 中使用 Docker，而不是极简 `web` WSL

---

## 七、系统信息 & 调试

```bash
whoami              # 当前用户
pwd                 # 当前路径
df -h               # 磁盘空间使用
free -h             # 内存使用
uname -a            # 内核版本
lsb_release -a      # 发行版信息（需要安装 lsb-release）
```

---

## 八、WSL 与 Windows 文件交互

* **访问 Windows 文件**：

```bash
cd /mnt/c/Users/<yourname>/Desktop
```

* **从 Windows 访问 WSL 文件**：

```text
\\wsl$\Ubuntu-24.04\home\<user>\
```

> 在资源管理器地址栏输入即可直接打开 WSL 文件系统

---

## 九、常用快捷操作

| 操作       | 命令                                                   |
| -------- | ---------------------------------------------------- |
| 启动默认 WSL | `wsl`                                                |
| 启动指定 WSL | `wsl -d Ubuntu-24.04`                                |
| 停止所有 WSL | `wsl --shutdown`                                     |
| 退出 WSL   | `exit`                                               |
| 切换默认用户   | `ubuntu config --default-user <username>`（Ubuntu 特有） |

---

## 十、注意事项总结

1. **家目录在 `/home/<user>`**，root 的家目录在 `/root`
2. **不要在 `/mnt/c` 做大量文件操作**，性能差
3. **WSL2 是 NAT 网络**，外部访问需要端口转发
4. **极简 WSL（如 Docker Desktop 自动创建的 `web`）不带 sudo / Python / apt**，学习开发建议用标准 Ubuntu

---

