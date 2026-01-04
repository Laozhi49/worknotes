
---

# 🐳 **Docker 基本操作总结（最常用版）**

---

# 1️⃣ **镜像（Image）操作**

### **查看本地镜像**

```bash
docker images
```

### **拉取镜像**

```bash
docker pull ubuntu:22.04
```

### **删除镜像**

```bash
docker rmi 镜像ID
```

---

# 2️⃣ **容器（Container）操作**

### **查看正在运行的容器**

```bash
docker ps
```

### **查看全部容器（包括停止的）**

```bash
docker ps -a
```

### **启动容器**

```bash
docker start 容器ID
```

### **停止容器**

```bash
docker stop 容器ID
```

### **删除容器**

```bash
docker rm 容器ID
```

---

# 3️⃣ **运行新容器**

### **交互式启动（进入 bash）**

```bash
docker run -it ubuntu:22.04 bash
```

### **后台运行**

```bash
docker run -d ubuntu:22.04
```

### **映射端口（主机 8080 → 容器 80）**

```bash
docker run -p 8080:80 -d nginx
```

### **挂载目录（主机目录 → 容器目录）**

```bash
docker run -it -v /host/path:/container/path ubuntu bash
```

---

# 4️⃣ **进入容器**

### **进入正在运行的容器**

```bash
docker exec -it 容器ID bash
```

### **进入容器主进程的 shell**

```bash
docker exec -it 容器ID /bin/bash
```

---

# 5️⃣ **文件复制**

### **把主机文件复制到容器**

```bash
docker cp /host/file 容器ID:/container/path/
```

### **把容器文件复制到主机**

```bash
docker cp 容器ID:/container/file /host/path/
```

---

# 6️⃣ **查看容器日志**

```bash
docker logs 容器ID
```

实时查看：

```bash
docker logs -f 容器ID
```

---

# 7️⃣ **构建镜像（Dockerfile）**

Dockerfile 示例：

```dockerfile
FROM ubuntu:22.04
COPY . /app
RUN apt update && apt install -y python3
CMD ["bash"]
```

构建镜像：

```bash
docker build -t myimage:v1 .
```

---

# 8️⃣ **导出 / 导入镜像**

### **导出镜像**

```bash
docker save -o myimage.tar myimage:v1
```

### **导入镜像**

```bash
docker load -i myimage.tar
```

---

# 9️⃣ **导出 / 导入容器**

### **导出容器**

```bash
docker export 容器ID > container.tar
```

### **导入容器快照**

```bash
docker import container.tar
```

---

# 🔟 **清理 Docker 垃圾文件**

删除停止容器：

```bash
docker container prune
```

删除无用镜像：

```bash
docker image prune
```

删除所有无用数据：

```bash
docker system prune -a
```

---

# 📌 总结（最常用命令一览）

| 功能        | 命令                          |
| --------- | --------------------------- |
| 查看容器      | `docker ps -a`              |
| 启动/停止     | `docker start/stop`         |
| 进入容器      | `docker exec -it 容器ID bash` |
| 运行新容器     | `docker run -it 镜像 bash`    |
| 文件复制      | `docker cp`                 |
| 构建镜像      | `docker build`              |
| 删除容器 / 镜像 | `docker rm / docker rmi`    |

---