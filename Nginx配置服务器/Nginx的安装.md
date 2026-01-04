
---

# 一、Nginx 的安装

## 安装依赖
```bash
pip install flask
```

## 安装nginx

参考官网：https://nginx.org/en/linux_packages.html#Ubuntu

```bash
# nstall the prerequisites
sudo apt install curl gnupg2 ca-certificates lsb-release ubuntu-keyring

# Import an official nginx signing key so apt could verify the packages authenticity. Fetch the key
curl https://nginx.org/keys/nginx_signing.key | gpg --dearmor \
    | sudo tee /usr/share/keyrings/nginx-archive-keyring.gpg >/dev/null

# Verify that the downloaded file contains the proper key
gpg --dry-run --quiet --no-keyring --import --import-options import-show /usr/share/keyrings/nginx-archive-keyring.gpg

# To set up the apt repository for stable nginx packages, run the following command
echo "deb [signed-by=/usr/share/keyrings/nginx-archive-keyring.gpg] \
https://nginx.org/packages/ubuntu `lsb_release -cs` nginx" \
    | sudo tee /etc/apt/sources.list.d/nginx.list

# Set up repository pinning to prefer our packages over distribution-provided ones
echo -e "Package: *\nPin: origin nginx.org\nPin: release o=nginx\nPin-Priority: 900\n" \
    | sudo tee /etc/apt/preferences.d/99nginx

# To install nginx, run the following commands
sudo apt update
sudo apt install nginx
```

安装完立刻就会：

* 创建 nginx 用户
* 注册 systemd 服务
* **默认开机自启**

> ❗ `nginx -t` / `systemctl reload nginx`
> **不会**影响是否自启
> 自启是在安装那一刻就设置好的

---

## 2️⃣ 重要目录结构（必背）

```text
/etc/nginx/
├── nginx.conf              # 总入口配置
├── conf.d/
│   ├── default.conf        # 默认站点（Welcome to nginx）
│   └── xxx.conf            # 你自己的站点
├── mime.types
└── ...
```

### nginx.conf 的核心只有一句：

```nginx
http {
    include /etc/nginx/conf.d/*.conf;
}
```

👉 **所有真正的 server 配置都写在 /etc/nginx/conf.d 文件夹里，一般不会直接修改nginx.conf**

---

## 3️⃣ 默认 Welcome to nginx 从哪来？

```text
/etc/nginx/conf.d/default.conf
```

```nginx
server {
    listen 80;
    server_name localhost;
    root /usr/share/nginx/html;
}
```

**只要有任何请求没匹配到别的 server，就会进它**