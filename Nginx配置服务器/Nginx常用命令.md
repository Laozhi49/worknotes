# Nginx 常用命令大全

---

## **1️⃣ 启动/停止/重载/查看状态**

| 命令                             | 功能               |
| ------------------------------ | ---------------- |
| `sudo systemctl start nginx`   | 启动 Nginx 服务      |
| `sudo systemctl stop nginx`    | 停止 Nginx 服务      |
| `sudo systemctl restart nginx` | 重启 Nginx（会停止再启动） |
| `sudo systemctl reload nginx`  | 平滑重载配置（不中断现有连接）  |
| `sudo systemctl status nginx`  | 查看 Nginx 服务状态    |
| `sudo systemctl enable nginx`  | 开机自启 Nginx       |
| `sudo systemctl disable nginx` | 取消开机自启           |

> 如果系统不支持 systemd，可以用：
>
> ```bash
> sudo service nginx start|stop|restart|reload|status
> ```

---

## **2️⃣ 配置管理**

| 命令                | 功能                              |
| ----------------- | ------------------------------- |
| `nginx -t`        | 测试 Nginx 配置文件是否正确               |
| `nginx -s reload` | 平滑重载配置（和 systemctl reload 相同效果） |
| `nginx -s stop`   | 立即停止 Nginx（类似 kill）             |
| `nginx -s quit`   | 平滑退出 Nginx                      |

* 配置文件路径通常在 `/etc/nginx/nginx.conf`，可用 `nginx -t -c /路径/nginx.conf` 指定测试。

---

## **3️⃣ 查看日志**

| 命令                                  | 功能       |
| ----------------------------------- | -------- |
| `tail -f /var/log/nginx/access.log` | 实时查看访问日志 |
| `tail -f /var/log/nginx/error.log`  | 实时查看错误日志 |

---

## **4️⃣ 进程管理（不常用，但有时用得上）**

```bash
ps aux | grep nginx   # 查看 nginx 进程
sudo pkill nginx      # 杀掉 nginx 进程
sudo killall nginx    # 杀掉 nginx 进程
```

---

💡 **小技巧：**

* 修改配置后一定要先 `nginx -t` 测试，然后再 `reload`，避免配置错误导致服务无法启动。
* `restart` 会中断正在处理的请求，生产环境建议用 `reload`。

---

