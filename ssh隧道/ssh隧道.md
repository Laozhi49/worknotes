情况是：

- 电脑 A：可以访问普通互联网，也可以通过公司 VPN 或其他网络通道访问 GitLab。
- 电脑 B：可以访问普通互联网，但是无法直接访问 GitLab。
- 两台电脑：已经通过 NetBird 组网，并且 B 上的 SSH 服务已经配置好。

这种情况，建议采用 **SSH 隧道 + 电脑 A 的 HTTP 代理**。

这样可以实现一个很方便的效果：

> 在电脑 B 的 VS Code 远程终端中执行 `git clone`、`git pull`，GitLab 请求通过电脑 A 的网络出去。

## **1. 整体网络结构**

**电脑 B**

远程开发机

VS Code Remote SSH / Git

NetBird + SSH 隧道

**电脑 A**

HTTP / SOCKS 代理服务

可以访问 GitLab

**GitLab**

公司内网 / VPN 可访问资源

## **2. 推荐使用 SSH 动态端口转发**

其实不一定要在电脑 A 安装额外的代理软件。

OpenSSH 自带 SOCKS5 动态端口转发功能，可以让电脑 B 通过电脑 A 的 SSH 连接访问 GitLab。

这里我们使用电脑 A 作为 SSH 服务端，电脑 B 作为 SSH 客户端。

注意：你目前配置的是 B 上的 SSH Server，因此还需要在 A 上开启 OpenSSH Server。

### **第一步：在电脑 A 安装 SSH Server**

管理员 PowerShell：

```
Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0
```

启动：

```
Start-Service sshd
Set-Service sshd -StartupType Automatic
```

检查：

```
Get-Service sshd
```

确保状态为 `Running`。

然后确认电脑 A 的 NetBird IP：

```
netbird status
```

假设：

```
电脑 A：100.100.1.10
电脑 B：100.100.1.20
```

### **第二步：从电脑 B 建立 SOCKS5 隧道**

在电脑 B 的 PowerShell 执行：

```
ssh -N -D 127.0.0.1:1080 用户名@100.100.1.10
```

例如：

```
ssh -N -D 127.0.0.1:1080 admin@100.100.1.10
```

输入电脑 A 的 Windows 账户密码。

这条命令的含义：

| 参数                   | 作用                 |
| -------------------- | ------------------ |
| `-N`                 | 不执行远程命令            |
| `-D`                 | 建立 SOCKS 动态代理      |
| `127.0.0.1:1080`     | 在电脑 B 本地监听 1080 端口 |
| `admin@100.100.1.10` | 连接电脑 A             |

这个 PowerShell 窗口需要保持运行。

此时电脑 B 就拥有了一个通过电脑 A 出口的 SOCKS5 代理。

## **3. 配置 Git 使用代理**

在电脑 B 新开一个 PowerShell 窗口。

执行：

```
git config --global http.proxy socks5h://127.0.0.1:1080
git config --global https.proxy socks5h://127.0.0.1:1080
```

这里特别使用 `socks5h`，让域名解析也通过代理进行。

然后测试：

```
curl.exe --proxy socks5h://127.0.0.1:1080 https://gitlab.com
```

如果你的 GitLab 是公司私有实例，就将地址换成实际的 GitLab 域名。

接着执行：

```
git clone https://你的GitLab地址/group/project.git
```

如果 GitLab 使用 HTTPS 认证，仍然需要电脑 B 提供相应的 GitLab 凭据。

## **4. 需要注意的两个问题**

**问题一：电脑 A 的 SSH 服务必须允许电脑 B 访问。**

电脑 A 的 Windows 防火墙需要允许来自电脑 B NetBird IP 的 TCP 22 端口连接。

**问题二：电脑 A 必须能够访问 GitLab 的实际网络。**

例如，如果 GitLab 只有连接公司 VPN 才能访问，那么电脑 A 必须保持公司 VPN 在线。

另外，如果公司 VPN 使用了严格的路由策略、DNS 或访问控制，SOCKS 代理还需要针对这些情况进一步调整。