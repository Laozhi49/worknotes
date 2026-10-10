
---

# 一、基础必会（最常用 ⭐⭐⭐⭐⭐）

在 **运行（Win + R）** 或 **CMD / PowerShell** 中可用：

| 命令              | 作用            |
| --------------- | ------------- |
| `cmd`           | 打开命令提示符       |
| `powershell`    | 打开 PowerShell |
| `exit`          | 退出当前窗口        |
| `cls`           | 清屏            |
| `help`          | 查看命令帮助        |
| `ver`           | 查看 Windows 版本 |
| `time` / `date` | 查看或设置时间/日期    |

---

# 二、文件与目录操作（非常常用）

| 命令           | 作用        |
| ------------ | --------- |
| `dir`        | 列出目录内容    |
| `cd`         | 切换目录      |
| `cd ..`      | 返回上一级     |
| `md 文件夹名`    | 创建文件夹     |
| `rd 文件夹名`    | 删除文件夹     |
| `copy 源 目标`  | 复制文件      |
| `xcopy`      | 复制目录（旧）   |
| `robocopy` ⭐ | 专业级复制（推荐） |
| `del 文件名`    | 删除文件      |
| `move 源 目标`  | 移动文件      |

示例：

```bat
robocopy D:\data E:\backup /E /R:1 /W:1
```

---

# 三、磁盘 & 系统管理（你刚好在用）

| 命令             | 作用          |
| -------------- | ----------- |
| `diskmgmt.msc` | 磁盘管理        |
| `diskpart`     | 磁盘分区工具（⚠危险） |
| `chkdsk`       | 检查磁盘        |
| `format`       | 格式化磁盘       |
| `defrag`       | 磁盘碎片整理      |
| `cleanmgr`     | 磁盘清理        |

⚠ **diskpart 非常强大，新手慎用**

---

# 四、系统维护 & 修复（很实用 ⭐⭐⭐⭐）

| 命令                                           | 作用     |
| -------------------------------------------- | ------ |
| `tasklist`                                   | 查看进程   |
| `taskkill`                                   | 结束进程   |
| `shutdown`                                   | 关机/重启  |
| `sfc /scannow`                               | 系统文件检查 |
| `dism /online /cleanup-image /restorehealth` | 系统修复   |
| `msconfig`                                   | 启动管理   |
| `services.msc`                               | 服务管理   |

示例：

```bat
shutdown /r /t 0
```

---

# 五、网络排错（非常常用 ⭐⭐⭐⭐）

| 命令              | 作用     |
| --------------- | ------ |
| `ipconfig`      | 查看 IP  |
| `ipconfig /all` | 完整网络信息 |
| `ping`          | 测试网络连通 |
| `tracert`       | 路由跟踪   |
| `netstat -ano`  | 端口占用   |
| `nslookup`      | DNS 查询 |
| `arp -a`        | ARP 表  |

示例：

```bat
ping www.baidu.com
```

---

# 六、用户 & 权限管理

| 命令                              | 作用      |
| ------------------------------- | ------- |
| `whoami`                        | 当前用户    |
| `net user`                      | 用户管理    |
| `net localgroup administrators` | 管理员组    |
| `runas`                         | 以其他用户运行 |

---

# 七、系统工具快速打开（Win + R 神器）

| 命令             | 打开       |
| -------------- | -------- |
| `control`      | 控制面板     |
| `appwiz.cpl`   | 程序和功能    |
| `sysdm.cpl`    | 系统属性     |
| `ncpa.cpl`     | 网络适配器    |
| `devmgmt.msc`  | 设备管理器    |
| `eventvwr.msc` | 事件查看器    |
| `regedit`      | 注册表      |
| `gpedit.msc`   | 组策略（专业版） |

---

# 八、PowerShell 常用（现代推荐）

| 命令                     | 作用   |
| ---------------------- | ---- |
| `Get-Process`          | 查看进程 |
| `Get-Service`          | 查看服务 |
| `Get-ChildItem` (`ls`) | 列目录  |
| `Set-ExecutionPolicy`  | 脚本策略 |
| `Get-ComputerInfo`     | 系统信息 |

---

# 九、进阶 & 运维级（了解即可）

| 命令         | 作用         |
| ---------- | ---------- |
| `bcdedit`  | 启动项        |
| `wevtutil` | 日志管理       |
| `schtasks` | 计划任务       |
| `wmic`     | 系统查询（逐渐淘汰） |

---

