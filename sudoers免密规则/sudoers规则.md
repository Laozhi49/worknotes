
---

## 1️⃣ sudoers 规则作用

`sudors` 文件控制哪些用户可以用 `sudo` 执行命令。默认使用 sudo 执行命令会要求输入密码，但可以通过 **NOPASSWD** 参数免密执行。

---

## 2️⃣ 基本语法

```
<USER> <HOST>=(<RUNAS>) NOPASSWD: <COMMAND>
```

* `<USER>`：运行 sudo 的用户
* `<HOST>`：规则生效的主机名，一般写 `ALL`
* `<RUNAS>`：以哪个用户身份执行命令，一般是 `root`
* `NOPASSWD:`：免密执行
* `<COMMAND>`：允许执行的命令 **绝对路径**

### 示例

1. 允许用户 `ubuntu` 免密删除 `/home/ubuntu/outputs`：

```text
ubuntu ALL=(root) NOPASSWD: /bin/rm -rf /home/ubuntu/outputs
```

* 绝对路径 `/bin/rm` 必须写全
* 参数必须匹配规则（`-rf` 可以写在命令里）

2. 允许用户 `www-data` 免密重启 nginx：

```text
www-data ALL=(root) NOPASSWD: /usr/sbin/service nginx restart
```

---

## 3️⃣ 使用 `/etc/sudoers.d/` 文件

比直接编辑 `/etc/sudoers` 更安全，方便管理：

```bash
sudo nano /etc/sudoers.d/<filename>
```

写入规则后保存即可。

**注意：**

* 文件权限必须是 `0440`：

```bash
sudo chmod 0440 /etc/sudoers.d/<filename>
```

* 文件必须以换行结尾

一般创建文件写入规则就可以了，这个权限不用管

---

## 4️⃣ 调用方式（Python / shell）

在 Python `subprocess` 里：

```python
import subprocess
subprocess.run(["sudo", "/bin/rm", "-rf", "/home/ubuntu/outputs"], check=True)
```

* `/bin/rm` 和 sudoers 里定义的命令要完全一致
* 参数顺序、绝对路径都要一致

在 shell 里：

```bash
sudo /bin/rm -rf /home/ubuntu/outputs
```

不会要求输入密码

---

## 5️⃣ 注意事项

| 项目   | 建议                                       |
| ---- | ---------------------------------------- |
| 安全性  | 只允许必要命令，使用绝对路径，不要用通配符 `*`                |
| 目录权限 | sudoers 规则不能绕过文件系统权限，Flask 仍然需要访问权限      |
| 文件名  | /etc/sudoers.d/<name> 不要随意覆盖其他规则         |
| 测试   | 用 `sudo -u <USER> sudo <COMMAND>` 测试是否免密 |

---
