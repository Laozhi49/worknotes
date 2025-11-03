要从 `~/.ssh/known_hosts` 文件中删除特定 IP（如 `192.168.60.30`）对应的 SSH 主机密钥记录，你可以使用以下几种方法：

---

### **方法 1：使用 `ssh-keygen -R`（推荐）**
这是最安全、最方便的方式，它会自动删除匹配的行：
```bash
ssh-keygen -R 192.168.60.30
```
**输出示例：**
```
# Host 192.168.60.30 found: line 15
/Users/你的用户名/.ssh/known_hosts updated.
Original contents retained as /Users/你的用户名/.ssh/known_hosts.old
```
- 这会直接删除 `192.168.60.30` 的记录，并备份原文件为 `known_hosts.old`。

---

### **方法 2：手动编辑 `known_hosts` 文件**
如果喜欢手动操作：
1. 打开文件：
   ```bash
   nano ~/.ssh/known_hosts
   ```
2. 找到包含 `192.168.60.30` 的行（按 `Ctrl+W` 可搜索），删除整行。
3. 保存退出：
   - `Ctrl+X` → `Y` → `Enter`。

---

### **方法 3：使用 `sed` 命令（适合批量删除）**
通过命令行直接删除：
```bash
sed -i '/192.168.60.30/d' ~/.ssh/known_hosts
```
- `-i`：直接修改原文件。
- `'/192.168.60.30/d'`：匹配包含该 IP 的行并删除。

---

### **验证是否删除成功**
查看文件内容，确认目标 IP 已不存在：
```bash
grep '192.168.60.30' ~/.ssh/known_hosts
```
- 如果无输出，说明已删除成功。

---

### **注意事项**
1. **权限问题**：如果提示权限不足，用 `sudo` 或确保你对文件有写权限。
2. **备份习惯**：建议操作前备份原文件：
   ```bash
   cp ~/.ssh/known_hosts ~/.ssh/known_hosts.bak
   ```
3. **Windows 用户**：如果使用 Git Bash/WSL，方法同样适用；路径可能是 `C:\Users\你的用户名\.ssh\known_hosts`。

---

### **为什么需要这样做？**
- 当树莓派重装系统、更换网卡或 IP 被其他设备占用时，SSH 会因主机密钥不匹配报错。删除旧记录后，下次连接会重新信任新密钥。

完成后，尝试重新连接树莓派即可恢复正常！ 🚀