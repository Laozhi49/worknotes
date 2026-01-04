在 **Ubuntu** 中临时挂代理（HTTP/HTTPS/SOCKS），最常见的方法有三种：**终端临时代理、APT 临时代理、系统级临时代理**。

---

# ✅ 一、终端临时代理（仅当前终端有效）

关闭终端就失效，非常方便。

### **1. HTTP/HTTPS 代理**

```bash
export http_proxy="http://127.0.0.1:7890"
export https_proxy="http://127.0.0.1:7890"
```

### **2. SOCKS5 代理**

```bash
export ALL_PROXY="socks5://127.0.0.1:7890"
```

### **取消代理**

```bash
unset http_proxy https_proxy ALL_PROXY
```

---

# ✅ 二、APT 临时代理（只对这次 apt 命令有效）

如果你不想修改配置文件，可以这样：

### **HTTP/HTTPS 代理**

```bash
sudo apt -o Acquire::http::Proxy="http://127.0.0.1:7890" \
         -o Acquire::https::Proxy="http://127.0.0.1:7890" update
```

---

# ✅ 三、curl / wget 单独走代理（仅一条命令有效）

### **curl**

```bash
curl -x socks5://127.0.0.1:7890 https://google.com
```

### **wget**

```bash
wget -e use_proxy=yes -e http_proxy=127.0.0.1:7890 https://google.com
```

---

# ❗代理不生效常见原因

* 终端用的是 `sudo`（sudo 不继承用户环境变量）
  → 用 `sudo -E` 或者直接在 apt 命令里写代理
* 程序要用 `ALL_PROXY` 而不是 http_proxy
* Clash/SS/V2Ray 没打开对应端口

---
