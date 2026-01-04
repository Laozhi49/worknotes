
---

# 一、服务器推荐的项目目录结构（example）

```text
pt2nb_server/
├── frontend/                 # 前端
│   └── index.html
├── backend/                  # 后端
│   ├── app.py
│   ├── pt2nb.sh
│   ├── pt2onnx.py
│   └── jobs/
└── nginx/
    └── pt2nb.conf
```

👉 **前后端完全分离**，这是非常好的习惯

---

# 三、前端代码应该怎么写（最低正确姿势）

## 1️⃣ 前端职责

只做 3 件事：

* 选文件
* 调 API
* 显示状态 / 下载

---

## 2️⃣ 核心前端模板(example)

```html
<!DOCTYPE html>
<html lang="zh">
<head>
  <meta charset="UTF-8">
  <title>PT → NB 转换</title>
  <style>
    body { font-family: sans-serif; }
    .container { width: 500px; margin: 50px auto; }
    input[type="file"] { display: block; margin-bottom: 20px; }
    button { padding: 10px 20px; font-size: 16px; }

    .loading {
      animation: blink 1.2s infinite;
    }
    @keyframes blink {
      0% { opacity: 0.3; }
      50% { opacity: 1; }
      100% { opacity: 0.3; }
    }
  </style>
</head>
<body>

<div class="container">
  <h2>PT → NB 转换</h2>

  <label>.pt 文件:</label>
  <input type="file" id="ptFile" accept=".pt">

  <label>校准图片（多选）:</label>
  <input type="file" id="images" accept="image/*" multiple>

  <button id="convertBtn">开始转换</button>

  <p id="status"></p>

  <button id="downloadBtn" style="display:none;">下载 NB</button>
</div>

<script>
let currentJobId = null;

const statusEl = document.getElementById("status");
const convertBtn = document.getElementById("convertBtn");
const downloadBtn = document.getElementById("downloadBtn");

convertBtn.onclick = async () => {
  const ptFile = document.getElementById("ptFile").files[0];
  const images = document.getElementById("images").files;

  if (!ptFile || images.length === 0) {
    alert("请选择 pt 文件和图片");
    return;
  }

  const form = new FormData();
  form.append("pt_file", ptFile);
  for (let i = 0; i < images.length; i++) {
    form.append("images", images[i]);
  }

  statusEl.innerText = "⏳ 任务提交中...";
  statusEl.className = "loading";
  convertBtn.disabled = true;
  downloadBtn.style.display = "none";

  const res = await fetch("/api/uploads", {
    method: "POST",
    body: form
  });

  if (!res.ok) {
    statusEl.innerText = "❌ 提交失败";
    convertBtn.disabled = false;
    return;
  }

  const data = await res.json();
  currentJobId = data.job_id;
  statusEl.innerText = "⏳ 后台转换中...";
  pollStatus(currentJobId);
};

function pollStatus(jobId) {
  const timer = setInterval(async () => {
    const res = await fetch(`/api/status/${jobId}`);
    const data = await res.json();

    if (data.status === "done") {
      clearInterval(timer);
      statusEl.innerText = "✅ 转换完成";
      statusEl.className = "";
      convertBtn.disabled = false;
      downloadBtn.style.display = "inline-block";
    } else if (data.status.startsWith("error")) {
      clearInterval(timer);
      statusEl.innerText = "❌ 转换失败";
      convertBtn.disabled = false;
    }
  }, 2000);
}

downloadBtn.onclick = async () => {
  const res = await fetch(`/api/download/${currentJobId}`);
  const blob = await res.blob();

  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = "output.nb";
  a.click();
};
</script>

</body>
</html>

```

👉 **前端永远只请求 `/api/...`**

---

# 四、后端 Flask / FastAPI 怎么写（example）

## 1️⃣ 后端职责（非常重要）

后端 **不是转换器**，而是：

* 文件接收
* 目录管理
* 调脚本
* 返回结果
* 清理后台
---

## 2️⃣ 后端app脚本example

```python
from flask import Flask, request, jsonify, send_file
import os
import subprocess
import uuid
import threading
import shutil
from cleanup import cleanup_jobs

threading.Thread(target=cleanup_jobs, daemon=True).start()

BASE_DIR = "jobs"

app = Flask(__name__)
os.makedirs(BASE_DIR, exist_ok=True)

# ---------- 工具函数 ----------

def create_job_dirs(job_id):
    job_dir = os.path.join(BASE_DIR, job_id)
    input_dir = os.path.join(job_dir, "input")
    output_dir = os.path.join(job_dir, "output")
    os.makedirs(input_dir, exist_ok=True)
    os.makedirs(output_dir, exist_ok=True)
    return job_dir, input_dir, output_dir


def run_job(job_id, pt_path, img_dir, nb_path):
    status_file = os.path.join(BASE_DIR, job_id, "status.txt")
    try:
        with open(status_file, "w") as f:
            f.write("running")

        subprocess.check_call([
            "./pt2nb.sh",
            pt_path,
            img_dir,
            nb_path
        ])

        with open(status_file, "w") as f:
            f.write("done")

    except Exception as e:
        with open(status_file, "w") as f:
            f.write(f"error:{str(e)}")


def get_status(job_id):
    status_file = os.path.join(BASE_DIR, job_id, "status.txt")
    if not os.path.exists(status_file):
        return "unknown"
    return open(status_file).read().strip()


# ---------- API ----------

@app.route("/uploads", methods=["POST"])
def upload():
    print("==== /uploads called ====")

    pt_file = request.files.get("pt_file")
    images = request.files.getlist("images")

    if not pt_file or not pt_file.filename.endswith(".pt"):
        return jsonify({"error": "invalid pt file"}), 400
    if not images:
        return jsonify({"error": "image required"}), 400

    job_id = str(uuid.uuid4())
    _, input_dir, output_dir = create_job_dirs(job_id)

    pt_path = os.path.join(input_dir, "model.pt")
    img_dir = os.path.join(input_dir, "images")
    nb_path = os.path.join(output_dir, "model.nb")
    os.makedirs(img_dir, exist_ok=True)

    pt_file.save(pt_path)
    for img in images:
        img.save(os.path.join(img_dir, img.filename))

    # 后台执行
    threading.Thread(
        target=run_job,
        args=(job_id, pt_path, img_dir, nb_path),
        daemon=True
    ).start()

    return jsonify({"job_id": job_id}), 202


@app.route("/status/<job_id>")
def status(job_id):
    status = get_status(job_id)
    return jsonify({"status": status})


@app.route("/download/<job_id>")
def download(job_id):
    nb_path = os.path.join(BASE_DIR, job_id, "output", "model.nb")
    if not os.path.exists(nb_path):
        return "not ready", 404
    return send_file(nb_path, as_attachment=True)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)

```

## 3️⃣ 后端清理后台脚本example

```python
import os
import subprocess
import time
from datetime import datetime, timedelta

BASE_DIR = "/home/ubuntu20/server/pt2nb_server/backend/jobs"
# EXPIRE_MINUTES = 10
# SLEEP_INTERVAL = 60  # 每分钟检查一次
KEEP_DAYS = 1
SLEEP_INTERVAL = 3600  # 每小时检查一次

def cleanup_jobs():
    print("[cleanup] background cleanup thread started")
    while True:
        now = datetime.now()
        for job_id in os.listdir(BASE_DIR):
            job_path = os.path.join(BASE_DIR, job_id)
            if not os.path.isdir(job_path):
                continue

            # 检查状态文件，跳过正在运行的任务
            status_file = os.path.join(job_path, "status.txt")
            status = "running"
            if os.path.exists(status_file):
                with open(status_file, "r") as f:
                    status = f.read().strip()
            if status == "running":
                continue

            # 检查创建时间
            ctime = datetime.fromtimestamp(os.path.getctime(job_path))
            # if now - ctime > timedelta(minutes=EXPIRE_MINUTES):
            if now - ctime > timedelta(days=KEEP_DAYS):
                try:
                    print(f"[cleanup] Removing old job: {job_id}")
                    # 调用 sudo rm -rf 删除文件夹
                    subprocess.run(
                        ["sudo", "rm", "-rf", job_path],
                        check=True
                    )
                except subprocess.CalledProcessError as e:
                    print(f"[cleanup] Failed to remove {job_id}: {e}")

        time.sleep(SLEEP_INTERVAL)

```

---

# 五、Shell 脚本怎么写（工程版）

## 1️⃣ 必须支持参数

```bash
#!/usr/bin/env bash

PT_PATH="$1"
IMG_DIR="$2"
NB_PATH="$3"

OUTPUT_DIR=$(dirname "$NB_PATH")
BASENAME=$(basename "$PT_PATH" .pt)
```

---

## 2️⃣ 一个合格的脚本应该做到：

* 不依赖当前目录
* 所有路径来自参数
* 能单独运行
* 不交互（不能卡住）


*当脚本使用了sudo时，可以使用sudoers规则给予免密*

---

# 六、Nginx 配置的标准写法

```nginx
server {
    listen 80;
    server_name sydpower_ai.com 192.168.1.174;

    client_max_body_size 2G;

    location / {
        root /home/ubuntu20/server/pt2nb_server/frontend;
        index index.html;
        try_files $uri $uri/ /index.html;
    }

    location /api/ {
        proxy_pass http://127.0.0.1:5000/;
        proxy_http_version 1.1;

        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;

        proxy_read_timeout 600s;
        proxy_send_timeout 600s;
    }
}
```

**其中，server_name写成sydpower_ai时，需要保证/etc/hosts里存在127.0.0.1   sydpower_ai映射域名（但是此域名只能本机使用，如需同一内网下的其他网络也可以访问此域名，需要更改/添加路由器的DNS域名解析），而192.168.1.174则保证同一个内网下的其他网络可以通过这个ip访问此网站**


*注意，此文件只能在/etc/nginx/conf.d目录下才能起效，因此编写完成后记得copy到/etc/nginx/conf.d*

---

## nginx 配置 5 大注意事项

### 1️⃣ `server_name` 一定要对

* 域名
* IP
* `_`（兜底）

---

### 2️⃣ `/api/` 和 Flask 路由要对齐

```nginx
location /api/ {
    proxy_pass http://127.0.0.1:5000/;
}
```

Flask 写：

```python
@app.route("/uploads")
```

浏览器访问：

```text
/api/uploads
```

---

### 3️⃣ 文件上传一定要设置大小

```nginx
client_max_body_size 2G;
```

否则直接 413

---

### 4️⃣ 修改 nginx 后永远三步

```bash
sudo nginx -t
sudo systemctl reload nginx

# 若nginx未启动
sudo systemctl start nginx

```

---

### 5️⃣ Welcome to nginx ≠ nginx 没生效

它只代表：

> **你访问的 Host 没命中你的 server**

---

# 七、什么时候“可以不用 nginx”？

✔ 本机自己测试
✔ 只你一个人用
✔ 不需要域名

❌ 内网共享
❌ 多人使用
❌ 文件大
❌ 稳定运行

---

# 八、一句话总结（给未来的你）

> **Nginx 管访问，前端管交互，后端管调度，脚本管执行**
