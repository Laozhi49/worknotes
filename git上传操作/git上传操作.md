# 命令行指引
您还可以按照以下说明从计算机中上传现有文件。

# 初始化仓库
```bash
# 如果仓库还未创建
git init
```

# 配置您的 Git 身份
开始使用 Git 并学习 如何配置它。

## 本地
Git 本地设置
本地配置您的 Git 身份，仅在此项目中使用：
```bash
git config --local user.name "Zhi Lao"
git config --local user.email "laozhi@ybn.sydpower.com"
```

## 全局
```bash
git config --global user.name "Zhi Lao"
git config --global user.email "laozhi@ybn.sydpower.com"
```

# 添加文件
使用 SSH 或 HTTPS 将文件推送到此存储库。如果不确定，建议使用 SSH。

## 创建一个新仓库
```bash
# ssh
git clone git@git.local:robotics/platform/sensor/imu.git
# https
git clone https://git.local/robotics/platform/sensor/imu.git

cd imu
git switch --create main
touch README.md
git add README.md
git commit -m "add README"
git push --set-upstream origin main
```

## 推送现有文件夹
### 前往您的文件夹
```bash
cd existing_folder
```
### 配置 Git 仓库
```bash
git init --initial-branch=main --object-format=sha1
# ssh
git remote add origin git@git.local:robotics/platform/sensor/imu.git
# https
git remote add origin https://git.local/robotics/platform/sensor/imu.git

git add .
git commit -m "Initial commit"

# 如果没有main分支，只有master，可以将master改名为main
git branch -M main

git push --set-upstream origin main
```
## 推送现有的 Git 仓库
### 前往您的仓库
```bash
cd existing_repo
```
### 配置 Git 仓库
```bash
git remote rename origin old-origin
# ssh
git remote add origin git@git.local:robotics/platform/sensor/imu.git
# https
git remote add origin https://git.local/robotics/platform/sensor/imu.git

git push --set-upstream origin --all
git push --set-upstream origin --tags
```


# 删除误上传的文件
在仓库目录下创建或修改 .gitignore，例如加入：
```bash
build/
```
这样会忽略整个 build 目录及其所有内容。
如果你当前已经把 build 里的文件提交过 git，还需要先从索引中移除：
```bash
git rm -r --cached build

# 然后再提交
git add .gitignore
git commit -m "ignore build directory"
```

# 上传到新分支
一般情况下，不能直接上传到main分支，这个时候就需要在仓库先创建新分支dev，然后再上传。
```bash
# 更新仓库，更新仓库前，可以先备份，以防拉取后覆盖仓库
git fetch origin

# 切换分支
git checkout -b dev origin/dev

...
```