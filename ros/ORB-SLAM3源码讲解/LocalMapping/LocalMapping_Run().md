
## 🔹 LocalMapping::Run() 主循环逻辑

这个函数是一个 **后台线程**，启动后会一直循环，直到 SLAM 停止或结束。

### 1. 前置处理

```cpp
mbFinished = false;
while(1)
{
    // Tracking 会看到 LocalMapping 正在忙碌
    SetAcceptKeyFrames(false);
```

* `mbFinished = false` ：标记线程未结束
* `SetAcceptKeyFrames(false)` ：在处理关键帧时，先 **不接受新关键帧**，避免冲突

---

### 2. 判断是否有新关键帧

```cpp
if(CheckNewKeyFrames() && !mbBadImu)
{
    ProcessNewKeyFrame();
    MapPointCulling();
    CreateNewMapPoints();
    ...
}
```

* `CheckNewKeyFrames()` ：看队列里有没有待处理的关键帧
* `!mbBadImu` ：IMU 初始化是否正常
* **关键步骤：**

  1. `ProcessNewKeyFrame()` → 把关键帧放进地图，做 BoW 转换，建立初步连接
  2. `MapPointCulling()` → 把最近创建但质量差的地图点删掉
  3. `CreateNewMapPoints()` → 在当前关键帧和相邻关键帧之间 **三角化新点**

---

### 3. 局部优化

```cpp
if(!CheckNewKeyFrames() && !stopRequested())
{
    if(mpAtlas->KeyFramesInMap()>2)
    {
        if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
        {
            Optimizer::LocalInertialBA(...);
        }
        else
        {
            Optimizer::LocalBundleAdjustment(...);
        }
    }
}
```

* 如果没有新的关键帧进来，就可以安全地做 **局部 BA**
* 两种情况：

  * **有 IMU** → 调用 `LocalInertialBA()` （会优化位姿、速度、IMU 偏置）
  * **无 IMU** → 调用 `LocalBundleAdjustment()` （优化关键帧位姿 + 地图点）

这一步就是所谓的 **局部 BA（Local Bundle Adjustment）**。

---

### 4. IMU 初始化

```cpp
if(!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
{
    InitializeIMU(...);
}
```

* 如果是 **VIO 模式**，在这里尝试做 **IMU 初始化**（解尺度、重力方向、偏置）。

---

### 5. KeyFrame Culling（关键帧冗余剔除）

```cpp
KeyFrameCulling();
```

* 如果有些关键帧的观测点和别的关键帧高度重合（信息冗余），就把它们删掉。
* 这样可以避免地图膨胀。

---

### 6. 插入回环检测模块

```cpp
mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
```

* 把处理好的关键帧交给 **LoopClosing 线程**，用于回环检测。

---

### 7. 线程管理 & 停止检查

```cpp
else if(Stop() && !mbBadImu)
{
    while(isStopped() && !CheckFinish())
    {
        usleep(3000);
    }
    if(CheckFinish())
        break;
}

ResetIfRequested();
SetAcceptKeyFrames(true);

if(CheckFinish())
    break;

usleep(3000);
```

* 如果收到 `Stop()` 请求，就暂停工作，直到恢复或结束
* `ResetIfRequested()`：检查是否需要重置
* `usleep(3000)`：休眠 3ms，避免 CPU 占用过高

---

### 8. 结束

```cpp
SetFinish();
```

* 标记线程已结束

---

## 🔹 总结：Run() 的流程

可以概括成下面这条流水线：

1. 等待新关键帧
2. **处理关键帧**：插入地图 → 剔除坏点 → 创建新点
3. **局部优化**：Local BA / Inertial BA
4. **IMU 初始化（如果需要）**
5. **关键帧剔除**（去冗余）
6. **送去回环检测**
7. 处理线程状态（暂停/重置/退出）

👉 它的核心作用就是 **维护局部地图质量**，保证 Tracking 有一个稳定的参考，同时为 LoopClosing 提供干净的关键帧。

---

要不要我帮你画一个 **LocalMapping::Run() 的流程图**，这样比文字更直观？
