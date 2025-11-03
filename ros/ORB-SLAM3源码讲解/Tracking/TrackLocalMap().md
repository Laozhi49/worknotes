非常好 👍，你现在把 `Tracking::TrackLocalMap()` 拿出来看了，这是 ORB-SLAM 里面 **最核心的 tracking 步骤** ——在有了初始位姿估计后，利用 **局部地图 (local map)** 来优化当前帧的位姿。

我来逐段给你详细讲一下逻辑：

---

## 1. 初始化和局部地图更新

```cpp
mTrackedFr++;

UpdateLocalMap();
SearchLocalPoints();
```

* `mTrackedFr++`：跟踪帧数计数器。
* `UpdateLocalMap()`：更新局部地图，通常会找出当前帧相关关键帧（共视关键帧 + 相邻关键帧），以及这些关键帧里的 MapPoints，作为候选局部地图点。
* `SearchLocalPoints()`：把这些局部地图点投影到当前帧，找匹配（只要在视野内并且尺度/角度合适，就尝试匹配）。
  👉 这样 `mCurrentFrame` 就会拥有更多候选的 MapPoints，而不仅仅是来自上一帧。

---

## 2. 先统计一下已有匹配点情况

```cpp
int aux1 = 0, aux2=0;
for(int i=0; i<mCurrentFrame.N; i++)
    if(mCurrentFrame.mvpMapPoints[i])
    {
        aux1++;
        if(mCurrentFrame.mvbOutlier[i])
            aux2++;
    }
```

* `aux1`：当前帧一共匹配到了多少 MapPoints；
* `aux2`：其中有多少被认为是外点（比如投影误差太大）。
  这里只是 debug 统计，不影响后续逻辑。

---

## 3. 位姿优化

```cpp
int inliers;
if (!mpAtlas->isImuInitialized())
    Optimizer::PoseOptimization(&mCurrentFrame);
else
{
    if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
    {
        Optimizer::PoseOptimization(&mCurrentFrame);
    }
    else
    {
        if(!mbMapUpdated)
            inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame);
        else
            inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame);
    }
}
```

* 如果没有 IMU：只做视觉 BA 优化（`PoseOptimization`，PnP + BA）。
* 如果有 IMU：

  * 在重定位后的几帧里，还是只依赖视觉优化；
  * 之后则用 **视觉 + IMU 联合优化**，分两种情况：

    * `PoseInertialOptimizationLastFrame()`：和上一帧一起做 IMU 约束优化；
    * `PoseInertialOptimizationLastKeyFrame()`：和参考关键帧一起做 IMU 优化。

👉 这里是 ORB-SLAM3 相比 ORB-SLAM2 最大的改进点之一（紧耦合视觉-惯性）。

---

## 4. 再统计一次匹配结果

```cpp
aux1 = 0, aux2 = 0;
for(int i =0; i<mCurrentFrame.N; i++)
    if(mCurrentFrame.mvpMapPoints[i])
    {
        aux1++;
        if(mCurrentFrame.mvbOutlier[i])
            aux2++;
    }
```

* 优化后，外点会被标记（残差太大），所以再统计一次。

---

## 5. 更新 MapPoint 统计信息

```cpp
mnMatchesInliers = 0;
for(int i =0; i<mCurrentFrame.N; i++)
{
    if(mCurrentFrame.mvpMapPoints[i])
    {
        if(!mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
            if(!mbOnlyTracking)
            {
                if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    mnMatchesInliers++;
            }
            else
                mnMatchesInliers++;
        }
        else if(mSensor==System::STEREO)
            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
    }
}
```

* 遍历当前帧的所有 MapPoint：

  * 如果是 inlier，就增加该点的 “被找到次数”；
  * 如果不是 only-tracking 模式，还要求这个点在地图中有至少一次有效观测才算 inlier；
  * 否则，在 localization-only 模式下，即使是临时 VO 点，也计入。
* 对于 stereo 传感器，如果是外点，直接清空 MapPoint。

最终 `mnMatchesInliers` 表示 **当前帧有效匹配点数量**。

---

## 6. 成功判定

```cpp
mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    return false;

if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
    return true;
```

* 如果刚刚重定位（当前帧在重定位后的 `mMaxFrames` 范围内），就要求更严格：至少 50 个 inlier 才算成功；
* 如果之前状态是 `RECENTLY_LOST`，那只要 inliers > 10，就认为 tracking 成功（宽松一些，帮助恢复）。

---

## 7. 针对不同传感器的阈值

```cpp
if (mSensor == System::IMU_MONOCULAR)
{
    if((mnMatchesInliers<15 && mpAtlas->isImuInitialized())||(mnMatchesInliers<50 && !mpAtlas->isImuInitialized()))
    {
        return false;
    }
    else
        return true;
}
else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
{
    if(mnMatchesInliers<15)
        return false;
    else
        return true;
}
else
{
    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}
```

* **IMU + 单目**：

  * 如果 IMU 已初始化，至少要 15 个 inliers；
  * 如果 IMU 没初始化，则要 50 个（因为不能信 IMU）。
* **IMU + 立体 / RGBD**：至少要 15 个 inliers。
* **纯视觉单目 / 立体**：至少要 30 个 inliers。

---

## 🌱 总结

`TrackLocalMap()` 是整个 tracking 最关键的一步，它做了：

1. 从当前帧的参考关键帧和共视关系里找出局部地图点；
2. 投影匹配，把更多地图点和当前帧建立联系；
3. 根据传感器情况（有无 IMU），做位姿优化；
4. 剔除外点，统计 inliers；
5. 根据 inliers 数量、重定位状态、传感器类型，判断 tracking 是否成功。

👉 可以理解为：

* `TrackWithMotionModel` 主要靠 **短时连续性**（上一帧）。
* `TrackReferenceKeyFrame` 靠 **参考关键帧**。
* `TrackLocalMap` 靠 **地图上下文**，把前两者的估计 refine 成最终位姿。

---
