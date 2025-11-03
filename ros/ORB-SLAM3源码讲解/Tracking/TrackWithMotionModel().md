这个 `Tracking::TrackWithMotionModel()` 是 ORB-SLAM 系列里 **利用运动模型预测位姿并进行跟踪** 的关键函数。相比 `TrackReferenceKeyFrame()` 和 `TrackLocalMap()`，它的特点是：
👉 用上一次的位姿和速度来预测当前帧位姿，再通过投影搜索找到 MapPoint 约束，最后做优化。

我给你分成几个逻辑块来讲：

---

## 1. **准备工作**

```cpp
ORBmatcher matcher(0.9,true);
UpdateLastFrame();
```

* 新建 ORB 特征匹配器，阈值 0.9，开启方向一致性检查。
* `UpdateLastFrame()`：更新上一帧的位姿，使它和参考关键帧保持一致；如果是 Localization 模式，还可能把临时 VO 点转成 MapPoints。

---

## 2. **位姿预测**

```cpp
if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
{
    PredictStateIMU();
    return true;
}
else
{
    mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
}
```

* 如果 IMU 已初始化，并且当前帧在重定位恢复后过了一段安全时间：

  * 使用 IMU 预积分预测当前帧的状态（位姿 + 速度 + bias），直接返回，不用视觉匹配。
* 否则（比如单目 / 立体 / RGBD 没有 IMU 或刚重定位完还不能信 IMU）：

  * 用运动模型预测：

    $$
    T_{cw}^{pred} = v \cdot T_{cw}^{last}
    $$

    其中 `mVelocity` 是上一帧和再上一帧之间的相对位姿。

---

## 3. **清空 MapPoint 引用**

```cpp
fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
```

* 先把当前帧的 `mvpMapPoints` 清空，准备重新匹配。

---

## 4. **基于投影的匹配**

```cpp
int th;
if(mSensor==System::STEREO) th=7;
else th=15;

int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,
                       mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
```

* 把上一帧的 MapPoints 投影到当前帧的图像上，在预测位置附近搜索 ORB 特征，建立匹配。
* 搜索窗口大小：立体时 7 像素，单目时 15 像素（更大，因为深度不确定）。
* 如果匹配数太少（`<20`），再扩大窗口（2 倍）。

---

## 5. **匹配数不足的处理**

```cpp
if(nmatches<20)
{
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        return true;
    else
        return false;
}
```

* 如果两次搜索后还是太少：

  * 对带 IMU 的模式，返回 `true`（因为还可以依赖 IMU 预测维持位姿）。
  * 否则（纯视觉）返回 `false`，跟踪失败。

---

## 6. **位姿优化**

```cpp
Optimizer::PoseOptimization(&mCurrentFrame);
```

* 用匹配到的 MapPoints 对当前帧位姿做一次非线性优化（重投影误差最小化）。

---

## 7. **剔除外点**

```cpp
for(int i =0; i<mCurrentFrame.N; i++)
{
    if(mCurrentFrame.mvpMapPoints[i])
    {
        if(mCurrentFrame.mvbOutlier[i])
        {
            // 去掉外点
            ...
            nmatches--;
        }
        else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
            nmatchesMap++;
    }
}
```

* 遍历当前帧所有 MapPoint：

  * 如果被标记为外点（投影误差太大），就清空。
  * 否则，如果这个点在地图里至少有一次有效观测，则 `nmatchesMap++`（计作可靠 inlier）。

---

## 8. **结果判定**

```cpp
if(mbOnlyTracking)
{
    mbVO = nmatchesMap<10;
    return nmatches>20;
}

if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    return true;
else
    return nmatchesMap>=10;
```

* 如果在 “仅跟踪模式”（Localization Mode）：

  * 如果内点少于 10，就标记成 Visual Odometry（纯 VO，没有和地图强约束）。
  * 返回 `nmatches > 20`。
* 如果有 IMU：直接返回 true（即使 inliers 少，也能靠 IMU 保持）。
* 否则（纯视觉）：需要至少 10 个地图内点才算成功。

---

## 🌱 总结

`TrackWithMotionModel()` 的流程：

1. 用 IMU 或运动模型预测位姿；
2. 通过投影搜索匹配 MapPoints；
3. 如果匹配点数足够 → 优化当前帧位姿 → 剔除外点 → 检查 inliers 数量；
4. 根据传感器类型和结果返回是否成功跟踪。

---

直观理解：

> 这是一个 **短期跟踪方法**：不依赖于关键帧或全局地图，只用上一帧的运动模型 + 临时匹配来“撑住”Tracking。能保证在帧间相机运动平稳的情况下，不会频繁丢失。

---

