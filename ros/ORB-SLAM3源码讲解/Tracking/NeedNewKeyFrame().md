好，这个函数是 **关键帧选择策略**，名字是 `Tracking::NeedNewKeyFrame()`，用来判断 **当前帧是否需要插入新的关键帧**。
在 SLAM 里，关键帧不是每一帧都建的，而是要根据条件来挑选，保证既有足够的约束，又不会太冗余。

我给你拆分讲一下：

---

## ① IMU 未初始化时的特殊规则

```cpp
if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && 
   !mpAtlas->GetCurrentMap()->isImuInitialized())
{
    if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
        return true;
    else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && 
             (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
        return true;
    else
        return false;
}
```

👉 如果是 **IMU 模式** 且 **IMU 还没初始化**，则只根据 **时间间隔** 判断。

* 单目 IMU：关键帧间隔 > 0.25s 就要新关键帧
* 双目/RGBD IMU：同样 > 0.25s 插入
  这是为了给 IMU 初始化提供足够的观测。

---

## ② 如果只做定位，不建图

```cpp
if(mbOnlyTracking)
    return false;
```

👉 纯定位模式下，**不建新关键帧**。

---

## ③ Local Mapping 被阻塞时不能插入

```cpp
if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    return false;
```

👉 如果局部建图因为 **回环检测 / 闭环优化** 被暂停，那就不能加关键帧。

---

## ④ 如果刚刚重定位过，暂时不要新关键帧

```cpp
const int nKFs = mpAtlas->KeyFramesInMap();
if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    return false;
```

👉 如果当前帧距离上次重定位时间太短，并且地图关键帧数量足够，就不插入。
避免在重定位后过快地插入大量关键帧。

---

## ⑤ 参考关键帧的匹配数

```cpp
int nMinObs = 3;
if(nKFs<=2) nMinObs=2;
int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);
```

👉 统计当前参考关键帧里，有多少 MapPoint 在当前帧还能看到（至少 2\~3 次观测的点）。

这个数会用于后面判断 **当前跟踪是否衰退**。

---

## ⑥ 检查近距离点（只对双目/RGBD）

```cpp
int nNonTrackedClose = 0;
int nTrackedClose= 0;

if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
{
    for(int i =0; i<N; i++)
    {
        if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
        {
            if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                nTrackedClose++;
            else
                nNonTrackedClose++;
        }
    }
}
bool bNeedToInsertClose;
bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);
```

👉 在双目/RGBD 里，可以直接从深度得到“近点”。

* `nTrackedClose`：已经跟踪到的近点数量
* `nNonTrackedClose`：还没跟踪上的近点数量
  如果近点没多少被跟踪，但有很多还可以新建的，那说明需要一个新关键帧来把这些点加入地图。

---

## ⑦ 设置参考比例阈值

```cpp
float thRefRatio = 0.75f;
if(nKFs<2) thRefRatio = 0.4f;
if(mSensor==System::MONOCULAR) thRefRatio = 0.9f;
...
```

👉 这是一个关键的比例，用来比较：
**当前帧跟踪到的点数 vs 参考关键帧的点数**。
不同模式（单目 / 双目 / IMU）会设不同的比例。

---

## ⑧ 插入关键帧的几个条件

```cpp
// 条件1a：距离上个关键帧太久了
const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;

// 条件1b：超过最小帧间隔，并且局部建图空闲
const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle);

// 条件1c：跟踪变弱（非单目类）
const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && 
                 mSensor!=System::IMU_STEREO && mSensor!=System::IMU_RGBD && 
                 (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose);

// 条件2：当前跟踪到的点数，相比参考关键帧下降太多（比例 < thRefRatio）
const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) 
                && mnMatchesInliers>15;
```

额外条件：

* **c3**：IMU 模式下，关键帧时间间隔超过 0.5s
* **c4**：IMU 单目，点数下降到 75 以下，或刚刚 lost

---

## ⑨ 最终判断

```cpp
if(((c1a||c1b||c1c) && c2)||c3 ||c4)
{
    if(bLocalMappingIdle || mpLocalMapper->IsInitializing())
        return true;
    else
    {
        mpLocalMapper->InterruptBA();
        if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
        {
            if(mpLocalMapper->KeyframesInQueue()<3)
                return true;
            else
                return false;
        }
        else
            return false;
    }
}
else
    return false;
```

👉 总结：

* 如果 **(c1a/c1b/c1c) + c2** 成立，或者 (c3/c4) 成立，就考虑建新关键帧。
* 如果局部建图空闲，直接建；
* 如果建图线程忙，就打断全局 BA（Bundle Adjustment），尽量让它能接收关键帧。
* 如果队列里关键帧太多，就暂时不建。

---

## 🌟 总结

这个函数的核心思想就是：

1. **保证建图需要**：点数下降、近点未利用、时间间隔过长
2. **保证跟踪需要**：如果 tracking 变弱，就赶紧建新关键帧
3. **保证系统稳定**：不要在建图阻塞 / 刚重定位 / 队列太满时乱加

---
