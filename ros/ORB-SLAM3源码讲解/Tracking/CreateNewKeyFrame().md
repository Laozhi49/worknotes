好的，这个 `Tracking::CreateNewKeyFrame()` 是 **跟踪线程在决定需要新关键帧后，真正生成关键帧并交给 Local Mapping 的过程**。我帮你分步骤详细拆开：

---

### 🔹 1. 先决条件检查

```cpp
if(mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
    return;

if(!mpLocalMapper->SetNotStop(true))
    return;
```

* 如果 Local Mapping 还在初始化并且 IMU 还没有完成初始化 → 不允许创建关键帧。
* 调用 `SetNotStop(true)`，保证 Local Mapping 在关键帧创建时不会被外部打断。

---

### 🔹 2. 创建关键帧对象

```cpp
KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
```

* 用 **当前帧** 构造出一个 **关键帧**，并加入当前的地图和关键帧数据库。

```cpp
if(mpAtlas->isImuInitialized()) 
    pKF->bImu = true;

pKF->SetNewBias(mCurrentFrame.mImuBias);
mpReferenceKF = pKF;
mCurrentFrame.mpReferenceKF = pKF;
```

* 如果 IMU 已经初始化，给关键帧标记 `bImu = true`，并设置 IMU 偏置。
* 更新参考关键帧（reference KF）为当前新建的关键帧。

---

### 🔹 3. 关键帧之间的链式关系

```cpp
if(mpLastKeyFrame)
{
    pKF->mPrevKF = mpLastKeyFrame;
    mpLastKeyFrame->mNextKF = pKF;
}
```

* 把这个新关键帧和上一个关键帧链接起来，形成一个 KF 链。

---

### 🔹 4. IMU 预积分对象重置

```cpp
if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
{
    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);
}
```

* 创建一个新的 IMU 预积分对象，用于之后 **从这个关键帧开始积累 IMU 数据**。

---

### 🔹 5. 双目/RGB-D 情况下生成新的 MapPoints

```cpp
if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR)
{
    mCurrentFrame.UpdatePoseMatrices();
    ...
}
```

如果是 **双目或 RGBD**，则利用深度信息从当前帧生成新的 MapPoints：

* 先把有深度的特征点 `(depth > 0)` 按深度排序。
* 遍历这些点：

  * 如果该点没有对应的 MapPoint，或 MapPoint 没有有效观测，就新建一个。
  * 用当前帧的投影函数 `UnprojectStereo` 或 `UnprojectStereoFishEye` 把像素点反投影成 3D 点。
  * 创建 `MapPoint`，加入到当前关键帧和地图。
  * 如果有左右目匹配，则同时在右目添加对应观测。
  * 更新该 MapPoint 的描述子、法向量和深度范围。

这样保证新关键帧不仅存储图像，还能带来新的三维点。

---

### 🔹 6. 把关键帧交给 Local Mapping

```cpp
mpLocalMapper->InsertKeyFrame(pKF);
mpLocalMapper->SetNotStop(false);
```

* 把新关键帧交给 Local Mapping 线程去处理（扩展地图、局部 BA）。
* 解除之前对 Local Mapping 的“不能停”的限制。

---

### 🔹 7. 更新 Tracking 的状态

```cpp
mnLastKeyFrameId = mCurrentFrame.mnId;
mpLastKeyFrame = pKF;
```

* 记录最后关键帧 ID 和指针。

---

## 📌 总结

`CreateNewKeyFrame()` 的主要作用：

1. **生成一个新的关键帧对象**（包含当前帧的所有信息）。
2. **建立关键帧链式关系**，并更新参考关键帧。
3. **重置 IMU 预积分**，为后续优化准备。
4. **双目/RGBD 情况下创建新的 MapPoints**，扩展地图。
5. **交给 Local Mapping** 线程去处理地图更新和优化。

👉 可以理解为：Tracking 判断需要新关键帧 → `NeedNewKeyFrame()` → 调用 `CreateNewKeyFrame()` → 交给 Local Mapping。

---
