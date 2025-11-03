这个函数 **`Tracking::TrackReferenceKeyFrame()`** 是 ORB-SLAM3 在正常追踪时的一种位姿估计方式：
👉 **当前帧和参考关键帧（Reference KeyFrame）做匹配，用 PnP 来估计相机位姿**。

我给你逐段拆开解释：

---

## 1️⃣ 计算 BoW 向量

```cpp
mCurrentFrame.ComputeBoW();
```

* 给当前帧计算 **Bag of Words (BoW)** 直方图。
* 这个向量用来加速和参考关键帧的特征匹配。
* ORB-SLAM 里用了 **DBoW2** 库，能快速找出两帧共享的词汇并匹配 ORB 特征。

---

## 2️⃣ BoW 匹配

```cpp
ORBmatcher matcher(0.7,true);
vector<MapPoint*> vpMapPointMatches;

int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
```

* 建立一个 ORB 特征匹配器（汉明距离阈值比例 0.7）。
* 调用 `SearchByBoW`：

  * 在 **参考关键帧 mpReferenceKF** 和 **当前帧 mCurrentFrame** 的 BoW 向量里，寻找候选匹配。
  * 返回匹配数 `nmatches`，以及匹配上的 MapPoint 集合 `vpMapPointMatches`。

检查匹配数：

```cpp
if(nmatches<15) {
    cout << "TRACK_REF_KF: Less than 15 matches!!\n";
    return false;
}
```

👉 匹配点太少（<15），认为追踪失败。

---

## 3️⃣ 初始化位姿

```cpp
mCurrentFrame.mvpMapPoints = vpMapPointMatches;
mCurrentFrame.SetPose(mLastFrame.GetPose());
```

* 把 BoW 匹配结果保存到当前帧的 MapPoint 容器。
* 初始化当前帧的位姿为上一帧的位姿（提供优化的初值）。

---

## 4️⃣ PnP 位姿优化

```cpp
Optimizer::PoseOptimization(&mCurrentFrame);
```

* 调用优化器，用 `mvpMapPoints` 和特征匹配做 **PnP + BA 优化**。
* 目标：优化出当前帧的相机位姿 `Tcw`。

---

## 5️⃣ 去除外点

```cpp
for(int i =0; i<mCurrentFrame.N; i++)
{
    if(mCurrentFrame.mvpMapPoints[i])
    {
        if(mCurrentFrame.mvbOutlier[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

            mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            mCurrentFrame.mvbOutlier[i]=false;

            if(i < mCurrentFrame.Nleft){
                pMP->mbTrackInView = false;
            }
            else{
                pMP->mbTrackInViewR = false;
            }

            pMP->mbTrackInView = false;
            pMP->mnLastFrameSeen = mCurrentFrame.mnId;
            nmatches--;
        }
        else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
            nmatchesMap++;
    }
}
```

解释：

* 遍历所有 MapPoint 匹配
* 如果某个点被优化判定为外点（`mvbOutlier[i] == true`）：

  * 把它从当前帧匹配中移除
  * 修改 MapPoint 的一些追踪标志位（比如不再认为在视野里）
* 如果是合法的点（观测数 > 0），就统计到 `nmatchesMap`（有效匹配数）。

---

## 6️⃣ 成功判定

```cpp
if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    return true;
else
    return nmatchesMap>=10;
```

* 如果用了 IMU（单目/双目/RGBD+IMU），直接返回 `true`（IMU 能提供额外约束，要求低一点）。
* 如果是纯视觉模式（单目/双目/RGBD 没 IMU），则需要 **≥10 个有效匹配点** 才算成功。

---

## ✅ 总结

`TrackReferenceKeyFrame()` 的完整流程：

1. 用 BoW 向量在参考关键帧和当前帧之间找匹配
2. 匹配数足够 → 初始化位姿（用上一帧）
3. 做 PnP 位姿优化（BA）
4. 剔除外点，只保留有效匹配
5. 判断是否追踪成功（≥10 inliers 或 IMU 模式）

👉 **作用**：在 Tracking 阶段，如果没有速度模型可用（或者失败），就用参考关键帧来定位当前帧。

---


# Optimizer::PoseOptimization(&mCurrentFrame)的原理

---

# 一句话概述

`PoseOptimization(frame)` 是 **给定当前帧与若干 3D MapPoints 的 2D-3D 对应关系，优化该帧的相机位姿 $T_{cw}$**，使所有观测的重投影误差最小化（用 g2o 做非线性最小二乘，带鲁棒核并且剔除外点）。

---

# 输入 / 输出

* **输入**

  * `mCurrentFrame.mvpMapPoints`：每个特征点对应的 MapPoint（或 `NULL`）
  * `mCurrentFrame.mvKeys` / `mvKeysRight`：该帧的关键点像素坐标（单目 / 立体）
  * 相机内参（fx, fy, cx, cy）以及是否为 stereo / rgbd / monocular
  * 初值：`mCurrentFrame` 当前的位姿估计（通常来自运动模型或上一帧）
* **输出**

  * 更新后的 `mCurrentFrame` 位姿（`SetPose()`）
  * `mCurrentFrame.mvbOutlier[i]` 标记哪些匹配被判为外点
  * （间接）可能清除一些 `mCurrentFrame.mvpMapPoints[i]`

---

# 高层步骤（实际源码中就是这些操作）

1. **构建优化器（g2o）**

   * 创建 `g2o::SparseOptimizer`、选择线性求解器与求解算法（通常是 Levenberg-Marquardt）。
2. **添加位姿顶点**

   * 新建一个 `VertexSE3`（或 `VertexSE3Expmap`），id = 0，初值设为 `mCurrentFrame` 的 pose，设置为可优化。
3. **针对每个有效 MapPoint 添加投影边**

   * 根据传感器类型选择边的类型（单目：`EdgeProjectXYZ2UV`；立体/双目：`EdgeStereoProjectXYZ` / 特殊边类）。
   * 每条边包含的东西：

     * 观测量（像素坐标 u,v，立体时还可能有右目或深度信息）
     * 被固定的 3D 点（MapPoint 的世界坐标）
     * 相机内参（作为边的参数或全局参数）
     * 信息矩阵（通常是单位矩阵 / 以观测噪声为依据）
     * 鲁棒核（Huber）用来弱化异常值影响
4. **第一次优化（若干次迭代）**

   * `optimizer.initializeOptimization()`，`optimizer.optimize(n_iters)`。
5. **计算每条边的残差/chi2，标记外点**

   * 若某条边的重投影误差超过阈值（基于 chi2 或像素阈值），把该匹配标记为外点：`mCurrentFrame.mvbOutlier[i] = true`，并在当前帧中把该 MapPoint 置空（或暂时保留但标记）。
6. **删除/屏蔽外点的边，重新优化（可选）**

   * 用去掉外点后的边再次优化（通常再做一次或两次迭代）。
7. **把优化结果写回 `mCurrentFrame`**

   * 更新位姿 `mCurrentFrame.SetPose(优化后的位姿)`。
   * `mvbOutlier` 表示最终哪些匹配被视为 outlier。

---

# 数学上是什么在最小化？

对每个观测 i（一个 2D 测量 $z_i=(u_i,v_i)$）和对应 3D 点 $X_i$：

$$
e_i = z_i - \pi( T_{cw} \cdot X_i )
$$

其中 $\pi(\cdot)$ 是投影函数：

$$
u = f_x \frac{X_c}{Z_c} + c_x, \quad v = f_y \frac{Y_c}{Z_c} + c_y
$$

目标是最小化带权重的平方和（并使用 Huber 抑制异常值）：

$$
\min_{T_{cw}} \sum_i \rho( e_i^T \Omega e_i )
$$

$\rho$ 为 Huber，$\Omega$ 为信息矩阵（一般取单位或根据像素噪声给定）。

求解器会线性化误差对位姿参数（SE3 参数化）并做迭代的高斯-牛顿 / LM 更新。

---

# 关于外点判定（实现细节）

* ORB-SLAM 系列常用流程：先一次完整优化，再根据每条边的 chi2（或投影误差像素值）判断是否超出阈值（单目通常用 2 DOF 的卡方阈值或像素阈值），被判为外点后把该匹配从帧中移除（`mCurrentFrame.mvpMapPoints[i]=NULL; mvbOutlier[i]=true`），然后再优化一次以稳固结果。
* 阈值实现细节（源码里可能写具体数字，例如 `th2 = 5.991` 或像素阈值 `5.0`）——具体数值会依实现版本有所不同，但思想如上。

---

# 关于不同传感器的区别

* **单目**：边只包含 2D 测量 `(u,v)`，自由度 2，用 2D reprojection error。
* **立体 (stereo)** / **RGB-D**：可以使用额外的深度或右目信息，边可以是 3D 投影或带深度的残差，使优化更稳定（等价于提供更强的约束）。
* 在 ORB-SLAM3，如果 IMU 存在且需要融合，则使用 `PoseInertialOptimization...` 类型的函数（会加入 IMU 预积分残差），而 `PoseOptimization` 保持**纯视觉**优化。

---

# 伪代码（便于你对照源码）

```cpp
bool Optimizer::PoseOptimization(Frame* pFrame) {
    // 1. 构建 g2o 优化器（LM）
    optimizer = createG2oOptimizer();

    // 2. 添加 pose 顶点 (id=0)
    VertexSE3* vSE3 = new VertexSE3();
    vSE3->setEstimate(pFrame->GetPose());
    optimizer.addVertex(vSE3);

    // 3. 为每个有对应 MapPoint 的观测添加边
    for(i = 0; i < pFrame->N; ++i) {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(!pMP || pMP->isBad()) continue;

        EdgeProjectXYZ2UV* e = new EdgeProjectXYZ2UV(pMP->GetWorldPos(), cameraParams);
        e->setVertex(0, vSE3);
        e->setMeasurement(pFrame->mvKeys[i].pt); // u,v
        e->setInformation(I); // 信息矩阵
        e->setRobustKernel(new Huber());
        optimizer.addEdge(e);
    }

    // 4. 优化（第一次）
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // 5. 标记外点
    for(each edge e) {
        if(e->chi2() > thresh) {
            mark mFrame.mvbOutlier[idx] = true;
            // optionally remove edge
        } else {
            mark inlier
        }
    }

    // 6. 再次优化（去除外点）
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // 7. 写回位姿与 outliers
    pFrame->SetPose(vSE3->estimate());
    return true;
}
```

---

# 其它实现细节与注意点

* **单顶点优化**：只有相机位姿这个变量被优化（MapPoint 是固定的）。这使优化问题规模小、速度快。
* **鲁棒核（Huber）**：在初期抑制离群点影响，后面真正用 chi2 判定剔除。
* **权重信息**：有时会根据特征尺度层级给不同信息权重（scales），但常见是使用统一权重并用 Huber。
* **与 RANSAC 的关系**：`PoseOptimization` 通常不是 RANSAC PnP，而是在已知足够多匹配（或 PnP 初值）的前提下做非线性优化。Relocalization 中会先用 PnP+RANSAC 找初值，再调用优化；TrackReferenceKeyFrame 里通常先做 BoW 匹配然后直接 `PoseOptimization`（有合适初值）。

---

# 举例：为什么要两轮优化？

第一次优化后可以找到大部分符合模型的 inliers，但初始时 outliers 仍会影响雅可比矩阵的估计。剔除明显 outliers 后，第二次优化收敛更稳、更准确。

---

如果你想更进一步，我可以：

* 把你本地 ORB-SLAM3 的 `Optimizer::PoseOptimization` 源码逐行注释（你可以贴源码片段，我来逐行解释）；
* 或者演示一个最小的示例（用 g2o）来实现单帧的 PoseOptimization，让你可以在本地编译运行并观测中间残差 / chi2 值。


