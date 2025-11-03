好，这个函数正好是你前面看的 **局部地图追踪流程**的最后一步，叫 `Tracking::SearchLocalPoints()`。
它的作用就是：
👉 把 `UpdateLocalPoints()` 收集到的局部地图点，投影到当前帧中，看哪些能找到匹配，从而增加当前帧和地图的约束。

---

### 逐步讲解

#### ① 清理当前帧已有的 MapPoint 匹配

```cpp
for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(),
                                 vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
{
    MapPoint* pMP = *vit;
    if(pMP)
    {
        if(pMP->isBad())
        {
            *vit = static_cast<MapPoint*>(NULL);
        }
        else
        {
            pMP->IncreaseVisible();
            pMP->mnLastFrameSeen = mCurrentFrame.mnId;
            pMP->mbTrackInView = false;
            pMP->mbTrackInViewR = false;
        }
    }
}
```

* 遍历当前帧已经匹配到的 MapPoints：

  * 如果点是坏点（被剔除了），清空。
  * 否则，更新这个点的一些统计信息：

    * `IncreaseVisible()`：表示这个点在当前帧视野里可见过一次。
    * `mnLastFrameSeen`：记录它最后被看到的帧 ID。
    * `mbTrackInView` / `mbTrackInViewR`：重置可见性标志，后面投影时会更新。

📌 这一步相当于 **清理和初始化当前帧已有点的状态**。

---

#### ② 遍历局部地图点，尝试投影

```cpp
int nToMatch=0;

for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(),
                                 vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
{
    MapPoint* pMP = *vit;

    if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
        continue;
    if(pMP->isBad())
        continue;
```

* 遍历 `UpdateLocalPoints()` 收集到的局部地图点 `mvpLocalMapPoints`。
* 排除：

  * 已经在当前帧看过的点（避免重复）
  * 坏点

---

```cpp
if(mCurrentFrame.isInFrustum(pMP,0.5))
{
    pMP->IncreaseVisible();
    nToMatch++;
}
```

* 调用 `isInFrustum()`，检查点能否投影到当前帧相机视野中。
* 这个函数会计算投影坐标、深度、视角夹角，更新点的投影属性（如 \`mTrack
