用 OpenCV 的 `stereoRectify`，输入已有的 **K、D、Tlr** 和图像大小，直接输出 `R1, R2, P1, P2, Q`。运行后就能得到 ORB-SLAM3 需要的 `R` 和 `P` 矩阵。

```python
import numpy as np
import cv2

# ---------- 左相机参数 ----------
K1 = np.array([[496.7810, 0, 304.7981],
               [0, 499.4897, 234.9977],
               [0, 0, 1]], dtype=np.float64)

D1 = np.array([0.0792, -0.0985, 0.0, 0.0, 0.0], dtype=np.float64)  # k1, k2, p1, p2, k3

# ---------- 右相机参数 ----------
K2 = np.array([[499.5256, 0, 335.2932],
               [0, 502.6719, 236.7066],
               [0, 0, 1]], dtype=np.float64)

D2 = np.array([0.0913, -0.1335, 0.0, 0.0, 0.0], dtype=np.float64)

# ---------- 相机外参 (Tlr: 左到右) ----------
Tlr = np.array([[1.0000, -0.0002,  0.0053, -0.0597883],
                [0.0002,  1.0000,  0.0029,  0.0000216],
                [-0.0053, -0.0029, 1.0000, -0.0000496],
                [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)

R = Tlr[:3, :3]
t = Tlr[:3, 3]

# ---------- 图像大小 ----------
image_size = (640, 480)  # (width, height)

# ---------- 立体校正 ----------
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, image_size, R, t,
                                            flags=cv2.CALIB_ZERO_DISPARITY,
                                            alpha=-1)

# ---------- 打印结果 ----------
def print_cv_mat(name, mat):
    print(f"{name}: !!opencv-matrix")
    print(f"   rows: {mat.shape[0]}")
    print(f"   cols: {mat.shape[1]}")
    print("   dt: d")
    print("   data: [" + ", ".join([f"{v:.12g}" for v in mat.flatten()]) + "]\n")

print_cv_mat("LEFT.R", R1)
print_cv_mat("LEFT.P", P1)
print_cv_mat("RIGHT.R", R2)
print_cv_mat("RIGHT.P", P2)
print_cv_mat("Q", Q)
```

运行后会直接在终端里输出 OpenCV YAML 格式的 `R` 和 `P`，复制到配置文件里就能用了。

⚠️ 注意：

* `flags=cv2.CALIB_ZERO_DISPARITY` 表示让左右相机的主点对齐（常用于深度估计）。
* `alpha=-1` 表示尽量保留所有视野（可改成 `0` 保证无黑边）。

