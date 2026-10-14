# 小球自由落体：用 MuJoCo 验证牛顿公式

用 MuJoCo 仿真一颗沿竖直方向自由下落的小球，把速度、路程和牛顿公式对照，并画出图表。

## 公式

取竖直向上为正，重力加速度 \(g = 9.81\,\mathrm{m/s^2}\)，小球从高度 \(z_0\) 静止释放：

- 速度：\(v_z(t) = -gt\)
- 高度：\(z(t) = z_0 - \tfrac{1}{2}gt^2\)
- 下落路程：\(s(t) = z_0 - z(t) = \tfrac{1}{2}gt^2\)

地面在 \(z = 0\)。小球半径 0.05 m，从 5 m 下落，大约 1.00 s 后接触地板并弹起。接触参数把回弹调得比较明显：第一次大约能弹回 3 m 以上。\(v = -gt\) 和 \(s = \tfrac{1}{2}gt^2\) 只在落地前成立，落地后仿真曲线会掉头，理论曲线继续按自由落体往下走。小球被限制在 z 轴滑动关节上，质量为 1 kg，不发生转动和水平漂移。积分器为 RK4，步长 0.001 s。默认仿真 4 秒，能看到几次弹起。

## 环境

需要 Python 3.12（本机默认 `python3` 若是 3.14，MuJoCo 轮子可能装不上）。

```bash
/home/stairmed/miniconda3/envs/robot-sim/bin/python -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

如果访问 PyPI 超时，可以改用镜像：

```bash
.venv/bin/python -m pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## 运行

```bash
.venv/bin/python freefall.py
```

终端会打印落地前的最大绝对误差，以及弹起后的最高点。图表写到 `output/`，竖线标出落地时刻：

- `velocity_time.png`：速度随时间，对照 \(v = -gt\)
- `distance_time.png`：下落路程随时间，对照 \(s = \tfrac{1}{2}gt^2\)
- `error.png`：速度误差和路程误差

落地前，速度最大误差应远小于 \(0.01\,\mathrm{m/s}\)，路程最大误差应远小于 \(1\,\mathrm{mm}\)。落地之后两条曲线分开，是因为碰撞改变了速度，自由落体公式不再适用。

加上 `--view` 会再用 MuJoCo 查看器播放下落和反弹（需要图形界面）。播完后窗口保持打开，按 `R` 从初始高度再落一次，关闭窗口后程序才退出：

```bash
.venv/bin/python freefall.py --view
```
