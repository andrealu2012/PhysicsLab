"""小球自由落体：用 MuJoCo 仿真对照牛顿公式。"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "models" / "freefall_ball.xml"
OUTPUT_DIR = ROOT / "output"

G = 9.81
DURATION = 4.0


def configure_font():
    """优先使用系统中文字体，找不到则回退英文标签。"""
    from matplotlib import font_manager

    candidates = [
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Source Han Sans SC",
        "WenQuanYi Micro Hei",
        "WenQuanYi Zen Hei",
        "Droid Sans Fallback",
    ]
    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name]
            plt.rcParams["axes.unicode_minus"] = False
            return True
    return False


def simulate(duration: float) -> dict[str, np.ndarray]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
    mujoco.mj_forward(model, data)

    z0 = float(data.xpos[ball_id, 2])
    n_steps = int(round(duration / model.opt.timestep))
    times = np.empty(n_steps + 1)
    heights = np.empty(n_steps + 1)
    velocities = np.empty(n_steps + 1)
    contacts = np.empty(n_steps + 1)

    times[0] = data.time
    heights[0] = z0
    velocities[0] = data.qvel[0]
    contacts[0] = data.ncon

    for i in range(1, n_steps + 1):
        mujoco.mj_step(model, data)
        times[i] = data.time
        heights[i] = data.xpos[ball_id, 2]
        velocities[i] = data.qvel[0]
        contacts[i] = data.ncon

    gravity = -float(model.opt.gravity[2])
    return {
        "t": times,
        "z": heights,
        "vz": velocities,
        "z0": np.array(z0),
        "g": np.array(gravity),
        "s": z0 - heights,
        "vz_theory": -gravity * times,
        "s_theory": 0.5 * gravity * times**2,
        "contact": contacts,
    }


def impact_index(result: dict[str, np.ndarray]) -> int:
    hit = np.flatnonzero(result["contact"] > 0)
    return int(hit[0]) if len(hit) else len(result["t"])


def report(result: dict[str, np.ndarray]) -> None:
    t = result["t"]
    hit = impact_index(result)
    free = slice(0, hit)
    vz_err = np.abs(result["vz"][free] - result["vz_theory"][free])
    s_err = np.abs(result["s"][free] - result["s_theory"][free])
    g = float(result["g"])
    z0 = float(result["z0"])

    print(f"重力加速度 g = {g:.2f} m/s^2")
    print(f"初始高度 z0 = {z0:.3f} m，仿真时长 {t[-1]:.3f} s，步数 {len(t) - 1}")
    if hit < len(t):
        print(f"首次接触地板 t = {t[hit]:.3f} s")
    print()
    print("落地前，仿真与自由落体公式的最大绝对误差")
    print(f"  速度 {vz_err.max():.3e} m/s")
    print(f"  路程 {s_err.max():.3e} m")
    if hit < len(t):
        print()
        print(f"落地后小球弹起，最高回到 z = {result['z'][hit:].max():.3f} m")
        print("v = -gt 和 s = (1/2)gt² 只描述落地前的自由落体，所以落地后两条曲线会分开")


def save_plots(result: dict[str, np.ndarray], use_chinese: bool) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    t = result["t"]
    hit = impact_index(result)
    t_hit = float(t[min(hit, len(t) - 1)])
    vz_err = result["vz"] - result["vz_theory"]
    s_err = result["s"] - result["s_theory"]
    impact_label = "落地" if use_chinese else "impact"

    if use_chinese:
        labels = {
            "t": "时间 t (s)",
            "vz": "速度 v_z (m/s)",
            "s": "下落路程 s (m)",
            "sim": "仿真",
            "theory_v": "理论  v = -gt",
            "theory_s": "理论  s = (1/2)gt²",
            "verr": "速度误差 (m/s)",
            "serr": "路程误差 (m)",
        }
    else:
        labels = {
            "t": "time t (s)",
            "vz": "velocity v_z (m/s)",
            "s": "distance s (m)",
            "sim": "simulation",
            "theory_v": "theory  v = -gt",
            "theory_s": "theory  s = (1/2)gt^2",
            "verr": "velocity error (m/s)",
            "serr": "distance error (m)",
        }

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(t, result["vz"], label=labels["sim"], linewidth=2)
    ax.plot(t, result["vz_theory"], "--", label=labels["theory_v"], linewidth=1.5)
    ax.axvline(t_hit, color="0.4", linestyle=":", label=impact_label)
    ax.set_xlabel(labels["t"])
    ax.set_ylabel(labels["vz"])
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "velocity_time.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(t, result["s"], label=labels["sim"], linewidth=2)
    ax.plot(t, result["s_theory"], "--", label=labels["theory_s"], linewidth=1.5)
    ax.axvline(t_hit, color="0.4", linestyle=":", label=impact_label)
    ax.set_xlabel(labels["t"])
    ax.set_ylabel(labels["s"])
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "distance_time.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    axes[0].plot(t, vz_err, color="C3")
    axes[0].axvline(t_hit, color="0.4", linestyle=":")
    axes[0].set_ylabel(labels["verr"])
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, s_err, color="C3")
    axes[1].axvline(t_hit, color="0.4", linestyle=":")
    axes[1].set_xlabel(labels["t"])
    axes[1].set_ylabel(labels["serr"])
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "error.png", dpi=150)
    plt.close(fig)

    print()
    print(f"图表已保存到 {OUTPUT_DIR}")


def view(duration: float) -> None:
    import threading
    import time

    import glfw
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    reset_requested = threading.Event()

    def on_key(keycode: int) -> None:
        if keycode == glfw.KEY_R:
            reset_requested.set()

    print("查看器中按 R 重新下落。小球落到地板上会弹起，关闭窗口后程序退出")
    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        while viewer.is_running():
            step_start = time.time()
            if reset_requested.is_set():
                reset_requested.clear()
                mujoco.mj_resetData(model, data)
                mujoco.mj_forward(model, data)
            if data.time < duration:
                mujoco.mj_step(model, data)
                viewer.sync()
                remaining = model.opt.timestep - (time.time() - step_start)
                if remaining > 0:
                    time.sleep(remaining)
            else:
                viewer.sync()
                time.sleep(0.02)


def main() -> None:
    parser = argparse.ArgumentParser(description="小球自由落体，对照牛顿公式")
    parser.add_argument("--view", action="store_true", help="用 MuJoCo 查看器观看下落")
    parser.add_argument("--duration", type=float, default=DURATION, help="仿真时长（秒）")
    args = parser.parse_args()

    use_chinese = configure_font()
    result = simulate(args.duration)
    report(result)
    save_plots(result, use_chinese)
    if args.view:
        view(args.duration)


if __name__ == "__main__":
    main()
