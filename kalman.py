#!/usr/bin/python

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt


def simulate_1d_robot(steps: int, seed: int, q: float, r: float):
    """
    1次元ロボットの真値と観測を生成する。

    状態遷移:
      x_t = x_{t-1} + u_t + w_t,   w_t ~ N(0, Q=q)

    観測:
      z_t = x_t + v_t,             v_t ~ N(0, R=r)

    入力 u_t は「±1m」を既知入力として扱う（どっちに動いたかは分かっている設定）。
    """
    rng = np.random.default_rng(seed)

    # 入力（行動）: ±1
    u = rng.choice([-1.0, 1.0], size=steps)

    x_true = np.zeros(steps, dtype=float)
    z_meas = np.zeros(steps, dtype=float)

    x = 0.0
    for t in range(steps):
        w = rng.normal(0.0, np.sqrt(q))  # process noise
        x = x + u[t] + w

        v = rng.normal(0.0, np.sqrt(r))  # measurement noise
        z = x + v

        x_true[t] = x
        z_meas[t] = z

    return u, x_true, z_meas


def kalman_filter_1d(u, z, x0=0.0, p0=1.0, q=0.2, r=2.0):
    """
    1次元カルマンフィルタ（F=1, H=1, B=1 の基本形）

    Predict:
      x_pred = x + u
      p_pred = p + q

    Update:
      k = p_pred / (p_pred + r)
      x = x_pred + k * (z - x_pred)
      p = (1 - k) * p_pred
    """
    steps = len(z)
    x_est = np.zeros(steps, dtype=float)
    p_est = np.zeros(steps, dtype=float)
    k_hist = np.zeros(steps, dtype=float)
    x_pred_hist = np.zeros(steps, dtype=float)

    x = float(x0)
    p = float(p0)

    for t in range(steps):
        # Predict
        x_pred = x + u[t]
        p_pred = p + q

        # Update
        k = p_pred / (p_pred + r)
        x = x_pred + k * (z[t] - x_pred)
        p = (1.0 - k) * p_pred

        x_est[t] = x
        p_est[t] = p
        k_hist[t] = k
        x_pred_hist[t] = x_pred

    return x_est, p_est, k_hist, x_pred_hist


def plot_results(t, x_true, z_meas, x_est, k_hist, p_est, outdir=None):
    # Figure 1: position
    plt.figure()
    plt.plot(t, x_true, label="True position")
    plt.plot(t, z_meas, label="Measurement", linestyle="None", marker="o", markersize=3)
    plt.plot(t, x_est, label="Kalman estimate")
    plt.xlabel("time step")
    plt.ylabel("position (m)")
    plt.title("1D Kalman Filter: True vs Measurement vs Estimate")
    plt.legend()
    plt.tight_layout()

    if outdir is not None:
        plt.savefig(os.path.join(outdir, "position.png"), dpi=200)

    # Figure 2: gain & variance
    plt.figure()
    plt.plot(t, k_hist, label="Kalman gain K")
    plt.plot(t, p_est, label="Estimate variance P")
    plt.xlabel("time step")
    plt.title("Kalman gain and estimate uncertainty")
    plt.legend()
    plt.tight_layout()

    if outdir is not None:
        plt.savefig(os.path.join(outdir, "gain_variance.png"), dpi=200)

    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=60)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--q", type=float, default=0.2, help="process noise variance Q")
    parser.add_argument("--r", type=float, default=2.0, help="measurement noise variance R")
    parser.add_argument("--x0", type=float, default=0.0, help="initial estimate x0")
    parser.add_argument("--p0", type=float, default=1.0, help="initial variance P0")
    parser.add_argument("--savefig", action="store_true", help="save figures to outputs/")
    args = parser.parse_args()

    u, x_true, z_meas = simulate_1d_robot(args.steps, args.seed, args.q, args.r)
    x_est, p_est, k_hist, _ = kalman_filter_1d(
        u, z_meas, x0=args.x0, p0=args.p0, q=args.q, r=args.r
    )

    t = np.arange(args.steps)

    outdir = None
    if args.savefig:
        outdir = "outputs"
        os.makedirs(outdir, exist_ok=True)

    plot_results(t, x_true, z_meas, x_est, k_hist, p_est, outdir=outdir)

    # ちょい便利：最終ステップの誤差も出す（READMEの結果欄に貼れる）
    mae_meas = float(np.mean(np.abs(z_meas - x_true)))
    mae_est = float(np.mean(np.abs(x_est - x_true)))
    print("Mean Absolute Error (Measurement vs True):", mae_meas)
    print("Mean Absolute Error (Kalman estimate vs True):", mae_est)


if __name__ == "__main__":
    main()

