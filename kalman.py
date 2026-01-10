import argparse
import numpy as np
import matplotlib.pyplot as plt

def simulate_1d_robot(steps: int, seed: int, q: float, r: float):
    rng = np.random.default_rng(seed)
    # 入力（行動）: ±1
    u = rng.choice([-1.0, 1.0], size=steps)
    #print(u) 
    x_true = np.zeros(steps, dtype=float)
    z_meas = np.zeros(steps, dtype=float)

    x = 0.0
    for t in range(steps):
        w = rng.normal(0.0, np.sqrt(q))  
        x = x + u[t] + w  # 状態遷移

        v = rng.normal(0.0, np.sqrt(r))  
        z = x + v # 観測＝真＋ノイズ

        x_true[t] = x
        z_meas[t] = z

    return u, x_true, z_meas

def kalman_filter(u, z, x0=0.0, p0=0.0, q=0.2, r=2.0):
    steps=len(z)
    x_est=np.zeros(steps, dtype=float)
    p_est=np.zeros(steps, dtype=float)
    k_hist=np.zeros(steps, dtype=float)

    x=float(x0)
    p=float(p0)

    for t in range(steps):
        x_pred=x+u[t]
        p_pred=p+q

        k=p_pred/(p_pred+r)   #カルマンゲイン
        x=x_pred+k * (z[t]-x_pred)  # 推定更新
        p=(1.0-k) * p_pred   # 分散更新

        x_est[t]=x
        p_est[t]=p
        k_hist[t]=k
    return x_est, p_est, k_hist

#def predict(u, x0=0.0):
#    x=float(x0)
#    x_pred=np.zeros(len(u), dtype=float)
#    for t in range(len(u)):
#        x=x+u[t]
#        x_pred[t]=x
#    return x_pred

def plot(t, x_true, z_meas, x_est):
    plt.figure()
    plt.plot(t, x_true, label="True position")
    plt.plot(t, z_meas, label="Measurement", linestyle="None", marker="o", markersize=3)
    plt.plot(t, x_est, label="kalman estimate")
    plt.xlabel("time step")
    plt.ylabel("position")
    plt.title("kalman estimate-True/Measurement")
    plt.legend()
    plt.tight_layout()
    plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=60)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--q", type=float, default=0.2)
    parser.add_argument("--r", type=float, default=2.0)
    parser.add_argument("--x0", type=float, default=0.0)
    parser.add_argument("--p0", type=float, default=0.0)
    args = parser.parse_args()

    u, x_true, z_meas = simulate_1d_robot(args.steps, args.seed, args.q, args.r)
    x_est, _, _ = kalman_filter(u, z_meas, x0=args.x0, p0=args.p0, q=args.q, r=args.r)
    t = np.arange(args.steps)
    plot(t, x_true, z_meas, x_est)

if __name__ == "__main__":
    main()

