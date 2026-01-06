import argparse
import numpy as np

def simulate_1d_robot(steps: int, seed: int, q: float, r: float):
    rng = np.random.default_rng(seed)

    # 入力（行動）: ±1
    u = rng.choice([-1.0, 1.0], size=steps)

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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=60)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--q", type=float, default=0.2)
    parser.add_argument("--r", type=float, default=2.0)
    args = parser.parse_args()

    u, x_true, z_meas = simulate_1d_robot(args.steps, args.seed, args.q, args.r)

    print("u[:5]     =", u[:5])
    print("x_true[:5]=", x_true[:5])
    print("z_meas[:5]=", z_meas[:5])

if __name__ == "__main__":
    main()

