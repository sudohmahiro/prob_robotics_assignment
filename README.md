これは確率ロボティクスの課題レポジトリです。

# 1次元カルマンフィルタによるロボット位置推定

## 概要
1次元ロボットが「右に1m or 左に1m」に移動する状況をシミュレーションをしノイズを含む観測値からカルマンフィルタで位置推定を行う。

## 問題設定
- 空間：1次元  
- 状態：ロボットの位置 x  
- 初期位置：x<sub>0</sub>  
- 初期分散：P<sub>0</sub>

### ロボット
- 各時刻でランダムに「右に1 m」または「左に1 m」移動
- 移動にはガウス分布に従うノイズが加わる

## 状態方程式
ロボットの状態は1次元位置 x<sub>t</sub> のみで表される。

$$
\begin{aligned}
x_t &= x_{t-1} + u_t + w_t \\
w_t &\sim \mathcal{N}(0, Q)
\end{aligned}
$$

- u<sub>t</sub>：制御指令
- w<sub>t</sub>：プロセスノイズ
- Q：プロセスノイズ分散

## 観測方程式
観測値は真の位置に観測ノイズが加わったものとする。

$$
\begin{aligned}
z_t &= x_t + v_t \\
v_t &\sim \mathcal{N}(0, R)
\end{aligned}
$$
- z<sub>t</sub>：観測値  
- v<sub>t</sub>：観測ノイズ  
- R：観測ノイズ分散


## アルゴリズム
### 予測ステップ
$$
\begin{aligned}
\hat{x}_{t|t-1} &= \hat{x}_{t-1} + u_t \\
P_{t|t-1} &= P_{t-1} + Q
\end{aligned}
$$

### 更新ステップ

#### カルマンゲイン

$$
K_t = \frac{P_{t|t-1}}{P_{t|t-1} + R}
$$

#### 状態更新

$$
\hat{x}_t = \hat{x}_{t|t-1} + K_t\left(z_t - \hat{x}_{t|t-1}\right)
$$
#### 分散更新

$$
P_t = (1 - K_t)P_{t|t-1}
$$

### 参考資料
- https://ushitora.net/archives/3274
- https://qiita.com/arutan_dev/items/091138f2d80e79a5c703
- https://disassemble-channel.com/the-theory-of-kalman-filter/
