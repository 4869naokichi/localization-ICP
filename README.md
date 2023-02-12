ICP（Iterative Closest Point）スキャンマッチングを用いた自己位置推定。
間違っている部分があるかもしれません。

## 理論
スキャン点群 $\boldsymbol{p}_i (i = 1, 2, \cdots, N)$ と地図の点群 $\boldsymbol{q}_j (j = 1, 2, \cdots, M)$ に対し以下の誤差関数（目的関数）を定義する。
```math
E(\boldsymbol{x}) = \dfrac{1}{N}\sum_{i=1}^N||(R\boldsymbol{p}_i+\boldsymbol{t})-\boldsymbol{q}_i||^2
```
ここで、$R$ は回転行列、$\boldsymbol{t}$ は並進ベクトルである。$\boldsymbol{p}_i$ の最近傍点を $\boldsymbol{q}_i$ とする。  
LRFによる自己位置推定はこの $E(\boldsymbol{x})$を最小化する最適化問題である。  
（当たり前？だけど）$N$ と $M$ は異なっても良い（レーザーの点の数と地図の点の数は異なっても良い）。

## 最近傍点の探索
$E(\boldsymbol{x})$ を計算するにあたり、あるレーザー点 $\boldsymbol{p}_i$ に最も近い地図点 $\boldsymbol{q}_j$ を見つけたい。全ての距離を計算すれば最も近い点はわかるが、これでは計算量が $O(M)$ になってしまう。そこでkd-tree（kd木）というものがよく使われる。kd木の説明は[こちら](https://myenigma.hatenablog.com/entry/2020/06/14/205753)が簡潔で分かりやすい。PythonだとSciPyにscipy.spatial.KDTreeというモジュールがあるようで、MATLABのStatistics and Machine Learning Toolboxにも関数が用意されているようです。

## 最適化
**点と点の対応が既知の場合は**、ICPアルゴリズムは「特異値分解」を用いて非常に単純に実装できます [1]。
1. それぞれの点群の重心を計算して、各点の座標を重心中心に変換する。
2. その座標行列を互いに掛けあわせた $W$ という行列を特異値分解する 。
```math
W = U\Sigma V^\top
```
3. 特異値分解によって得られた $U$ と $V$ を使って、$E(\boldsymbol{x})$ を最小にする $\boldsymbol{t}$ と $R$ は次式で計算できる。（$\boldsymbol{\mu}_p$ と $\boldsymbol{\mu}_q$ はそれぞれの点群の重心の座標）
```math
\begin{align}
R & = UV^\top \\
\boldsymbol{t} & = \boldsymbol{\mu}_p - R\boldsymbol{\mu}_q
\end{align}
```
4. 収束するまで1-3を繰り返す。

なぜこれで最適化できるのかは参考資料[2]を参照してください。

## 疑問点など
- 特異値分解を用いる手法の適用のために「レーザー点と地図点が精度良く対応している必要がある」が、最近傍点で対応付けて良いのか。

## 参考文献
[1] [ICPアルゴリズムを利用したSLAM](https://myenigma.hatenablog.com/entry/20140617/1402971928)  
[2] [Iterative Closest Point Algorithm](http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/17-icp.pdf)
