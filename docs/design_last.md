# 最終設計書: `autoware_pose_twist_fusion`（自己位置推定フュージョンパッケージ）

> 本パッケージは Autoware の `autoware_ekf_localizer` を置き換える自己位置推定ノードである。
> 起動パラメータで **3 種類の推定アルゴリズム**（EKF / LIO-SAM IMU Preintegration / 自作 Factor-Graph Fusion）を切り替えて実行できる。

- 対象 Autoware バージョン: Autoware Core 1.8.0 系（本リポジトリ `main`）
- 版: **design_last（rev.7 相当・2026-08-16）**
- 位置づけ: `design.md`（rev.6）と 5 回分のレビュー記録（`review.md` 〜 `review_v5.md`）を統合し、
  **改訂履歴の記述を本文から排除して「確定仕様だけ」で構成し直した最終設計書**。
  併せて rev.6 時点で残っていた内部矛盾 18 件を解消し、要求（`requirement.md`）との対応を §0 で明示した。
- ステータス: **実装着手可（Implementation-Ready）**。残る未確定事項は §9.2 のみ（いずれも実装をブロックしない）。

**本書の読み方**

- 本文は「そう決まっている仕様」だけを書く。過去版で撤回された記述・その経緯は本文に持ち込まない。
  設計判断の根拠は §8（Design Decisions, D1〜D45）に集約し、本文からは ID で参照する。
- rev.6 からの実質変更点（矛盾の解消・パラメータの追加/既定変更）は **付録 A** に一覧する。
  `design.md` からの移行時はここだけを見れば差分が分かる。

---

## 0. 要求との対応（トレーサビリティ）

`requirement.md` の各項に対する本設計の実現方法と、**要求からの逸脱**を明示する。

| # | 要求 | 実現 | 判定 |
| --- | --- | --- | --- |
| R1 | 自己位置推定機能を持つ | 本パッケージ全体 | ✅ |
| R2 | `ekf_localizer` の代わりに実行し、同じ出力トピックを出す（`kinematic_state` / `pose_with_covariance` / `pose_with_covariance_no_yawbias`） | §3.1。実 Autoware では `pose_with_covariance` が名前空間外の絶対名 `/localization/pose_with_covariance` として存在するため、**絶対名（#2a）と名前空間内（#2b）の両方**を同一内容で発行してドロップイン互換と要求の双方を満たす | ✅ |
| R3 | 3 種類の機能を起動時パラメータで切替。元アルゴリズムに準拠し、入出力のみ再配線 | §4.1（pluginlib）、`fusion_method`（read_only）。§5.1 / §5.2 / §5.3 | ✅ |
| R4 | 自作融合は GTSAM を使用 | §5.3 全体 | ✅ |
| R5 | 入力は GNSS 自己位置 / IMU / 車速 / NDT 自己位置 | §5.3.3。要求記載の `/sensing/vehicle_celocity_converter/twist_with_covariance` は綴り誤りであり、実在する `/sensing/vehicle_velocity_converter/twist_with_covariance` を使う（§2.4 / D19） | ✅ |
| R6 | 因子グラフ1：**内界センサ情報のみ**で更新、**更新周期 20 ms** | §5.3.2。内界センサのみ・20 ms グリッドという要求は満たす | ✅ |
| R7 | 因子グラフ1：情報を Subscribe したら因子を追加し、指定周期で**最適化**をかける | §5.3.2。**逸脱**: Graph1 は GTSAM のグラフを構築せず、IMU プリインテグレーションによる解析的伝播＋車速の EKF 形式 1 ステップ更新で `μ₁, Σ₁` を得る（D9b） | ⚠️ **意図的逸脱**（下記 §0.1） |
| R8 | 因子グラフ1：因子グラフ2の更新タイミングでリセット | §5.3.7 (4)。アンカー更新時に `X*,V*,B*,Σ*` で初期化し、以降を replay する | ✅ |
| R9 | 因子グラフ2：**すべての入力**で更新、**更新周期 500 ms** | §5.3.2。`graph2_optimize_interval_ms: 500` | ✅ |
| R10 | 因子グラフ2：因子グラフ2の更新タイミングでリセット | §5.3.2 / §5.3.7 (3)。**逸脱**: 全リセットではなく、**オーバーラップ付き固定ラグ窓 + 窓外の周辺化**とする（D23） | ⚠️ **意図的逸脱**（下記 §0.1） |
| R11 | 入力 Subscribe 時刻の因子グラフ1の自己位置＋共分散の範囲を考慮して、因子追加時の共分散を補正する。確率的信頼度のロジック案を提案する | §5.3.6（層 A: χ² ゲート / 層 B: `w = exp(−½max(0,d²−dof))` による連続重み付け / 層 C: ロバスト核 / 低速緩和）。到着時点の `N(μ₁(t), Σ₁(t))` を使うという要求の意味論を INV-3（凍結）で保証する | ✅ |
| — | （要求に無い拡張）外部 SLAM LiDAR オドメトリを Graph2 の第 5 入力として追加 | §5.3.3c。**Q/D7 で合意済みの拡張**。要求どおりの入力構成に戻す場合は `slam.use: false` を設定する（機能は残す） | ⚠️ **要求範囲外の拡張**（合意済み） |

### 0.1 要求からの逸脱 2 件の根拠

**(1) R7 — Graph1 を「因子グラフの最適化」として実装しない**

- 要求の字義どおりに 20 ms ごとの因子グラフ最適化を行うと、共分散を得るために `marginalCovariance()` を 50 Hz で呼ぶ必要があり、計算コストが実時間要件（§11 試験 10）に見合わない。
- 一方で **Graph1 が扱う因子は「IMU プリインテグレーション（連続ノード間）」と「車速・非ホロノミック（各ノードの unary）」だけであり、グラフは閉路を持たない鎖状（Markov 連鎖）である**。線形ガウス近似の下では、鎖状グラフの最新ノードにおける MAP 周辺分布は前向き再帰（＝フィルタ形式）の解と厳密に一致する。したがって本実装は「因子グラフ1を最適化した結果の最新ノード周辺分布」と数学的に等価な量を、桁違いに安い計算で得ている。
- 厳密に異なるのは「過去ノードの再線形化を行わない」点のみである。Graph1 の役割は (i) 高レート出力の生成、(ii) §5.3.6 の到達可能範囲の提示であり、いずれも最新時刻の分布しか使わないため、再線形化の欠如は問題にならない。過去の再線形化は Graph2 が固定ラグ窓のバッチ最適化として担当する。
- 用語としては、要求との対応を保つため本書でも「因子グラフ1（Graph1）」と呼び続ける。実装上は `Graph1` クラス（伝播器 + EKF 更新器）である。

**(2) R10 — Graph2 を全リセットせず固定ラグ窓にする**

- 要求どおり 500 ms ごとに全リセットすると、リセット直後に届いた遅延測定（最大 300 ms）を貼るべきノードが既に破棄されており、**測定が原理的に使えない**。要求 R11（Subscribe した情報を対応する因子グラフに追加する）と両立しない。
- そこで「窓を捨てずにずらす」固定ラグ窓（`graph2_lag_ms` 既定 1000 ms）とし、**窓外だけを周辺化して境界 prior 1 個に畳む**。これは「リセットして周辺化結果を prior として引き継ぐ」という要求の意図（情報を捨てずに計算量を有界にする）をより厳密に実現したものであり、周辺化のタイミングも要求どおり **Graph2 の更新タイミング（500 ms 毎）** である。
- 逆に `graph2_lag_ms = graph2_optimize_interval_ms` とすれば要求の字義（毎 tick リセット）に近づくが、その設定は §6.2 の検証条件 `graph2_lag_ms ≥ optimize_interval + max_measurement_delay + margin` に違反するため**起動失敗**となる。遅延測定を諦める構成は本設計では提供しない。

---

## 1. 目的とスコープ

`ekf_localizer` と同一の出力インタフェースを維持したまま、内部の推定アルゴリズムを差し替え可能にする。これにより下流（`stop_filter` → `/localization/kinematic_state`、`twist2accel`、`pose_instability_detector`、`ndt_scan_matcher` の初期姿勢フィードバックなど）へ**ドロップイン互換**で接続できる。

対象アルゴリズム:

| モード ID | 概要 | 元実装 |
| --- | --- | --- |
| `ekf` | Autoware 標準 2D EKF | `autoware_ekf_localizer` |
| `lio_sam_imu_preintegration` | IMU プリインテグレーション + ファクタグラフ | `LIO-SAM/src/imuPreintegration.cpp` |
| `factor_graph_fusion` | 自作。GTSAM による 2 段ファクタグラフ融合 | 新規 |

**アルゴリズムは元実装に準拠**し、入出力のみ本設計書の出力インタフェース要求に合わせて再配線する。

---

## 2. 調査結果（既存実装のインタフェースとアルゴリズム）

### 2.1 `autoware_ekf_localizer` の出力

`src/ekf_localizer.cpp:70-84` のパブリッシャ宣言と、`ekf_localizer.launch.xml` および
`tier4_localization_launch/launch/pose_twist_fusion_filter/pose_twist_fusion_filter.launch.xml` のリマップ、
`localization.launch.xml:30,38` の `push-ros-namespace`（`localization` → `pose_twist_fusion_filter`）を突き合わせた結果:

| ノード内トピック名 | リマップ後（実トピック） | 型 | 意味 |
| --- | --- | --- | --- |
| `ekf_odom` | `/localization/pose_twist_fusion_filter/kinematic_state` | `nav_msgs/Odometry` | 融合結果オドメトリ（**ヨーバイアス除去後**） |
| `ekf_pose_with_covariance` | **`/localization/pose_with_covariance`**（名前空間外の絶対名） | `geometry_msgs/PoseWithCovarianceStamped` | 融合結果姿勢（ヨーバイアス除去後） |
| `ekf_biased_pose_with_covariance` | `/localization/pose_twist_fusion_filter/biased_pose_with_covariance` | `geometry_msgs/PoseWithCovarianceStamped` | ヨーバイアスを**含む**姿勢 |
| `ekf_twist_with_covariance` | `/localization/pose_twist_fusion_filter/twist_with_covariance` | `geometry_msgs/TwistWithCovarianceStamped` | 融合結果 twist |
| `ekf_pose` / `ekf_biased_pose` / `ekf_twist` | `.../pose`, `.../biased_pose`, `.../twist` | `*Stamped` | 可視化・簡易参照用 |
| `estimated_yaw_bias` | `.../estimated_yaw_bias` | `autoware_internal_debug_msgs/Float64Stamped` | 推定ヨーバイアス |
| `debug/processing_time_ms` | `.../debug/processing_time_ms` | `autoware_internal_debug_msgs/Float64Stamped` | 処理時間 |
| `/diagnostics` | 同左（絶対名） | `diagnostic_msgs/DiagnosticArray` | 診断 |

> `pose_with_covariance` は launch で**絶対名**が指定されているため名前空間 `pose_twist_fusion_filter` の**外**に出る。
> 本パッケージも**この実トピック名を維持**する（RViz 設定 `autoware.rviz` 等が参照しており、変えるとドロップイン互換が壊れる）。
> 要求文書の `/localization/pose_twist_fusion_filter/pose_with_covariance` は名前空間内トピックとしても**追加で**発行する（§3.1 #2b）。

**実消費者**:

| トピック | 消費者 |
| --- | --- |
| `.../kinematic_state` | `stop_filter` → `/localization/kinematic_state` |
| `.../twist_with_covariance` | `stop_filter`（`use_twist_with_covariance: True`） |
| `.../biased_pose_with_covariance` | `ndt_scan_matcher`（`input_initial_pose_topic`）、`ar_tag_based_localizer`、`lidar_marker_localizer` |
| `/localization/pose_with_covariance` | RViz（`autoware.rviz`, `planning_bev.rviz`, `planning_tpv.rviz`） |

EKF の状態は `[x, y, yaw, yaw_bias, vx, wz]`。`ekf_pose`（=真値）はヨーバイアスを差し引いた姿勢、`ekf_biased_pose` はセンサが観測する（バイアスを含む）姿勢。z / roll / pitch は状態に含まれず、`Simple1DFilter` で入力姿勢から低域通過させて付加する。

**出力レート**: EKF 予測タイマ周期（既定 `predict_frequency = 50 Hz` → 20 ms 周期）。TF は `tf_rate = 50 Hz`。
**タイムスタンプ**: 予測タイマの `this->now()`（`ekf_localizer.cpp:149`）。EKF は**現在時刻へ外挿した推定**を publish する。
**QoS**: 全パブリッシャ・サブスクライバとも `rclcpp::QoS{1}`（Reliable / Volatile）。

### 2.2 要求で指定された 3 トピックと実装現状の対応

現行 Autoware では `kinematic_state` は存在、`pose_with_covariance` は**名前空間外**に存在、`pose_with_covariance_no_yawbias` の名称は存在しない。

**確定（D1）**: 現行命名規約を維持する。すなわち `pose_with_covariance` = **ヨーバイアス除去後の真値**、`biased_pose_with_covariance` = **ヨーバイアス込み**。要求の `pose_with_covariance_no_yawbias`（＝「ヨーバイアス無し」）は **真値のエイリアス**として発行する。

### 2.3 `LIO-SAM` `imuPreintegration` のアルゴリズム構造

`src/imuPreintegration.cpp` は 2 クラス構成:

- **`IMUPreintegration`**: GTSAM `ISAM2` によるファクタグラフ最適化。
  - IMU 積分器を 2 本持つ:
    - `imuIntegratorOpt_`: 最適化グラフ用（`odometryHandler` でノード間を積分）。
    - `imuIntegratorImu_`: 高レート伝播用（`imuHandler` で最新最適化結果から前方積分し高レート odom を出力）。
  - `odometryHandler`（LiDAR オドメトリ受信 = 低レート）: `PriorFactor<Pose3>`（LiDAR 姿勢, `:412`）+ `ImuFactor`（`:405`）+ `BetweenFactor<ConstantBias>` を追加し `optimizer.update()`。
  - `imuHandler`（IMU 受信 = 高レート）: 最新バイアスで前方積分し `odomTopic+"_incremental"` を高レート発行。
  - **リセット規律（`:354-380`）**: キー数が 100 に達すると `marginalCovariance(X/V/B(key-1))` を取得 → `resetOptimization()` → **X・V・B の 3 つに `PriorFactor` を張り直す**。
  - **補正後の再積分（`:454-462`）**: 最適化完了後、`imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom)` の直後に、補正時刻以降の `imuQueImu` を**再積分**して現在時刻の高レート状態を作り直す。
  - **入力共分散の切替（`:303,412`）**: LiDAR オドメトリの `pose.covariance[0] == 1` を「degenerate フラグ」とみなし、大きいノイズモデル `correctionNoise2` に切り替える。
  - **発散検知（`:433,472`）**: `failureDetection()`（速度 30 m/s 超・バイアス 1.0 超）で `resetParams()`。
- **`TransformFusion`**: LiDAR オドメトリ（低レート・高精度）と IMU オドメトリ（高レート）を合成し、高レートで最終姿勢と TF を出力。

この「**低レート最適化 + 高レート前方積分**」「**一定間隔でのグラフリセット（周辺化引き継ぎ）**」という構造は、要求の自作 Fusion（因子グラフ 1: 高レート、因子グラフ 2: 低レート）と本質的に同型であり、実装を共通化しやすい。

> 自作モードの Graph2 は、この LIO-SAM 型の「全リセット」ではなく**固定ラグ窓の周辺化**を採る（§0.1 (2) / D23）。
> **Graph1 側のアンカー引き継ぎ + 再積分**は LIO-SAM と同型である（ただし車速の replay を追加・D15）。
> `lio_sam_imu_preintegration` モードは元実装どおり全リセットを踏襲する。

### 2.4 実センサトピック名

| 要求記載 | 実 Autoware トピック | 型 |
| --- | --- | --- |
| `/sensing/gnss/pose_with_covariance` | 同左 ✅ | `geometry_msgs/PoseWithCovarianceStamped` |
| `/sensing/imu/imu_data` | 同左 ✅（`imu_corrector` 出力、QoS depth 10 Reliable） | `sensor_msgs/Imu` |
| `/sensing/vehicle_celocity_converter/twist_with_covariance` | ⚠️ 綴り誤り。**`/sensing/vehicle_velocity_converter/twist_with_covariance`**（自作モード）／ `/localization/twist_estimator/twist_with_covariance`（`ekf` モード） | `geometry_msgs/TwistWithCovarianceStamped` |
| `/localization/pose_estimator/pose_with_covariance`（NDT） | 同左 ✅ | `geometry_msgs/PoseWithCovarianceStamped` |

> **モード別に車速入力を変える理由（D19）**: `ekf_localizer` が実際に購読しているのは `/localization/twist_estimator/twist_with_covariance` であり、これは `gyro_odometer` が
> `vehicle_velocity_converter` の車速と IMU 角速度を合成した結果である。自作モードは IMU を**生で**ファクタ化するため、
> `gyro_odometer` 出力をそのまま車速ファクタに使うと**同じ IMU 情報の二重計上**になる。
> よって自作モードの車速入力は既定で **`/sensing/vehicle_velocity_converter/twist_with_covariance`（車速のみ）** とし、
> `ekf` モードでは互換性のため `/localization/twist_estimator/twist_with_covariance` を使う。入力トピックはモードごとにパラメータで指定する。

---

## 3. 出力インタフェース仕様（全モード共通・ekf 互換）

### 3.1 出力トピック一覧

ノードは名前空間 `/localization/pose_twist_fusion_filter` 配下で起動される前提。`#2a` のみ launch で絶対名にリマップする。

| # | ノード内名 | 実トピック | 型 | 区分 | frame_id / child |
| --- | --- | --- | --- | --- | --- |
| 1 | `kinematic_state` | `.../pose_twist_fusion_filter/kinematic_state` | `nav_msgs/Odometry` | **必須（R2）** | `map` / `base_link` |
| 2a | `pose_with_covariance` | `/localization/pose_with_covariance` | `PoseWithCovarianceStamped` | **必須**（ekf 実装互換） | `map` |
| 2b | `ns_pose_with_covariance` | `.../pose_twist_fusion_filter/pose_with_covariance` | `PoseWithCovarianceStamped` | **必須（R2）** | `map` |
| 3 | `pose_with_covariance_no_yawbias` | `.../pose_twist_fusion_filter/pose_with_covariance_no_yawbias` | `PoseWithCovarianceStamped` | **必須（R2）** | `map` |
| 4 | `biased_pose_with_covariance` | `.../pose_twist_fusion_filter/biased_pose_with_covariance` | `PoseWithCovarianceStamped` | 互換（**下流必須**: ndt_scan_matcher 等） | `map` |
| 5 | `twist_with_covariance` | `.../pose_twist_fusion_filter/twist_with_covariance` | `TwistWithCovarianceStamped` | 互換（**下流必須**: stop_filter） | `base_link` |
| 6 | `pose` / `biased_pose` | `.../pose`, `.../biased_pose` | `PoseStamped` | 互換・可視化 | `map` |
| 7 | `twist` | `.../twist` | `TwistStamped` | 互換・可視化 | `base_link` |
| 8 | `estimated_yaw_bias` | `.../estimated_yaw_bias` | `Float64Stamped` | 互換 | — |
| 9 | `debug/processing_time_ms` | `.../debug/processing_time_ms` | `Float64Stamped` | デバッグ | — |
| 10 | `debug/covariance_correction` | `.../debug/covariance_correction/*` | `Float64Stamped` 群 | デバッグ（自作モード: 入力別の `d²` / `w` / 棄却数 / `\|c\|` / `\|u\|`） | — |
| 11 | `/diagnostics` | 同左 | `DiagnosticArray` | 互換 | — |
| TF | `map → base_link` | tf2 | — | 互換（`enable_tf`） | — |

`#2a`, `#2b`, `#3` は**常に同一内容**（同一 stamp・同一値・同一共分散）を発行する。

**入出力サービス・入力トピック（互換）**:

| 種別 | 名前 | 型 | 備考 |
| --- | --- | --- | --- |
| Service | `trigger_node_srv` | `std_srvs/SetBool` | `false` で推定停止・状態リセット、`true` で再開（ekf と同一） |
| Sub | `initialpose`（→ `/initialpose3d`） | `PoseWithCovarianceStamped` | 初期姿勢設定 |

### 3.2 ヨーバイアスの扱い（統一規約 / D1）

**マッピング表（実装はこの表を唯一の正とする）**:

```
[EKF モード]
  ekf_module の「バイアス除去後」姿勢
      → #2a pose_with_covariance
      → #2b ns_pose_with_covariance
      → #3  pose_with_covariance_no_yawbias
      → #1  kinematic_state.pose
      → #6  pose
      → TF (map → base_link)

  ekf_module の「バイアス込み」姿勢
      → #4 biased_pose_with_covariance     ← ★ここにのみ出す
      → #6 biased_pose

  推定ヨーバイアス値
      → #8 estimated_yaw_bias

[LIO-SAM / factor_graph_fusion モード]
  ヨーバイアス状態を持たない
      → #1,#2a,#2b,#3,#4,#6 すべて同一の推定姿勢
      → #8 estimated_yaw_bias は 0.0 を発行（トピックは存在させる）
```

> **禁止**: バイアス込み姿勢を `pose_with_covariance_no_yawbias` に割り当ててはならない。上表が唯一の正である。

### 3.3 出力セマンティクスの厳密定義（D20）

**ドロップイン互換の一致条件**を以下で定義する。§11 の互換テスト（試験 1）はこの表を検証する。

| 項目 | 規定 |
| --- | --- |
| トピック名 / 型 | §3.1 の表と完全一致 |
| `header.frame_id` | pose 系 = `pose_frame_id`（既定 `map`）、twist 系 = `base_frame_id`（既定 `base_link`） |
| `child_frame_id` | `kinematic_state` は `base_link` 固定（ekf 実装と同じ） |
| **タイムスタンプ** | 出力タイマ発火時刻 `this->now()` に**外挿**した推定を stamp する（ekf と同一方針）。センサ時刻を stamp する方式は採らない（下流の `pose_instability_detector` が出力周期を前提にしているため）。外挿量が `max_extrapolation_ms`（既定 100 ms）を超える場合は外挿せず最終推定時刻を stamp し、診断 WARN |
| QoS（出力） | `rclcpp::QoS{1}`, Reliable, Volatile（ekf と同一） |
| QoS（入力） | IMU / 車速 = `rclcpp::QoS{100}` Reliable（プリインテグレーションでの取りこぼしは推定誤差に直結するため ekf の depth 1 より深くする）、pose 系 = `rclcpp::QoS{10}` Reliable、`initialpose` = `QoS{1}` |
| publish rate | `output_rate_hz`（既定 50 Hz）。TF は `tf_rate_hz`（既定 50 Hz） |
| **共分散の座標系** | pose の 6×6 は ROS 規約 `[x,y,z,rx,ry,rz]`、位置は `map` 系、回転は `map` 系の微小回転。twist の 6×6 は `base_link` 系。GTSAM 内部表現からの変換は §5.3.5 |
| **共分散の値** | 全モードで**必ず有効値**を出す。ゼロ行列・NaN・負の対角は禁止（§5.2.1 / §5.3.9） |
| 未推定次元 | 対角に `undetermined_variance`（既定 1e4）を入れる（0 を入れない） |
| **出力する推定** | 全出力トピック・TF は `published_estimate`（アンカージャンプ smoothing 適用後）。ゲート・NIS・NEES・Graph2 初期値には `internal_estimate` を使い、両者を混用しない。smoothing は「補正オフセット `C` の減衰」として定義し、`X_published = X_internal · C` により常に車両運動を含める。pose と twist に同時に適用して `d(pose_published)/dt = twist_published` を保つ。smoothing 中の出力共分散は、`Σ_internal` を published 基準へ**輸送**（pose = `J_r⁻¹(−c)`、twist = `Ad_{C⁻¹}`）したうえで未適用オフセット分を加算した **MSE 行列**（共分散ではない）（§5.3.7 / D27・D31・D32・D40） |

**6DoF 結果の出力規約（D20）**:

- `factor_graph_fusion` / `lio_sam_imu_preintegration` は 6DoF 状態を持つ。**既定は 6DoF をそのまま出力**する（`kinematic_state` は本来 3D odometry であり、z/roll/pitch を捨てる理由がない）。
- EKF との厳密な数値比較や 2D 前提の下流デバッグのために `output.flatten_to_2d`（既定 `false`）を用意する。`true` の場合:
  - z / roll / pitch は**推定値をそのまま使わず**、`pose_estimator` 入力を `Simple1DFilter` に通した値で置換（ekf と同一挙動）。
  - twist は `linear.y = linear.z = angular.x = angular.y = 0`。
  - **`ekf` モードでは本パラメータの値によらず常に `true` として扱う**（EKF は 2D 状態のため。`false` を指定しても無視し、起動時に診断 INFO を出す）。
- `kinematic_state.twist` の定義（全モード共通）:
  - `linear` = **`base_link` 系の並進速度** `v_b = R(X)ᵀ · V_world`（GTSAM の `V(k)` は world 系のため必ず変換する）。
  - `angular` = **`base_link` 系の角速度** `ω_b`（IMU ジャイロからバイアスを引いた値を `base_link` に回転したもの）。
  - EKF モードは `linear.x = vx`, `angular.z = wz`、他は 0（現行と同一）。
- **`twist.covariance`** は world 系の速度共分散 `Σ_V` を `R(X)ᵀ Σ_V R(X)` で `base_link` 系に変換し、角速度ブロックは IMU ジャイロノイズ + バイアス分散から与える。

---

## 4. アーキテクチャ設計

### 4.1 方針: 共通ノード + プラグイン（pluginlib）

3 モードを 1 ノードに `if/else` で詰め込むのではなく、**`pluginlib` による戦略パターン**を採用する（D2）。

```
autoware_pose_twist_fusion (ament_cmake, rclcpp_component)
├─ PoseTwistFusionNode                     … 唯一の実行ノード
│    - パラメータ fusion_method で plugin を動的ロード（起動時のみ / D17）
│    - 出力パブリッシャ（§3 の共通トピック）を一元管理
│    - 各プラグインが要求する入力サブスクライバを生成・配線
│    - 出力レート用タイマ（50 Hz）を保持
│    - initialpose / trigger サービス / TF / 診断を共通処理
│    └─ OutputStage                        … ★ smoothing はここが持つ（プラグインではない・D43）
│         - smoothing 状態 c / u / t_left / prev_snapshot_ を private に保持
│         - InternalEstimate → PublishedEstimate の導出（§5.3.7）
│
├─ FusionPlugin (基底 pluginlib インタフェース)
│    virtual InputRequirements required_inputs() const;
│    virtual void on_imu(const Imu &);
│    virtual void on_vehicle_twist(const TwistWithCovarianceStamped &);
│    virtual void on_ndt_pose(const PoseWithCovarianceStamped &);      // 送信元ごとに分離（D17）
│    virtual void on_gnss_pose(const PoseWithCovarianceStamped &);
│    virtual void on_slam_odometry(const Odometry &);
│    virtual void set_initial_pose(const PoseWithCovarianceStamped &);
│    virtual void reset();                                             // trigger_node(false) 等
│    // ★D43: プラグインが返すのは **internal estimate のみ**。const・非ブロッキング。
│    //   smoothing 状態を持たないので const で成立する。
│    virtual InternalEstimate estimate_internal(const rclcpp::Time & stamp) const;
│    virtual EstimateGeneration generation() const;   // ★ アンカー更新の検出に使う世代番号
│    virtual DiagnosticStatus diagnostics() const;
│
├─ EkfFusionPlugin                 … 既存 ekf_module を内部利用
├─ LioSamImuPreintegrationPlugin   … imuPreintegration アルゴリズムを移植
└─ FactorGraphFusionPlugin         … 自作 GTSAM 2 段グラフ
```

> **プラグイン境界の規約（D43）**: プラグインは `internal_estimate` だけを返し、**smoothing には一切関与しない**。
> smoothing 状態（`c` / `u` / `t_left` / `prev_snapshot_`）は `PoseTwistFusionNode::OutputStage` の private メンバであり、
> output callback group からのみ書き換えられる（D37）。この分離により
> (a) `estimate_internal()` を `const` として宣言でき（状態を書き換えない）、
> (b) 3 モードで smoothing 挙動が完全に共通化され、
> (c) 「ゲート／NIS に published を渡す」という取り違えが型レベルで起こらなくなる。
> `InternalEstimate` と `PublishedEstimate` は別型とし、暗黙変換を禁止する。

**利点**:
- 出力インタフェース（§3）とライフサイクル（initialpose、trigger サービス、TF、診断）を 1 箇所で共通化 → ドロップイン互換を担保。
- モード追加が容易（プラグイン追加のみ）。
- 各アルゴリズムは元実装のコアを最小改変で内包でき「アルゴリズムは元に準拠」（R3）を満たす。

代替案（単一ノード分岐 / 3 ノード別実行）との比較は §8 D2。

### 4.2 データフロー（自作モード）

```
/sensing/imu/imu_data ─────────────→ ImuBuffer(deque, 3.5 s) ──────┐
                                                                    │
/sensing/vehicle_velocity_converter ─→ VehicleTwistBuffer(3.5 s) ───┤
                    /twist_with_cov                                 │     (Graph1 replay 用)
                                                                    ▼
                                                    Graph1（内界のみ / 20 ms）
                                                    IMU predict + 車速 EKF 更新
                                                       ├→ StateHistory(1.5 s, 20 ms 刻み)
                                                       └→ internal estimate（μ₁,Σ₁）
                                                                    │
/sensing/gnss/pose_with_covariance ─┐                               ├──→ OutputStage（§5.3.7）
/localization/pose_estimator/...  ──┼→ [§5.3.4 遅延補正]            │     smoothing はここだけ
外部 LIO-SAM lidar odometry ────────┘   [§5.3.6 共分散補正]         │        │
                                          ↑ μ₁(t),Σ₁(t)            │   InternalEstimateSnapshot
                                          │ ＝ internal estimate    │      （atomic swap）
                                          │      （smoothing 前）   │        │
                                    PendingMeasurementQueue         │        ▼
                                    （補正済み noise model 付き）    │   出力タイマ(50 Hz)
                                          │  drain（tick 時）        │        │
                                          ▼                         │        ▼
                              Graph2MeasurementHistory              │   §3 トピック
                              （窓内の採用済み測定を保持）           │
                                          │  [t_begin, t_end] 抽出   │
                                          ▼                         │
                Graph2（固定ラグ窓 [t_end−lag, t_end] / 別スレッド） │
                 tick 500 ms ごとに窓内をバッチ再構築 → 最適化       │
                 窓外は周辺化して境界 prior に畳む                   │
                 → 吸収済み測定だけを history から prune             │
                                          │                         │
                          新アンカー X*,V*,B*,Σ*（t_end）───────────┘
                                          │
                                          ▼
              Graph1 リセット + IMU/車速 replay（§5.3.7）
```

> バッファ長 3.5 s は §6.2 の検証式
> `graph2_lag_ms + graph2_optimize_interval_ms × (1 + optimizer_failure_limit) + 500 = 1000 + 500×4 + 500 = 3500` から決まる。
> 最適化が連続失敗すると窓が右へ伸びるため（D35）、その分を含んでいる。

### 4.3 スレッド・実行モデル（D17）

- Executor: **`rclcpp::executors::MultiThreadedExecutor`**（コンポーネント実行時は `component_container_mt`）。
- Callback group:

| Group | メンバ | 種別 |
| --- | --- | --- |
| `sensor_cb_group` | IMU, 車速 | MutuallyExclusive（時系列順序を保証） |
| `measurement_cb_group` | GNSS, NDT, SLAM, initialpose | MutuallyExclusive |
| `output_cb_group` | 出力タイマ (50 Hz), TF タイマ, 診断タイマ | MutuallyExclusive（**smoothing 状態の排他はこれで担保**） |
| `service_cb_group` | `trigger_node_srv` | MutuallyExclusive |

- **Graph2 の構築・最適化は ROS callback では実行しない**。専用ワーカスレッド（`std::thread` + condition_variable）で走らせる。
  500 ms タイマ（tick）は「最適化要求」をキューに積むだけ。最適化中に次の要求が来た場合は**要求を上書き**し、多重実行しない（診断 WARN）。
- 測定 callback（GNSS / NDT / SLAM）は **Graph2 に直接触らない**。§5.3.4 の遅延補正・§5.3.6 の共分散補正まで callback 側で済ませ、
  結果を `PendingMeasurementQueue` に push するだけで戻る。グラフへの反映は次の tick でワーカがまとめて行う（§5.3.2）。
  → 共分散補正は「測定到着時点の Graph1 予測分布」を使うという要求 R11 の意味論が保たれ、かつ callback は O(1)。
- 共有状態と保護:

| 共有物 | 保護 |
| --- | --- |
| `ImuBuffer` / `VehicleTwistBuffer` / `StateHistory` | `std::mutex buffer_mutex_`（保持時間は数十 µs 以内。ワーカはコピーを取ってから離す） |
| **Graph1（逐次伝播器 + EKF 更新）** | **`std::mutex graph1_mutex_`。読み書きを問わず Graph1 に触るすべての経路が取得する**（sensor callback の IMU/車速適用、ワーカの (4) リセットと (5) replay、測定 callback の区間再伝播、initialpose、`reset()`）（D36） |
| Graph2（窓グラフ + オプティマイザ） | ワーカスレッド専有。外部から触らない |
| **`Graph2MeasurementHistory`** | ワーカスレッド専有。外部から触らない（`PendingMeasurementQueue` からの move のみで増える）。§5.3.2a |
| `PendingMeasurementQueue` | 測定 callback が push、ワーカが tick で drain。`std::mutex pending_mutex_`（要素は「stamp + 測定値 + §5.3.6 で補正済みの noise model」のみで軽量） |
| **`InternalEstimateSnapshot`** | `std::shared_ptr<const InternalEstimateSnapshot>` を `std::atomic_store/load` で**コピー&スワップ**。出力タイマは load するだけで**ロックしない**。**smoothing 状態は含めない**（D37） |
| **smoothing 状態 `c` / `u` / `t_left` / `prev_snapshot_`** | **`OutputStage` の private メンバ = output callback group 専有**（MutuallyExclusive なので追加の排他は不要）。ワーカも sensor callback も触らない（D37・D43）。§5.3.7 (6) |

**Graph1 のロック規律（D36）**

- **リセット・replay を実行するのは Graph2 ワーカスレッド**（§5.3.7 (4)(5)）であり、その間も IMU（100 Hz）と車速（50 Hz）の callback は Graph1 を書き換え続ける。片側だけがロックを取っても競合は防げない。
- 規約: `Graph1` クラスのすべての public メソッド（`applyImu()` / `applyWheelSpeed()` / `resetToAnchor()` / `replayFrom()` / `snapshot()` / `propagateInterval()`）は
  **入口で `graph1_mutex_` を取得する**。内部呼び出し用に `*_locked()` の private 版を用意し、二重ロックを避ける。
- **最悪保持時間**: replay の対象は §5.3.7 (5) の定義どおり **`t_end`（アンカー時刻）以降**であり、その長さは
  「Graph2 の最適化所要時間 + tick 1 周期」＝最悪 `graph2_optimize_interval_ms + p99(最適化) ≈ 800 ms` 相当。
  IMU 80 サンプル + 車速 40 サンプルの再適用で、1 サンプルあたり数 µs のオーダー（プリインテグレーションの更新と EKF 更新）なので **1 ms 未満**を見込む。
  100 Hz の IMU callback が 1 ms ブロックされても取りこぼしは起きない（QoS depth 100）。試験 22 ② で実測し、
  `p99 > 2 ms` なら「ワーカ側で新 Graph1 インスタンスを構築 → `graph1_mutex_` 下で atomic swap し、swap 後に
  swap 中に届いたサンプルだけを追い replay する」方式へ切り替える（設計上の逃げ道として記載するが、既定は単純な mutex）。
- **ロック順序は `graph1_mutex_` → `buffer_mutex_` に固定**する（replay はバッファを読みながら Graph1 を更新するため）。
  逆順で取得する経路を作らないことを実装規約とし、debug ビルドでは順序チェック付きラッパを使う。

- **出力デッドライン保証**: 出力タイマは `estimate_internal()` を呼ぶが、その実体はスナップショット（アンカー + プリインテグレータ状態）からの前方積分のみで、
  グラフ最適化にも `graph1_mutex_` にも一切触れない（スナップショットは shared_ptr の load のみ）。よって最悪実行時間は O(1)。
  実測を `debug/processing_time_ms` に出す。
- `fusion_method` は **起動時のみ有効**（`read_only` パラメータとして宣言。実行中変更は拒否）。プラグイン切替はノード再起動で行う。

---

## 5. モード別設計

### 5.1 `ekf` モード

- 既存 `ekf_module`（`ekf_module.cpp` / `state_transition.cpp` / `measurement.cpp` 等）を**そのまま内部利用**する（可能ならライブラリ化して依存、困難なら該当ソースを本パッケージへ vendored 移植）。
- 入力: `pose_with_cov`（NDT）, `twist_with_cov`（= `/localization/twist_estimator/twist_with_covariance`）。既存と同一。
- 出力マッピング: **§3.2 の表に厳密に従う**（`ekf_pose_with_covariance` → #1/#2a/#2b/#3/#6/TF、`ekf_biased_pose_with_covariance` → #4 のみ）。
- アルゴリズム不変（遅延補償・Mahalanobis ゲート・スムーズ更新・ピッチ z 補正すべて維持）。
- `output.flatten_to_2d` は本モードでは常に `true` として扱う（§3.3）。
- smoothing（§5.3.7）は本モードでは実質発動しない（EKF は連続更新でアンカージャンプを持たない）が、`OutputStage` は共通に通す。

### 5.2 `lio_sam_imu_preintegration` モード

- `imuPreintegration.cpp` の `IMUPreintegration` + `TransformFusion` のロジックを**元アルゴリズムに忠実**に移植。
- **前提（D3）**: 本モードを使う場合、**LIO-SAM の他コンポーネント（`imageProjection` / `featureExtraction` / `mapOptimization`）は本パッケージ外で別途起動**する。したがって本ノードは `imuPreintegration` 相当の処理のみを担い、**LiDAR オドメトリはそのまま入力として受け取る**（NDT 等への差し替えは行わない）。
- **入力**:
  - IMU: `/sensing/imu/imu_data`（元 `imuTopic`）。
  - LiDAR オドメトリ（低レート・絶対姿勢、`odometryHandler` 入力）: 外部 LIO-SAM `mapOptimization` が出力する LiDAR オドメトリトピック（`nav_msgs/Odometry`、既定 `lio_sam/mapping/odometry` 相当、パラメータで指定可能）。
  - **車速による前進拘束は追加しない**（元実装は IMU のみ。忠実移植を優先）。
  - degenerate フラグ（`pose.covariance[0] == 1`）の解釈も元実装どおり踏襲する。
- 座標系: 元実装の LiDAR/IMU 座標変換（`lidar2Imu`）を踏襲しつつ、出力を Autoware の `base_link`/`map` 規約へ写像する薄いアダプタを追加（D3）。

#### 5.2.1 出力共分散の生成（D39）

LIO-SAM の `imuPreintegration` は odometry の covariance に姿勢情報を入れない（`pose.covariance[0]` をフラグに流用している）。
そのままでは §3.3 の「共分散必須」に反するため、以下で生成する。

**X・V・B は IMU 前方積分で強く結合する**（`p(t) = p + v·Δt + …`、`v(t) = v + R(a−b_a)·Δt + …`）ため、
個別に伝播すると交差項 `2Δt·P_pv`・`−Δt·P_vb` 等が落ち、符号次第で**過小評価（過信）になる**（D33）。
`PreintegratedImuMeasurements` の共分散伝播もそもそも 9×9（+ バイアス 6×6）の joint で定義されている。
よって **joint 15×15 で伝播する**（自作モードの D10 と同じ規律）。

```
アンカー（最終最適化ノード k）:
  Σ_anchor = optimizer.jointMarginalCovariance({X(k), V(k), B(k)})   # 15×15
             並び = [ξ_X(6) ; δv(3) ; δb(6)]、ξ_X は GTSAM body tangent

高レート伝播時刻 t（アンカーから Δt 前方積分）:
  NavState X̂(t) = pim.predict(NavState(X*,V*), B*, H_state, H_bias)
      H_state = ∂(NavState出力)/∂(NavState入力)   … 9×9
      H_bias  = ∂(NavState出力)/∂(bias)           … 9×6
  H = [ H_state , H_bias ]                                    # 9×15

  Σ_nav(t) = H · Σ_anchor · Hᵀ + pim.preintMeasCov()          # 9×9（[ξ_X(6); δv(3)]）
  Σ_B(t)   = Σ_anchor[9:15, 9:15] + Q_bias·Δt                 # バイアスランダムウォーク

  出力 pose 共分散 = Σ_nav(t)[0:6, 0:6]（§5.3.5 の逆変換で ROS 表現へ）
  出力 twist の linear 共分散 = R(X̂)ᵀ · Σ_nav(t)[6:9, 6:9] · R(X̂)（world → base_link）
  出力 twist の angular 共分散 = ジャイロノイズ + Σ_B(t) のジャイロブロック
```

- `pim.predict()` のヤコビアン引数は GTSAM のバージョンで有無・並びが異なる。取得できない構成では
  `NavState::update()` のヤコビアン、または `numericalDerivative` によるフォールバックを使い、**どちらでも試験 9・12 を通す**こと。
- 実装簡略化のため、`lio_sam.covariance_source` パラメータで切替可能とする:
  - `propagated`（既定）: 上式（joint 15×15）。
  - `fixed`: `lio_sam.fallback_pose_stddev` / `fallback_twist_stddev` から対角行列を作る（デバッグ・移植初期用）。
  - **個別伝播（blockwise）オプションは用意しない**。保守性を持たない近似であり、`fixed` という明示的な逃げ道がある以上、中途半端な近似を選べるようにする利益がない（D33・D39）。
- いずれの場合も出力前に §5.3.9 の共分散妥当性チェック（対称化・PSD 化）を通す。

### 5.3 `factor_graph_fusion` モード（自作・本設計の中核）

#### 5.3.1 フレーム規約と IMU 外部パラメータ（D13）

**ナビゲーション状態の body フレームは `base_link` とする。**（LIO-SAM は IMU フレームを body にするが、本パッケージは Autoware 出力・車速ファクタの双方が `base_link` 基準なので `base_link` を選ぶ方が変換が減る。）

```
world / navigation frame : map （ENU、重力は -Z）
body frame               : base_link
sensor frames            : imu_link, gnss_link, (lidar frame)

変換取得:
  T_base_imu  = TF (base_link ← imu_link)   … 起動時に 1 度取得しキャッシュ
  T_base_gnss = TF (base_link ← gnss_link)  … GNSS のアンテナ位置補正に使用
```

**TF 取得に失敗した場合の fallback**:

| 変換 | 有効化フラグ | fallback パラメータ | 内容 | 取得も fallback も無い場合 |
| --- | --- | --- | --- | --- |
| `T_base_imu` | `imu.use_tf_extrinsic`（既定 `true`） | **`imu.extrinsic_fallback`** | 6 自由度（x,y,z,roll,pitch,yaw） | 起動失敗（診断 ERROR） |
| `T_base_gnss` | `gnss.use_tf_extrinsic`（既定 `true`） | **`gnss.lever_arm_fallback`** | 並進 `r_bg = (x,y,z)` のみ（回転は GNSS 位置測定に不要・§5.3.3b） | 起動失敗（診断 ERROR） |

- `use_tf_extrinsic: false` を明示した場合は TF を引かず常に fallback を使う（この場合は起動失敗にしない）。
- **GNSS は `gnss.lever_arm_fallback` を使う**（`imu.extrinsic_fallback` を流用してはならない）。

**IMU 測定値の `base_link` への変換**（`R = R_base_imu`, `r = t_base_imu`）:

```
ω_b = R · ω_imu
a_b = R · a_imu − ω_b × (ω_b × r) − ω̇_b × r
                 └ 遠心項          └ オイラー項
```

- `imu.lever_arm_compensation` で段階選択:
  - `rotation_only`: 回転のみ（`|r| < 0.2 m` 相当で誤差 < 0.05 m/s² @ ω=0.5 rad/s）。
  - `centrifugal`（**既定**）: 遠心項まで補償。`ω̇` は数値微分が高ノイズなためオイラー項は無視し、その分を加速度計ノイズに加算する（`imu.euler_term_sigma`、既定 0.02 m/s²）。
  - `full`: `ω̇` を 20 Hz LPF 付き数値微分で推定して補償（高精度 IMU 向けオプション）。

**IMU メッセージの解釈**:

| 項目 | 規定 |
| --- | --- |
| `linear_acceleration` | **specific force**（重力を含む）。`/sensing/imu/imu_data` は `imu_corrector` によりバイアス補正済みだが、残留バイアスを状態 `B(k)` で推定する |
| `angular_velocity` | 角速度 [rad/s]。同上 |
| `orientation` | **使用しない**（無視する）。姿勢はグラフで推定 |
| 重力 | `imu.gravity`（既定 9.80665）。GTSAM `PreintegrationParams::MakeSharedU(g)`（Z 上向き = ENU） |
| covariance = -1 | 「未提供」を意味する ROS 規約。この場合はパラメータのノイズ値を使う（§5.3.9） |
| 軸系 | REP-105 / REP-145 準拠（x 前・y 左・z 上）を前提。異なる場合は TF で吸収 |

#### 5.3.2 状態定義とノード生成規則（D9・D23・D24・D28・D29・D34・D35）

**状態ノード**: `X(k) ∈ Pose3`（map ← base_link）, `V(k) ∈ Vector3`（map 系速度）, `B(k) ∈ imuBias::ConstantBias`（base_link 系の加速度計・ジャイロバイアス）。LIO-SAM に準拠。

**Graph1（高レート・内界のみ・要求 R6/R7）**:
- ノード周期 = **20 ms 固定グリッド**（`graph1_node_interval_ms`）。
- Graph1 は「アンカーからの前方積分」であり、**グラフ最適化を毎回走らせない**（§0.1 (1) / D9b）。
  IMU プリインテグレーションによる**解析的伝播**（`PreintegratedImuMeasurements::predict()` + 共分散伝播）で `μ₁(t), Σ₁(t)` を生成する。
  車速による補正は、**車速メッセージを受信するたびに 1 ステップの EKF 形式更新**（§5.3.3a の測定モデル: 車速 1 次元 + 非ホロノミック 2 次元）を `μ₁, Σ₁` に適用して行う。
- **車速サンプルの使用規則（明確化）**: **同一 20 ms グリッド区間に届いた車速サンプルのうち、最新の 1 個だけ**を使う。
  既定の車速レート 50 Hz ではちょうど 1 個であり実質「全サンプル使用」になるが、車速レートが 50 Hz を超える場合も
  区間あたり 1 回に制限することで、過サンプリングされた相関の強い信号を独立扱いして共分散を過小評価することを避ける（D30 と同じ思想）。
  この規則は通常受信時と replay とで完全に同一である。
- **車速サンプルのゲート（D42）**: 車速には **層 A（χ² ゲート、dof = 1、閾値 `gate_chi2.dim1`）のみ**を適用する。
  適用箇所は Graph1 の EKF 更新の直前（`d² = (v̂_x − z_v)² / (H Σ₁ Hᵀ + σ_v²)`）。棄却されたサンプルは
  `VehicleTwistBuffer` に**棄却フラグ付きで保存**し、Graph1 でも Graph2 でも使わない（replay 時も同じ判定を再現する）。
  **層 B（連続重み付け）は車速には適用しない**（1 次元スカラーに対する共分散膨張は非ホロノミック拘束と併せて二重の減衰になり、
  停車時の挙動が不安定になるため）。層 C も対象外。
- 生成した各グリッド点を **`StateHistory` に格納**（§5.3.4）。
- **この「IMU 伝播 + 車速 EKF 更新」の処理列は、通常受信時と §5.3.7 (5) の replay とで完全に同一**でなければならない（D15 / 試験 18）。

**Graph2（低レート・全入力・要求 R9/R10）: オーバーラップ付き固定ラグ窓 + バッチ再構築**

```
  過去 ←                                                        → 現在
                        ← graph2_lag_ms (1000 ms) →
        ┌───────────────────────────────────────────┐
  ──────┤ marginal prior          optimization window├──── t
     t_begin              t_next_begin             t_end
   （左端・境界ノード） （次 tick の境界・強制ノード） （右端・tick 時刻＝アンカー）
        ↑                        ↑                       ↑
   これより過去は        次 tick でここまでを      遅延測定 (≤300 ms) は
   prior 1 個へ畳み済み   周辺化して畳む            必ずこの窓の中に落ちる
```

**窓の定義**:

| 記号 | 意味 |
| --- | --- |
| `t_end` | 窓の**最新端（右端）** = tick 時刻。ここに必ずノードを作る。**Graph1 へ渡すアンカー**もこのノード |
| **`t_begin`** | 窓の**過去側境界（左端）** ≡ **現在保持している境界 prior が張られているノードの時刻**（実装識別子 `t_prior_`）。`t_end − graph2_lag_ms` から毎 tick 計算するのではなく、**周辺化 (g) が成功したときにのみ更新される状態変数**。正常時は `t_begin = 前 tick の t_next_begin` となり、結果的に `t_end − graph2_lag_ms` に一致する |
| `t_next_begin` | **次 tick の境界** = `t_end + graph2_optimize_interval_ms − graph2_lag_ms`。**今回の窓の強制ノードとして必ず生成する**（D29） |
| 境界 prior | `t_begin` より古い領域を周辺化して得た `(X,V,B)@t_begin` 上の線形ファクタ 1 個 |

> **`t_begin` を「prior の時刻」と定義する理由（D35）**: 最適化に失敗した tick では周辺化 (g) を実行しないので、
> 境界 prior は**前 tick の時刻のまま**残る。それにもかかわらず `t_begin = t_end − graph2_lag_ms` と毎 tick 計算すると、
> (i) 窓の左端が prior のノードより新しくなり **prior をどのノードに張るかが未定義**になり、
> (ii) `t_begin` より古いという理由で (a-4) が測定を捨てるが、その測定は**まだどの prior にも吸収されていない**（＝情報の純損失）。
> `t_begin ≡ t_prior_` と定義すれば、失敗中は窓が右へ伸びるだけで、prior・測定・IMU 区間のどれも整合が崩れない。
> 代償は窓長が `graph2_lag_ms + 失敗回数 × graph2_optimize_interval_ms` まで伸びることであり、
> `optimizer_failure_limit`（既定 3 回）で頭打ちになる。バッファ長の下限は §6.2 でこの伸びを含めて検証する。

- 実装識別子は `t_window_begin_` / `t_window_end_` / `t_next_window_begin_` とする。`begin` / `end` は時間軸の左右（過去／未来）と一致させる。

**遅延測定が必ず窓内に入る条件（§6.2 で強制）**:

```
graph2_lag_ms ≥ graph2_optimize_interval_ms + max_measurement_delay_ms + lag_margin_ms
既定:  1000 ≥ 500 + 300 + 100 = 900   ✓
```

導出: tick 直後（時刻 `T`）に到着した測定は次の tick `T+interval` まで待たされる。その測定の stamp は最古で `T − max_delay`。
処理時点の境界は `t_begin = T + interval − lag` なので、`T − max_delay > T + interval − lag` すなわち `lag > interval + max_delay` が必要。

- それでも境界より古い stamp の測定は**破棄**する（診断 WARN `"before graph2 window"`）。これは
  `max_measurement_delay_ms` を超える異常遅延か、時刻同期異常の場合に限られる（INV-5 の明示的例外・下記 (a-4)）。

##### 5.3.2a `Graph2MeasurementHistory`（D28 / D34）

毎 tick バッチ再構築は、「窓内で既に採用した測定を保持し続けるストア」がなければ成立しない
（`PendingMeasurementQueue` は tick で drain されるため次 tick では消えてしまう）。責務を次のように分離する:

| 構造 | 所有 | 寿命 | 責務 |
| --- | --- | --- | --- |
| `PendingMeasurementQueue` | 測定 callback ↔ ワーカ（`pending_mutex_`） | 次 tick まで | スレッド間の受け渡しのみ |
| **`Graph2MeasurementHistory`** | Graph2 ワーカ専有（ロック不要） | 周辺化 prior に吸収されるまで | 窓内の採用済み測定の**唯一の正**。毎 tick ここから全ファクタを作り直す |

```cpp
enum class Contribution {
  kNotYetBuilt,                    //  まだ一度もグラフ構築を通っていない
  kFactorAdded,                    //  直近 tick で実際にファクタとして投入された
  kIntentionallyExcluded,          //  設計上の理由で意図的に張らなかった
                                   //   （SLAM ジャンプ検出・SLAM 補間アンカー保持）
};

struct AcceptedMeasurement {
  rclcpp::Time            stamp;
  MeasurementSource       source;          // GNSS / NDT / SLAM のみ（車速は含まない・§5.3.3a'）
  MeasurementValue        value;           // 位置 3 / Pose3 / odom Pose3+cov
  gtsam::SharedNoiseModel noise;           // ★§5.3.6 で補正済み。以後 **再計算しない**
  double                  nis;             // d²（受信時点。SLAM relative はサンプル対単位・§5.3.3c）
  double                  weight;          // w（受信時点）
  gtsam::Key              assigned_node{}; // 直近 tick で割り当てたノードキー（prune 判定に使う）
  Contribution            contribution{Contribution::kNotYetBuilt};
};
std::deque<AcceptedMeasurement> measurement_history_;   // stamp 昇順を維持
```

- **共分散補正（§5.3.6 の層 A/B）は「測定到着時点の Graph1 予測分布」で一度だけ確定し、以後 tick ごとに再計算しない（INV-3）。** 理由:
  1. 要求 R11 の意味論そのものが「到着時点で内界センサから到達可能な範囲か」であり、事後推定で測り直すと定義が変わる。
  2. 再計算すると「測定 → 推定 → その測定の重み」の自己参照ループが生じ、外れ値が自分を正当化しうる。
  3. 到着順序に依存しない再構築（試験 17 ④）を保証するには、重みが tick に依存してはならない。
- history のサイズ上限は `graph2.measurement_history_max`（既定 512）。超過時は最古から捨てて診断 WARN（異常入力レートに対する暴走防止）。
- `reset()` / `trigger_node(false)` / 再初期化（§5.3.8）で **全消去**する。

**tick（`graph2_optimize_interval_ms`、既定 500 ms）ごとの手続き**:

```
(a) 測定の収集
  (a-1) PendingMeasurementQueue を drain（ロック区間は move のみ）
  (a-2) Graph2MeasurementHistory へ stamp 昇順を保ったまま挿入（out-of-order 到着もここで整列）
  (a-3) history から [t_begin, t_end] の測定を全件取得（＝前 tick で採用済みの測定も必ず再投入される）
  (a-4) stamp < t_begin のものは history から削除 + 診断 WARN "before graph2 window"
        ★これは INV-5 の明示的な例外である（§5.3.7 の不変条件表を参照）。
          contribution が kNotYetBuilt でも削除する ＝ その測定は本当に失われる。
          異常遅延・時刻同期異常でのみ発生するため、削除件数をカウンタとして診断に出す。
        stamp > t_end（tick 時刻より未来）のものは history に残し、次 tick で採用する
(b) ノードタイムライン生成（この段階で out-of-order は解消される）
      {t_begin, t_next_begin, t_end}                       ← t_next_begin を必ず含める（D29）
      ∪ 前回窓のノード時刻のうち t_begin 以上のもの
      ∪ (a-3) の測定 stamp
      ∪ 強制ノード（間隔が graph2_max_node_interval_ms = 100 ms を超える区間を分割）
      を昇順に並べ、node_merge_tolerance_ms(5 ms) 以内の時刻は 1 ノードに併合
      ★ ただし t_begin は「併合の吸収先」にしない（D34）:
          |stamp − t_begin| ≤ node_merge_tolerance_ms の測定は t_begin へ併合せず、
          max(stamp, t_begin + node_merge_tolerance_ms) に独立ノードを作る
      → 各測定に「割り当てノードキー」を確定して AcceptedMeasurement::assigned_node に記録
(c) 区間ごとの IMU プリインテグレーション
      連続ノード (t_i, t_{i+1}) ごとに ImuBuffer から該当区間を取り出し、
      直近のバイアス推定で PreintegratedImuMeasurements を作り直す
      → ノードが途中に挿入されても区間は必ず正しく分割される
(d) ファクタグラフ構築
      境界 prior（LinearContainerFactor、キー = t_begin ノード）
      + 区間ごとの ImuFactor + BetweenFactor<ConstantBias>
      + history 由来の測定ファクタ（GNSS / NDT / SLAM）
      + 車速ファクタ（各ノードへ最寄り 1 サンプル・§5.3.3a'）
      ★ 境界ノード t_begin には unary 測定ファクタ（GNSS / NDT / SLAM 絶対 / 車速）を張らない。
        そこに掛かる情報は既に境界 prior へ吸収済みであり、張ると二重計上になる。
        相対ファクタ（SLAM BetweenFactor / ImuFactor）は t_begin を始点として張ってよい（未吸収のため）。
      → 実際に張った測定の contribution を kFactorAdded に、
        設計上の理由で張らなかったものを kIntentionallyExcluded に更新する
(e) 初期値
      前回窓の解（共通ノード）＋ StateHistory からの補間（新規ノード）
(f) 最適化（§5.3.7）→ アンカー X*,V*,B* @ t_end と marginal covariance を取得
(g) 周辺化と prune（§5.3.7 (3)）
      境界ノード j* = t_next_begin のノード（(b) で必ず存在する）
      j* 以下のみに依存するファクタを周辺化 → 次 tick の境界 prior として保存
      t_begin ← t_next_begin（★ t_begin は「prior の時刻」なのでここでのみ更新する・D35）
      assigned_node ≤ j* **かつ** contribution ≠ kNotYetBuilt の測定を history から削除
      ※ 最適化が失敗した tick では (g) を実行しない（history も prior も t_begin も更新しない・§5.3.9）
```

**境界ノードに併合された測定の情報消失を防ぐ（D34）**

規則の組み合わせには、測定が**黙って捨てられる**穴がある:

```
1. tick 時、drain された新規測定の stamp が t_begin の 5 ms 以内だった
2. (b) の併合規則により assigned_node = 境界ノード t_begin
3. (d) の規則「境界ノードには unary 測定ファクタを張らない」により ファクタが張られない
4. (g) の prune 規則「assigned_node ≤ j* を削除」により 吸収済みとして削除される
   → その測定は一度もグラフに寄与しないまま消える（二重計上ではなく情報の純損失）
```

`t_begin` は前 tick の `t_next_begin` であり、外界センサの stamp がそこへ 5 ms 以内に落ちる確率は 10 Hz の入力なら 1 tick あたり数 % ある。
症状は「たまに GNSS/NDT が 1 個だけ効かない」であり、発散しないので発見が難しい。**構造 + 検証の二段で潰す**:

| 段 | 対策 |
| --- | --- |
| **構造（主対策）** | `t_begin` を併合の吸収先から除外する。`\|stamp − t_begin\| ≤ node_merge_tolerance_ms` の測定は `max(stamp, t_begin + node_merge_tolerance_ms)` に**独立ノード**を作ってそこへ張る。ずらした時間差 `Δt = ノード時刻 − stamp`（最大 5 ms）はノイズへ加算する: 位置 `σ² += (graph2.boundary_merge_velocity_mps × Δt)²`（既定 20 m/s、5 ms で +0.1 m 相当）、姿勢 `σ² += (graph2.boundary_merge_yaw_rate_rps × Δt)²`（既定 1.0 rad/s、5 ms で +0.005 rad 相当）。`t_next_begin`（= 次の `j*`）は unary ファクタを張ってよいので併合の吸収先にしてよい |
| **検証（保険）** | `AcceptedMeasurement::contribution` を持ち、(g) の prune では `contribution == kNotYetBuilt` の要素を**絶対に削除しない**。debug ビルドでは「削除対象がすべて `kFactorAdded` または `kIntentionallyExcluded`」であることを assert する（INV-5）。release ビルドでは削除せず診断 ERROR + カウンタを上げる |

- `kIntentionallyExcluded` に該当するのは、SLAM のジャンプ検出で張らなかった対（§5.3.3c）と SLAM の補間アンカー（§5.3.3c）である。
  これらは「意図的に使わない」ことが設計で決まっているので prune してよい。
  （車速は history に入らないため、`graph2_max_time_offset_ms` 超過による不使用はこの enum の対象外である。）

**設計上の補足**:

- **なぜ incremental でなくバッチか（D24）**: (b)(c) のとおりノード挿入のたびに IMU 区間の再分割が起きるため、
  iSAM2 への単純な factor 追加では表現できない。窓を毎回組み直せば、**測定の到着順序に依存しない**（試験 17）。
- **なぜ `t_next_begin` を強制ノードにするか（D29）**: 次 tick の周辺化境界に**必ずノードが存在する**ようにすれば、
  チェーン型ファクタ（`ImuFactor` / `BetweenFactor<ConstantBias>` / SLAM `BetweenFactor`）が境界を跨ぐことが**原理的に起こらない**。
  跨ぐファクタが無ければ「境界以下のみに依存するファクタ」と「境界より新しいファクタ」に厳密に二分でき、
  周辺化（Schur 補元）は近似なしで実行でき、history の prune も単純規則で完結する。
  この強制ノードが無い場合、`X(j*−1) → X(j*+1)` を跨ぐ `BetweenFactor` を「周辺化するか残すか」で必ず二重計上か情報欠落のどちらかが発生する。
  追加コストはノード 1 個（`X,V,B` の 3 変数）にすぎない。
  **これはアルゴリズムの成立条件（INV-2）であって選択肢ではないため、実行時パラメータとしては公開しない**。
  「無効化した場合に何が壊れるか」は試験 20 ⑦ のテスト専用フック（`Graph2Builder` の friend テスト）でのみ再現する。
- **コスト**: ラグ 1000 ms・測定 30 Hz 想定でノード 15〜30 個（`X,V,B` で 45〜90 変数）、IMU 100 Hz で区間あたり数〜十数サンプル。
  疎な `LevenbergMarquardtOptimizer` で数 ms〜数十 ms のオーダーであり、tick 500 ms に対して十分な余裕がある（試験 10 で p99 < 300 ms を確認）。
- **オプティマイザ**: 既定 `graph2.optimizer: levenberg_marquardt`（バッチ）。`isam2` も選択可能にするが、その場合も
  **窓を毎 tick 作り直して `ISAM2` を新規構築する**（incremental 更新はしない）。iSAM2 の incremental 性を活かす構成は
  out-of-order 対応と両立しないため Phase 2 以降の検討事項とする（§9.2）。
- ノード時刻は**測定時刻をそのまま使う**（固定グリッドへスナップしない）。
  20 ms グリッドへスナップすると最大 10 ms のずれ、15 m/s 走行時に **15 cm** の量子化誤差が入り、cm 級を狙う本設計では許容できない。
- 上限 `graph2_max_nodes`（既定 80）を超える場合は、古い側から強制ノードを間引いて（測定ノードは残す）上限内に収め、診断 WARN（暴走防止）。
- **ノード周期・最適化周期・ラグ長は独立パラメータ**（`graph2_max_node_interval_ms` / `graph2_optimize_interval_ms` / `graph2_lag_ms`）。

---

#### 5.3.3 入力とファクタ

| 入力 | トピック | 要求 | 追加先 | GTSAM ファクタ |
| --- | --- | --- | --- | --- |
| IMU | `/sensing/imu/imu_data` | R5 | Graph1(伝播), Graph2 | `ImuFactor`（`PreintegratedImuMeasurements`）+ `BetweenFactor<ConstantBias>`（D6・LIO-SAM 準拠） |
| 車速 | `/sensing/vehicle_velocity_converter/twist_with_covariance` | R5 | Graph1(区間あたり最新 1 サンプル), Graph2(**各ノードへ最寄り 1 サンプル**・§5.3.3a') | **`WheelSpeedFactor`（自作）** + 任意で `NonHolonomicFactor` |
| GNSS | `/sensing/gnss/pose_with_covariance` | R5 | Graph2 のみ | **`GnssLeverArmFactor`（自作・§5.3.3b）**: 位置のみ絶対拘束。レバーアーム `r_bg` を**姿勢依存の測定モデル**として扱う |
| NDT | `/localization/pose_estimator/pose_with_covariance` | R5 | Graph2 のみ | `PriorFactor<Pose3>`（姿勢絶対拘束） |
| SLAM（LiDAR オドメトリ） | 外部 LIO-SAM `mapOptimization`（既定 `lio_sam/mapping/odometry`） | **拡張（D7）** | Graph2 のみ | 既定 `BetweenFactor<Pose3>`（相対）／`PriorFactor<Pose3>`（絶対、選択可）（§5.3.3c） |

- 絶対姿勢入力（GNSS / NDT / SLAM）は §5.3.6 の共分散補正を通した noise model で追加する。車速は層 A のみ（§5.3.2 / D42）。
- SLAM は要求（`requirement.md`）に無い拡張入力である。要求どおりの入力構成で評価する場合は `slam.use: false` を設定する。

##### 5.3.3a 車速ファクタの測定モデル（D11）

**`WheelSpeedFactor`: `NoiseModelFactor2<Pose3, Vector3>`（残差 1 次元）**

```
状態:      X_k = T^map_base ,  V_k = v^map （map 系の base_link 速度）
測定:      z_v = twist.linear.x                （base_link 前進速度 [m/s]）
予測:      v̂_x = e_xᵀ · R(X_k)ᵀ · V_k
残差:      r = v̂_x − z_v                        （1 次元）
ノイズ:    σ_v² = twist.covariance[0]（= linear.x 分散）。0 / 負 / NaN なら
           wheel_speed.fallback_stddev（既定 0.2 m/s）を使用

ヤコビアン:
  ∂r/∂X_k = [ e_xᵀ · skew(R(X_k)ᵀ V_k) ,  0₁ₓ₃ ]   （GTSAM tangent 順 [ω; ν]、右摂動）
  ∂r/∂V_k = e_xᵀ · R(X_k)ᵀ
```

**`NonHolonomicFactor`（既定 ON、`wheel_speed.use_nonholonomic: true`）**: `NoiseModelFactor2<Pose3, Vector3>`（残差 2 次元）

```
残差:  r = [ e_yᵀ R(X_k)ᵀ V_k ,  e_zᵀ R(X_k)ᵀ V_k ]ᵀ    （横滑り・上下速度 ≈ 0）
ノイズ: σ_lat（既定 0.1 m/s）, σ_vert（既定 0.05 m/s）
無効化: |z_v| < nonholonomic_min_speed_mps（既定 0.5）では追加しない（停車時の過拘束回避）
```

- Graph1 では同じ測定モデルを**EKF 形式の 1 ステップ更新**として適用する（グラフを組まない、§5.3.2）。数式は同一。
- 車速のレバーアーム: 車速は後輪車軸中心相当で `base_link` 前方成分として与えられるため、追加補正はしない（Autoware の `vehicle_velocity_converter` が既に `base_link` 基準で出す）。

##### 5.3.3a' Graph2 における車速ファクタの結び付け（D30）

車速は `PendingMeasurementQueue` ではなく `VehicleTwistBuffer` に入るため（§4.2）、Graph1 と Graph2 で扱いを変える:

| グラフ | 車速の使い方 | 理由 |
| --- | --- | --- |
| **Graph1** | 20 ms グリッド区間あたり最新 1 サンプルで EKF 形式の 1 ステップ更新（既定 50 Hz では全サンプル相当） | 高レート推定の速度・ヨー安定性に直結する |
| **Graph2** | **各ノード時刻に最寄りの 1 サンプルだけ**を `WheelSpeedFactor`（+ `NonHolonomicFactor`）として張る | 車速 stamp をすべてノード化すると 1 s 窓・50 Hz で約 50 ノード増え、「15〜30 ノード想定」（§5.3.2 のコスト見積）と矛盾する |

**割り当て規則（`wheel_speed.graph2_binding: nearest`（既定））**:

```
1. 窓 [t_begin, t_end] の VehicleTwistBuffer サンプル s_i（層 A で棄却されていないもの）を、
   最も近いノード時刻 t_k へ割り当てる    k(i) = argmin_k |t_{s_i} − t_k|
2. 各ノード t_k について、割り当てられたサンプルのうち最も近い 1 個だけを採用する
      （採用されなかったサンプルは Graph2 では使わない ＝ 同一サンプルが 2 ノードで使われることはない）
3. |t_{s} − t_k| > wheel_speed.graph2_max_time_offset_ms（既定 30 ms）なら、そのノードには張らない
4. 境界ノード t_begin には張らない（§5.3.2 (d)。既に境界 prior へ吸収済み）
5. 時刻ずれ分をノイズへ加算する:
      σ_v'² = σ_v² + (graph2_time_offset_accel × Δt)²      （既定 3.0 m/s²、Δt = |t_s − t_k|）
      → Δt = 10 ms なら +0.03 m/s 相当で、σ_v 既定 0.2 m/s に対して十分小さい
```

- **なぜ間引いてよいか**: 車速は車両運動の帯域（数 Hz）に対して 50 Hz と大幅に過サンプリングされており、隣接サンプルは強く相関する。
  全サンプルを独立とみなして 50 本のファクタを張る方が、むしろ情報量を過大評価して共分散を過小にする。
  ノードあたり 1 本へ間引くのは、計算量削減であると同時に**相関を無視することによる過信の緩和**でもある。
- 代替として `wheel_speed.graph2_binding: interpolate`（前後サンプルの線形補間値を使い、補間誤差をノイズに加算）と
  `none`（Graph2 では車速を使わず Graph1 のみ）を選べるようにする。既定は `nearest`。
- `graph2_binding: none` は、車速を Graph1 だけで使い Graph2 は外界＋IMU に専念させる構成であり、
  二重計上を最も保守的に避けたい場合の逃げ道として常設する（試験 11 ③ で NEES 比較）。

##### 5.3.3b GNSS ファクタとレバーアーム補正（D25）

GNSS が測るのは **アンテナ位置**であり、`base_link` 位置ではない。アンテナのレバーアームを
`r_bg = t_base_gnss`（`base_link` 系で表したアンテナ位置）とすると、測定モデルは

```
状態:   X_k = T^map_base ∈ Pose3   （p = X_k.translation(), R = X_k.rotation()）
測定:   z ∈ R³                      （map 系のアンテナ位置）
予測:   ĥ(X_k) = p + R · r_bg
残差:   r = ĥ(X_k) − z              （3 次元・map 系）
```

**姿勢に依存するファクタである**点が重要で、`z − R̂·r_bg` を前処理して「base_link 位置の測定」に変換する方式は、
その時点の姿勢推定 `R̂` を**定数として固定**してしまうため採らない（姿勢誤差がそのまま位置測定のバイアスになり、
かつ GNSS が姿勢を拘束する経路が失われる）。

**`GnssLeverArmFactor : gtsam::NoiseModelFactor1<gtsam::Pose3>`**

```
ヤコビアン（GTSAM tangent 順 [ω; ν]、右摂動 X ⊕ ξ = X · Expmap(ξ)）:

  ĥ(X ⊕ ξ) = p + R·Expmap(ω)·r_bg + R·J_l(ω)·ν
           ≈ p + R·r_bg − R·skew(r_bg)·ω + R·ν

  ∂r/∂X = [ −R · skew(r_bg) ,  R ]      （3×6）
```

- `r_bg = 0`（アンテナが `base_link` 上）の場合はヤコビアンが `[0, R]` となり、GTSAM の `GPSFactor` と厳密に一致する。
  実装上は場合分けせず同一ファクタで扱う。
- ビルド先の GTSAM がレバーアーム対応の GPS ファクタ（`gtsam::GPSFactorArm` 等）を提供している場合はそれを使ってもよい。
  **CMake でシンボルの有無を確認**し、無ければ自作実装にフォールバックする（`GNSS_FACTOR_USE_GTSAM_ARM` マクロ）。
  どちらを使う場合も試験 14（`numericalDerivative` との一致）を必ず通す。
- **ノイズモデルと座標系**: 残差が map 系の 3 次元位置なので、ROS メッセージの共分散の左上 3×3（既に map 系）を
  **回転変換せずそのまま**使う。§5.3.5 の body 系変換は適用しない。
- **層 A/B（§5.3.6）で使うイノベーションと予測共分散**も同じ測定モデルで作る:
  ```
  ν   = ( p₁ + R(μ₁)·r_bg ) − z                       （3 次元・map 系）
  J   = [ −R(μ₁)·skew(r_bg) , R(μ₁) ]                 （3×6）
  Σ_ĥ = J · Σ₁ · Jᵀ                                    （body tangent の Σ₁ を map 系アンテナ位置へ写す）
  S   = Σ_ĥ + R_z,pos ,   d² = νᵀ S⁻¹ ν               （dof = 3）
  ```
  これにより「レバーアーム由来の姿勢不確かさ」もゲートに正しく反映される。
- `r_bg` の取得元は §5.3.1 の表に従う（TF `base_link ← gnss_link` の並進、または `gnss.lever_arm_fallback`）。

##### 5.3.3c SLAM（LiDAR オドメトリ）入力の扱い（拡張入力 / D7・D38）

LiDAR オドメトリは**短時間の相対運動が非常に高精度**である一方、map フレーム絶対値は長期的にドリフトしうる。

- **主案（既定）: 相対拘束 `BetweenFactor<Pose3>`**。拘束もゲートも**「生の連続サンプル対 `(i−1, i)`」を単位**にする。
  これはノードタイムラインに依存しない（SLAM の stamp は必ずノードになる・§5.3.2 (b)）ので、到着時に凍結でき INV-3 と両立する。

  ```
  [SLAM サンプル i の到着時（callback スレッド・§5.3.4 の手続きの一部）]
    前サンプル i−1（history 内、または補間アンカー）との対を作る:
      ΔT_slam  = T_odom(t_{i-1})⁻¹ · T_odom(t_i)
      Σ_Δ      = 相対共分散（下記「相対共分散の作り方」）
      Δt       = t_i − t_{i-1}

    予測側は **Graph1 の同区間の相対運動**（絶対共分散ではない）:
      ΔT_g1    = μ₁(t_{i-1})⁻¹ · μ₁(t_i)                     （StateHistory から SE(3) 補間・§5.3.4）
      Σ_rel,g1 = 区間 [t_{i-1}, t_i] の IMU プリインテグレーション共分散の pose ブロック
                 + 同区間の車速更新で減る分（実装は Graph1::propagateInterval() で
                   「その区間だけを再伝播して得た共分散」を使う）
                 ※ アンカー誤差は両端で共通なので相対量では相殺する。
                   Σ₁(t_{i-1}) や Σ₁(t_i)（絶対共分散）を使ってはならない（過大評価でゲートが効かなくなる）

    残差とゲート（dof = 6）:
      ν  = Log( ΔT_g1⁻¹ · ΔT_slam )
      S  = Σ_rel,g1 + Σ_Δ
      d² = νᵀ S⁻¹ ν
      層 A: d² > gate_chi2.dim6(22.46) → この 1 区間の BetweenFactor を張らない（kIntentionallyExcluded）
      層 B: w = exp(−½ max(0, d²−6)) を掛けて Σ_Δ' = Σ_Δ / w
      → {ΔT_slam, Σ_Δ', d², w} を対の情報として **到着時に凍結**し history に保持（INV-3）

    ジャンプ検出（下記）を通過した対のみ採用する。
  ```

  - **ファクタの張り方**: tick では、対 `(i−1, i)` の 2 つの stamp に割り当てられたノード
    `k(t_{i-1})` と `k(t_i)` の間に `BetweenFactor<Pose3>(X(k(t_{i-1})), X(k(t_i)), ΔT_slam, noise(Σ_Δ'))` を張る。
    **時刻補間は不要**（SLAM stamp は必ずノード時刻になるため）。`node_merge_tolerance_ms` で 2 つが同一ノードに併合された場合
    （SLAM レートが 5 ms より速い異常時）はその対を捨てて診断 WARN。
  - **history との関係**: `Graph2MeasurementHistory` には**オドメトリ姿勢サンプルそのもの**（`T_odom(t)` と共分散）と、
    そのサンプルを終点とする**対の凍結情報** `{ΔT_slam, Σ_Δ', d², w}` を保持する。ファクタは毎 tick その場で生成し直すが、
    **`ΔT_slam` と `Σ_Δ'` は再計算しない**。
  - **境界での扱い**: 対の始点 `t_{i-1}` が `t_begin` より古い場合、その対は張れない。始点サンプルを
    **補間アンカーとして 1 個だけ例外的に残す**（prune 規則の例外・`kIntentionallyExcluded` として扱う）。
    始点が `t_begin` ちょうどに割り当てられる場合、`BetweenFactor` は `t_begin` を**始点**とする相対ファクタなので張ってよい
    （境界 prior に吸収されていない・§5.3.2 (d)）。
  - **境界を跨いだ対の扱い（INV-2 との整合）**: 対の 2 点が `j* = t_next_begin` を跨ぐ場合、そのままでは周辺化境界を跨ぐファクタになる。
    `t_next_begin` は強制ノードなので、**対を `(t_{i-1} → j*)` と `(j* → t_i)` に分割**し、`ΔT_slam` を `j*` で補間して 2 本にする。
    分割で生じる補間誤差は `Σ_Δ'` に `(slam.interp_sigma_per_s × 分割位置までの時間)²` を加算して吸収する（既定 0.05 m/s・0.01 rad/s）。
    分割後の 2 本は合計で元の 1 本と同じ情報量になるよう、`Σ_Δ'` を区間長で按分する
    （`Σ_Δ',前 = α Σ_Δ'`, `Σ_Δ',後 = (1−α) Σ_Δ'`, `α = (j*−t_{i-1})/Δt`。和が `Σ_Δ'` に戻る）。
  - **相対共分散の作り方**:
    **最優先は「SLAM 側が相対増分の共分散を直接出せるならそれを使う」**（`slam.covariance_is_relative: true`）。
    出せない場合に限り、連続 2 サンプルの絶対共分散を**独立と仮定して**合成する:
    ```
    Σ_Δ,indep = Ad_{ΔT}⁻¹ Σ_{k-1} Ad_{ΔT}⁻ᵀ + Σ_k        （Ad は Pose3::AdjointMap）
    ```

    > **独立仮定は保守側ではない（D33）**。厳密には交差共分散 `P₁₂ = Cov(ξ_{k-1}, ξ_k)` を含み
    >
    > ```
    > Σ_Δ,true = J₁ Σ_{k-1} J₁ᵀ + J₂ Σ_k J₂ᵀ + J₁ P₁₂ J₂ᵀ + J₂ P₂₁ J₁ᵀ
    >            （ΔT = T_{k-1}⁻¹T_k に対し J₁ = −Ad_{ΔT}⁻¹, J₂ = I）
    > Σ_Δ,indep − Σ_Δ,true = 2·sym( Ad_{ΔT}⁻¹ P₁₂ )
    > ```
    >
    > であり、**独立仮定が保守側になるのは `sym(Ad_{ΔT}⁻¹ P₁₂) ⪰ 0` のときに限る**（必要十分条件）。
    > 実運用の大半（連続 2 時刻の絶対姿勢が共通のマップ・共通のドリフトを共有し `P₁₂` が正相関）では成り立つが、
    > (a) 2 サンプルの間にループ閉じ込み・再定位が入り誤差が反相関になった場合、
    > (b) `mapOptimization` が `Σ_k` を毎回同じ固定値で出しており `P₁₂` と無関係な場合、には成り立たない。
    > よって「保守側だから安全」を根拠にしてはならない。**計算の都合による近似であり、保守性は保証しない**。

    運用上の扱い:
    - `slam.relative_covariance_scale`（既定 1.0）は「保守性を担保する係数」ではなく**チューニングノブ**である。
      試験 11 の NEES で `slam.use: true/false` を比較し、NEES が dof の 95 % 信頼区間に入る値を実測で決める。
    - 上記 (a) の反相関ケースは、下記のジャンプ検出（区間除外）で実装的に排除する。

  - **ジャンプ検出（D38）**: 「ジャンプ」は絶対量ではなく**推測航法から見た逸脱**で定義する。固定距離閾値は使わない
    （15 m/s × 100 ms = 1.5 m であり、固定 1.0 m のような閾値では正常走行が全棄却される）。2 段構えで判定する:

    ```
    (i) 主判定 — 推測航法残差の χ² 検定（上の d²）
        d² = νᵀ (Σ_rel,g1 + Σ_Δ)⁻¹ ν  >  gate_chi2.dim6 (22.46)  → 張らない
        ループ閉じ込みによる基準更新は「IMU + 車速で説明できない相対増分」として必ずここに現れる。
        走行速度に依存しない（速いほど Σ_rel,g1 も大きくなる）。

    (ii) 副判定 — 動力学的上界（Σ の見積り誤りに対する保険・スカラー）
        |Δp_slam|  >  slam.max_speed_mps × Δt + slam.jump_margin_m      → 張らない
        |Δθ_slam|  >  slam.max_yaw_rate_rps × Δt + slam.jump_margin_rad → 張らない
        既定: max_speed_mps = 20.0, max_yaw_rate_rps = 1.0,
              jump_margin_m = 0.30, jump_margin_rad = 0.05
        Δt = 100 ms なら 2.3 m / 0.15 rad が上界（15 m/s 走行の 1.5 m は通る）
    ```

    - (i) だけでは `Σ_Δ` を過大申告する SLAM に対して無力（`d²` が小さくなる）なため、(ii) を保険として常時併用する。
      逆に (ii) だけでは低速時のループ閉じ込みを見逃すため、(i) が主判定である。
    - どちらで棄却したかを `debug/covariance_correction` に別カウンタで出す。(ii) が発火する場合は
      **時刻同期または `slam.frame_id` の設定ミスを疑う**（正常なループ閉じ込みは通常 (i) で捕まる）。

- **副案: 絶対拘束 `PriorFactor<Pose3>`**（`slam.use_as: absolute`）。map フレームで運用し、NDT と同様に単一測定単位で扱う。
- 参照フレーム整合: 外部 LIO-SAM の出力座標系と Autoware `map` の対応を `slam.frame_id` で明示し、必要なら静的変換を適用。
- **情報の二重計上リスク**: LIO-SAM の `mapOptimization` オドメトリは既に **IMU プリインテグレーション結果を含む**（初期推定・IMU ファクタ経由）。
  これを生 IMU と同じ Graph2 に入れると同一情報を二重に数えて共分散を過小評価する。対策:
  1. 既定で `slam.covariance_scale`（既定 3.0）を掛けて拘束を意図的に弱める。
  2. LIO-SAM 側で **LiDAR のみのオドメトリ**が取れる場合はそちらを推奨。
  3. 試験 11 ① で `slam.use: true/false` 間の共分散一貫性（NEES）を評価し、`covariance_scale` を確定する。
  4. 二重計上が許容できないと判断された場合の逃げ道として `slam.use: false` を常に用意する（要求どおりの入力構成でもある）。

#### 5.3.4 時刻処理と遅延測定（D12）

```cpp
struct StateSample {
  rclcpp::Time   stamp;
  gtsam::Pose3   pose;      // μ₁
  gtsam::Vector3 velocity;
  gtsam::Matrix6 pose_cov;  // Σ₁ (GTSAM body tangent, §5.3.5)
  gtsam::Matrix3 vel_cov;
};
// リングバッファ。20 ms 刻み × history_length_ms(既定 1500) = 75 サンプル
```

**測定受信時の手続き（GNSS / NDT / SLAM 共通・callback スレッドで完結）**:

```
1. stamp 妥当性
     dt = now() − msg.header.stamp
     dt < −future_tolerance_ms(20)        → drop, 診断 WARN "future stamp"
     dt > max_measurement_delay_ms(300)   → drop, 診断 WARN "stale measurement"
     stamp が StateHistory の最古より前     → drop, 診断 WARN "too old"
     stamp < 現在の Graph2 窓境界 t_begin   → drop, 診断 WARN "before graph2 window"
       （t_begin はワーカが tick ごとに更新するため std::atomic 相当で公開する。
         callback 側の読みが 1 tick 古くても、実害は「本来 drop できた測定が (a-4) で drop される」だけ）
2. μ₁(t), Σ₁(t) の取得（★ smoothing 前の internal estimate から取る・§5.3.7）
     前後サンプル (t_i ≤ t ≤ t_{i+1}) を二分探索
     α = (t − t_i) / (t_{i+1} − t_i)
     μ₁(t) = gtsam::interpolate(pose_i, pose_{i+1}, α)  // SE(3) 上の補間
     Σ₁(t) = (1−α)·Σ_i + α·Σ_{i+1}                       // 一次近似（下記の注記を参照）
3. §5.3.5 で R_z を GTSAM 表現へ変換（GNSS は §5.3.3b のとおり map 系のまま扱う）
4. §5.3.6 で共分散補正 → 重み w、補正後 R_z'
   ★ SLAM relative（slam.use_as: relative）のみ、この手順を**サンプル対 (i−1, i) 単位**で行う（§5.3.3c）。
     直前サンプルが手元に無い（初回・長い dropout 後）場合は対を作らず、その 1 サンプルは
     「次の対の始点」としてだけ history に入れる（kIntentionallyExcluded）。
5. PendingMeasurementQueue に {stamp, 測定値, 補正後 noise model, ソース種別, d², w} を push（§4.3）
     → 次の Graph2 tick で drain され Graph2MeasurementHistory へ移される（§5.3.2a）。
       以後その測定は**周辺化 prior へ吸収されるまで毎 tick 再投入される**。
       ここで確定した noise model・d²・w は**以後変更しない**（INV-3）
```

> **共分散補間の性格**: `Σ(t) = (1−α)Σ_i + αΣ_{i+1}` は保守側になる保証を持たない。
> **計算量を抑えるための一次近似**であり、PSD 性が保たれること（PSD 行列の凸結合は PSD）だけが保証される。
> 区間長は 20 ms で `Σ_i` と `Σ_{i+1}` の差は小さいため、初期実装としては十分合理的である。
> 厳密性が必要な場合は `timing.exact_interpolation: true` で、**`t_i` の状態から測定時刻 `t` まで IMU を部分積分**して
> `μ₁(t), Σ₁(t)` を直接生成する経路に切り替えられるようにする（コストは区間内 IMU サンプル数に比例）。

- **なぜ 300 ms までなら扱えるか**: Graph2 は **`graph2_lag_ms`（既定 1000 ms）の固定ラグ窓**を保持しており、
  「tick 待ち最大 500 ms + 測定遅延 300 ms」を吸収できるため。§6.2 で下限を強制する。
- **StateHistory は Graph1 リセット時に全消去しない**。§5.3.7 の replay で作り直すのは `t_end` 以降のサンプルのみで、
  `t_end` より古いサンプルは `history_length_ms` に達するまで保持する。これにより、アンカー更新直後に届いた遅延測定でも
  `μ₁(t), Σ₁(t)` を引ける。`history_length_ms ≥ max_measurement_delay_ms + graph2_optimize_interval_ms + margin` を検証する。
- 出力タイマの stamp は §3.3 のとおり `now()` 外挿。外挿は最新の Graph1 状態から IMU で前方積分して行う。

#### 5.3.5 共分散表現と ROS ↔ GTSAM 変換（D14）

**規約**:

```
GTSAM Pose3 tangent:  ξ = [ωx, ωy, ωz, νx, νy, νz]ᵀ     （回転が先）
GTSAM の retract:     T ⊕ ξ = T · Expmap(ξ)              （右摂動 = body 系）
  ※ GTSAM 4 は既定で GTSAM_POSE3_EXPMAP 有効（完全 SE(3) Expmap）。
     ビルドフラグを CMake で確認し、無効ならビルドエラーにする（§10）。
  ※ したがって marginalCovariance(X(k)) は「X(k) の body 系右摂動」の共分散。

ROS PoseWithCovariance.covariance: 行優先 6×6、順序 [x, y, z, rx, ry, rz]
  摂動の定義:  t' = t + δt        （δt は map 系）
               R' = Expmap(δθ) · R （δθ は map 系の微小回転）
```

**変換式**:

```
Π : 順序入替行列（ROS 順 [t; θ] → GTSAM 順 [θ; t]）     P_perm = Π · P_ros · Πᵀ
J : 一次近似で  δθ = R · ω ,  δt = R · ν   （R = pose の回転行列）
    ⇒ J = blockdiag(R, R)

  ROS → GTSAM body tangent:
    P_gtsam = J⁻¹ · P_perm · J⁻ᵀ = blkdiag(Rᵀ, Rᵀ) · Π · P_ros · Πᵀ · blkdiag(R, R)

  GTSAM body tangent → ROS（上式の逆）:
    P_ros   = Πᵀ · blkdiag(R, R) · P_gtsam · blkdiag(Rᵀ, Rᵀ) · Π
```

**適用上の規則**:

| 場面 | 使う回転 `R` |
| --- | --- |
| イノベーション `ν = Log(μ₁(t)⁻¹ ∘ z)` の tangent 空間 | **`R(μ₁(t))`**（`z` の回転ではない）。`ν` は `μ₁` における右摂動なので、`Σ₁` と `R_z` の**両方を `R(μ₁)` 基準に揃える** |
| Graph1 の `Σ₁` | 既に `μ₁` の body tangent → 変換不要 |
| 入力メッセージの `R_z` | 上式で `R(μ₁)` を使って変換 |
| 出力 pose の共分散 | 逆変換で ROS 表現へ。`R = R(推定姿勢)` |
| **smoothing 由来の膨張項** | `c cᵀ` / `u uᵀ` の加算は **GTSAM tangent 空間で行い、その後に逆変換を 1 回だけ通す**。`c` / `u` は `X_published` の右摂動 tangent（`[ω; ν]` 順）だが **`Σ_internal` は `X_internal` 基準**なので、**加算前に pose は `J_r⁻¹(−c)`、twist は `Ad_{C⁻¹}` で published 基準へ輸送する**（D40・§5.3.7）。ROS 表現へ変換した後に足すのは**禁止**（順序・座標系ともに不一致） |
| **出力 twist の共分散** | 既に `base_link` 系のため回転は不要。**逆順序入替 `Πᵀ`（GTSAM 順 `[ω;ν]` → ROS 順 `[ν;ω]`）のみ**を適用する: `P_twist,ros = Πᵀ · M_twist · Π` |

- 一次近似（`Expmap` の左ヤコビアン `J_l ≈ I`）は `|ω| ≲ 0.1 rad`（≈ 6°）の 1σ で誤差 < 1 % であり、
  自己位置推定の姿勢共分散としては十分。`covariance.use_exact_jacobian: false`（既定）で切替可能にしておく。
- **NG 例（実装禁止）**: `Pose3::Logmap(mu.inverse()*z)` の残差と、ROS メッセージの共分散をそのまま足す。
  必ず本節の変換を通す。ユーティリティ `covariance_convert.{hpp,cpp}` に集約し、単体テストで往復一致を検証する（試験 12）。
- **GNSS**: `GnssLeverArmFactor` の残差は **map 系の 3 次元位置**なので、`P_ros` の左上 3×3 を**そのまま**使う（body 系へ回転しない）。
  ゲート側で必要な予測共分散は `Σ_ĥ = J Σ₁ Jᵀ`（`J = [−R(μ₁)skew(r_bg), R(μ₁)]`）で body tangent から map 系へ写す（§5.3.3b）。

#### 5.3.6 共分散補正ロジック（要求 R11 の核心）

**やりたいこと**: 絶対姿勢入力（GNSS / NDT / SLAM）を Graph2 に加える際、その入力時刻 `t` における **Graph1 の予測分布 `N(μ₁(t), Σ₁(t))`（＝内界センサで到達可能な範囲）** と入力値 `z` を突き合わせ、`z` の確率的信頼度に応じてファクタ共分散を補正する。

**適用対象（D42）**: 層 A/B/C は**絶対姿勢入力（GNSS / NDT / SLAM）にのみ**適用する。
車速は §5.3.2 のとおり **Graph1 の EKF 更新前に層 A（dof=1）だけ**を適用し、層 B/C は適用しない。

**3 層構成と適用順**:

```
[層 0] 妥当性チェック（常時 ON・§5.3.9）
        NaN/Inf、非対称、非 PSD、covariance=0 → 修復 or drop
              ↓
[層 A] ハードゲート（既定 ON）
        d² > gate_chi2[dof] なら factor を追加しない（drop）＋ 棄却カウンタ
              ↓  通過したもののみ
[層 B] 連続重み付け（既定 ON・主ロジック）
        R_z' = R_z / w
              ↓
[層 C] ロバストカーネル（既定 OFF）
        noiseModel::Robust を被せる
```

- **既定は A + B の 2 層**。C は `robust_kernel: none` が既定で、A/B のチューニング後に必要なら有効化する。
  3 層同時 ON は過剰減衰を招くため、`robust_kernel != none` かつ層 A/B も ON の場合は**起動時に診断 WARN** を出す。
- 各層の作用値（`d²`, `w`, 棄却数、採用数）を入力ソース別に `debug/covariance_correction` と診断に出す。

**層 A: カイ二乗ゲート**

```
ν  = 入力に応じた次元のイノベーション（§5.3.5 の tangent 表現）
S  = H Σ₁(t) Hᵀ + R_z        （H = 測定モデルのヤコビアン）
d² = νᵀ S⁻¹ ν
d² > gate_chi2[dof] → 棄却
```

- NDT / SLAM 絶対（`Pose3` 全次元）は `H = I₆` で `S = Σ₁ + R_z` に帰着する。
- GNSS は `H = [−R(μ₁)skew(r_bg), R(μ₁)]`、`ν` と `R_z` は map 系（§5.3.3b）。
- **SLAM 相対**: 絶対姿勢 vs `μ₁` ではなく、**連続サンプル対の相対量**で評価する。
  `ν = Log(ΔT_g1⁻¹ ΔT_slam)`、`S = Σ_rel,g1 + Σ_Δ`（`Σ_rel,g1` は**区間のプリインテグレーション共分散**であって絶対の `Σ₁` ではない）。
  詳細と凍結の単位は §5.3.3c。**relative モードでは INV-3 の凍結タイミングを「サンプル対の成立時（＝後続サンプルの到着時）」と読み替える**。
- **車速**（Graph1 内）: `H = ∂r/∂[X;V]`（§5.3.3a）、`S = H Σ₁ Hᵀ + σ_v²`、dof = 1。
- `μ₁, Σ₁` は必ず **smoothing 前の internal estimate** を使う（D27）。

| 入力 | `ν` の次元 | dof | 既定閾値（p = 0.999） |
| --- | --- | --- | --- |
| GNSS（位置のみ） | 3 | 3 | 16.27 |
| NDT（`Pose3`） | 6 | 6 | 22.46 |
| SLAM 相対（`Pose3`・**サンプル対単位**） | 6 | 6 | 22.46 |
| SLAM 絶対（`Pose3`） | 6 | 6 | 22.46 |
| 車速（スカラー・**層 A のみ / Graph1 内**） | 1 | 1 | 10.83 |

**層 B: 予測分布尤度による連続的信頼度重み付け（主ロジック・要求 R11 のロジック案）**

```
w   = exp( −½ · max(0, d² − dof) )     # 信頼度重み ∈ (0,1]
w   = clamp(w, w_min, 1)
R_z' = R_z / w                          # surprising ほど共分散拡大 ＝ 低信頼
```

意味:
- `z` が Graph1 の到達可能範囲内（`d² ≲ dof`）なら `w≈1` → 申告共分散そのまま採用。
- 到達不能なほど飛んでいる（GNSS マルチパス跳び・NDT 発散）と `d²≫dof` → `w→w_min` で寄与を滑らかに縮小。
- `max(0, d²−dof)` は、整合時の期待値 `E[d²]=dof` を差し引き「予想を超えた驚き」だけを罰するため。
- `w_min`（既定 0.05）で完全棄却を避け、長時間 DR ドリフト時に絶対入力へ復帰できる余地を残す。

> **性格の明示（D22）**: 本式は標準的な NIS ゲートでも、厳密なベイズ推論でもない。
> **適応的共分散インフレーションのヒューリスティック**である（`R'=R/w` は変分ベイズ的な適応 KF の共分散膨張と同族だが、
> `w` の関数形は経験的に選んだもの）。したがって:
> - `w` の関数形・`w_min` は**チューニング対象**として明示的に扱う。
> - 試験 5（GNSS 外れ値）と試験 13（パラメータ感度）で妥当性を数値検証することを**実装完了の必須条件**とする。
> - 代替関数形（`w = dof/max(dof,d²)`、`w = 1/(1+max(0,d²−dof))`）を `weight_function` パラメータで差し替えられるようにする。

**層 C: GTSAM ロバストノイズモデル**

`noiseModel::Robust`（Huber / Cauchy / DCS / Geman-McClure）を測定ファクタに被せ、最適化残差ベースでも外れ値を自動減衰。
層 B が「Graph1 予測 vs 入力」の**事前**ゲート、層 C が「グラフ全体最適 vs 入力」の**事後**ロバスト化として相補的に働く。

**低速時の緩和**

車速が小さい（停止/低速）ときは Graph1 の到達範囲が非常に狭くなり `w` が過敏になりうる。
`|v| < low_speed_relax_mps`（既定 0.5）のとき:
- `Σ₁` の位置対角に下限 `low_speed_min_pos_stddev`（既定 0.10 m）、姿勢対角に `low_speed_min_rot_stddev`（既定 0.01 rad）を課す。
- 実質的にゲートが緩み、停車中の NDT/GNSS 更新が不当に棄却されなくなる。

---

#### 5.3.7 アンカー更新・状態引き継ぎ・測定 replay（D10・D15・D23・D27・D29・D31・D32・D35・D37・D40）

Graph2 最適化完了時（ワーカスレッド、アンカー時刻 = 窓の最新端 `t_end`、次 tick の窓境界 `t_next_begin = t_end + interval − lag`）:

```
(1) 最適化結果の取得（アンカーノード k*、t_end に対応）
      X* = result.at<Pose3>(X(k*))
      V* = result.at<Vector3>(V(k*))
      B* = result.at<ConstantBias>(B(k*))
      Σ* = optimizer.jointMarginalCovariance({X(k*), V(k*), B(k*)})   # 15×15（既定・D10）

(2) 妥当性チェック（§5.3.9）
      発散判定（|V*| > max_velocity_mps, |B*| > max_bias）→ 破棄して前アンカー継続 + 診断 ERROR

(3) ★ 窓の周辺化と history prune（次 tick の境界 prior 生成）
      境界ノード j* = t_next_begin のノード（§5.3.2 (b) で強制生成済み・必ず存在する）
      部分グラフ G_old = { 現在の境界 prior } ∪ { 依存キーがすべて j* 以下のファクタ }
      G_old を (X,V,B)@j* について周辺化（Schur 補元）し、
      LinearContainerFactor 1 個として保存 → 次 tick の境界 prior

      t_begin ← t_next_begin（★ t_begin ≡ 境界 prior の時刻。ここでのみ更新する・D35）

      続いて Graph2MeasurementHistory を prune:
        assigned_node ≤ j* かつ contribution ≠ kNotYetBuilt → prior に吸収済み or 意図的除外 → 削除
        assigned_node ≤ j* かつ contribution == kNotYetBuilt → ★実装バグ（INV-5 違反）。
                                                              削除せず診断 ERROR（debug ビルドでは assert）
        assigned_node > j*            → 未吸収           → 保持（次 tick で再投入）
        SLAM の補間アンカー（j* 以下で最新の 1 サンプル）→ 例外的に保持（§5.3.3c）

      ※ 「stamp ≤ t_next_begin なら削除」という時刻基準では誤る。
        node_merge_tolerance_ms(5 ms) により stamp > t_next_begin でも境界ノードへ併合された測定があり、
        それは吸収済みなので削除しなければならない。**判定は必ず割り当てノードキーで行う**。
      ※ 実装簡略化オプション graph2.marginalization: prior_triple では、
        G_old 上の marginalCovariance から X/V/B の独立 PriorFactor 3 本に落とす（相関を捨てる近似）。
        既定は linear_container。

(4) Graph1 リセット（同じアンカーを共有・要求 R8）
      μ₁ ← X*,  v₁ ← V*,  b₁ ← B*,  Σ₁ ← Σ*（15×15 をそのまま引き継ぐ・D10）
      imuIntegrator->resetIntegrationAndSetBias(B*)

(5) ★ Graph1 measurement replay（D15）
      t_end 以降の ImuBuffer と VehicleTwistBuffer を timestamp 順に merge して再生する:

        for each event in merge_by_stamp(ImuBuffer, VehicleTwistBuffer) where stamp > t_end:
            IMU    → predict / integrateMeasurement（新バイアス B* で）
            車速   → 層 A ゲート（§5.3.2 / D42）→ 通過したものだけ
                     WheelSpeedFactor と同一の測定モデルで EKF 形式の 1 ステップ更新
                     （+ NonHolonomicFactor 相当の更新。適用条件・20 ms 区間 1 回の制限は通常処理と完全に同一）
            20 ms グリッド点に到達 → μ₁, Σ₁ を StateHistory へ書き戻す

      StateHistory は t_end 以降のみ作り直し、t_end より古いサンプルは保持する（§5.3.4）。

(6) InternalEstimateSnapshot の publish（D37）
      ワーカは **internal 側だけ**を公開する:
        snapshot = { generation++, アンカー(X*,V*,B*,t_end), プリインテグレータ状態,
                     Σ（joint 15×15）, StateHistory への参照 }
        std::atomic_store(&internal_snapshot_, std::make_shared<const InternalEstimateSnapshot>(...))
      ※ smoothing 状態（c, u, t_left）はワーカが触らない。オフセットの作り直しは
        **OutputStage が次の出力ステップで**行う（下記「アンカー更新の検出」）。
      ※ published pose 自体はどこにも保持しない（INV-4）
```

**replay が必要な理由（D15）**: Graph1 の通常動作は「IMU 伝播 + 20 ms ごとの車速 EKF 更新」であり、IMU だけを replay すると

```
補正直前:  t_end ─ IMU ─ 車速更新 ─ IMU ─ 車速更新 ─ IMU ─→ now
補正直後:  t_end ─ IMU ─────────── IMU ─────────── IMU ─→ now
```

となって、**アンカー更新の前後で Graph1 のアルゴリズムそのものが変わってしまう**（車速情報が最大 `interval + 最適化時間` 分だけ失われ、
共分散が不当に膨らむ／速度推定が IMU バイアスに引きずられる）。よって `VehicleTwistBuffer` を持ち、**両バッファを stamp 順に merge して replay する**。

- merge は stamp の昇順。同一 stamp の場合は **IMU → 車速** の順（通常処理と同じ順序）に固定し、replay の決定性を保証する。
- **通常処理と replay は同一の関数を呼ぶ**こと（`Graph1::applyImu()` / `applyWheelSpeed()` を共用し、経路を二重化しない）。試験 18 で等価性を検証する。

**二重計上・情報消失を防ぐ不変条件**

固定ラグ窓 + バッチ再構築では、同じ測定が「境界 prior」と「再投入されたファクタ」の両方に入ると、
黙って共分散を過小評価する（発散しないので気づきにくい）。逆に、一度もファクタにならないまま捨てられると情報が純減する。
次の 5 つを実装不変条件とする:

| # | 不変条件 | 担保手段 |
| --- | --- | --- |
| INV-1 | 窓内の各測定は、境界 prior・測定ファクタの**両方に寄与することは無い**（どちらか一方、または `kIntentionallyExcluded` としてどちらにも寄与しない） | prune を `assigned_node ≤ j*` で行う（(3)）。境界ノードには unary 測定ファクタを張らない（§5.3.2 (d)） |
| INV-2 | 周辺化境界を跨ぐファクタは存在しない | `t_next_begin` を強制ノード化（§5.3.2 (b) / D29）。チェーン型ファクタは必ず境界で分割される。SLAM の対が跨ぐ場合は `j*` で分割（§5.3.3c） |
| INV-3 | 測定の重み（`w`・補正後 noise）は到着時に一度だけ決まり、再構築で変化しない | `AcceptedMeasurement::noise` を凍結（§5.3.2a）。SLAM relative は「サンプル対の成立時」が凍結タイミング（§5.3.3c / §5.3.6） |
| **INV-4** | `published` 側は独立した状態を持たず、`X_published = X_internal · Expmap(c)` として毎ステップ導出される | `PublishedEstimate` を `InternalEstimate` + `c` からのみ構築可能な型にする（独立した pose メンバを持たせない）。試験 21 ⑦ |
| INV-5 | **一度もファクタとして寄与していない測定（`contribution == kNotYetBuilt`）は、(3) の prune で削除されない** | `t_begin` を併合の吸収先から外す（構造的対策）＋ 削除禁止（§5.3.2 (b)(g) / D34） |

**INV-5 の明示的な例外（これ以外で `kNotYetBuilt` を捨ててはならない）**:

1. §5.3.2 (a-4): `stamp < t_begin` の測定（異常遅延・時刻同期異常）。診断 WARN + カウンタを必ず出す。
2. §5.3.9 の「連続 `optimizer_failure_limit` 回失敗による窓の強制再構築」: `stamp ≤ t_anchor` の測定。削除件数を診断 WARN に出す。

いずれも「ここで捨てた測定は本当に失われる」ことを診断で可視化する。

- INV-1 の自動検証: デバッグビルドで、prune 時に「削除する測定の `assigned_node` がすべて `j*` 以下」であること、
  および「history 内に `assigned_node ≤ j*` かつ `contribution != kNotYetBuilt` の要素が残っていない」ことを `assert` する。
- INV-2 の自動検証: グラフ構築後に、全ファクタのキー集合が「すべて `j*` 以下」か「すべて `j*` 以上」のどちらかに分類できることを確認する。
- INV-5 の自動検証: prune 対象に `kNotYetBuilt` が現れたら assert（debug）/ 診断 ERROR（release）。試験 20 ⑥。
- **INV-1/INV-2 が破れたときの症状は「NEES が dof を系統的に上回る（＝共分散が過小＝過信）」**である。
  試験 11・20 で数値的に監視する。INV-5 が破れた場合は逆に情報が減るため NEES は dof を**下回る**（保守側）方向に出るが、
  精度（RMSE）が劣化するので試験 20 ② の参照バッチ解との一致で検出する。

> **NEES の向き（本書共通の定義）**: `NEES = eᵀ P⁻¹ e`（`e` = 真値との誤差、`P` = 申告共分散）で `E[NEES] = dof`。したがって
>
> | 観測 | 意味 |
> | --- | --- |
> | **NEES > dof**（統計的に有意に大きい） | `P` が**小さすぎる** ＝ **過信（overconfident）**。二重計上・相関の不当な破棄・ノイズ過小申告の症状 |
> | **NEES < dof** | `P` が**大きすぎる** ＝ **保守的（conservative）**。情報の取りこぼし・過剰な共分散膨張の症状 |

**Graph1 へ渡す X/V/B 間の相互相関の扱い（D10・D33）**

> **「相関を捨てれば保守側」は誤りである。** ブロック対角化は Loewner 順序で保守性を持たない。反例:
>
> ```
> P = [ 1   0.9 ]      blkdiag(P) = I,   I − P = [ 0   −0.9 ]  → 固有値 ±0.9 で非 PSD
>     [ 0.9  1  ]                                 [ −0.9  0  ]
> ```
>
> しかも本設計で問題になるのはまさにその過小評価側である。IMU 積分後の位置–速度相関 `P_xv` は通常**正**であり、
> Graph1 の伝播 `x' = x + v·Δt` では `Var(x') = P_xx + 2Δt·P_xv + Δt²·P_vv` なので、
> `P_xv > 0` を捨てると `Var(x')` を**系統的に過小評価**する。これは §5.3.6 の層 A/B を不当に厳しくし、
> 正常な GNSS/NDT を棄却する方向に効く。「安全側だから許容」という論拠は成立しない。

- **既定（`reset.carry_cross_covariance: true`）**: `jointMarginalCovariance({X(k*),V(k*),B(k*)})` で 15×15 を取得し、
  Graph1 の伝播でも 15×15 の相関付き共分散を保持する。**これが数学的に正しい引き継ぎであり、近似を入れる理由がない。**
  - コストが問題にならない根拠: Graph1 は IMU プリインテグレーションによる解析的共分散伝播（D9b）を行っており、
    そもそも**内部で 15×15 の同時共分散を持たなければ正しく動かない**。`carry_cross_covariance` が効くのは
    **リセット時の初期化 1 回だけ**であり、追加コストは 500 ms に 1 回の `jointMarginalCovariance`（15×15）のみ。
- **オプション（`reset.carry_cross_covariance: false`）**: LIO-SAM と同一の近似（marginal のみ引き継ぎ）。
  GTSAM の `jointMarginalCovariance` が使えない構成へのフォールバックとしてのみ残す。
  この場合は**保守性が保証されない**ため、次のいずれかで補う:
  - `reset.cross_covariance_inflation: 3.0`（既定・`false` 選択時のみ有効）: `n` 個のブロックに分割したとき
    **`n · blkdiag(P) ⪰ P` は常に成立する**（各ブロックに Cauchy–Schwarz を適用し
    `xᵀPx ≤ (Σᵢ aᵢ)² ≤ n Σᵢ aᵢ²`、`aᵢ = √(xᵢᵀPᵢᵢxᵢ)`）。X/V/B の 3 ブロックなので係数 3 で**保証付きの保守側**になる。
    代償として初期共分散が最大 3 倍に膨らみ、ゲートが緩む。
  - `1.0`（無補正）は、保守性がないことを承知の上でのチューニングとして扱う（起動時に診断 WARN）。
- **Graph2 の窓境界 prior**（(3)）は既定で `linear_container`、すなわち**相互相関を保持する**。
  Graph2 は窓を跨いで情報を伝える経路がここしかなく、相関を捨てると精度劣化が大きい。
  簡略化オプション `graph2.marginalization: prior_triple` も保守性を持たない近似であり、採用する場合は同じ `cross_covariance_inflation` を適用する。
- **検証**: 試験 11 ② で `carry_cross_covariance` の ON/OFF を比較し、
  **`false + inflation 1.0` で NEES が dof を系統的に上回る（＝申告共分散が過小＝過信）**ことを回帰試験として明示的に確認する。
  `inflation 3.0` では逆に NEES が dof を下回る（保守側）ことも併せて確認する。

**内部推定 / 出力推定の分離（D27）**

| 名称 | 定義 | 用途 |
| --- | --- | --- |
| `internal_estimate` | Graph2 のアンカーを**即時適用**して伝播した推定 `(μ₁, Σ₁, v₁, b₁)` | Graph1 の伝播、`StateHistory`、§5.3.6 の層 A/B（d²・NIS・w）、Graph2 の初期値、発散判定、NEES 評価 |
| `published_estimate` | `internal_estimate` に出力側 smoothing を掛けたもの | §3.1 の全出力トピック・TF のみ |

**smoothing の定義（D32）**

出力は常に「アンカー + 前方積分」から作られるため、アンカーが飛ぶと出力も飛ぶ。そこで大きなジャンプは滑らかに馴染ませるが、
**smoothing を推定器内部に持ち込んではならない**。smoothing の対象は**絶対 pose ではなく補正オフセット**である:

```
X_published(t) = X_internal(t) · C(t)          # C は base_link 側（右）に掛ける補正オフセット
C(t) = Expmap( c(t) ) ,  c(t) ∈ R⁶ ,  c → 0    # C → I で smoothing 完了
```

`X_internal(t)` の項がそのまま残るため、**車両本来の運動は常に出力に入る**。smoothing はその上に重畳する補正だけを扱う。
**発動判定はアンカー更新イベント時に行う**（出力ステップ毎に評価すると「前回出力からの車両運動」が混入し、
出力レートを変えると発動条件が変わってしまう）。

**アンカー更新の検出と smoothing 状態の所有（D37・D43）**

smoothing 状態は `OutputStage`（output callback group）の専有である:

```
[OutputStage が保持する状態（他スレッドは触らない）]
  prev_snapshot_   : std::shared_ptr<const InternalEstimateSnapshot>   # 直前ステップで使ったもの
  c, u, t_left     : smoothing 状態
  ※ published pose は保持しない（INV-4）

[毎出力ステップの先頭]
  s = std::atomic_load(&internal_snapshot_)
  if (s->generation != prev_snapshot_->generation) {      # ← アンカー更新の検出
      # 新旧を **同一時刻 t_now** で評価する
      X_internal_old = prev_snapshot_->propagate(t_now)
      X_internal_new = s->propagate(t_now)
      X_pub_hold     = X_internal_old · Expmap(c)         # 更新前に publish していた姿勢
      c ← Log( X_internal_new⁻¹ · X_pub_hold )            # ★ published を動かさずオフセットへ移す
      → 下記の発動判定を行う
      prev_snapshot_ = s
  }
```

- `prev_snapshot_` は `shared_ptr` なので、ワーカが新しいスナップショットへ差し替えても**古い方は生存**している（前方積分に必要）。
- `X_pub_hold` は関数ローカルの一時値であり、状態としては持たない（INV-4 は維持される）。
- この構成なら、アンカー更新から最初の出力までの遅れは最大 1 ステップ（20 ms）で、その間 published pose は
  古いスナップショットから前方積分され続ける（＝連続性が保たれる）。ワーカ側の完了時刻に依存しないので決定性も上がる。

```
[アンカー更新を検出したステップでの発動判定]
  c_rot = c[0:3],  c_lin = c[3:6]

  |c_lin| ≤ max_anchor_jump_m（既定 0.03）かつ |c_rot| ≤ max_anchor_jump_rad（既定 0.003）
      → c ← 0（即時適用。通常はこちら）, IDLE
  それ以外 → SMOOTHING へ遷移し、以下をラッチ（診断 INFO + カウンタ "anchor jump smoothing"）:
      T_apply = clamp( max(|c_lin| / smoothing_max_lin_rate_mps,
                           |c_rot| / smoothing_max_ang_rate_rps),
                       dt, smoothing_max_duration_ms / 1000 )
      u       = −c / T_apply           # ラッチした補正レート [rad/s; m/s]。c を T_apply で 0 へ運ぶ
      t_left  = T_apply
      ※ T_apply が smoothing_max_duration_ms で頭打ちになった場合は |u| がレート上限を超える。
        診断を WARN "anchor jump rate exceeded" に格上げする
        （1 s 以上 pose を古いまま保持するより、速度整合を保ったまま速く吸収する方が下流に安全）
      ※ smoothing 中に次のアンカー更新が来た場合も同じ手順で c を作り直す（published は動かさない）。
        このとき t_left は引き継ぎ、u ← −c / t_left として**方向だけ**を張り直す（合計吸収時間を延ばさない）。
        ただし |c| が発動時の smoothing_relatch_ratio(2.0) 倍を超えた場合、および t_left < dt の場合は
        T_apply から再ラッチする（診断 WARN）。

[毎出力ステップ（dt = 1/output_rate_hz）]
  [IDLE]      X_published = X_internal              # c = 0 と等価
  [SMOOTHING] c ← c + u · dt                        # 絶対 pose ではなくオフセットを減衰させる
              X_published = X_internal · Expmap(c)  # 車両運動は X_internal 経由で必ず入る
              t_left ← t_left − dt
              終了条件: t_left ≤ 0 または（|c_lin| ≤ smoothing_done_m(0.005) かつ
                                           |c_rot| ≤ smoothing_done_rad(0.0005)）
                  → c ← 0, u ← 0, IDLE（残差は完了閾値以下なので不連続は無視できる）
```

- **`max_anchor_jump_*` の既定値の根拠**: これは「これ以下なら smoothing せず即時適用してよい」閾値である。
  50 Hz 出力で `x` m を 1 ステップに載せると見かけ速度は `50x` m/s になるため、大きく取ってはならない。
  既定 **0.03 m / 0.003 rad** は、試験 3 の合否基準（位置ジャンプ < 5 cm、yaw ジャンプ < 5 mrad）に対して
  **余裕を持って下回る**値として選ぶ（等号で接する値にすると境界ケースで試験が原理的に落ちる）。
  見かけ速度は 1.5 m/s であり、`twist2accel` から見ても許容範囲に収まる。
  副作用として smoothing の発動頻度が上がるので、**発動時の診断は INFO + カウンタ**とし、
  WARN は `smoothing_max_duration_ms` を超えてレート上限を破ったときだけに出す（アラート疲れの防止）。
- **`c ← c + u·dt` が Lie 群上で厳密である理由**: ラッチ後の `u` は `−c₀/T_apply` すなわち `c₀` のスカラー倍であり、
  `c` は常に `c₀` と平行なままである。同一方向の tangent ベクトルどうしは可換（`Exp(a)·Exp(b) = Exp(a+b)`）なので、
  `Expmap(c + u·dt) = Expmap(c) · Expmap(u·dt)` が**近似なしで成立**する。実装は `c` をベクトルのまま持てばよく、
  BCH 近似も再 `Log` も不要（アンカー更新イベント時のみ `Log` を 1 回呼ぶ）。
- **出力 twist の整合（既定 `output.twist_smoothing_consistency: true`）**: `X_p = X_i · C` を微分すると、
  body 系の twist（`ξ = (X⁻¹Ẋ)^∨`、GTSAM tangent 順 `[ω; ν]`）は

  ```
  ξ_published = Ad_{C⁻¹} · ξ_internal + u          （厳密式）
              ≈ ξ_internal + u                     （|c| ≪ 1 のとき。Ad_{C⁻¹} = I + O(|c|)）
  ```

  となる（`u ∥ c` により `J_r(c)·u = u` が厳密に成立するため、右辺第 2 項は `u` そのものでよい）。成分で書くと

  ```
  ω_published,b = ω_internal,b + u[0:3]
  v_published,b = v_internal,b + u[3:6]            # ★ 追加の回転変換は不要
  ```

  `u` は `X_published` 基準の**右摂動（body 系）tangent** であり、その並進成分は既に `base_link` 系なので、
  `R(X_published)ᵀ` を掛けるのは**二重回転**であり誤りである。

  厳密式（`Ad_{C⁻¹}` 込み）を使うか近似式を使うかは `output.exact_smoothing_adjoint`（既定 `false`）で選択する。
  smoothing 中の `|c|` は最大で `smoothing_max_duration_ms × smoothing_max_lin_rate_mps ≈ 1 m` に達しうるため、
  `Ad_{C⁻¹}` の効きは並進で `|ω_i|·|c_lin|`（1 rad/s 旋回時に 1 m で 1 m/s）になりうる。
  **旋回中の大ジャンプでは無視できない**ので、試験 21 ⑩ で近似誤差が判定閾値（0.01 m/s）を超える条件が現れた場合は既定を `true` に変える。
  `twist_smoothing_consistency: false` にすると pose のみ smoothing・twist は internal 即時になるが、
  `pose_instability_detector` が smoothing 期間中に反応しうる。既定は `true`。
- **レート既定値の根拠**: `smoothing_max_lin_rate_mps = 1.0` / `smoothing_max_ang_rate_rps = 0.2` は、
  車両の実速度に対して十分小さく（15 m/s 走行時に 6.7 % の速度誤差）、かつ
  **`smoothing_max_duration_ms = 1000` 以内で 1 m / 0.2 rad までのジャンプをレート上限内に吸収できる**値として選ぶ。
  1 m を超えるジャンプは「実質的な再定位」であり、レート上限を破ってでも 1 s で吸収しきる方針を採る（診断 WARN 格上げ）。
  なお `max_anchor_jump_m`（0.03）はこれとは独立の「即時適用の上限」であり、両者を混同しないこと。
- **出力共分散の整合（D40）**: smoothing 中は「公称値 `Σ_internal`」と「実際に publish している姿勢」がずれる。
  この不整合を下流へ正直に伝えるため、出力共分散を次のように膨らませて出す。

  **加算は GTSAM body tangent 空間（順序 `[ω; ν]`）で行い、その後に §5.3.5 の変換を 1 回だけ通して ROS 表現にする。**
  **かつ `Σ_internal` は `X_internal` 基準の右摂動共分散、`c cᵀ` は `X_published` 基準のバイアス項なので、
  足す前に `Σ_internal` を published 基準へ輸送しなければならない。輸送作用素は pose と twist で異なる。**

  ```
  導出（pose）:
    真値 X_true = X̂_internal · Exp(ξ_int),  Cov(ξ_int) = Σ_pose,internal
    出力 X̂_published = X̂_internal · C,      C = Exp(c)
    published 基準の誤差:
        ξ_pub = Log( X̂_published⁻¹ · X_true ) = Log( Exp(−c) · Exp(ξ_int) )
              ≈ −c + J_r⁻¹(−c) · ξ_int                （ξ_int について 1 次）
    ⇒  bias  = −c                       → bias·biasᵀ = c cᵀ
       Cov   = J_r⁻¹(−c) Σ_pose,internal J_r⁻ᵀ(−c)
    ※ J_r は SE(3) の右ヤコビアン。J_r⁻¹(−c) = J_l⁻¹(c)。GTSAM では Pose3::LogmapDerivative(−c)
      （バージョンにより引数が Vector6 か Pose3 か異なるため CMake / 試験 21 ⑪ で確認する）。

  導出（twist）:
    ξ_published = Ad_{C⁻¹} · ξ_internal + u        （§5.3.7 の厳密式・これは exact であって近似ではない）
    ⇒  bias = u  → u uᵀ,   Cov = Ad_{C⁻¹} Σ_twist,internal Ad_{C⁻¹}ᵀ
  ```

  ```
  # --- GTSAM body tangent 空間（[ω; ν]、X_published 基準の右摂動） ---
  J = J_r⁻¹(−c)                                          # = Pose3::LogmapDerivative(−c)、6×6
  M_pose,published  = J · Σ_pose,internal · Jᵀ            + c cᵀ    （c = 未適用の残りオフセット）
  M_twist,published = Ad_{C⁻¹} Σ_twist,internal Ad_{C⁻¹}ᵀ + u uᵀ    （u = 現在の補正レート）

  # --- ROS 表現へ（§5.3.5・R = R(X_published)） ---
  P_pose,ros  = Πᵀ · blkdiag(R,R) · M_pose,published · blkdiag(Rᵀ,Rᵀ) · Π
  P_twist,ros = Πᵀ · M_twist,published · Π              （twist は base_link 系のため回転は不要。
                                                          逆順序入替 Πᵀ だけを掛ける）
  ```

  > **作用素を取り違えないこと**: `Ad_{C⁻¹}` は「同一時刻の tangent ベクトル（速度・力・微小変位）を別の基準へ写す」作用素であり、
  > 群要素の合成 `Exp(−c)·Exp(ξ)` から生じる 1 次項は `J_r⁻¹(−c)` になる。両者はどちらも `I + O(|c|)` だが
  > 互いに `O(|c|)` だけ異なる（`J_r⁻¹(−c) = J_l⁻¹(c) ≈ I − ½ad_c`、`Ad_{C⁻¹} ≈ I − ad_c`）。
  > **pose 共分散 = `J_r⁻¹(−c)`、twist 共分散 = `Ad_{C⁻¹}`** が正しい対応である。
  >
  > **効き方の大きさ**: 支配項 `c cᵀ`（`|c| ≤ smoothing_max_duration × rate ≈ 1 m`）に対して、輸送の効果は
  > `|c| × σ_rot` のオーダー（1 m × 0.01 rad = 0.01 m）で、`Σ_internal` の位置 σ（0.1 m 級）に対しては 10 % 程度になりうる。
  > 6×6 の行列積 2 回を 50 Hz で行うだけなので、**近似せず常に適用する**（切替パラメータは設けない。
  > `exact_smoothing_adjoint` が効くのは twist の**平均値**の式のみ）。`c = 0` のとき `J = Ad = I` で恒等式に戻る。

  > **禁止事項**:
  > 1. `Σ_pose,internal` を先に ROS 表現へ変換してから、`[ω;ν]` 順のままの `c cᵀ` を足すこと。順序も座標系も一致しない。
  >    加算は必ず変換の**前**に行う（試験 21 ④）。
  > 2. 輸送（`J_r⁻¹(−c)` / `Ad_{C⁻¹}`）を省いて `Σ_internal` に直接 `c cᵀ` を足すこと。また pose 側に `Ad_{C⁻¹}` を使うこと。
  >    試験 21 ⑪⑫ で `numericalDerivative` / モンテカルロと照合する。

  > **確率的意味**: `c cᵀ` / `u uᵀ` はランク 1 であり、統計的に導出された共分散ではない。
  > これは「published 推定の平均が internal からわざと `c` だけずれている」という**既知バイアスの二乗**であり、
  > 加算後の行列は共分散ではなく **MSE（平均二乗誤差）行列** `E[(x̂−x)(x̂−x)ᵀ] = Cov + bias·biasᵀ` である。
  > 下流（`ndt_scan_matcher` の初期姿勢、`pose_instability_detector`）はいずれも「推定値からの実際のずれの大きさ」を見るので、
  > 共分散そのものより MSE を渡す方が正しい。ただし **NEES / NIS 評価にこの膨張後の行列を使ってはならない**
  > （評価は `internal_estimate` の `Σ_internal` で行う。試験 11・21）。
  > smoothing 完了時（`c → 0`, `u → 0`）に両者とも `Σ_internal` へ連続的に戻る。

- **NIS / ゲート / NEES 評価には必ず `internal_estimate` を使う**。`published_estimate` を使うと、
  smoothing 中の意図的なバイアスをセンサ異常と誤判定して層 A/B が誤作動する。
- `initialpose` 受信時と初期化直後は smoothing せず**即時適用**する（`c ← 0`）。
- smoothing の残量 `|c|` と補正レート `|u|` を `debug/covariance_correction` 群に出力し、常時 0 でないなら Graph2 のアンカー更新が荒れている兆候として監視する。

#### 5.3.8 初期化シーケンス（D16・D26・D41）

```
        ┌──────────────┐
        │ UNINITIALIZED│  出力なし（診断 ERROR "not initialized"）
        └──────┬───────┘
               │ initial pose 取得（下記いずれか）
               ▼
        ┌──────────────┐
        │  BIAS_INIT   │  静止判定中は IMU 平均を取る（最大 bias_init_timeout_s = 2.0）
        └──────┬───────┘  車両が動いた／タイムアウト → 既定バイアスで先へ
               │
               ▼
        ┌──────────────┐
        │   RUNNING    │  通常推定。出力あり
        └──────┬───────┘
               │ 発散検知 / trigger_node(false) / initialpose 再受信
               ▼
        （UNINITIALIZED へ戻る、または RUNNING のままアンカー再設定）
```

**初期値の決定規則**:

| 量 | 取得元 | 既定共分散 |
| --- | --- | --- |
| `X0` | `initialization.pose_source` = `initialpose`（既定, `/initialpose3d`）／ `ndt`（最初の NDT 姿勢）／ `gnss`（GNSS 位置 + 重力から roll/pitch + **下記の yaw 取得元**） | 入力の共分散 × `initialization.pose_cov_scale`（既定 4.0）。入力が無効なら `init_pose_stddev`（位置 1.0 m / 姿勢 0.1 rad） |
| `V0` | **0 と仮定**。ただし車速が届いていれば `V0 = R(X0) · [z_v, 0, 0]ᵀ` | `init_vel_stddev`（既定 0.5 m/s） |
| `B0` | 静止判定（`\|z_v\| < 0.05 m/s` かつ `\|ω\| < 0.01 rad/s` が `bias_init_static_s` = 1.0 秒継続）なら IMU 平均から**下記の式で**算出。非静止なら `imu.initial_bias`（既定 0） | `init_accel_bias_stddev` 0.1 m/s², `init_gyro_bias_stddev` 0.01 rad/s |
| 重力方向 | 静止時: 加速度平均から roll/pitch を推定して `X0` の姿勢を補正（下記「可分性」の規則に従う）。非静止時: `X0` の roll/pitch をそのまま使う | — |

**`pose_source: gnss` のときの yaw 取得元（D41）**

静止 IMU から得られるのは重力方向、すなわち **roll/pitch だけ**である。yaw（方位）は重力と無関係なので決まらない。
しかも yaw が未知のまま起動すると、位置しか観測しない GNSS では yaw を直接拘束できず、
その間 IMU バイアスと yaw が縺れて収束が極端に遅くなる。よって yaw の出所を明示的に規定する。

`initialization.gnss_yaw_source`（既定 `auto`）で規定する。`auto` は上から順に試し、最初に成立したものを使う:

| # | 取得元 | 成立条件 | 初期 yaw 標準偏差 |
| --- | --- | --- | --- |
| 1 | **dual-antenna heading** | GNSS 入力が姿勢付き（`orientation` が単位クォータニオンでなく、yaw 分散が `gnss_yaw_max_stddev_rad`（既定 0.2）未満） | メッセージの yaw 分散 × `pose_cov_scale` |
| 2 | **course over ground（走行方向）** | 車速 `z_v ≥ gnss_yaw_min_speed_mps`（既定 2.0）が `gnss_yaw_min_duration_s`（既定 1.0）継続し、その間の GNSS 位置差 `Δp` が `\|Δp\| ≥ 2.0 m`。yaw = `atan2(Δp_y, Δp_x)`（後退中は不可なので車速の符号で判定） | `atan2` の伝播誤差 `σ_pos·√2 / \|Δp\|`（下限 0.05 rad） |
| 3 | **`initialpose` の yaw** | `/initialpose3d` を受信済み（位置は GNSS、yaw だけ流用） | 入力の yaw 分散 × `pose_cov_scale` |
| 4 | **拒否**（既定の最終手段） | 上記いずれも不成立 | **RUNNING へ遷移せず `UNINITIALIZED` を維持**し、診断 ERROR `"gnss init: yaw source unavailable"` を出し続ける |

- **`yaw = 0` + 大分散という fallback は既定にしない**。理由: (a) tangent 空間での線形化は yaw 誤差が数十度を超えると破綻し、
  GNSS 位置ファクタのヤコビアンが意味を失う、(b) 誤った yaw のまま出力すると下流（planning / ndt_scan_matcher の初期姿勢）へ
  「もっともらしいが完全に誤った姿勢」を渡すことになり、沈黙して壊れるより危険。
  試験・シミュレーション用途に限り `gnss_yaw_source: zero_with_large_covariance`（yaw 標準偏差 `gnss_yaw_fallback_stddev_rad`、既定 1.0 rad、
  §6.2 で `≤ 1.5 rad` に制限）を明示指定できるようにするが、**既定ではない**し、選択時は起動時に診断 WARN を出す。
- 2（course over ground）を採用した場合、初期化完了までは静止状態から動き出す必要がある。この待ち時間は
  `bias_init_timeout_s` とは別に `gnss_yaw_timeout_s`（既定 30.0）で監視し、超過したら診断 ERROR に格上げする。
- どの経路でも roll/pitch は重力から（静止時）または `X0` から取り、`b_a` との可分性は下表の `gnss` 行に従う。

**初期 IMU バイアスの算出式（D26）**

静止中も重力を観測するため、加速度計バイアスは `b_a = mean(accel)` **ではない**。GTSAM の specific force 規約に合わせて確定する。

```
規約（§5.3.1 と整合）:
  world = map（ENU, Z 上向き）、gravity vector  g_w = (0, 0, −g)ᵀ,  g = imu.gravity = 9.80665
  GTSAM PreintegrationParams::MakeSharedU(g) は n_gravity = (0,0,−g) を設定する
  IMU が出す specific force:  f = R_wb^T · (a_w − g_w) + b_a + n
     → 静止（a_w = 0）のとき  f = −R_wb^T · g_w + b_a = R_wb^T · (0,0,g)ᵀ + b_a
       （水平に置いた IMU は +Z に約 +9.8 m/s² を出力する。実センサの挙動と一致）

したがって、静止区間の平均を ā_meas, ω̄_meas（いずれも §5.3.1 で base_link へ回転済み）として:

  b_g = ω̄_meas                                （地球自転 15 °/h は無視。車載グレードでは検出限界以下）
  b_a = ā_meas − R_wb^T · (0, 0, g)ᵀ           （R_wb = R(X0)、map ← base_link の回転）
```

- 符号規約を反転させた実装（`f = R^T(a_w + g_w)` 等）と混ざると重力 2 倍の誤差（≈ 19.6 m/s²）になるため、
  **`MakeSharedU` を使うこと**と上式の組を単体試験で固定する（試験 19）。
- **roll/pitch と `b_a` の可分性**: 静止 IMU 1 台からは「傾き」と「加速度計バイアス」を分離できない（同じ観測を説明する）。
  よって次の規則で一方を既知として扱う:

  | 条件 | 扱い |
  | --- | --- |
  | `pose_source` = `initialpose` / `ndt`（roll/pitch が与えられる） | **姿勢を信頼**し、上式で `b_a` を算出する |
  | `pose_source` = `gnss`（roll/pitch が無い） | **`b_a = imu.initial_bias`（既定 0）と仮定**し、`ā_meas` の向きから roll/pitch を推定して `X0` を補正する |
  | 静止判定が成立しない | `b_a`・`b_g` とも `imu.initial_bias`、roll/pitch は `X0` のまま。共分散は `init_*_bias_stddev` の 2 倍に膨らませる |

  いずれの場合も `b_a` は状態 `B(k)` として走行中に推定され続けるため、初期値の誤差は絶対入力が入り次第収束する。

- `X0` の共分散を入力の 4 倍に膨らませるのは、初期姿勢が誤っていても絶対入力で回復できるようにするため。
- **必要センサが揃うまで**: IMU が `initialization.required_imu_samples`（既定 20）届くまで RUNNING に入らない。
- **これらは「必須デフォルト」**であり、§9.2 の「チューニング項目」とは分離する。
  実装完了条件は「パラメータ未指定でも既定値だけで起動・収束すること」。

#### 5.3.9 異常系・妥当性チェック（D18・D35）

**入力共分散の妥当性**:

```
1. NaN / Inf を含む            → drop + 診断 WARN
2. 全要素 0                    → 「未提供」とみなし fallback_covariance を使用（診断 INFO、レート制限）
3. 対角に負値                  → drop + 診断 WARN
4. IMU covariance[0] == -1     → ROS 規約「未提供」。パラメータのノイズ値を使用
5. 非対称                      → (P + Pᵀ)/2 で対称化
6. 非 PSD                      → 固有値分解し負固有値を eps(1e-12) にクリップ。クリップしたら診断 WARN
```

出力直前にも同じ 5・6 を適用し、**§3.3 の「NaN/ゼロ共分散を出さない」を保証**する。

**センサ dropout / timeout**:

| 入力 | timeout パラメータ（既定） | 超過時の挙動 | 診断 |
| --- | --- | --- | --- |
| IMU | `failure.imu_timeout_ms`（100 ms） | 前方積分停止。最後の推定を保持して出力（外挿は `max_extrapolation_ms` まで） | 200 ms で WARN、1 s で ERROR |
| 車速 | `wheel_speed.timeout_ms`（200 ms） | 車速ファクタ／EKF 更新を行わない（IMU のみで継続） | 500 ms で WARN |
| NDT | `ndt.timeout_ms`（1000 ms） | Graph2 から NDT ファクタが消えるだけ。DR 継続 | 1 s で WARN、3 s で ERROR |
| GNSS | `gnss.timeout_ms`（2000 ms） | 同上（GNSS は元々間欠） | 5 s で WARN のみ |
| SLAM | `slam.timeout_ms`（1000 ms） | 同上 | 3 s で WARN |

- 全絶対入力（NDT / GNSS / SLAM）が同時に timeout → `dead_reckoning_only` 状態。診断 ERROR、共分散が単調増大することを確認できるようにする。
- 復帰条件: 対象トピックを 1 回受信し、かつ層 A のゲートを通過したら通常復帰。
  ただし dropout 継続時間が `long_dropout_s`（既定 5.0）を超えていた場合は、復帰初回のゲート閾値を
  `recovery_gate_scale`（既定 10.0）倍に緩めて再捕捉できるようにする。

**GTSAM 最適化失敗**:

```
try { values = optimizer.optimize(); }   // §5.3.2 (f)。graph2.optimizer が isam2 でも同じ扱い
catch (const gtsam::IndeterminantLinearSystemException & e) {
   → 今回の tick の結果は破棄。前アンカーを維持（出力は DR で継続）
   → 境界 prior は **前 tick の状態のまま**（§5.3.2 (g) を実行しない ＝ 半端に更新しない）
   → **t_begin も更新しない**（D35）。t_begin ≡ 境界 prior の時刻なので、失敗中は窓が右へ伸びるだけになる:
        窓長 = graph2_lag_ms + 連続失敗回数 × graph2_optimize_interval_ms
      これにより「prior は古いのに t_begin だけ進み、未吸収の測定が (a-4) で捨てられる」事故を防ぐ。
      §5.3.4 の callback 側 drop 判定が読む t_begin も同じ値（更新されない）なので、両者は常に整合する。
      窓が伸びる分の IMU / 車速サンプルが必要になるため、バッファ長の下限に
      optimizer_failure_limit × graph2_optimize_interval_ms を含める（§6.2）
   → **Graph2MeasurementHistory は prune しない**。drain 済みの測定は既に history に入っており、
      次 tick でそのまま再投入される（その結果 stamp < t_begin になったものはその時点で drop・(a-4)）
   → 診断 ERROR + 失敗カウンタ++
   → 連続 optimizer_failure_limit(3) 回失敗:
        窓を破棄し、「前アンカー（時刻 t_anchor）+ 膨らませた共分散（failure_cov_inflation = 10.0 倍）」を
        唯一の境界 prior として Graph2 を強制再構築（窓長は以後の tick で回復）。
        **t_begin ← t_anchor**（新しい prior の時刻）。
        このとき **history から stamp ≤ t_anchor の測定を削除する**（前アンカーに既に反映済みのため・INV-1。
        contribution に関わらず削除する INV-5 の明示的例外。削除件数を診断 WARN に出す）
   → さらに optimizer_failure_reinit_limit(10) 回失敗:
        UNINITIALIZED へ戻す（§5.3.8 から再初期化）
}
catch (const std::exception & e) { 同上（型別にログ） }
```

- 発散検知（LIO-SAM `failureDetection()` 相当）: `|V*| > max_velocity_mps`（既定 40）、
  `|b_a| > max_accel_bias`（既定 1.0 m/s²）、`|b_g| > max_gyro_bias`（既定 0.5 rad/s）で同じ再構築フローに入る。

---

## 6. パラメータ設計

### 6.1 パラメータファイル（抜粋）

```yaml
/**:
  ros__parameters:
    fusion_method: factor_graph_fusion   # ekf | lio_sam_imu_preintegration | factor_graph_fusion（起動時のみ / read_only）
    output_rate_hz: 50.0
    tf_rate_hz: 50.0
    enable_tf: true
    pose_frame_id: map
    base_frame_id: base_link

    output:
      flatten_to_2d: false             # true で z/roll/pitch を Simple1DFilter 由来に置換（ekf モードは常に true 扱い）
      max_extrapolation_ms: 100
      twist_smoothing_consistency: true # smoothing 中も d(pose)/dt と twist を整合させる（§5.3.7 / D31）
      exact_smoothing_adjoint: false    # true で ξ_pub = Ad_{C⁻¹}ξ_int + u の厳密式を使う（twist の平均値のみ）
      undetermined_variance: 1.0e4

    covariance:
      use_exact_jacobian: false        # §5.3.5 の一次近似 J_l ≈ I を厳密ヤコビアンに切り替える

    qos:
      imu_depth: 100
      twist_depth: 100
      pose_depth: 10
      output_depth: 1

    # --- factor_graph_fusion 固有 ---
    factor_graph_fusion:
      graph1_node_interval_ms: 20
      graph2_max_node_interval_ms: 100
      graph2_optimize_interval_ms: 500
      graph2_lag_ms: 1000              # 固定ラグ窓長（§5.3.2）
      node_merge_tolerance_ms: 5
      graph2_max_nodes: 80

      graph2:
        optimizer: levenberg_marquardt # levenberg_marquardt（既定・バッチ） | isam2（毎 tick 再構築）
        marginalization: linear_container  # linear_container（既定・相関保持） | prior_triple（近似）
        lag_margin_ms: 100             # ラグ窓の余裕（§6.2 の検証で使用）
        measurement_history_max: 512   # Graph2MeasurementHistory の上限（§5.3.2a）
        # t_next_begin の強制ノード化は INV-2 の成立条件であり、設定で無効化できない（パラメータを持たない）
        boundary_merge_velocity_mps: 20.0  # t_begin 近傍の測定をずらした際の位置ノイズ加算の上界（§5.3.2 (b)）
        boundary_merge_yaw_rate_rps: 1.0   # 同・姿勢ノイズ加算の上界（§5.3.2 (b) / D44）

      buffer:                          # replay 用バッファ（§5.3.7 (5)）
        # 最適化失敗中は t_begin を固定するため窓が最大 optimizer_failure_limit(3) × interval(500 ms) 伸びる
        imu_buffer_ms: 3500
        vehicle_twist_buffer_ms: 3500

      timing:
        history_length_ms: 1500        # StateHistory の長さ（max_delay + optimize_interval + margin）
        max_measurement_delay_ms: 300
        future_tolerance_ms: 20
        exact_interpolation: false     # true で μ₁,Σ₁ を IMU 部分積分で厳密生成（§5.3.4）

      reset:
        carry_cross_covariance: true   # true: jointMarginalCovariance で 15×15 を厳密に引き継ぐ（既定・D10）
        cross_covariance_inflation: 3.0  # carry_cross_covariance:false のときのみ有効。
                                         #   3.0 = ブロック数 n=3 の Cauchy–Schwarz 境界（保証付き保守側）。
                                         #   1.0 は無補正（保守性なし・起動時 WARN）
        # 「これ以下のジャンプは smoothing せず即時適用」の閾値。
        # 50 Hz 出力で x m を 1 ステップに載せると見かけ 50x m/s になるため小さく取る。
        # 試験 3 の合否基準（5 cm / 5 mrad）を余裕をもって下回る値（§5.3.7 / D45）
        max_anchor_jump_m: 0.03
        max_anchor_jump_rad: 0.003
        smoothing_max_lin_rate_mps: 1.0
        smoothing_max_ang_rate_rps: 0.2
        smoothing_max_duration_ms: 1000  # 吸収に掛ける時間の上限。超過分はレート上限を破って吸収（WARN 格上げ）
        smoothing_done_m: 0.005          # smoothing 完了とみなす残差 |c_lin|（これ以下で c ← 0）
        smoothing_done_rad: 0.0005
        smoothing_relatch_ratio: 2.0     # smoothing 中に |c| が発動時の何倍を超えたら T_apply から再ラッチするか

      covariance_correction:
        enable_gate: true                # 層 A
        enable_weighting: true           # 層 B
        weight_function: exp_excess_nis  # exp_excess_nis | inverse_nis | inverse_linear
        w_min: 0.05
        gate_chi2:                       # dof ごと（p = 0.999）
          dim1: 10.83                    # 車速（層 A のみ・Graph1 内・D42）
          dim3: 16.27                    # GNSS 位置
          dim6: 22.46                    # NDT / SLAM
        low_speed_relax_mps: 0.5
        low_speed_min_pos_stddev: 0.10
        low_speed_min_rot_stddev: 0.01
        recovery_gate_scale: 10.0
      robust_kernel: none                # 層 C: none | huber | cauchy | dcs | gm（既定 OFF）
      robust_kernel_param: 1.345

      imu:
        frame_id: imu_link
        use_tf_extrinsic: true
        extrinsic_fallback: {x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}
        lever_arm_compensation: centrifugal   # rotation_only | centrifugal | full
        euler_term_sigma: 0.02
        factor_type: imu_factor          # imu_factor（推奨/LIO-SAM 準拠） | combined_imu_factor
        gravity: 9.80665
        use_orientation: false           # sensor_msgs/Imu.orientation は使わない
        # 実装必須デフォルト（チューニング項目ではない）
        accel_noise_stddev: 3.9e-3       # m/s²/√Hz  (LIO-SAM 既定に準拠)
        gyro_noise_stddev: 1.5e-3        # rad/s/√Hz
        accel_bias_rw_stddev: 6.4e-4     # m/s³/√Hz
        gyro_bias_rw_stddev: 3.5e-5      # rad/s²/√Hz
        integration_cov: 1.0e-8
        initial_bias: {ax: 0.0, ay: 0.0, az: 0.0, gx: 0.0, gy: 0.0, gz: 0.0}

      wheel_speed:
        use: true
        topic: /sensing/vehicle_velocity_converter/twist_with_covariance
        fallback_stddev: 0.2
        timeout_ms: 200                  # §5.3.9 のセンサ timeout（D44）
        graph2_binding: nearest          # nearest（既定） | interpolate | none（§5.3.3a'）
        graph2_max_time_offset_ms: 30    # ノード時刻とのずれ許容
        graph2_time_offset_accel: 3.0    # 時刻ずれをノイズへ換算する加速度 [m/s²]
        use_nonholonomic: true
        nonholonomic_lat_stddev: 0.1
        nonholonomic_vert_stddev: 0.05
        nonholonomic_min_speed_mps: 0.5

      gnss:
        use: true
        topic: /sensing/gnss/pose_with_covariance
        frame_id: gnss_link
        use_tf_extrinsic: true
        lever_arm_fallback: {x: 0.0, y: 0.0, z: 0.0}   # TF 取得不可時の r_bg（§5.3.3b）
        fallback_stddev: {x: 1.0, y: 1.0, z: 3.0}
        timeout_ms: 2000

      ndt:
        use: true
        topic: /localization/pose_estimator/pose_with_covariance
        fallback_stddev: {pos: 0.2, rot: 0.02}
        timeout_ms: 1000

      slam:                              # ★ 要求範囲外の拡張入力。要求どおりの構成では use: false
        use: true
        topic: lio_sam/mapping/odometry
        use_as: relative                 # relative（推奨, BetweenFactor） | absolute（PriorFactor）
        frame_id: map
        covariance_is_relative: false    # true なら SLAM が出す相対共分散を優先（最も正しい・§5.3.3c）
        relative_covariance_scale: 1.0   # 独立仮定の近似に対するチューニングノブ（保守性の保証ではない）
        # ジャンプ検出の主判定は推測航法残差の χ² 検定（gate_chi2.dim6）。
        # 以下は保険の動力学的上界（固定距離閾値は使わない・§5.3.3c / D38）
        max_speed_mps: 20.0
        max_yaw_rate_rps: 1.0
        jump_margin_m: 0.30
        jump_margin_rad: 0.05
        interp_sigma_per_s: {pos: 0.05, rot: 0.01}  # 周辺化境界で対を分割する際の補間誤差（§5.3.3c）
        covariance_scale: 3.0            # 二重計上対策の意図的な弱め（§5.3.3c）
        timeout_ms: 1000

      initialization:
        pose_source: initialpose         # initialpose | ndt | gnss
        gnss_yaw_source: auto            # auto（dual_antenna → course → initialpose → 拒否）
                                         #  | dual_antenna | course | initialpose
                                         #  | zero_with_large_covariance（試験用・既定にしない）
        gnss_yaw_max_stddev_rad: 0.2     # dual-antenna heading を信頼する上限
        gnss_yaw_min_speed_mps: 2.0      # course over ground の成立条件
        gnss_yaw_min_duration_s: 1.0
        gnss_yaw_timeout_s: 30.0         # yaw 取得待ちの上限（超過で診断 ERROR 格上げ）
        gnss_yaw_fallback_stddev_rad: 1.0  # zero_with_large_covariance 選択時のみ使用（§6.2 で ≤ 1.5 に制限）
        pose_cov_scale: 4.0
        init_pose_stddev: {pos: 1.0, rot: 0.1}
        init_vel_stddev: 0.5
        init_accel_bias_stddev: 0.1
        init_gyro_bias_stddev: 0.01
        required_imu_samples: 20
        bias_init_static_s: 1.0
        bias_init_timeout_s: 2.0

      failure:
        max_velocity_mps: 40.0
        max_accel_bias: 1.0
        max_gyro_bias: 0.5
        optimizer_failure_limit: 3
        optimizer_failure_reinit_limit: 10
        failure_cov_inflation: 10.0
        imu_timeout_ms: 100
        long_dropout_s: 5.0

    # --- lio_sam_imu_preintegration 固有 ---
    lio_sam:
      imu_topic: /sensing/imu/imu_data
      lidar_odometry_topic: lio_sam/mapping/odometry
      covariance_source: propagated      # propagated（joint 15×15） | fixed
      fallback_pose_stddev: {pos: 0.3, rot: 0.03}
      fallback_twist_stddev: {lin: 0.3, ang: 0.03}
      # 以降は元実装の *.param.yaml を踏襲

    # --- ekf 固有 ---
    ekf:
      # autoware_ekf_localizer の ekf_localizer.param.yaml をそのまま踏襲
      input_twist_topic: /localization/twist_estimator/twist_with_covariance
```

JSON schema（`schema/pose_twist_fusion.schema.json`）を ekf に倣って用意し、範囲・enum を機械的に検証する。

### 6.2 パラメータ検証

起動時に以下を検証し、違反時は**例外を投げてノードを起動失敗**させる（黙って壊れた状態で動かさない）。
「WARN」と明記した行のみ起動を許可する。

| 検証 | 条件 |
| --- | --- |
| 周期の整合 | `0 < graph1_node_interval_ms < graph2_max_node_interval_ms ≤ graph2_optimize_interval_ms` |
| **ラグ窓** | `graph2_lag_ms ≥ graph2_optimize_interval_ms + max_measurement_delay_ms + graph2.lag_margin_ms`（既定 1000 ≥ 500+300+100） |
| **履歴長** | `history_length_ms ≥ max_measurement_delay_ms + graph2_optimize_interval_ms + graph1_node_interval_ms × 5`（既定 1500 ≥ 900） |
| **バッファ長** | `imu_buffer_ms` と `vehicle_twist_buffer_ms` がともに `graph2_lag_ms + graph2_optimize_interval_ms × (1 + optimizer_failure_limit) + 500` 以上（既定 `1000 + 500×4 + 500 = 3500`）。最適化失敗中は `t_begin` を固定するため窓が伸びる分を含む（§5.3.9 / D35） |
| 遅延許容 | `0 < max_measurement_delay_ms < history_length_ms` |
| 重み下限 | `0 < w_min ≤ 1` |
| 出力レート | `output_rate_hz > 0` かつ `1000/output_rate_hz ≥ graph1_node_interval_ms`（出力がグラフより速くならない） |
| ノイズ | すべての `*_stddev` / `*_cov` が `> 0`（有限） |
| 閾値 | すべての `gate_chi2.*` が `> 0` |
| enum | `fusion_method`, `weight_function`, `use_as`, `lever_arm_compensation`, `robust_kernel`, `covariance_source`, `pose_source`, `gnss_yaw_source`, `graph2.optimizer`, `graph2.marginalization`, `wheel_speed.graph2_binding` |
| ノード上限 | `graph2_max_nodes ≥ graph2_lag_ms / graph2_max_node_interval_ms + 10`（強制ノードだけで上限に達しない） |
| **境界ノード** | `0 < graph2_optimize_interval_ms < graph2_lag_ms`（`t_begin < t_next_begin < t_end` を保証。等号では境界ノードが窓端と縮退する） |
| **境界併合ノイズ** | `graph2.boundary_merge_velocity_mps > 0` かつ `graph2.boundary_merge_yaw_rate_rps > 0`（§5.3.2 (b)） |
| **車速結合** | `graph2_binding = nearest/interpolate` のとき `graph2_max_time_offset_ms ≥ 1000 / 車速レート想定値 / 2`（既定 30 ≥ 10）。かつ `graph2_max_time_offset_ms ≤ graph2_max_node_interval_ms / 2`（隣接ノードへの取り違えを防ぐ） |
| **車速 timeout** | `wheel_speed.timeout_ms > 1000 / 車速レート想定値`（既定 200 > 20） |
| **history 上限** | `graph2.measurement_history_max ≥ (GNSS+NDT+SLAM の想定合計レート) × graph2_lag_ms/1000 × 4`（既定 512 は 30 Hz・1 s に対して十分） |
| **smoothing** | `smoothing_max_lin_rate_mps > 0`、`smoothing_max_ang_rate_rps > 0`、`smoothing_max_duration_ms ≥ 1000 / output_rate_hz`（最低 1 ステップは掛かる）、`0 < smoothing_done_m < max_anchor_jump_m`、`0 < smoothing_done_rad < max_anchor_jump_rad`、`smoothing_relatch_ratio > 1` |
| **相互相関** | `carry_cross_covariance: false` のとき `cross_covariance_inflation ≥ 1.0`。`1.0 ≤ x < 3.0` を指定した場合は「保守性が保証されない設定」として**起動時に診断 WARN**（起動は許可） |
| **smoothing 閾値** | `max_anchor_jump_m < 0.05`（試験 3 の許容ジャンプ）かつ `max_anchor_jump_rad < 0.005`。**等号を許さない**（等号だと試験 3 が境界ケースで原理的に落ちる・D45）。これを満たさない値は**起動失敗** |
| **SLAM ジャンプ検出** | `slam.max_speed_mps > 0`、`slam.max_yaw_rate_rps > 0`、`jump_margin_m > 0`、`jump_margin_rad > 0` |
| **廃止パラメータ** | `slam.max_relative_jump_m` / `slam.max_relative_jump_rad` / `graph2.force_boundary_node` が指定されていたら**起動失敗**（黙殺を避ける。メッセージで代替と理由を説明する） |
| **GNSS 初期化 yaw** | `pose_source = gnss` のとき `gnss_yaw_source` が enum のいずれか。`zero_with_large_covariance` のときのみ `0 < gnss_yaw_fallback_stddev_rad ≤ 1.5` かつ**起動時に診断 WARN**。`gnss_yaw_min_speed_mps > 0`、`gnss_yaw_max_stddev_rad > 0`（§5.3.8） |
| 三層警告 | `robust_kernel != none` かつ `enable_gate && enable_weighting` → 診断 WARN（起動は許可） |
| **モード不整合** | `fusion_method: ekf` かつ `output.flatten_to_2d: false` → 診断 INFO（`true` として扱う旨を通知。起動は許可） |

---

## 7. 起動（launch）設計

`launch/pose_twist_fusion.launch.xml` を新設し、`pose_twist_fusion_filter.launch.xml` の `ekf_localizer` include を本ノードへ差し替え可能にする。
`fusion_method` を launch 引数で受け、既存の入出力トピック配線（§3.1）を維持する。既存 launch との差分は最小（include 先の 1 行置換 + param ファイル指定）。

**リマップ規約（既存 `pose_twist_fusion_filter.launch.xml` と同一にする）**:

```xml
<arg name="output_pose_with_covariance_name" value="/localization/pose_with_covariance"/>  <!-- 絶対名を維持（§2.1） -->
<arg name="output_odom_name"                 value="kinematic_state"/>
<arg name="output_biased_pose_with_covariance_name" value="biased_pose_with_covariance"/>
<!-- 新規 -->
<arg name="output_ns_pose_with_covariance_name"     value="pose_with_covariance"/>
<arg name="output_pose_with_covariance_no_yawbias_name" value="pose_with_covariance_no_yawbias"/>
```

**TF authority**: `map → base_link` を publish するノードは**システム内で本ノードのみ**とする。

- `enable_tf: true`（既定）で本ノードが publish。
- 他の localization ノード（`ndt_scan_matcher` 等）は `map → base_link` を出さない構成であることを launch のコメントで明記し、
  起動時に `tf` に同一親子の別 publisher がいないかを 1 度だけチェックして、いれば診断 WARN を出す。
- 複数 fusion ノードを比較目的で同時起動する場合は、比較側を `enable_tf: false` にする運用とする。

---

## 8. 設計判断の記録（Design Decisions）

| ID | 決定 | 根拠 / 参照 |
| --- | --- | --- |
| **D1** | `pose_with_covariance` = ヨーバイアス除去後の**真値**、`biased_pose_with_covariance` = **ヨーバイアス込み**、`pose_with_covariance_no_yawbias` = 真値の**エイリアス**。バイアス込み姿勢を `no_yawbias` に割り当てることは**禁止** | 現行命名規約維持。§3.2 のマッピング表が唯一の正 |
| **D2** | pluginlib による戦略パターン | 単一ノード分岐は肥大化・結合度大。3 ノード別実行は出力配線・診断・initialpose 処理が重複。§4.1 |
| **D3** | LIO-SAM モードは `imuPreintegration` 相当のみ担い、他部は外部起動。LiDAR オドメトリを直接入力、車速拘束なし | 元アルゴリズム忠実移植（R3）。§5.2 |
| **D4** | グラフの区切りでは周辺化共分散を新 prior として引き継ぐ（`lio_sam` モードは全リセット、`factor_graph_fusion` モードは固定ラグ窓の境界周辺化） | LIO-SAM `imuPreintegration.cpp:354-380`。§5.3.7 / D23 |
| **D5** | 必須 3 トピック以外も発行（`twist_with_covariance` 等） | 下流 `stop_filter` / `ndt_scan_matcher` が必要 |
| **D6** | IMU ファクタは `ImuFactor` + `BetweenFactor<ConstantBias>`（`CombinedImuFactor` はオプション） | `imuPreintegration.cpp:405` の実装に準拠 |
| **D7** | SLAM（LiDAR オドメトリ）を Graph2 入力に追加。既定は相対拘束 `BetweenFactor<Pose3>` | 要求範囲外の拡張（合意済み）。`slam.use: false` で要求どおりの構成に戻せる。§5.3.3c |
| **D8** | 互換トピック・TF・診断をすべて発行し完全ドロップイン互換 | §3.1 |
| **D9** | **Graph1 = 20 ms 固定グリッド、Graph2 = 測定時刻トリガ**（5 ms 集約 / 100 ms 強制 / 500 ms tick）。ノードは固定ラグ窓内で毎 tick バッチ再構築 | 固定グリッドへのスナップは 15 m/s で最大 15 cm の量子化誤差。§5.3.2 |
| **D9b** | Graph1 では `marginalCovariance()` を毎周期呼ばず、プリインテグレーションによる**解析的共分散伝播 + EKF 形式の車速更新**で `Σ₁` を得る | 20 ms 周期での周辺化はコスト過大。鎖状グラフなのでフィルタ解 = MAP 解。§0.1 (1) / §5.3.2 |
| **D10** | リセット時は **X・V・B の 3 状態すべて**を引き継ぐ。既定 `carry_cross_covariance: true`（`jointMarginalCovariance` で 15×15 を厳密に引き継ぐ）。`false` は近似であり**保守側である保証はない**ため、選ぶ場合は `cross_covariance_inflation`（既定 3.0 = Cauchy–Schwarz 境界）で保証付き保守化する | §5.3.7。相関を捨てると `Var(x') = P_xx + 2Δt·P_xv + Δt²·P_vv` の交差項が落ち、正相関の典型ケースで**過小評価**する |
| **D11** | 車速は自作 **`WheelSpeedFactor`（`NoiseModelFactor2<Pose3,Vector3>`, 残差 1 次元）**。既定で `NonHolonomicFactor`（残差 2 次元）も併用 | 測定モデル `v̂_x = e_xᵀ R(X)ᵀ V` を明記。§5.3.3a |
| **D12** | `StateHistory`（20 ms 刻み・**1.5 s**）を保持し、測定 stamp で SE(3) 補間して `μ₁(t), Σ₁(t)` を得る。遅延 `max_measurement_delay_ms`(300) 超と窓境界より古い stamp は drop。補間の性格は「保守的」ではなく**一次近似** | §5.3.4 |
| **D13** | **body フレーム = `base_link`**。IMU は TF `base_link←imu_link` で回転 + 遠心項補償して使う。`Imu.orientation` は無視 | §5.3.1 |
| **D14** | ROS ↔ GTSAM 共分散変換を `P_gtsam = blkdiag(Rᵀ,Rᵀ)·Π·P_ros·Πᵀ·blkdiag(R,R)` と規定。`R` は常に `μ₁` の回転を使う | §5.3.5。ROS 生共分散の直接加算は禁止 |
| **D15** | Graph2 補正後、`t_end` 以降の **IMU と車速を timestamp 順に merge して replay** し、Graph1 と `StateHistory` を作り直す（IMU は新バイアスで再積分、車速は §5.3.3a と同一モデルで EKF 更新） | IMU のみの replay ではアンカー前後で Graph1 のアルゴリズムが変わってしまう。§5.3.7 (5) |
| **D16** | 初期化を `UNINITIALIZED → BIAS_INIT → RUNNING` の状態機械として規定。X0/V0/B0 と初期共分散に**実装必須デフォルト**を与える | §5.3.8 |
| **D17** | MultiThreadedExecutor + 4 callback group。**Graph2 最適化は専用ワーカスレッド**、出力は shared_ptr の atomic swap でロックなし。`fusion_method` は `read_only` | §4.3 |
| **D18** | センサ timeout / 共分散妥当性 / GTSAM 例外 / 発散検知の挙動を全て規定。ゼロ・NaN 共分散は出さない | §5.3.9 |
| **D19** | 自作モードの車速入力は `vehicle_velocity_converter`（車速のみ）を既定とする。`ekf` モードは互換のため `twist_estimator`（gyro_odometer 出力） | gyro_odometer 出力は IMU 角速度を含むため生 IMU と二重計上になる。§2.4 |
| **D20** | 6DoF 結果は**既定でそのまま出力**。`output.flatten_to_2d: true` で EKF 互換の 2D セマンティクスに射影（`ekf` モードは常に `true` 扱い）。`kinematic_state.twist` は `base_link` 系（`R(X)ᵀ V`） | §3.3 |
| **D21** | 層 A（ゲート）+ 層 B（重み）を既定 ON、層 C（ロバスト核）を既定 OFF。適用順を固定し、作用値を診断に出す | §5.3.6 |
| **D22** | 層 B の `w = exp(−½ max(0,d²−dof))` は**ヒューリスティック**であることを明記し、代替関数形を差し替え可能にする。感度評価を実装完了条件に含める | §5.3.6（要求 R11 の「ロジック案」に対する回答） |
| **D23** | Graph2 は「500 ms ごとの全リセット」ではなく **`graph2_lag_ms`（既定 1000 ms）のオーバーラップ付き固定ラグ窓**とし、窓外のみを周辺化して境界 prior に畳む。`graph2_lag_ms ≥ optimize_interval + max_measurement_delay + margin` を起動時に強制 | リセット直後に届いた遅延測定の貼り先ノードが消えている境界問題を解消。要求 R10 からの意図的逸脱（§0.1 (2)）。§5.3.2 / §6.2 |
| **D24** | Graph2 は incremental なノード追加をやめ、**tick ごとに窓内をバッチ再構築**。既定オプティマイザは `levenberg_marquardt` | out-of-order 測定は既存 IMU ファクタの分割・再プリインテグレーションを要求するため、逐次 factor 追加では表現できない。§5.3.2 / 試験 17 |
| **D25** | GNSS は **`GnssLeverArmFactor : NoiseModelFactor1<Pose3>`**（残差 `p + R(X)·r_bg − z`、`∂r/∂X = [−R·skew(r_bg), R]`）。前処理で `z − R̂·r_bg` を作る方式は禁止 | レバーアーム補正は姿勢依存であり、姿勢を定数固定すると誤差がバイアスになる。共分散は map 系のまま使う。§5.3.3b |
| **D26** | 初期バイアスは `b_g = ω̄_meas`、**`b_a = ā_meas − R_wb^T·(0,0,g)ᵀ`**（GTSAM `MakeSharedU` の specific force 規約）。roll/pitch と `b_a` は同時推定できないため、`pose_source` に応じてどちらを既知とするか規定 | `b_a = mean(accel)` は誤り（静止中も重力を観測する）。§5.3.8 / 試験 19 |
| **D27** | **`internal_estimate`（アンカー即時適用）と `published_estimate`（smoothing 後）を分離**。smoothing は出力側のみ。ゲート・NIS・NEES・Graph2 初期値は必ず internal を使う | 出力 pose・出力共分散・内部状態の三者不整合を防ぐ。§5.3.7 |
| **D28** | **`Graph2MeasurementHistory`** を新設し、窓内の採用済み測定（補正後 noise model 込み）を周辺化されるまで保持して**毎 tick 再投入**する。`PendingMeasurementQueue` はスレッド間受け渡し専用。共分散補正の結果は到着時に**凍結**し再計算しない（INV-3） | 毎 tick バッチ再構築（D24）は、測定の永続ストアが無いと前 tick の測定を失って成立しない。凍結は自己参照ループと到着順依存を防ぐため。§5.3.2a |
| **D29** | Graph2 のノードタイムラインに**次 tick の境界時刻 `t_next_begin` を強制ノードとして必ず含める**（実行時パラメータ化しない）。history の prune は stamp ではなく**割り当てノードキー基準**（`assigned_node ≤ j*` かつ `contribution != kNotYetBuilt`）。境界ノードには unary 測定ファクタを張らない | 境界を跨ぐチェーン型ファクタを原理的に消し、周辺化を厳密化して二重計上・情報欠落の両方を防ぐ（INV-1〜3）。§5.3.2 / §5.3.7 (3) |
| **D30** | 車速は **Graph1 = 20 ms 区間あたり最新 1 サンプル、Graph2 = 各ノードへ最寄り 1 サンプル**。サンプル→最寄りノードの一意割り当てで再利用を禁止し、時刻ずれ `Δt` は `σ² += (a_max·Δt)²` でノイズに加算。`graph2_binding: none` の逃げ道を常設 | 車速 stamp を全てノード化すると 1 s 窓・50 Hz で約 50 ノード増え、D24 のコスト前提が崩れる。過サンプリングされた相関の強い信号を全数独立扱いすると共分散を過小評価する。§5.3.3a' |
| **D31** | アンカージャンプの smoothing は**レート制限方式**（`smoothing_max_lin_rate_mps` 等、上限時間で頭打ち）。**出力 twist にも smoothing 由来の速度を加算**して `d(pose_published)/dt = twist_published` を保ち、twist 共分散も `+u uᵀ` で膨張させる | 固定ステップ吸収は大ジャンプで非物理的な見かけ速度を生む。pose だけ smoothing すると下流（`pose_instability_detector` / `twist2accel`）から見て pose と twist が矛盾する。§5.3.7 |
| **D32** | smoothing の対象を**絶対 pose ではなく補正オフセット `C`** とする: `X_published(t) = X_internal(t) · Expmap(c(t))`、`c ← c + u·dt`。**published pose を独立した状態として保持しない**（INV-4）。発動判定は出力ステップ毎ではなく**アンカー更新イベント時**に `c = Log(X_internal⁻¹·X_pub_hold)` で行う | 絶対 pose を積分する方式は車両運動を出力に含めないため、走行中に published pose が実質停止し、終端で走行距離相当のジャンプを起こす。`u ∥ c₀` なので `c` の加算は Lie 群上でも厳密。§5.3.7 |
| **D33** | 「相互相関・交差共分散を捨てる近似は保守側」という主張を**設計書から全廃**する。保守性が必要な箇所では**保証付きの手段**（`n·blkdiag(P) ⪰ P` の Cauchy–Schwarz 境界、明示的な `covariance_scale`、ジャンプ検出による区間除外）を使い、根拠を式で示す | `blkdiag([[1,0.9],[0.9,1]]) − P` は非 PSD。SLAM 相対共分散が保守側になるのは `sym(Ad_{ΔT}⁻¹P₁₂) ⪰ 0` のときに限る。§5.3.7 / §5.3.3c / §5.3.4 |
| **D34** | 境界ノード `t_begin` を `node_merge_tolerance_ms` の**吸収先にしない**。近傍の測定は `max(stamp, t_begin + tolerance)` に独立ノードを作って張り、ずらし分をノイズへ加算する。あわせて `AcceptedMeasurement::contribution` を持ち、**一度もファクタになっていない測定を prune で削除することを禁止**（INV-5） | 「境界ノードに unary を張らない」×「`assigned_node ≤ j*` を削除」の組み合わせは、境界に併合された新規測定を**使わないまま捨てる**。二重計上ではなく情報の純損失で、発散しないため発見が難しい。§5.3.2 (b)(g) |
| **D35** | **`t_begin ≡ 境界 prior が張られているノードの時刻`**（実装 `t_prior_`）と定義し、`t_end − graph2_lag_ms` から毎 tick 計算しない。周辺化 (g) が成功したときにのみ更新する。失敗中は窓が伸びるので、バッファ長の下限に `optimizer_failure_limit × interval` を加える | 最適化失敗時に prior だけ古く `t_begin` だけ進むと、(i) prior を張るノードが窓内に存在しない、(ii) 未吸収の測定が「窓より古い」として捨てられる、が同時に起きる。§5.3.2 / §5.3.9 / §6.2 |
| **D36** | **Graph1 の全 public 操作を `graph1_mutex_` で保護**する（sensor callback 側も含む）。ロック順序は `graph1_mutex_ → buffer_mutex_` に固定。replay 中の保持時間は 1 ms 未満を目標とし、試験 22 ② で実測して超過するなら「別インスタンス構築 + atomic swap + 追い replay」へ切り替える | リセット・replay は Graph2 ワーカスレッドが実行するので「sensor_cb_group 内でのみ触るからロック不要」は成立せず、実際にはデータ競合。§4.3 |
| **D37** | smoothing 状態 `c` / `u` / `t_left` / `prev_snapshot_` は **output callback group 専有**。ワーカは世代番号付き `InternalEstimateSnapshot` だけを atomic swap し、`c` の作り直しは出力側が「新旧スナップショットを同一時刻 `t_now` で評価」して行う | `c` を双方が書く設計はスナップショットに含めても不変オブジェクトにならずロストアップデートが起きる。また出力側で作り直す方が「同一時刻で評価する」要求を自然に満たす。§4.3 / §5.3.7 (6) |
| **D38** | SLAM 相対拘束の**単位を「連続サンプル対」**とし、`d² / w / Σ_Δ'` を対の成立時に凍結する。予測側は `ΔT_Graph1` と**区間プリインテグレーション共分散**（絶対 `Σ₁` ではない）。ジャンプ検出は (i) この `d²` の χ² 検定（主）+ (ii) `v_max·Δt + margin` の動力学的上界（保険）の 2 段。固定距離閾値は使わない | 単一絶対姿勢でゲートすると、相対拘束の品質と無関係な理由（SLAM の絶対ドリフト）で棄却される。固定 1.0 m の閾値は 15 m/s × 100 ms = 1.5 m と矛盾する。§5.3.3c / §5.3.6 |
| **D39** | `lio_sam_imu_preintegration` モードの共分散伝播を **joint 15×15**（`jointMarginalCovariance` → `predict()` のヤコビアン → `preintMeasCov()` 加算）に統一。個別伝播オプションは用意しない | 個別伝播は X/V/B の交差項を落とし、D33 に自ら違反する。`fixed` という明示的な逃げ道がある以上、中途半端な近似を選べるようにする利益がない。§5.2.1 |
| **D40** | smoothing 中の出力共分散は、**`Σ_internal` を published 基準へ輸送してから**バイアス項を足す。**pose は `J_r⁻¹(−c)`、twist は `Ad_{C⁻¹}`**（作用素が異なる）。近似オプションは設けず常に適用する | `Σ_internal` は `X_internal` 基準、`c cᵀ` は `X_published` 基準で、そのまま足すのは座標系不一致。`Ad_{C⁻¹}` は同一時刻の tangent ベクトルの写像であり、群の合成 `Exp(−c)Exp(ξ)` の 1 次項は `J_r⁻¹(−c)`。§5.3.7 |
| **D41** | `initialization.pose_source: gnss` の yaw 取得元を `gnss_yaw_source`（`auto` = dual-antenna → course over ground → `initialpose` → **初期化拒否**）で規定。`yaw = 0` + 大分散は既定にせず、試験用の明示指定 + 起動 WARN に限る | GNSS 位置 + 重力では roll/pitch しか決まらず yaw は不定。誤った yaw で RUNNING に入ると、下流へ「もっともらしいが誤った姿勢」を渡すことになり、沈黙して停止するより危険。§5.3.8 |
| **D42**（本版で新規） | **共分散補正（層 A/B/C）の適用対象を絶対姿勢入力（GNSS / NDT / SLAM）に限定**する。車速は **Graph1 の EKF 更新直前に層 A（dof = 1、`gate_chi2.dim1`）のみ**を適用し、層 B/C は適用しない。棄却したサンプルは `VehicleTwistBuffer` に棄却フラグ付きで残し、Graph1・Graph2 とも使わない（replay でも同じ判定を再現する） | 旧版は「dof 表に車速がある」「(d) が車速を unary 測定ファクタとして扱う」一方で「`MeasurementSource` から車速を除外」「補正は絶対姿勢入力に対するもの」と書いており、車速に層 A/B を適用するかが未定義だった。1 次元スカラーへの共分散膨張は非ホロノミック拘束と併せて二重の減衰になるため層 B は適用しない。§5.3.2 / §5.3.6 |
| **D43**（本版で新規） | **プラグインは `internal_estimate` のみを返す**（`estimate_internal()` は `const`）。smoothing 状態と `published_estimate` の導出は `PoseTwistFusionNode::OutputStage` が排他的に所有する | 旧版のプラグイン IF は `estimate() const` でありながら、smoothing が毎ステップ `c`/`t_left` を書き換える設計で、const では実装できなかった。責務を分ければ const が成立し、3 モードで smoothing 挙動が共通化され、internal/published の取り違えも型で防げる。§4.1 / §5.3.7 |
| **D44**（本版で新規） | 参照されていながら定義が無かったパラメータを新設する: `graph2.boundary_merge_yaw_rate_rps`（境界併合時の姿勢ノイズ加算、既定 1.0）、`wheel_speed.timeout_ms`（車速 dropout 判定、既定 200） | §5.3.2 (b) は `max_yaw_rate` を、§5.3.9 は「車速 timeout 200 ms」を使っていたが、いずれも §6.1 に対応するパラメータが無く、SLAM 専用パラメータの流用と読める状態だった。§5.3.2 / §5.3.9 / §6.1 |
| **D45**（本版で新規） | `reset.max_anchor_jump_m` / `_rad` の既定を **0.03 m / 0.003 rad** とし、§6.2 で「試験 3 の許容値（0.05 m / 0.005 rad）**未満**」を**起動失敗条件**として強制する。`smoothing_done_*` も 0.005 m / 0.0005 rad へ下げる | 既定を試験 3 の合否閾値と等値（0.05 / 0.005）にすると、即時適用が定義上ちょうど閾値に達しうるため試験が境界ケースで原理的に落ちる。余裕を持たせる。§5.3.7 / §6.2 |

---

## 9. Open Questions

### 9.1 合意済み事項（確定）

要求の解釈に関する論点は §0 のトレーサビリティ表に、技術的な決定は §8 の D1〜D45 に集約した。
過去 5 回のレビューで挙がった論点（Q1〜Q51）はすべて D1〜D45 のいずれかに反映済みであり、
本書では個別の Q 番号を追跡しない（`design.md` §9.1〜9.2e に履歴が残っている）。

### 9.2 残る事項（実装をブロックしない）

いずれも「実装必須デフォルトは §6.1 に記載済み」で、実測により**調整する**項目:

- `covariance_correction.w_min` と `weight_function` の最終選定（試験 5・13 で決定）。
- `slam.covariance_scale` の最終値（試験 11 ① の NEES 結果で決定）。
- `slam.use_as` の既定（`relative` 前提だが、外部 SLAM の実出力品質次第で `absolute` に変える余地）。
- `slam.use` の既定（本書は `true`。要求どおりの入力構成で評価する場合は `false`。試験 11 ① の結果で運用既定を決める）。
- `output.exact_smoothing_adjoint` の既定（`false` 前提。試験 21 ⑩ で旋回中の `Ad_{C⁻¹}` 効果が判定閾値を超えるなら `true` へ）。
- `slam.max_speed_mps` / `max_yaw_rate_rps` / `jump_margin_m` / `jump_margin_rad`（副判定の上界）を車両仕様に合わせて設定する。主判定は χ² なのでチューニング不要。
- `slam.interp_sigma_per_s`（周辺化境界で対を分割する際の補間誤差。試験 20 の一致精度で確認）。
- `initialization.gnss_yaw_source` の運用既定（`auto` 前提。dual-antenna 非搭載車両では course over ground に依存するため、試験 24 ② で「静止状態からの GNSS 初期化が成立しない」ことを明示的に確認する）。
- `graph1_mutex_` の保持時間（試験 22 ② で p99 を実測。2 ms を超えるなら atomic swap 方式へ切替・D36）。
- IMU ノイズ値（§6.1 は LIO-SAM 既定に準拠。実機 IMU の Allan 分散で置換）。
- `graph2_lag_ms` の最終値（既定 1000 ms。§6.2 の下限を満たす範囲で、試験 10 の計算コストと試験 4 の遅延耐性から決定）。
- `graph2.marginalization` の既定（`linear_container` 前提。`prior_triple` との差を試験 3 で測る）。
- `graph2.optimizer` に `isam2` を選んだ場合の incremental 化（窓を跨いだ再利用）は **Phase 2 以降の検討事項**。現設計は毎 tick 再構築であり、これは out-of-order 対応と引き換えの意図的な選択（D24）。
- `wheel_speed.graph2_binding` の既定（`nearest` 前提。`none` との差を試験 11 ③ の NEES で確認し、二重計上が有意なら `none` へ）。
- `reset.smoothing_max_lin_rate_mps` / `smoothing_max_ang_rate_rps` の最終値（試験 21 で下流 `pose_instability_detector` / `twist2accel` の反応を見て決定）。

---

## 10. リスク・留意点

| リスク | 内容 | 対策 |
| --- | --- | --- |
| GTSAM バージョン差 | 本環境 `libgtsam.so.4` = 4.x。`CombinedImuFactor` / `GPSFactor` / `GPSFactorArm` / `Pose3::LogmapDerivative` の有無と API 差異 | `find_package(GTSAM 4 REQUIRED)`、CI でバージョン固定。レバーアーム GPS ファクタは CMake で存在確認し、無ければ自作 `GnssLeverArmFactor` を使う（§5.3.3b）。`LogmapDerivative` は試験 21 ⑪ で数値微分と照合 |
| **`GTSAM_POSE3_EXPMAP` フラグ** | 無効ビルドでは `Pose3::retract` が Expmap でなくなり、§5.3.5 の共分散変換の前提が崩れる | CMake で `GTSAM_POSE3_EXPMAP` / `GTSAM_ROT3_EXPMAP` を確認し、無効なら**ビルドエラー**にする |
| 実時間性 | 500 ms tick ごとの**窓バッチ再構築**（ラグ 1000 ms 分の IMU 再プリインテグレーション + 最適化）の計算コスト | `debug/processing_time_ms` で監視。§4.3 のワーカ分離で出力を守る。上限超過時は `graph2_lag_ms` を縮めて §6.2 の下限まで下げるか、`max_measurement_delay_ms` を下げる。試験 10 |
| **replay の経路二重化** | Graph1 の通常処理と replay で別コードを書くと、アンカー更新前後で挙動が変わる | 通常処理と replay で**同一の `Graph1::applyImu()` / `applyWheelSpeed()` を共用**することを実装規約とし、試験 18 で等価性を検証 |
| **internal / published の取り違え** | ゲートや NIS に smoothing 後の推定を使うと、smoothing 中に絶対入力を誤って棄却する | 型レベルで分離（`InternalEstimate` / `PublishedEstimate` を別型にし、暗黙変換を禁止）。プラグインは internal しか返さない（D43）。§5.3.7 / D27 |
| **測定の二重計上（周辺化境界）** | 境界 prior に吸収済みの測定を history に残して再投入すると、共分散を静かに過小評価する（発散しないので気づきにくい） | INV-1〜3（§5.3.7）を実装不変条件とし、debug ビルドで `assert`、試験 20 で回帰検知。NEES 監視（試験 11）で系統的な過信を検出 |
| **history の取り違え** | `StateHistory`（Graph1 の状態列）と `Graph2MeasurementHistory`（測定ストア）は名前が似ており混同しやすい | 前者は `state_history_`、後者は `measurement_history_` と識別子を明確に分け、所有スレッドも分離（前者は `buffer_mutex_`、後者はワーカ専有）。§4.3 の共有物表を唯一の正とする |
| **smoothing 由来の見かけ速度** | 大きなアンカージャンプを短時間で吸収すると、下流 `twist2accel` に非物理的な加速度が伝わる。逆に即時適用の閾値を大きく取ると 1 ステップで飛ぶ | レート制限 smoothing + twist 整合（D31）。`max_anchor_jump_*` を試験 3 の閾値未満に強制（D45・§6.2）。レート上限を破った場合は診断 WARN を格上げし、`debug/covariance_correction` 群に `\|c\|` と `\|u\|` を出す |
| **published pose の独立積分** | published 側を独立した積分器として持つと、車両運動が出力に反映されない／二重に反映される | published pose は `X_internal · Expmap(c)` から**毎回導出**し、状態として持たない（INV-4・D32）。型で強制し、試験 21 ⑦⑧（10–15 m/s 走行条件）で回帰検知 |
| **「近似は保守側」という無根拠な記述** | 相関・交差項を捨てる近似を「安全側」と書くと、実際には過小評価（過信）している箇所をレビューで見逃す | D33 を規約とする: 保守性を主張する箇所には必ず**成立条件または保証付き境界を式で示す**。示せない場合は「保守性は保証しない近似」と書く。NEES 監視（試験 11）で過信を数値検出 |
| **測定の情報消失（境界併合）** | 境界ノードに併合された測定が「ファクタを張られないまま吸収済み扱いで prune」され、黙って消える。二重計上と違い共分散は**過大**になるので NEES では検出しにくい | `t_begin` を併合の吸収先から外す（構造）＋ `contribution` による削除禁止（INV-5）。INV-5 の例外（(a-4)・失敗時再構築）は必ず診断カウンタで可視化する。試験 20 ②③⑥。§5.3.2 (b)(g) / D34 |
| **Graph1 のデータ競合** | ワーカの replay と sensor callback が同時に Graph1 を書き換える。症状は再現性の低い推定飛びで、デバッグが極めて困難 | 全 public 操作を `graph1_mutex_` で保護し、ロック順序を固定。ThreadSanitizer ビルドを CI に入れ、試験 17・18 を TSan 下で走らせる。§4.3 / D36 |
| **smoothing 状態の二重書き込み** | `c` をワーカと出力側の双方が書くとロストアップデートで補正が消え、published pose が飛ぶ | `c/u/t_left` を `OutputStage` の private メンバに閉じ込め、output callback group からのみ触る（D37・D43） |
| **NEES の向きの取り違え** | 「NEES が低い＝過信」と解釈すると、過信（NEES 高）を正常と読み、保守化（NEES 低）を危険と読む。**判定が反転して危険側に倒れる** | §11 冒頭に定義表（`E[NEES] = dof`、高＝過信 / 低＝保守）を置き、解析スクリプト側でも「高い＝過信」と出力文言に埋め込む。試験 11 の期待方向を明記 |
| 診断の不統一 | 3 モードで診断・初期化挙動が違うと `pose_instability_detector` が誤検知 | 診断は `PoseTwistFusionNode` 側で一元化（プラグインは `DiagnosticStatus` を返すのみ）。試験 2・16 |
| ライセンス | LIO-SAM コードは ROS1/独自スタイル・BSD-3 | ファイル冒頭に BSD-3 表記を継承。`package.xml` の `license` に併記 |
| 情報の二重計上（SLAM ⊃ IMU） | SLAM odometry は既に IMU 情報を含む（§5.3.3c） | `covariance_scale` で弱める。NEES 評価。`slam.use: false` 常設（要求どおりの構成でもある） |
| ヒューリスティック依存 | 層 B の重み関数は理論保証がない（D22） | 代替関数形を差し替え可能に。試験 5・13 を実装完了条件に |
| TF 競合 | `map → base_link` の重複 publish | §7 の TF authority 規約 + 起動時チェック |

---

## 11. テスト・受け入れ基準

`test/` 配下に実装する。

- **マージ必須**: 1・2・3・12・17・18・20・21・22・24
- **`slam.use: true` を採用する場合のみマージ必須**: 23
- **Phase 完了条件**: 4〜11・13〜16・19

> **NEES / NIS の定義（本章の合否判定はすべてこれに従う）**:
>
> ```
> NEES = (x̂ − x_true)ᵀ · P⁻¹ · (x̂ − x_true)      E[NEES] = dim(x)   （真値が必要。シミュレーション専用）
> NIS  = νᵀ · S⁻¹ · ν  （ν = イノベーション）      E[NIS]  = dim(ν)   （実機でも計算できる）
>
> NEES（NIS）が dof より有意に **大きい** → 申告共分散 P（S）が**小さすぎる ＝ 過信**
> NEES（NIS）が dof より有意に **小さい** → 申告共分散 P（S）が**大きすぎる ＝ 保守的**
> ```
>
> 有意性は自由度 `N·dof`（N = サンプル数）のカイ二乗 95 % 区間で判定する。
> `P` には **必ず `internal_estimate` の `Σ_internal`** を使う（smoothing 由来の MSE 行列は使わない・§5.3.7）。

| # | 種別 | 内容 | 合否基準 |
| --- | --- | --- | --- |
| 1 | 出力互換 | 全出力トピックの存在・型・`frame_id` / `child_frame_id` / QoS / publish rate を §3.1・§3.3 の表と照合。`#2a`/`#2b`/`#3` の内容一致も検証 | 全項目一致（launch_test）。要求 R2 の 3 トピックが必ず含まれること |
| 2 | EKF 回帰 | 同一 rosbag を本家 `ekf_localizer` と `fusion_method:=ekf` で再生し出力比較 | 位置差 < 1 cm、yaw 差 < 1 mrad、共分散相対差 < 1 %（実質同一実装のため厳しくてよい） |
| 3 | リセット連続性 | 500 ms アンカー更新の前後で pose / twist / 共分散を検査 | 位置ジャンプ < 5 cm、yaw ジャンプ < 5 mrad、共分散の対角が単調に不連続増加しない。**`max_anchor_jump_m` / `_rad` の既定（0.03 / 0.003）がこの基準を厳密に下回っていることをパラメータ側でも検証する**（§6.2 の起動失敗条件と一致） |
| 4 | 遅延センサ | GNSS / NDT に 100 / 300 / 500 ms の遅延を注入。tick 直後・tick 直前の両方のタイミングで注入する（§5.3.2 の境界条件） | 300 ms までは推定精度劣化 < 10 % かつ **注入タイミングに依らず採用される**、500 ms は drop され診断 WARN が出る |
| 5 | 外れ値（GNSS） | **(a) 単体試験**: 層 A/B を直接呼び、`Σ₁` と `R_z` を**固定値で与えて** 5 m / 20 m / 100 m のイノベーションを入力する。既定条件は `Σ_ĥ = diag(0.1², 0.1², 0.2²)`（1 s DR 相当）、`R_z,pos = diag(0.5², 0.5², 1.0²)`（GNSS 単独測位相当）とし、`w_min = 0.05` / `weight_function = exp_excess_nis`。**(b) 統合試験**: rosbag に同じジャンプを注入し発散しないことのみを見る | **(a)** `S = Σ_ĥ + R_z` の x 成分は `0.1² + 0.5² = 0.26 m²` なので `d²(5 m) = 25/0.26 ≈ 96.2`、`d²(20 m) ≈ 1538`、`d²(100 m) ≈ 3.8e4`。判定は「`d²` の計算値が解析値と 1e-6 以内で一致」「`dof = 3`・`gate_chi2.dim3 = 16.27` に対し 5 m でも**層 A で棄却**される」「ゲートを OFF にしたとき `w = clamp(exp(−½(96.2−3)), 0.05, 1) = 0.05 = w_min`」。**距離だけで採用/棄却を規定しない**。`Σ_ĥ`・`R_z` の**標準偏差を 10 倍**（分散 100 倍・`S_x = 26 m²`）に緩めた第 2 条件（`d²(5 m) = 25/26 ≈ 0.96 < 16.27` → 採用・`w = 1`）も必ず実施し、**閾値が距離ではなく NIS で決まる**ことを試験として明示する **(b)** いずれの条件でも推定が発散しない |
| 6 | 発散（NDT） | 人工的な姿勢ジャンプ・共分散増大 | 層 A/B が作動、DR で継続、復帰時に再捕捉 |
| 7 | センサ dropout | IMU / 車速 / GNSS / NDT / SLAM を個別に停止・復帰 | §5.3.9 の表どおりの診断遷移（車速は `wheel_speed.timeout_ms` = 200 ms を使うこと）。復帰後 1 s 以内に定常復帰 |
| 8 | IMU バイアス | ステップ状バイアス / ドリフトを注入 | バイアス推定が 5 s 以内に追従、位置誤差が発散しない |
| 9 | 共分散健全性 | 全出力の 6×6 を全周期チェック。`lio_sam_imu_preintegration` モードの joint 15×15 伝播（D39）の結果も対象に含める | 対称（\|P−Pᵀ\|∞ < 1e-9）、PSD（最小固有値 ≥ −1e-12）、NaN/Inf なし、全ゼロでない。**LIO-SAM モードで `Σ_nav(t)` をモンテカルロ（IMU ノイズを実際に注入した 10⁴ 本の伝播）と照合し、位置・速度ブロックの相対差 < 5 %。個別伝播（X/V/B を独立に伝播）で計算した参照値は同条件で位置分散を過小評価することを併せて示す（D39 の回帰試験）** |
| 10 | 実時間性 | 30 分連続再生で処理時間を計測 | 出力タイマ実行時間の p99 < 2 ms、Graph2 最適化の p99 < 300 ms、出力周期ジッタ < 5 ms |
| 11 | 二重計上 / NEES | ① `slam.use` の on/off ② `reset.carry_cross_covariance` の on/off（off 側は `cross_covariance_inflation` = 1.0 と 3.0 の両方）③ `wheel_speed.graph2_binding` = `nearest` / `none` の各組み合わせで NEES を比較 | ① NEES が dof の 95 % 信頼区間に収まる `slam.covariance_scale` を選定（この試験で既定値を確定） ② **`carry_cross_covariance: false` + `inflation 1.0` では NEES が dof を系統的に上回る（＝共分散が過小＝過信）ことを確認**する。`inflation 3.0` では逆に dof を下回る（保守側）こと、既定の `true` では信頼区間に収まることも確認（＝「相関を捨てるのは保守側」という誤解の回帰試験・D33） ③ `nearest` と `none` の NEES 差が有意なら既定を見直す |
| 12 | 共分散変換（単体） | `covariance_convert` の ROS→GTSAM→ROS 往復。twist 側の `Πᵀ · M · Π` も含む | 往復誤差 < 1e-9。既知の解析解（純並進・純回転・複合）と一致。twist は `[ω;ν]` → `[ν;ω]` の並べ替えが正しいことを成分単位で確認 |
| 13 | パラメータ感度 | `w_min` ∈ {0.01, 0.05, 0.2}、`weight_function` 3 種の総当たり | 位置 RMSE の変動が既定設定の ±20 % 以内であること、または最良設定を既定に採用 |
| 14 | 自作ファクタ（単体） | `WheelSpeedFactor` / `NonHolonomicFactor` / `GnssLeverArmFactor`（`r_bg = 0` と非零の両方）の残差・ヤコビアン | GTSAM `numericalDerivative` との差 < 1e-6。`r_bg = 0` では `GPSFactor` と残差・ヤコビアンが一致 |
| 15 | 初期化 | 各 `pose_source`、静止/走行中初期化、センサ欠損下の初期化 | すべて `RUNNING` へ到達、または明示的に ERROR 診断で停止（沈黙しない） |
| 16 | 統合 | 3 モードで logging_simulator / AWSIM の自動運転スタックを起動 | 下流ノードが正常動作、`pose_instability_detector` の誤検知なし |
| **17** | **時刻順序** | GNSS / NDT / SLAM を意図的に順不同で投入（例: stamp 10.10 → 10.30 → 10.20 → 10.40）。さらに Graph2 tick の直後に 300 ms 遅延測定を投入するケースを含める | ① crash しない ② Graph2 のノードタイムラインが常に時刻昇順 ③ IMU ファクタ区間が挿入ノードで正しく分割されている（区間の和が窓全体と一致・重複なし） ④ 同じ測定集合を stamp 順に投入した場合と推定結果が許容差内（位置 < 1 mm、yaw < 0.1 mrad）で一致 ⑤ tick 直後の 300 ms 遅延測定が drop されずファクタとして採用される |
| **18** | **replay 等価性** | 同一の IMU / 車速入力列に対し、(a) アンカー更新なしで連続伝播した場合と、(b) 途中でアンカー更新（同一の X\*,V\*,B\* を与える）を挟んで §5.3.7 (5) の replay を行った場合を比較 | 両者の `StateHistory` が全グリッド点で一致（位置 < 1e-9 m、共分散相対差 < 1e-9）。**車速 replay を落とすと失敗する**ことを退行検知として確認する。車速の層 A 棄却判定も replay で同一に再現されること |
| **19** | **初期バイアス** | 既知の姿勢・既知のバイアスで合成した静止 IMU 列（水平・roll 10°・pitch −5° の 3 条件）を投入 | `b_g`・`b_a` の推定が真値と一致（`b_a` 誤差 < 1e-6 m/s²）。重力の符号規約ミス（≈ 19.6 m/s² の誤差）が起きないことを保証 |
| **20** | **測定 history の永続性・非二重計上** | ① `t=10.0` の tick で GNSS@9.8・NDT@9.9 を投入し、`t=10.5`・`t=11.0` の tick を進める ② 同じ測定集合を「周辺化が起きない十分長いラグ窓（例 `graph2_lag_ms = 10000`）での 1 回のバッチ最適化」で解いた参照と比較 ③ **両方の境界に測定 stamp を意図的に一致させる（`t_next_begin ± 2 ms` と `t_begin ± 2 ms` の両方、`node_merge_tolerance_ms` 内外の両方）** | ① 9.8/9.9 の測定が `t=10.5` の窓（`[9.5, 10.5]`）でも**ファクタとして再投入されている**（グラフのファクタ数で確認） ② 参照バッチ解と一致（位置 < 1 mm、yaw < 0.1 mrad） ③ 各測定がグラフ全体で**ちょうど 1 回**寄与する（INV-1: prune 後の history に `assigned_node ≤ j*` かつ `contribution != kNotYetBuilt` が残らない） ④ 全ファクタが周辺化境界を跨がない（INV-2。SLAM 対の分割も含む） ⑤ 30 分連続再生で history サイズが `measurement_history_max` に張り付かない ⑥ **`t_begin ± 2 ms` に置いた測定が必ずファクタとして寄与する**（INV-5: `contribution == kNotYetBuilt` のまま prune される要素が 0 件） ⑦ **テスト専用フックで境界ノード強制生成を無効化すると ③ か ④ のどちらかが必ず失敗する**（実行時パラメータを廃止した代わりの回帰試験） ⑧ INV-5 の例外経路（(a-4) と失敗時再構築）で捨てた件数が診断カウンタに正しく現れる |
| **21** | **smoothing 整合性** | **0.01 m / 0.1 m / 1 m / 5 m** のアンカージャンプを注入し、出力 pose・twist・共分散を全周期記録。**走行条件を必ず 3 通り実施する: (A) 静止 (B) 直進定速 10 m/s (C) 定速 15 m/s + 旋回 0.3 rad/s**。(B)(C) は「published pose が補正レートだけで動き車両運動を含まない」欠陥を検出するために必須 | ① **0.01 m（< `max_anchor_jump_m` = 0.03）は即時適用**（smoothing なし・`c` が 1 ステップで 0） ② **0.1 m / 1 m / 5 m は必ず SMOOTHING 経路に入り**、`smoothing_max_duration_ms` 以内に吸収完了する（0.1 m は即時適用されてはならない・D45） ③ smoothing 中の `‖d(pose_published)/dt − twist_published‖` < 0.01 m/s（`twist_smoothing_consistency: true` 時、全条件） ④ **共分散加算が GTSAM tangent 空間で行われている**こと（`c` を人工的に `[ω]` 側のみ非零にしたとき、ROS 出力の膨張が `rx,ry,rz` ブロックに現れ `x,y,z` に現れないことで検証・§5.3.5） ⑤ 出力共分散が smoothing 完了時に `Σ_internal` へ連続的に戻る ⑥ `pose_instability_detector` が誤検知しない（試験 16 と併せて確認） ⑦ **条件 (B)(C) で、smoothing 中の `d(pose_published)/dt` が車速（10 / 15 m/s）± 補正レート上限の範囲に入る**（回帰検知の主眼） ⑧ **smoothing 終了時の pose 不連続が `smoothing_done_m` 以下** ⑨ smoothing 中にさらにアンカー更新（0.5 m）を重ねても、残差 `\|c\|` が単調に減少し `smoothing_max_duration_ms × 2` 以内に IDLE へ戻る ⑩ 条件 (C) で `exact_smoothing_adjoint` の true/false 差を測定し、③ の閾値を超えるなら既定を `true` に変更する **⑪ 共分散の輸送が正しいこと**: `c` を既知の値（並進 1 m + 回転 0.1 rad）に固定し、`M_pose = J_r⁻¹(−c) Σ_int J_r⁻ᵀ(−c) + c cᵀ` を、`X_int` にモンテカルロ摂動（10⁵ サンプル、`Σ_int` から生成）を与えて実測した MSE 行列と照合する（相対差 < 2 %）。`J_r⁻¹(−c)` を `I` に置換した実装と `Ad_{C⁻¹}` に置換した実装はいずれもこの基準を外すことを確認し、**作用素の取り違えを回帰検知する** **⑫** twist 側は `Ad_{C⁻¹} Σ_twist,int Ad_{C⁻¹}ᵀ + u uᵀ` で同じ照合を行う（こちらは `Ad` が正しい） **⑬** `OutputStage` が smoothing 状態を専有していること（プラグインの `estimate_internal()` を直接呼んでも smoothing が掛からない）を型・API レベルで確認（D43） |
| **22** | **スレッド安全性** | ① ThreadSanitizer ビルドで試験 17・18 と 30 分の rosbag 再生を実行 ② Graph2 最適化を人工的に 400 ms へ引き延ばし、replay と sensor callback を意図的に競合させる ③ アンカー更新の直後 1 ステップで出力 pose・twist を記録する | ① TSan の data race 検出が **0 件**（特に Graph1・smoothing 状態・`StateHistory`） ② `graph1_mutex_` の保持時間 p99 < 2 ms（超過するなら D36 の atomic swap 方式へ切替） ③ アンカー更新をまたぐ 1 ステップで published pose の不連続が `smoothing_done_m` 以下（出力側での `c` 作り直しが正しく効いている・D37） ④ 出力周期ジッタ < 5 ms（試験 10 と同一基準） |
| **23** | **SLAM 相対拘束**（`slam.use: true` 時のみ必須） | ① 合成データで 15 m/s 直進 + 100 ms 間隔の SLAM オドメトリを与える（正常系） ② 途中に 0.3 m のループ閉じ込み由来の基準ジャンプを注入 ③ SLAM stamp を Graph2 の周辺化境界 `j*` を跨ぐように配置 | ① **全区間で `BetweenFactor` が張られる**（固定距離閾値 1.0 m 方式ならここで全棄却される。回帰検知の主眼） ② 注入した区間だけが χ² 主判定で棄却される（低速 1 m/s 条件でも検出できること） ③ 対が `j*` で 2 本に分割され、分割前後で推定結果が一致（位置 < 1 mm）かつ INV-2 を満たす ④ `d²` / `w` が到着時（対の成立時）に凍結され、tick を跨いでも変化しない（INV-3） |
| **24** | **GNSS 初期化 yaw** | `pose_source: gnss` で ① dual-antenna heading あり ② heading なし・静止のまま ③ heading なし・2 m/s 以上で 2 m 以上走行 ④ `zero_with_large_covariance` を明示指定 | ① 1 s 以内に RUNNING、初期 yaw 誤差 < 0.1 rad ② **RUNNING に入らず診断 ERROR `"gnss init: yaw source unavailable"` が出続ける**（沈黙して誤った yaw で走り出さない） ③ 走行開始から `gnss_yaw_min_duration_s + 1 s` 以内に RUNNING、yaw 誤差 < 0.2 rad ④ 起動時に診断 WARN が出て RUNNING に入り、初期 yaw 分散が `gnss_yaw_fallback_stddev_rad²` になっている |

---

## 付録 A. `design.md`（rev.6）からの変更点

本書は `design.md` rev.6 の内容を確定仕様として引き継いだうえで、内部矛盾 **18 件**を解消した。
仕様として実質的に変わったのは「新規パラメータ 2 個」「既定値 4 個」「プラグイン IF」「車速のゲート適用範囲」である。

### A.1 版間の取り残しの修正（記述のみ・仕様不変）

| # | 箇所 | 旧記述 | 修正 |
| --- | --- | --- | --- |
| A-1 | §4.2 データフロー図 | `ImuBuffer(2.5 s)` / `VehicleTwistBuffer(2.5 s)` | **3.5 s**（§6.1 / §6.2 と一致させた） |
| A-2 | §5.3.7 (5) 本文 | 「IMU と同じ 2.5 s 深さのリングバッファ」 | 記述を削除し §6.2 の検証式へ一本化 |
| A-3 | §5.3.7「レート既定値の根拠」 | 「`max_anchor_jump_m = 1.0 m` を 1 s 以内で吸収できる値として選ぶ」 | `max_anchor_jump_m` とは独立の説明に書き換え（`smoothing_max_duration_ms` 以内に 1 m / 0.2 rad を吸収する値、と定義） |
| A-4 | §5.3.7 `exact_smoothing_adjoint` の説明 | 「`\|c\| ≤ max_anchor_jump_m` 相当（1 m / 0.1 rad）」 | 「`\|c\|` は最大で `smoothing_max_duration × rate ≈ 1 m`」に修正（`max_anchor_jump_m` は無関係） |
| A-5 | §9.2 Q8 | 「相互相関は**既定で捨てる**（明示的近似）」 | D10 と正面から矛盾していた。Q 表を廃止し §8 の D10（既定 `true`）に一本化 |
| A-6 | §9.2 Q12 | 「`StateHistory` **1 s**」 | D12・§6.1 と同じ **1.5 s** に統一（Q 表廃止により消滅） |
| D-1 | §5.3.5 | 「順序入替 `Π`（`[ω;ν] → [ν;ω]`）」 | 同節の定義（`Π : [t;θ] → [θ;t]`）および式 `Πᵀ·M·Π` と向きが逆だった。**「逆順序入替 `Πᵀ`」**に修正 |
| D-2 | §5.3.2 / §5.3.3a' | Graph1 の車速が「全サンプル」と「20 ms あたり最大 1 回」で併記 | **「20 ms 区間あたり最新 1 サンプル」に統一**（既定 50 Hz では全サンプル相当であることを注記） |
| D-5 | §4.3 | replay 保持時間の見積りを `graph2_lag_ms + optimize_interval`（1.5 s）としていた | replay 範囲は `t_end` 以降なので `optimize_interval + p99(最適化) ≈ 0.8 s` に修正（過大見積りだったため実害なし） |
| D-4 | §11 冒頭 | 試験 23・24 のマージ必須／Phase 区分が未指定 | 24 をマージ必須、23 を `slam.use: true` 時のみマージ必須と明記 |

### A.2 未定義だった仕様の確定（実質的な仕様追加）

| # | 内容 | 決定 |
| --- | --- | --- |
| B-1 | §5.3.2 (b) の境界併合ノイズ加算で使う `max_yaw_rate` が §6.1 に無く、SLAM 専用の `slam.max_yaw_rate_rps` を流用しているように読めた | **`graph2.boundary_merge_yaw_rate_rps`（既定 1.0）を新設**（D44）。§6.2 に検証も追加 |
| B-2 | §5.3.9 のセンサ timeout 表に「車速 200 ms」があるが §6.1 に対応パラメータが無かった | **`wheel_speed.timeout_ms`（既定 200）を新設**（D44）。§6.2 に検証も追加 |
| B-3 | 車速に層 A/B を適用するかが未定義（dof 表には車速があるが、`MeasurementSource` からは除外されていた） | **層 A のみを Graph1 の EKF 更新直前に適用し、層 B/C は適用しない**と確定（D42）。棄却サンプルはフラグ付きで保持し replay でも同一判定を再現する |
| C-1 | プラグイン IF が `estimate(...) const` なのに smoothing が毎ステップ状態を書き換えており実装不能。smoothing の所有者も未定義 | **プラグインは `estimate_internal() const` で internal のみ返し、smoothing は `PoseTwistFusionNode::OutputStage` が専有**すると確定（D43） |
| C-2 | INV-4 が本文にはあるが不変条件表に無かった | 不変条件表に **INV-4** を追加（§5.3.7） |
| C-3 | INV-1 の文言「ちょうど一方に寄与」が `kIntentionallyExcluded`（どちらにも寄与しない）と矛盾 | 「**両方に寄与することは無い**（どちらか一方、またはどちらにも寄与しない）」に修正 |
| C-4 | INV-5「`kNotYetBuilt` は削除されない」と §5.3.2 (a-4)（`stamp < t_begin` は無条件削除）が字面上衝突 | **INV-5 の明示的例外を 2 件（(a-4) と失敗時の窓再構築）として列挙**し、いずれも診断カウンタで可視化することを必須化（§5.3.7） |

### A.3 既定値の変更

| パラメータ | 旧既定 | 新既定 | 理由 |
| --- | --- | --- | --- |
| `reset.max_anchor_jump_m` | 0.05 | **0.03** | 試験 3 の合否基準（< 5 cm）と**等値**だったため、即時適用がちょうど閾値に達しうる境界ケースで試験が原理的に落ちる。余裕を持たせる（D45） |
| `reset.max_anchor_jump_rad` | 0.005 | **0.003** | 同上（試験 3 は < 5 mrad） |
| `reset.smoothing_done_m` | 0.01 | **0.005** | §6.2 の検証 `smoothing_done_m < max_anchor_jump_m` を満たすため |
| `reset.smoothing_done_rad` | 0.001 | **0.0005** | 同上 |

あわせて §6.2 の検証を「`max_anchor_jump_m ≤ 0.05` なら WARN」から
「**`max_anchor_jump_m < 0.05` を満たさなければ起動失敗**」へ格上げした（試験 3 との整合を設定で壊せないようにするため）。

### A.4 試験の修正

| # | 旧 | 新 |
| --- | --- | --- |
| 試験 21 ①/⑬ | ①「0.1 m は即時適用（smoothing なし）」と ⑬「既定 0.05 m で 0.1 m のジャンプが必ず smoothing 経路に入る」が**正面から矛盾**していた（① は既定が 1.0 m だった頃の記述） | 注入量を **0.01 / 0.1 / 1 / 5 m** とし、①「0.01 m は即時適用」②「0.1 m 以上は必ず smoothing」に整理。旧 ⑬ は ② に統合し、⑬ は `OutputStage` の所有検証（D43）に差し替え |
| 試験 3 | — | `max_anchor_jump_*` の既定が合否基準を**厳密に下回る**ことをパラメータ側でも検証（等号を許さない） |
| 試験 7 | 「車速 200 ms」の根拠パラメータが無かった | `wheel_speed.timeout_ms` を使うことを明記 |
| 試験 12 | ROS↔GTSAM 往復のみ | twist 側の `Πᵀ · M · Π`（`[ω;ν] → [ν;ω]`）を成分単位で検証する項目を追加 |
| 試験 18 | IMU / 車速の replay 等価性 | 車速の**層 A 棄却判定**も replay で同一に再現されることを追加（D42） |
| 試験 20 | ①〜⑦ | INV-5 例外経路の件数が診断カウンタに現れることを ⑧ として追加 |

### A.5 本書で削除したもの

- rev.2〜rev.6 の改訂履歴テーブル（本文冒頭の 5 つの表）と、本文中の「rev.X の記述の撤回」ブロック引用。
  → 確定した結論のみを本文に残し、なぜそう決めたかは §8 の D 表に集約した。
- §9.1 / §9.2 / §9.2b〜§9.2e の Q 表（Q1〜Q51）。
  → すべて D1〜D45 に反映済み。履歴が必要な場合は `design.md` を参照する。
- 廃止済みパラメータへの言及（`slam.max_relative_jump_m` / `_rad`、`graph2.force_boundary_node`、`propagated_blockwise`）。
  → §6.2 の「廃止パラメータ指定時は起動失敗」という検証行にのみ残した。
