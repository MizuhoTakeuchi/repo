# 実装計画: `autoware_pose_twist_fusion`

対応設計書: [`design.md`](./design.md)（rev.6）。レビュー: [`review.md`](./review.md) / [`review_v2.md`](./review_v2.md) / [`review_v3.md`](./review_v3.md) / [`review_v4.md`](./review_v4.md) / [`review_v5.md`](./review_v5.md)。
本書はパッケージ構成・依存・フェーズ分割・検証計画を定義する。

- 改訂: **rev.6（2026-08-16）** — design.md rev.6（review_v5 反映）に追随。
  境界併合による情報消失の防止（D34・INV-5）、窓境界 `t_begin` の定義変更（D35）、
  Graph1 の mutex 化とロック順序（D36）、smoothing 状態の output thread 専有（D37）、
  SLAM 相対拘束のサンプル対単位化と χ² ジャンプ検出（D38）、LIO-SAM モードの joint 15×15 伝播（D39）、
  smoothing 共分散の輸送 `J_r⁻¹(−c)` / `Ad_{C⁻¹}`（D40）、GNSS 初期化 yaw の取得元（D41）、
  試験 22（スレッド安全性・マージ必須）・23（SLAM 相対）・24（GNSS 初期化 yaw）の追加を反映。
- 前改訂: rev.5（2026-08-16） — design.md rev.5（review_v4 反映）に追随。
  smoothing のオフセット方式化（D32・INV-4）、`carry_cross_covariance` 既定の `true` 化と
  `cross_covariance_inflation`（D10 改訂・D33）、SLAM 相対共分散のジャンプ検出（§5.3.3c）、
  試験 5 の NIS 基準化・試験 21 の走行条件追加とマージ必須化を反映。
- 前改訂: rev.4（2026-08-16） — design.md rev.4（review_v3 反映）に追随。
  `Graph2MeasurementHistory`（D28）、周辺化境界ノードの強制生成とキー基準 prune（D29）、
  Graph2 の車速ファクタ結合規則（D30）、レート制限 smoothing と twist 整合（D31）、窓端記号の改名を反映。
- 前改訂: rev.3（2026-08-16） — design.md rev.3（review_v2 反映）に追随。
  Graph2 の固定ラグ窓 + バッチ再構築（D23・D24）、車速 replay 用バッファ（D15）、
  `GnssLeverArmFactor`（D25）、初期バイアス式（D26）、internal/published 推定の分離（D27）を反映。
- 前改訂: rev.2（2026-08-16） — design.md rev.2 に追随。共分散変換・状態履歴・車速ファクタ・
  スレッド分離・初期化・異常系のコンポーネントと Phase 2 の細分化を追加。

---

## 1. パッケージ構成

```
src/universe/autoware_universe/localization/autoware_pose_twist_fusion/
├─ package.xml
├─ CMakeLists.txt                      … GTSAM_POSE3_EXPMAP 検証を含む（design §10）
├─ README.md                           … design.md の要約 + 使い方
├─ plugins.xml                         … pluginlib 宣言
├─ include/autoware/pose_twist_fusion/
│   ├─ pose_twist_fusion_node.hpp
│   ├─ fusion_plugin_base.hpp          … FusionPlugin 基底 / InputRequirements / EstimateResult
│   ├─ output_publisher.hpp            … design §3 共通出力の一元管理
│   ├─ covariance_convert.hpp          … ★ ROS ↔ GTSAM 共分散変換（design §5.3.5 / D14）
│   └─ diagnostics_builder.hpp         … ★ 3 モード共通の診断生成（design §10）
├─ src/
│   ├─ pose_twist_fusion_node.cpp      … executor / callback group / TF / trigger / initialpose
│   ├─ output_publisher.cpp            … 共分散健全化（対称化・PSD 化）を含む
│   ├─ covariance_convert.cpp
│   ├─ diagnostics_builder.cpp
│   └─ plugins/
│       ├─ ekf_fusion_plugin.{hpp,cpp}
│       ├─ lio_sam_imu_preintegration_plugin.{hpp,cpp}
│       │   └─ lio_sam_covariance.{hpp,cpp}      … ★ 出力共分散の前方伝播（design §5.2.1）
│       └─ factor_graph_fusion/
│           ├─ factor_graph_fusion_plugin.{hpp,cpp}
│           ├─ imu_buffer.{hpp,cpp}              … ★ IMU リングバッファ（3.5 s・rev.6）+ 再積分（D15）
│           ├─ vehicle_twist_buffer.{hpp,cpp}    … ☆ 車速リングバッファ（3.5 s、replay 用・D15/rev.3・rev.6）
│           ├─ measurement_queue.{hpp,cpp}       … ☆ PendingMeasurementQueue（スレッド間受け渡し専用・D24/D28）
│           ├─ measurement_history.{hpp,cpp}     … ◇ Graph2MeasurementHistory（窓内の採用済み測定・prune 規則・D28/D29）
│           │                                       ◆ contribution フラグと INV-5（D34・rev.6）
│           ├─ state_history.{hpp,cpp}           … ★ StateHistory + SE(3) 補間（D12）
│           ├─ graph1_propagator.{hpp,cpp}       … ★ 解析的伝播 + 車速 EKF 更新（D9b）
│           │                                       ※ 通常受信と replay で同一 API を共用（D15）
│           │                                       ◆ graph1_mutex_ で全操作を保護 + 区間相対共分散 API（D36/D38・rev.6）
│           ├─ graph2_window.{hpp,cpp}           … ☆ ラグ窓のノードタイムライン構築・区間分割・境界ノード強制（D23・D24・D29）
│           │                                       ◇ 車速サンプル → ノードの一意割り当て（D30）
│           │                                       ◆ t_begin を併合の吸収先から除外 / t_prior_ の保持（D34/D35・rev.6）
│           ├─ graph2_optimizer.{hpp,cpp}        … ★ 窓のバッチ最適化 + 専用ワーカスレッド（D17・D24）
│           ├─ graph2_marginalizer.{hpp,cpp}     … ☆ 窓外の周辺化 → 境界 prior（D23）+ ◇ history prune（D29 / INV-1〜3）
│           ├─ anchor_manager.{hpp,cpp}          … ★ X/V/B 引き継ぎ・replay 起動・internal/published 分離（D10・D15・D27）
│           ├─ smoothing_state.{hpp,cpp}         … ◆ output thread 専有の c/u/t_left と共分散輸送（D37・D40・rev.6）
│           ├─ slam_pair_gate.{hpp,cpp}          … ◆ SLAM 連続サンプル対のゲート・重み凍結・境界跨ぎ分割（D38・rev.6）
│           ├─ covariance_corrector.{hpp,cpp}    … 層 A/B/C（design §5.3.6 / D21・D22）
│           ├─ imu_frame_adapter.{hpp,cpp}       … ★ imu_link → base_link 変換（D13）
│           ├─ initializer.{hpp,cpp}             … ★ 初期化状態機械（D16）
│           └─ factors/
│               ├─ wheel_speed_factor.{hpp,cpp}      … ★ NoiseModelFactor2<Pose3,Vector3>（D11）
│               ├─ nonholonomic_factor.{hpp,cpp}     … ★ 同上・残差 2 次元
│               ├─ gnss_lever_arm_factor.{hpp,cpp}   … ☆ NoiseModelFactor1<Pose3>（D25/rev.3）
│               └─ factor_builders.{hpp,cpp}         … IMU / GNSS / NDT / SLAM → GTSAM ファクタ
├─ config/pose_twist_fusion.param.yaml
├─ schema/pose_twist_fusion.schema.json
├─ launch/pose_twist_fusion.launch.xml
└─ test/
    ├─ test_covariance_convert.cpp        … 試験 12
    ├─ test_wheel_speed_factor.cpp        … 試験 14（numericalDerivative 比較）
    ├─ test_gnss_lever_arm_factor.cpp     … 試験 14（r_bg = 0 / 非零）
    ├─ test_graph2_window.cpp             … 試験 17（out-of-order / 区間分割）+ 車速割り当て（D30）
    ├─ test_measurement_history.cpp       … ◇ 試験 20（永続性・非二重計上・INV-1〜3）
    ├─ test_replay_equivalence.cpp        … 試験 18（IMU + 車速 replay の等価性）
    ├─ test_initial_bias.cpp              … 試験 19（重力規約と b_a）
    ├─ test_covariance_corrector.cpp      … 試験 5 の単体部分
    ├─ test_state_history.cpp             … 試験 4 の単体部分
    ├─ test_anchor_manager.cpp            … 試験 3 + ◇ 試験 21（smoothing 整合性）
    ├─ test_initializer.cpp               … 試験 15
    ├─ test_covariance_health.cpp         … 試験 9（+ ◆ LIO-SAM joint 伝播のモンテカルロ照合・D39）
    ├─ test_thread_safety.cpp             … ◆ 試験 22（TSan / ロック保持時間 / アンカー跨ぎ連続性・rev.6）
    ├─ test_slam_pair_gate.cpp            … ◆ 試験 23（SLAM 相対拘束・rev.6）
    └─ test_output_interface.launch.py    … 試験 1
```

★ = design.md rev.2 で新規に必要と確定したコンポーネント。☆ = design.md rev.3（review_v2 反映）で追加。◇ = design.md rev.4（review_v3 反映）で追加。◆ = design.md rev.6（review_v5 反映）で追加。

## 2. 依存関係（package.xml）

- ビルド: `ament_cmake_auto`, `autoware_cmake`, `eigen3_cmake_module`, `pluginlib`
- 共通: `rclcpp`, `rclcpp_components`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_srvs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `diagnostic_msgs`, `autoware_internal_debug_msgs`, `autoware_utils_*`
- EKF: `autoware_kalman_filter`, `autoware_localization_util`（既存 `ekf_module` の再利用/移植）
- 自作 & LIO-SAM: `GTSAM`（`find_package(GTSAM 4 REQUIRED)`、`libgtsam.so.4`）, `Eigen3`
- テスト: `ament_cmake_gmock`, `ament_cmake_ros`（launch_test）
- ライセンス: LIO-SAM 由来コードは BSD-3 表記を継承（`package.xml` に併記）

**CMake で必須の検証（design §10）**:

```cmake
# GTSAM_POSE3_EXPMAP が無効だと covariance_convert の前提が崩れるためビルドを止める
include(CheckCXXSymbolExists)
# GTSAMConfig.h の GTSAM_POSE3_EXPMAP / GTSAM_ROT3_EXPMAP を確認し、無効なら FATAL_ERROR
```

## 3. フェーズ分割（マイルストーン）

各フェーズ末に「ビルド通過 + 該当テスト green」を完了条件とする。

### Phase 0 — 骨組みと出力互換（土台）

- [ ] パッケージ雛形・CMake（GTSAM フラグ検証込み）・pluginlib 宣言。
- [ ] `PoseTwistFusionNode`：param 読取 + **パラメータ検証**（design §6.2）、プラグイン動的ロード（`fusion_method` は `read_only`）、
      **MultiThreadedExecutor と 4 callback group**（D17）、出力タイマ、TF、`trigger_node_srv`、`initialpose`。
- [ ] `OutputPublisher`：design §3.1 の全トピック（**`/localization/pose_with_covariance` の絶対名リマップ**、
      `ns_pose_with_covariance`、`pose_with_covariance_no_yawbias`、`biased_pose_with_covariance`、`estimated_yaw_bias` を含む）+ TF + 診断。
      出力直前の共分散健全化（対称化・PSD クリップ・NaN 検査）を実装。
- [ ] `covariance_convert`：ROS ↔ GTSAM 変換（D14）。**Phase 2 より先に作り、単体テストを通す**。
- [ ] `diagnostics_builder`：3 モード共通の診断キー・レベル定義。
- [ ] **Passthrough プラグイン**（NDT をそのまま出力）で配線検証。
- **完了条件**: 試験 1（出力互換）・試験 12（共分散変換）green。既存 `pose_twist_fusion_filter.launch.xml` の
  include を差し替えても下流（`stop_filter` / `ndt_scan_matcher`）が起動する。

### Phase 1 — EKF プラグイン（互換性の基準確立）

- [ ] `EkfFusionPlugin`：既存 `ekf_module` を内部利用（まずライブラリ依存、不可なら vendored 移植）。
- [ ] 入出力を design §3.2 の**マッピング表に厳密に**従って配線
      （`ekf_biased_pose_with_covariance` → `biased_pose_with_covariance` のみ。`no_yawbias` へは割り当てない）。
- [ ] 入力 twist は `/localization/twist_estimator/twist_with_covariance`（D19）。
- [ ] 本家 `ekf_localizer` と rosbag で数値比較。
- **完了条件**: 試験 2（EKF 回帰、位置差 < 1 cm / yaw 差 < 1 mrad）green。

### Phase 2 — 自作 Factor-Graph Fusion（中核）

レビュー反映により 4 サブフェーズに分割する。

#### 2a. 基盤（フレーム・時刻・バッファ）
- [ ] `imu_frame_adapter`：TF `base_link←imu_link` 取得、回転 + 遠心項補償（D13）。
- [ ] `imu_buffer`：**3.5 s** リングバッファ（rev.6: 最適化失敗中の窓伸長を賄う・D35）、アンカー以降の再積分 API（D15）。
- [ ] ☆ `vehicle_twist_buffer`：**3.5 s** リングバッファ + `imu_buffer` との stamp 順 merge イテレータ（D15 / rev.3・rev.6）。
- [ ] `state_history`：20 ms × 1.5 s、SE(3) 補間、遅延/未来/窓境界の stamp 判定（D12）。**アンカー以降のみ再構築し、それより古いサンプルは保持**する。
- [ ] `initializer`：`UNINITIALIZED → BIAS_INIT → RUNNING` 状態機械（D16）。
      ◆ **rev.6: `gnss_yaw_source`（`auto` = dual-antenna → course over ground → `initialpose` → 拒否）を実装**（D41）。
      yaw が得られない間は RUNNING に入らず診断 ERROR を出し続ける。`zero_with_large_covariance` は明示指定時のみ + 起動 WARN。
- **完了条件**: 試験 15（初期化）・**試験 24（GNSS 初期化 yaw）**・`test_state_history` green。

#### 2b. ファクタ
- [ ] `wheel_speed_factor` / `nonholonomic_factor`：残差 + 解析ヤコビアン（D11）。
- [ ] ☆ `gnss_lever_arm_factor`：残差 `p + R(X)r_bg − z`、`∂r/∂X = [−R·skew(r_bg), R]`（D25）。
      GTSAM 側にレバーアーム対応 GPS ファクタがあれば CMake で検出して切替。
- [ ] `factor_builders`：`ImuFactor` + `BetweenFactor<ConstantBias>`（D6）、GNSS（上記・共分散は map 系のまま）、
      NDT `PriorFactor<Pose3>`、SLAM `BetweenFactor<Pose3>`（相対共分散合成込み）／`PriorFactor<Pose3>`（D7）。
- [ ] ◆ **rev.6: `slam_pair_gate`**（D38）：連続サンプル対で `ΔT_slam` / `ΔT_g1` / 区間プリインテグレーション共分散から
      `d² / w / Σ_Δ'` を作り**到着時に凍結**。χ² 主判定 + 動力学的上界の副判定。周辺化境界 `j*` を跨ぐ対の分割も含む。
- **完了条件**: 試験 14（`numericalDerivative` 比較、差 < 1e-6、`r_bg=0` で `GPSFactor` と一致）・**試験 23（SLAM 相対）** green。

#### 2c. 2 段グラフとアンカー
- [ ] `graph1_propagator`：解析的共分散伝播 + 車速の EKF 形式 1 ステップ更新（D9b）。`marginalCovariance()` は使わない。
      **通常受信と replay が同一の `applyImu()` / `applyWheelSpeed()` を通る**ように API を設計する（D15）。
      ◆ **rev.6: 全 public メソッドが入口で `graph1_mutex_` を取得**（内部は `*_locked()`）。ロック順序 `graph1_mutex_ → buffer_mutex_`（D36）。
      ◆ **rev.6: 任意区間の相対共分散を返す API**（SLAM 対のゲート用・D38）を追加する。
- [ ] ◇ `measurement_history`：`Graph2MeasurementHistory`（stamp 昇順維持・窓抽出・`assigned_node` 記録・
      prune・SLAM 補間アンカーの例外保持・上限超過時の WARN）（D28・D29）。
      **補正済み noise / d² / w は到着時に凍結し、再構築で再計算しない。**
      ◆ **rev.6: `contribution` フラグを持ち、`kNotYetBuilt` の要素を prune しない（INV-5）**。
      debug で assert、release で診断 ERROR（D34）。
- [ ] ☆ `graph2_window`：ラグ窓（`graph2_lag_ms`）のノードタイムライン生成（stamp 順ソート・5 ms 集約・100 ms 強制分割・
      `t_begin`/`t_next_begin`/`t_end` 確定、**`t_next_begin` の強制ノード化**）と、区間ごとの IMU プリインテグレーション（D23・D24・D29）。
      ◇ 車速サンプル → 最寄りノードの一意割り当てと時刻ずれのノイズ加算（D30）。境界ノードには unary 測定ファクタを張らない。
      ◆ **rev.6: `t_begin` を併合の吸収先にしない**（近傍測定は `t_begin + tolerance` 以降に独立ノード化 + ノイズ加算・D34）。
      ◆ **rev.6: `t_begin` は状態変数 `t_prior_`**（周辺化成功時のみ更新）であり、`t_end − lag` から毎 tick 計算しない（D35）。
- [ ] `graph2_optimizer`：窓の**バッチ再構築 + 最適化**（既定 `LevenbergMarquardtOptimizer`）、**専用ワーカスレッド**、
      `IndeterminantLinearSystemException` の捕捉・**窓のロールバック**・強制再構築（D18・D24）。
- [ ] ☆ `graph2_marginalizer`：次 tick の境界ノードについて、**除去される側の部分グラフのみ**を周辺化して
      `LinearContainerFactor` を生成（`prior_triple` 近似も実装）。窓内に残るファクタを周辺化に含めない（二重計上防止）（D23）。
      ◇ 周辺化と同時に `measurement_history` を prune し、INV-1/INV-2 を debug ビルドで `assert`（D29）。
- [ ] `anchor_manager`：X/V/B 引き継ぎ（**既定は `jointMarginalCovariance` による 15×15 厳密引き継ぎ**。
      `carry_cross_covariance: false` を選ぶ経路では `cross_covariance_inflation` を必ず適用）、
      **IMU + 車速 replay の起動**、`internal_estimate` / `published_estimate` の分離、
      ◆ **オフセット方式のジャンプ平滑化**（`X_published = X_internal·Expmap(c)`、`c ← c + u·dt`、
      発動判定はアンカー更新イベント時。**published pose を状態として持たない = INV-4**）+ twist 整合
      （`smoothing_max_*_rate` / `twist_smoothing_consistency` / `exact_smoothing_adjoint`）、
      スナップショットの atomic swap（D10・D15・D17・D27・D31・D32）。型を分けて取り違えを防ぐ
      （`PublishedEstimate` は `InternalEstimate` と `c` からの生成関数でのみ構築可能とする）。
      ◆ **rev.6: `c/u/t_left` は output thread 専有**。ワーカは世代番号付き `InternalEstimateSnapshot` のみ publish し、
      アンカー更新の検出と `c` の作り直しは output thread が新旧スナップショットを同一時刻で評価して行う（D37）。
      ◆ **rev.6: 出力共分散は輸送してから加算**（pose = `J_r⁻¹(−c)`、twist = `Ad_{C⁻¹}`）。近似オプションは設けない（D40）。
      ◆ **rev.6: `max_anchor_jump_m/rad` の既定を 0.05 / 0.005 に変更**し、通常発動の診断を INFO + カウンタへ降格。
- **完了条件**: 試験 3（リセット連続性、位置ジャンプ < 5 cm）・試験 10（実時間性）・**試験 17・18・20・21・22** green。

#### 2d. 共分散補正と異常系
- [ ] `covariance_corrector`：層 0（妥当性）→ 層 A（ゲート、dof 別閾値）→ 層 B（重み、`weight_function` 差し替え可）
      → 層 C（ロバスト核、既定 OFF）。低速緩和と復帰時ゲート緩和（D21・D22）。
- [ ] センサ timeout / dropout / 発散検知（D18）。
- [ ] `debug/covariance_correction`（`d²`, `w`, 棄却数）の publish。
- **完了条件**: 試験 5・6・7・8・9・11・13 green。合成データ（既知軌跡 + 外れ値 GNSS 注入）で外れ値が自動減衰し軌跡追従。

### Phase 3 — LIO-SAM IMU Preintegration プラグイン

- [ ] `imuPreintegration.cpp` の `IMUPreintegration`/`TransformFusion` を移植（リポジトリの LIO-SAM は既に ROS2 化済みのため差分小）。
- [ ] 入力配線（IMU=`/sensing/imu/imu_data`、絶対姿勢=外部 `mapOptimization` の LiDAR オドメトリ）。
      LIO-SAM 他部は外部起動前提、車速拘束なし、degenerate フラグ解釈も踏襲（D3）。
- [ ] `lio_sam_covariance`：アンカーの **joint 15×15**（`jointMarginalCovariance`）を `predict()` のヤコビアン `[H_state H_bias]` で
      前方伝播し `preintMeasCov()` を加算（design §5.2.1・**rev.6: D39**。X/V/B の個別伝播は禁止）。
      **ゼロ共分散を出さない**。
- [ ] 出力を `base_link`/`map` 規約へ写像するアダプタ。
- **完了条件**: 試験 9（共分散健全性）を本モードでも green。高レート姿勢を安定発行。

### Phase 4 — 統合・検証・ドキュメント

- [ ] 3 モードを AWSIM / logging_simulator rosbag で切替検証。
- [ ] TF authority チェック（design §7）の実装と launch コメント整備。
- [ ] §9.3 の残パラメータを試験 5・11・13 の結果で確定し、`config/` と schema に反映。
- [ ] `README.md`、schema、param 完成。CI（`autoware_lint_common`）通過。
- **完了条件**: 試験 16（統合）green。`pose_instability_detector` の誤検知なし。

## 4. 検証計画（design §11 との対応）

| 試験 # | 内容 | 実装場所 | Phase |
| --- | --- | --- | --- |
| 1 | 出力互換（topic/type/frame/QoS/rate） | `test_output_interface.launch.py` | 0 |
| 2 | EKF 回帰 | rosbag 比較スクリプト | 1 |
| 3 | リセット連続性 | `test_anchor_manager.cpp` + rosbag | 2c |
| 4 | 遅延センサ（100/300/500 ms） | `test_state_history.cpp` + 注入再生 | 2a |
| 5 | GNSS 外れ値（5/20/100 m） | `test_covariance_corrector.cpp` + 注入再生 | 2d |
| 6 | NDT 発散 | 注入再生 | 2d |
| 7 | センサ dropout | 注入再生 | 2d |
| 8 | IMU バイアス step/drift | 合成データ | 2d |
| 9 | 共分散健全性（対称・PSD・非 NaN・非ゼロ） | `test_covariance_health.cpp` | 0 / 2d / 3 |
| 10 | 実時間性（p99 出力 < 2 ms、Graph2 < 300 ms） | 30 分再生 + `processing_time_ms` | 2c |
| 11 | 二重計上 / NEES（`slam.use` / **`carry_cross_covariance`** / `graph2_binding` の on/off） | 解析スクリプト | 2d |
| 12 | 共分散変換の往復・解析解一致 | `test_covariance_convert.cpp` | 0 |
| 13 | パラメータ感度（`w_min` × `weight_function`） | 総当たり再生 | 2d |
| 14 | 車速ファクタのヤコビアン | `test_wheel_speed_factor.cpp` | 2b |
| 15 | 初期化シーケンス | `test_initializer.cpp` | 2a |
| 16 | 統合（3 モード + スタック起動） | 手動 / CI シナリオ | 4 |
| **17** | **時刻順序（out-of-order 測定）** | `test_graph2_window.cpp` + 注入再生 | 2c |
| **20** | **測定 history の永続性・非二重計上（INV-1〜3）** | `test_measurement_history.cpp` + 参照バッチ比較 | 2c |
| **21** | **smoothing 整合性（pose 微分 vs twist、静止 / 10 m/s 直進 / 15 m/s 旋回の 3 条件）** | `test_anchor_manager.cpp` + 注入再生 | 2c |
| **18** | **replay 等価性（IMU + 車速）** | `test_replay_equivalence.cpp` | 2c |
| **19** | **初期バイアス（重力規約）** | `test_initial_bias.cpp` | 2a |
| **22** | **スレッド安全性（TSan・`graph1_mutex_` 保持時間・アンカー更新跨ぎの連続性）** | `test_thread_safety.cpp` + TSan CI ジョブ + 30 分再生 | 2c |
| **23** | **SLAM 相対拘束（15 m/s 正常系 / ループ閉じ込み / 境界跨ぎ対の分割）** | `test_slam_pair_gate.cpp` | 2b / 2c |
| **24** | **GNSS 初期化 yaw（dual-antenna / 静止で拒否 / course / 明示 fallback）** | `test_initializer.cpp` | 2a |

**マージ必須**: 試験 1・2・3・12・**17・18・20・21・22**（20・21 は design rev.4 / rev.5、22 は rev.6 で追加）。

> **rev.6 で CI に追加するジョブ**: ThreadSanitizer ビルド（試験 17・18・22 を実行）。
> D36・D37 のスレッド設計は静的レビューで見落とされた種類の欠陥なので、機械的検出手段を常設する。

## 5. 確定事項と残課題（design §9 と連動）

**確定（2026-08-16 第 1・2 回）**: Q1〜Q6 + SLAM 入力追加 → design §9.1。

**レビューを受けて確定（rev.2）**: Q7〜Q20 → design §9.2。特に実装への影響が大きいもの:

| 決定 | 実装への影響 |
| --- | --- |
| D9（ノード生成規則） | `graph2_optimizer` の設計を「固定グリッド」から「測定時刻トリガ」に変更 |
| D9b（Graph1 は解析伝播） | Graph1 を iSAM2 で組まない。`graph1_propagator` として独立実装 |
| D10・D15（X/V/B 引き継ぎ + 再積分） | `anchor_manager` / `imu_buffer` が新規に必要 |
| D11（車速ファクタ） | カスタム GTSAM factor 2 種を新規実装 |
| D12（状態履歴） | `state_history` が新規に必要。全測定経路がこれを経由 |
| D13（IMU extrinsic） | `imu_frame_adapter` が新規に必要 |
| D14（共分散変換） | `covariance_convert` を Phase 0 に前倒し（全モードが依存） |
| D16（初期化） | `initializer` が新規に必要 |
| D17（スレッド） | ノード側を MultiThreadedExecutor 前提に。Graph2 は専用ワーカ |
| D18（異常系） | 例外処理・timeout 監視を全プラグインに横断実装 |
| D19（twist 入力の分離） | モードごとに購読トピックが異なる。`InputRequirements` で表現 |
| D20（6DoF 出力） | `output.flatten_to_2d` の実装。`kinematic_state.twist` は `base_link` 系へ変換 |

**第 2 回レビューを受けて確定（rev.3）**: Q21〜Q27 → design §9.2b。実装への影響:

| 決定 | 実装への影響 |
| --- | --- |
| D23（固定ラグ窓） | `graph2_optimizer` を「全リセット」から「窓 + 境界 prior」に変更。`graph2_marginalizer` が新規に必要。パラメータ検証に窓長条件を追加 |
| D24（バッチ再構築） | `graph2_window` が新規に必要。iSAM2 の incremental 更新は使わない。測定は `measurement_queue` 経由でワーカへ渡す |
| D15 改（車速 replay） | `vehicle_twist_buffer` が新規に必要。`graph1_propagator` の API を replay と共用できる形に設計し直す |
| D25（GNSS レバーアーム） | `gnss_lever_arm_factor` が新規に必要。GNSS 共分散は map 系のまま扱う（`covariance_convert` を通さない経路） |
| D26（初期バイアス式） | `initializer` に重力ベクトル規約と `pose_source` 別の可分性処理を実装 |
| D27（internal/published 分離） | `anchor_manager` と `output_publisher` の境界を再定義。`covariance_corrector` は internal のみを参照 |

**第 5 回レビューを受けて確定（rev.6）**: Q39〜Q51 → design §9.2e。実装への影響:

| 決定 | 実装への影響 |
| --- | --- |
| D34（境界併合の情報消失） | `AcceptedMeasurement` に `contribution` を追加。`graph2_window` の併合規則から `t_begin` を除外。試験 20 に ⑥⑦ を追加 |
| D35（`t_begin` = prior の時刻） | `graph2_window` に状態 `t_prior_` を持たせる。callback へ公開する `t_begin` も同じ値。バッファ既定を 3500 ms に拡大 |
| D36（Graph1 の mutex 化） | `graph1_propagator` の API を「public はロック取得、private `*_locked()`」の 2 層に。TSan CI を追加 |
| D37（smoothing 状態の所有） | スナップショットから `c/u/t_left` を除去し `generation` を追加。`output_publisher` 側に `SmoothingState` を private で持つ |
| D38（SLAM 対単位のゲート） | `slam_pair_gate` が新規に必要。`graph1_propagator` に区間相対共分散 API を追加。`BetweenFactor` の貼り方を「補間」から「サンプル stamp のノード対」へ変更。境界跨ぎ対の分割処理 |
| D39（LIO-SAM joint 伝播） | `lio_sam_covariance` を `jointMarginalCovariance` + `predict()` ヤコビアンで再実装。GTSAM のヤコビアン取得可否を CMake で判定 |
| D40（共分散の輸送） | `anchor_manager` / `output_publisher` に `J_r⁻¹(−c)`（`Pose3::LogmapDerivative`）と `Ad_{C⁻¹}` を実装。試験 21 ⑪⑫（モンテカルロ照合） |
| D41（GNSS 初期化 yaw） | `initializer` に yaw 取得元の状態機械を追加。GNSS メッセージの orientation 有効性判定と course over ground の実装 |
| Q51（`max_anchor_jump_*` 既定） | `config/*.param.yaml` と schema の既定値変更。診断レベルの見直し（WARN → INFO + カウンタ） |

**残課題（実測で確定・design §9.3）**:
- `w_min` / `weight_function`（試験 5・13 で決定）
- `slam.covariance_scale`（試験 11 の NEES で決定）
- `slam.use_as` の既定
- ~~`reset.carry_cross_covariance` の既定~~ → rev.5 で `true` に確定（試験 11 で `false` 側の劣化量を測る）
- `output.exact_smoothing_adjoint` の既定（試験 21 ⑩で決定。**共分散側の輸送は既定で常時 ON・D40**）
- ~~`slam.max_relative_jump_m` / `_rad`~~ → **rev.6 で廃止**。`slam.max_speed_mps` / `max_yaw_rate_rps` / `jump_margin_*` を車両仕様で設定
- `slam.interp_sigma_per_s`（境界跨ぎ対の分割誤差。試験 20 の一致精度で確認・rev.6）
- `initialization.gnss_yaw_source` の運用既定（試験 24 で確認・rev.6）
- `graph1_mutex_` の保持時間（試験 22 の p99 が 2 ms 超なら atomic swap 方式へ・rev.6）
- IMU ノイズ値（実機 Allan 分散で置換）
- `graph2_lag_ms` の最終値（試験 4・10 で決定）
- `graph2.marginalization` の既定（試験 3 で決定）
