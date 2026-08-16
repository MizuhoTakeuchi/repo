# 実装計画（最終版）: `autoware_pose_twist_fusion`

対応設計書: **[`design_last.md`](./design_last.md)**（最終版 / rev.7 相当）。
本書はパッケージ構成・依存・フェーズ分割・検証計画・要求適合性の確認手順を定義する。

- 版: **implementation_plan_last（2026-08-16）**
- 位置づけ: `implementation_plan.md`（rev.6）を `design_last.md` に追随させ、改訂履歴の記述を排除して
  「これから何をどの順で作るか」だけで再構成したもの。design_last で確定した新規決定（D42〜D45）と
  矛盾解消の結果を全節へ反映した。`implementation_plan.md` からの差分は**付録 B** に一覧する。
- 本書中の `D**` は `design_last.md` §8 の Design Decision ID、`R*` は同 §0 の要求 ID、
  `INV-*` は同 §5.3.7 の実装不変条件を指す。

---

## 0. 実装方針（設計から引き継ぐ 5 つの前提）

実装に入る前に、設計書が「実装規約」として要求している事項を先に押さえる。
これらは後から直すとコストが跳ね上がる種類のものである。

| # | 前提 | 実装への効き方 |
| --- | --- | --- |
| P1 | **プラグインは `internal_estimate` しか返さない**（D43） | smoothing / published 導出は `PoseTwistFusionNode::OutputStage` が独占する。プラグイン API を `estimate_internal(stamp) const` + `generation()` にすることで、`const` が成立し 3 モードで smoothing 挙動が共通化される。**先に IF を確定してから各プラグインを書く** |
| P2 | **通常処理と replay は同一関数を通る**（D15 / 試験 18） | `Graph1::applyImu()` / `applyWheelSpeed()` を唯一の入口とし、replay 専用の経路を作らない。車速の層 A 棄却判定（D42）も同じ関数の中で行い、replay で同一に再現されること |
| P3 | **internal と published を型で分ける**（D27 / INV-4） | `InternalEstimate` / `PublishedEstimate` を別型にし暗黙変換を禁止。`PublishedEstimate` は `InternalEstimate` と `c` からの生成関数でのみ構築可能とし、独立した pose メンバを持たせない |
| P4 | **INV-1〜5 を debug ビルドの `assert` に落とす**（D28・D29・D34） | 二重計上・情報消失は発散しないため、テストで気づけるのは不変条件を機械化したときだけ。`measurement_history` と `graph2_marginalizer` に assert を最初から入れる |
| P5 | **「近似は保守側」と書かない・実装しない**（D33） | 相関を捨てる近似を既定にしない。`carry_cross_covariance: true`、`marginalization: linear_container`、LIO-SAM の joint 15×15 伝播（D39）がいずれも既定 |

---

## 1. パッケージ構成

```
src/universe/autoware_universe/localization/autoware_pose_twist_fusion/
├─ package.xml
├─ CMakeLists.txt                      … GTSAM_POSE3_EXPMAP 検証を含む（design §10）
├─ README.md                           … design_last.md の要約 + 使い方
├─ plugins.xml                         … pluginlib 宣言
├─ include/autoware/pose_twist_fusion/
│   ├─ pose_twist_fusion_node.hpp
│   ├─ fusion_plugin_base.hpp          … FusionPlugin 基底 / InputRequirements
│   │                                     InternalEstimate / PublishedEstimate（別型・P3）
│   │                                     estimate_internal() const / generation()（D43）
│   ├─ output_stage.hpp                … ★ smoothing の所有者（c/u/t_left/prev_snapshot_）
│   │                                     internal → published の導出と出力共分散の輸送（D37・D40・D43）
│   ├─ smoothing_state.hpp             … ★ OutputStage の private メンバ型（他から参照不可）
│   ├─ output_publisher.hpp            … design §3 共通出力の一元管理
│   ├─ covariance_convert.hpp          … ROS ↔ GTSAM 共分散変換（D14）。twist の Πᵀ 経路を含む
│   ├─ param_validator.hpp             … ★ design §6.2 の検証（起動失敗 / WARN を分けて実装）
│   └─ diagnostics_builder.hpp         … 3 モード共通の診断生成
├─ src/
│   ├─ pose_twist_fusion_node.cpp      … executor / callback group / TF / trigger / initialpose
│   ├─ output_stage.cpp                … ★ アンカー更新の検出・発動判定・c の減衰・MSE 行列生成
│   ├─ output_publisher.cpp            … 共分散健全化（対称化・PSD 化）を含む
│   ├─ covariance_convert.cpp
│   ├─ param_validator.cpp
│   ├─ diagnostics_builder.cpp
│   └─ plugins/
│       ├─ ekf_fusion_plugin.{hpp,cpp}              … flatten_to_2d を常に true 扱い（D20）
│       ├─ lio_sam_imu_preintegration_plugin.{hpp,cpp}
│       │   └─ lio_sam_covariance.{hpp,cpp}         … joint 15×15 前方伝播（D39・design §5.2.1）
│       └─ factor_graph_fusion/
│           ├─ factor_graph_fusion_plugin.{hpp,cpp}
│           ├─ imu_buffer.{hpp,cpp}                 … IMU リングバッファ（3.5 s）+ 再積分（D15）
│           ├─ vehicle_twist_buffer.{hpp,cpp}       … 車速リングバッファ（3.5 s）
│           │                                          + 層 A 棄却フラグの保持（D42）
│           ├─ measurement_queue.{hpp,cpp}          … PendingMeasurementQueue（受け渡し専用・D28）
│           ├─ measurement_history.{hpp,cpp}        … Graph2MeasurementHistory（D28・D29）
│           │                                          contribution フラグと INV-5 + その例外 2 件（D34）
│           ├─ state_history.{hpp,cpp}              … StateHistory + SE(3) 補間（D12）
│           ├─ graph1_propagator.{hpp,cpp}          … 解析的伝播 + 車速 EKF 更新（D9b）
│           │                                          通常受信と replay で同一 API（D15・P2）
│           │                                          車速の層 A ゲート（D42）
│           │                                          graph1_mutex_ で全 public 操作を保護（D36）
│           │                                          区間相対共分散 API propagateInterval()（D38）
│           ├─ graph2_window.{hpp,cpp}              … ノードタイムライン構築・区間分割（D23・D24）
│           │                                          t_next_begin の強制ノード化（D29・INV-2）
│           │                                          t_begin を併合の吸収先にしない（D34）
│           │                                          t_prior_ の保持（D35）
│           │                                          車速サンプル → ノードの一意割り当て（D30）
│           ├─ graph2_optimizer.{hpp,cpp}           … 窓のバッチ最適化 + 専用ワーカ（D17・D24）
│           ├─ graph2_marginalizer.{hpp,cpp}        … 窓外の周辺化 → 境界 prior（D23）
│           │                                          + history prune と INV-1/2/5 の assert（D29・D34）
│           ├─ anchor_manager.{hpp,cpp}             … X/V/B 引き継ぎ・replay 起動・スナップショット publish
│           │                                          （D10・D15・D27・D37）★ smoothing は持たない
│           ├─ slam_pair_gate.{hpp,cpp}             … SLAM 連続サンプル対のゲート・凍結・境界跨ぎ分割（D38）
│           ├─ covariance_corrector.{hpp,cpp}       … 層 A/B/C（D21・D22）。適用対象は絶対姿勢入力のみ（D42）
│           ├─ imu_frame_adapter.{hpp,cpp}          … imu_link → base_link 変換（D13）
│           ├─ initializer.{hpp,cpp}                … 初期化状態機械 + gnss_yaw_source（D16・D41）
│           └─ factors/
│               ├─ wheel_speed_factor.{hpp,cpp}         … NoiseModelFactor2<Pose3,Vector3>（D11）
│               ├─ nonholonomic_factor.{hpp,cpp}        … 同上・残差 2 次元
│               ├─ gnss_lever_arm_factor.{hpp,cpp}      … NoiseModelFactor1<Pose3>（D25）
│               └─ factor_builders.{hpp,cpp}            … IMU / GNSS / NDT / SLAM → GTSAM ファクタ
├─ config/pose_twist_fusion.param.yaml
├─ schema/pose_twist_fusion.schema.json
├─ launch/pose_twist_fusion.launch.xml
└─ test/
    ├─ test_covariance_convert.cpp       … 試験 12（往復 + twist の Πᵀ 並べ替え）
    ├─ test_param_validator.cpp          … ★ design §6.2 の全行（起動失敗 / WARN の別を含む）
    ├─ test_output_stage.cpp             … ★ 試験 21 の単体部分 + 試験 22 ③（D37・D40・D43・D45）
    ├─ test_wheel_speed_factor.cpp       … 試験 14（numericalDerivative 比較）
    ├─ test_gnss_lever_arm_factor.cpp    … 試験 14（r_bg = 0 / 非零）
    ├─ test_graph2_window.cpp            … 試験 17（out-of-order / 区間分割）+ 車速割り当て（D30）
    │                                       + t_begin 併合除外（D34）+ t_prior_ の固定（D35）
    ├─ test_measurement_history.cpp      … 試験 20（永続性・非二重計上・INV-1〜5 とその例外）
    ├─ test_replay_equivalence.cpp       … 試験 18（IMU + 車速 replay。層 A 棄却の再現を含む）
    ├─ test_initial_bias.cpp             … 試験 19（重力規約と b_a）
    ├─ test_covariance_corrector.cpp     … 試験 5 の単体部分（NIS 基準）
    ├─ test_state_history.cpp            … 試験 4 の単体部分
    ├─ test_anchor_manager.cpp           … 試験 3（リセット連続性）
    ├─ test_initializer.cpp              … 試験 15 + 試験 24（GNSS 初期化 yaw）
    ├─ test_covariance_health.cpp        … 試験 9（+ LIO-SAM joint 伝播のモンテカルロ照合・D39）
    ├─ test_thread_safety.cpp            … 試験 22（TSan / ロック保持時間 / アンカー跨ぎ連続性）
    ├─ test_slam_pair_gate.cpp           … 試験 23（SLAM 相対拘束）
    └─ test_output_interface.launch.py   … 試験 1
```

★ = `implementation_plan.md`（rev.6）から**新設・移動**したコンポーネント（付録 B 参照）。

## 2. 依存関係（package.xml）

- ビルド: `ament_cmake_auto`, `autoware_cmake`, `eigen3_cmake_module`, `pluginlib`
- 共通: `rclcpp`, `rclcpp_components`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_srvs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `diagnostic_msgs`, `autoware_internal_debug_msgs`, `autoware_utils_*`
- EKF: `autoware_kalman_filter`, `autoware_localization_util`（既存 `ekf_module` の再利用/移植）
- 自作 & LIO-SAM: `GTSAM`（`find_package(GTSAM 4 REQUIRED)`、`libgtsam.so.4`）, `Eigen3`
- テスト: `ament_cmake_gmock`, `ament_cmake_ros`（launch_test）
- ライセンス: LIO-SAM 由来コードは BSD-3 表記を継承（`package.xml` に併記）

**CMake で必須の検証・機能判定**:

```cmake
# 1) GTSAM_POSE3_EXPMAP / GTSAM_ROT3_EXPMAP が無効だと covariance_convert の前提が崩れる → FATAL_ERROR
# 2) レバーアーム対応 GPS ファクタ（gtsam::GPSFactorArm 等）の有無 → GNSS_FACTOR_USE_GTSAM_ARM
# 3) PreintegratedImuMeasurements::predict() のヤコビアン引数の有無 → LIO_SAM_PREDICT_JACOBIAN
#    無い場合は NavState::update() / numericalDerivative へフォールバック（D39）
# 4) Pose3::LogmapDerivative の引数型（Vector6 / Pose3）→ SMOOTHING_LOGMAP_DERIVATIVE_ARG
#    smoothing 共分散の輸送 J_r⁻¹(−c) に必要（D40）
```

2〜4 はいずれも「無ければフォールバックするが、試験（14 / 9 / 21 ⑪）は必ず通す」方針とする。

## 3. フェーズ分割（マイルストーン）

各フェーズ末に「ビルド通過 + 該当テスト green」を完了条件とする。

### Phase 0 — 骨組み・出力互換・出力段（土台）

**このフェーズの狙い**: モードに依存しない部分（出力 IF / 共分散変換 / smoothing / パラメータ検証）を先に固めきる。
D43 により smoothing がプラグインの外に出たため、**Phase 0 の時点で smoothing を完成させて単体試験できる**（付録 B-1）。

- [ ] パッケージ雛形・CMake（上記 4 種の検証／機能判定込み）・pluginlib 宣言。
- [ ] `fusion_plugin_base.hpp`：**P1 の API を先に確定**する。
      `InputRequirements` / `InternalEstimate`（P3 の型分離）/ `estimate_internal(stamp) const` / `generation()` /
      `on_imu` / `on_vehicle_twist` / `on_ndt_pose` / `on_gnss_pose` / `on_slam_odometry` / `set_initial_pose` / `reset` / `diagnostics`。
- [ ] `PoseTwistFusionNode`：param 読取 + **`param_validator`**（design §6.2 の全行。**起動失敗と診断 WARN を明確に分ける**。
      廃止パラメータ `slam.max_relative_jump_m/_rad`・`graph2.force_boundary_node` の指定は起動失敗）、
      プラグイン動的ロード（`fusion_method` は `read_only`）、**MultiThreadedExecutor と 4 callback group**（D17）、
      出力タイマ、TF、`trigger_node_srv`、`initialpose`。
- [ ] `covariance_convert`：ROS ↔ GTSAM 変換（D14）。**twist 側は `P_twist,ros = Πᵀ · M · Π`（`[ω;ν]` → `[ν;ω]`）**を別 API として用意する。
- [ ] `OutputStage`（D37・D40・D43）:
      - smoothing 状態 `c` / `u` / `t_left` / `prev_snapshot_` を **private メンバ**として保持（他から参照不可）。
      - 毎ステップ先頭で `generation` 比較 → アンカー更新検出 → 新旧スナップショットを**同一時刻 `t_now`** で評価 → `c ← Log(X_new⁻¹·X_pub_hold)`。
      - 発動判定（`max_anchor_jump_m/rad`）、レート制限ラッチ、`c ← c + u·dt` の減衰、完了判定。
      - 出力共分散: **輸送してから加算**（pose = `J_r⁻¹(−c)`、twist = `Ad_{C⁻¹}`）→ MSE 行列 → `covariance_convert` で ROS 表現へ。
      - twist 整合（`twist_smoothing_consistency` / `exact_smoothing_adjoint`）。
      - `PublishedEstimate` は `InternalEstimate` + `c` からのみ構築（INV-4 を型で強制・P3）。
- [ ] `OutputPublisher`：design §3.1 の全トピック（**`/localization/pose_with_covariance` の絶対名リマップ**、
      `ns_pose_with_covariance`、`pose_with_covariance_no_yawbias`、`biased_pose_with_covariance`、`estimated_yaw_bias` を含む）+ TF + 診断。
      出力直前の共分散健全化（対称化・PSD クリップ・NaN 検査）。`#2a/#2b/#3` が同一内容であることを実装で保証する。
- [ ] `diagnostics_builder`：3 モード共通の診断キー・レベル定義。smoothing 発動は **INFO + カウンタ**、
      レート上限超過のみ WARN（D45）。
- [ ] **Passthrough プラグイン**（NDT をそのまま internal として返す）で配線検証。
- **完了条件**: 試験 1（出力互換）・試験 12（共分散変換）・**試験 21 の単体部分（①②③④⑤⑧⑨⑪⑫⑬）**・
  `test_param_validator` green。既存 `pose_twist_fusion_filter.launch.xml` の include を差し替えても
  下流（`stop_filter` / `ndt_scan_matcher`）が起動する。

### Phase 1 — EKF プラグイン（互換性の基準確立）

- [ ] `EkfFusionPlugin`：既存 `ekf_module` を内部利用（まずライブラリ依存、不可なら vendored 移植）。
- [ ] 入出力を design §3.2 の**マッピング表に厳密に**従って配線
      （`ekf_biased_pose_with_covariance` → `biased_pose_with_covariance` のみ。`no_yawbias` へは割り当てない）。
- [ ] 入力 twist は `/localization/twist_estimator/twist_with_covariance`（D19）。
- [ ] `output.flatten_to_2d` は値によらず `true` として扱い、`false` 指定時は起動時に診断 INFO を出す（D20）。
- [ ] 本家 `ekf_localizer` と rosbag で数値比較。
- **完了条件**: 試験 2（EKF 回帰、位置差 < 1 cm / yaw 差 < 1 mrad / 共分散相対差 < 1 %）green。

### Phase 2 — 自作 Factor-Graph Fusion（中核・要求 R4〜R11）

4 サブフェーズに分割する。

#### 2a. 基盤（フレーム・時刻・バッファ・初期化）

- [ ] `imu_frame_adapter`：TF `base_link←imu_link` 取得、回転 + 遠心項補償（D13）。
      TF / fallback の優先順位は design §5.3.1 の表（IMU = `imu.extrinsic_fallback`、GNSS = `gnss.lever_arm_fallback`）に従う。
- [ ] `imu_buffer`：**3.5 s** リングバッファ、アンカー以降の再積分 API（D15・D35）。
- [ ] `vehicle_twist_buffer`：**3.5 s** リングバッファ + `imu_buffer` との stamp 順 merge イテレータ（同一 stamp は IMU → 車速）。
      **層 A 棄却フラグを各サンプルに保持**し、Graph2 側の割り当てから除外できるようにする（D42）。
- [ ] `state_history`：20 ms × 1.5 s、SE(3) 補間、遅延/未来/窓境界の stamp 判定（D12）。
      **アンカー以降のみ再構築し、それより古いサンプルは保持**する。
- [ ] `initializer`：`UNINITIALIZED → BIAS_INIT → RUNNING` 状態機械（D16）。
      初期バイアス式 `b_a = ā_meas − R_wbᵀ(0,0,g)ᵀ`（`MakeSharedU` 規約）と `pose_source` 別の roll/pitch 可分性（D26）。
      **`gnss_yaw_source`（`auto` = dual-antenna → course over ground → `initialpose` → 拒否）を実装**（D41）。
      yaw が得られない間は RUNNING に入らず診断 ERROR を出し続ける。`zero_with_large_covariance` は明示指定時のみ + 起動 WARN。
- **完了条件**: 試験 15（初期化）・**試験 24（GNSS 初期化 yaw）**・試験 19（初期バイアス）・`test_state_history` green。

#### 2b. ファクタとゲート

- [ ] `wheel_speed_factor` / `nonholonomic_factor`：残差 + 解析ヤコビアン（D11）。
- [ ] `gnss_lever_arm_factor`：残差 `p + R(X)r_bg − z`、`∂r/∂X = [−R·skew(r_bg), R]`（D25）。
      GTSAM 側にレバーアーム対応 GPS ファクタがあれば CMake 判定で切替。共分散は **map 系のまま**扱う。
- [ ] `factor_builders`：`ImuFactor` + `BetweenFactor<ConstantBias>`（D6）、GNSS（上記）、
      NDT `PriorFactor<Pose3>`、SLAM `BetweenFactor<Pose3>` / `PriorFactor<Pose3>`（D7）。
- [ ] `covariance_corrector`：層 0（妥当性）→ 層 A（dof 別閾値）→ 層 B（`weight_function` 差し替え可）→ 層 C（既定 OFF）。
      **適用対象は絶対姿勢入力（GNSS / NDT / SLAM）のみ**（D42）。低速緩和と復帰時ゲート緩和も含む。
- [ ] `slam_pair_gate`（D38）：連続サンプル対で `ΔT_slam` / `ΔT_g1` / **区間**プリインテグレーション共分散から
      `d² / w / Σ_Δ'` を作り**到着時（対の成立時）に凍結**。χ² 主判定 + 動力学的上界の副判定。
      周辺化境界 `j*` を跨ぐ対の分割（`Σ_Δ'` の区間長按分 + `interp_sigma_per_s` 加算）も含む。
- **完了条件**: 試験 14（`numericalDerivative` 比較、差 < 1e-6、`r_bg=0` で `GPSFactor` と一致）・
  試験 5 の単体部分（NIS 基準・2 条件）・**試験 23（SLAM 相対拘束）** green。

#### 2c. 2 段グラフとアンカー

- [ ] `graph1_propagator`（D9b・要求 R6/R7）:
      - 解析的共分散伝播 + 車速の EKF 形式 1 ステップ更新。`marginalCovariance()` は使わない。
      - **20 ms 区間あたり車速は最新 1 サンプル**（既定 50 Hz では全サンプル相当）。
      - **車速の層 A ゲート**（dof=1、`gate_chi2.dim1`）を EKF 更新の直前に適用し、棄却をバッファへ記録（D42）。
      - **通常受信と replay が同一の `applyImu()` / `applyWheelSpeed()` を通る**（P2・D15）。
      - **全 public メソッドが入口で `graph1_mutex_` を取得**（内部は `*_locked()`）。ロック順序 `graph1_mutex_ → buffer_mutex_`（D36）。
      - **任意区間の相対共分散を返す `propagateInterval()`**（SLAM 対のゲート用・D38）。
- [ ] `measurement_history`：`Graph2MeasurementHistory`（stamp 昇順維持・窓抽出・`assigned_node` 記録・prune・
      SLAM 補間アンカーの例外保持・上限超過時の WARN）（D28・D29）。
      **補正済み noise / d² / w は到着時に凍結し、再構築で再計算しない**（INV-3）。
      **`contribution` フラグを持ち、`kNotYetBuilt` の要素を prune しない**（INV-5・D34）。
      **INV-5 の例外 2 件**（`stamp < t_begin` の (a-4) 破棄／連続失敗時の窓再構築）は**専用カウンタで診断に出す**。
- [ ] `graph2_window`（D23・D24・D29・D34・D35）:
      - ノードタイムライン生成（stamp 順ソート・5 ms 集約・100 ms 強制分割・`t_begin`/`t_next_begin`/`t_end` 確定）。
      - **`t_next_begin` の強制ノード化**（実行時パラメータにしない。INV-2 の成立条件）。
      - **`t_begin` を併合の吸収先にしない**（近傍測定は `t_begin + tolerance` 以降に独立ノード化し、
        `boundary_merge_velocity_mps` / `boundary_merge_yaw_rate_rps` でノイズ加算・D34・D44）。
      - **`t_begin` は状態変数 `t_prior_`**（周辺化成功時のみ更新）。`t_end − lag` から毎 tick 計算しない（D35）。
        callback へ公開する `t_begin` も同じ値。
      - 区間ごとの IMU プリインテグレーション。車速サンプル → 最寄りノードの一意割り当てと時刻ずれのノイズ加算（D30）。
      - 境界ノードには unary 測定ファクタを張らない。
- [ ] `graph2_optimizer`：窓の**バッチ再構築 + 最適化**（既定 `LevenbergMarquardtOptimizer`）、**専用ワーカスレッド**、
      `IndeterminantLinearSystemException` の捕捉・窓のロールバック（`t_begin` も prior も history も更新しない）・
      連続失敗時の強制再構築（D18・D24・D35）。
- [ ] `graph2_marginalizer`：次 tick の境界ノードについて、**除去される側の部分グラフのみ**を周辺化して
      `LinearContainerFactor` を生成（`prior_triple` 近似も実装）。窓内に残るファクタを周辺化に含めない（D23）。
      周辺化と同時に `measurement_history` を prune し、**INV-1 / INV-2 / INV-5 を debug ビルドで `assert`**（D29・D34・P4）。
- [ ] `anchor_manager`（D10・D15・D27・D37）:
      - X/V/B 引き継ぎ（**既定は `jointMarginalCovariance` による 15×15 厳密引き継ぎ**。
        `carry_cross_covariance: false` を選ぶ経路では `cross_covariance_inflation` を必ず適用）。
      - **IMU + 車速 replay の起動**（`t_end` 以降・stamp 順 merge）。
      - **世代番号付き `InternalEstimateSnapshot` の atomic swap**。
      - ★ **smoothing 状態は持たない**（`c/u/t_left` は `OutputStage` の専有・D37・D43）。
        `anchor_manager` は published pose を一切扱わない。
- **完了条件**: 試験 3（リセット連続性、位置ジャンプ < 5 cm）・試験 10（実時間性）・
  **試験 17・18・20・21（走行条件込みの全項）・22** green。

#### 2d. 異常系と数値検証

- [ ] センサ timeout / dropout / 発散検知（D18）。車速は **`wheel_speed.timeout_ms`（既定 200）**を使う（D44）。
- [ ] `debug/covariance_correction` の publish（入力別 `d²` / `w` / 棄却数、SLAM のジャンプ検出は主判定・副判定を別カウンタ、
      smoothing の `|c|` / `|u|`、INV-5 例外の破棄件数）。
- [ ] 合成データ（既知軌跡 + 外れ値 GNSS 注入）での挙動確認。
- **完了条件**: 試験 4・5（統合部分）・6・7・8・9・11・13 green。

### Phase 3 — LIO-SAM IMU Preintegration プラグイン

- [ ] `imuPreintegration.cpp` の `IMUPreintegration` / `TransformFusion` を移植（リポジトリの LIO-SAM は既に ROS2 化済みのため差分小）。
- [ ] 入力配線（IMU=`/sensing/imu/imu_data`、絶対姿勢=外部 `mapOptimization` の LiDAR オドメトリ）。
      LIO-SAM 他部は外部起動前提、車速拘束なし、degenerate フラグ解釈も踏襲（D3）。
- [ ] `lio_sam_covariance`：アンカーの **joint 15×15**（`jointMarginalCovariance`）を `predict()` のヤコビアン `[H_state H_bias]` で
      前方伝播し `preintMeasCov()` を加算（D39）。**X/V/B の個別伝播は実装しない**。
      `covariance_source: fixed` は明示的な逃げ道としてのみ用意する。**ゼロ共分散を出さない**。
- [ ] 出力を `base_link`/`map` 規約へ写像するアダプタ。`estimated_yaw_bias` は 0.0 を発行（D1）。
- **完了条件**: 試験 9（共分散健全性 + モンテカルロ照合、位置・速度ブロックの相対差 < 5 %）green。高レート姿勢を安定発行。

### Phase 4 — 統合・要求適合性・ドキュメント

- [ ] 3 モードを AWSIM / logging_simulator rosbag で切替検証。
- [ ] **§5 の要求適合性チェックリストを実行**（要求どおりの入力構成 `slam.use: false` での回帰を含む）。
- [ ] TF authority チェック（design §7）の実装と launch コメント整備。
- [ ] design §9.2 の残パラメータを試験 5・10・11・13・20・21 の結果で確定し、`config/` と schema に反映。
- [ ] `README.md`、schema、param 完成。CI（`autoware_lint_common`）通過。
- **完了条件**: 試験 16（統合）green。`pose_instability_detector` の誤検知なし。§5 のチェックリストが全項 ✅。

## 4. 検証計画（design_last.md §11 との対応）

| 試験 # | 内容 | 実装場所 | Phase | 区分 |
| --- | --- | --- | --- | --- |
| 1 | 出力互換（topic/type/frame/QoS/rate、`#2a/#2b/#3` 一致） | `test_output_interface.launch.py` | 0 | **必須** |
| 2 | EKF 回帰 | rosbag 比較スクリプト | 1 | **必須** |
| 3 | リセット連続性（既定値が閾値を厳密に下回ることの検証を含む） | `test_anchor_manager.cpp` + rosbag | 2c | **必須** |
| 4 | 遅延センサ（100/300/500 ms・tick 前後の両タイミング） | `test_state_history.cpp` + 注入再生 | 2a / 2d | Phase |
| 5 | GNSS 外れ値（NIS 基準・2 条件） | `test_covariance_corrector.cpp` + 注入再生 | 2b / 2d | Phase |
| 6 | NDT 発散 | 注入再生 | 2d | Phase |
| 7 | センサ dropout（車速は `wheel_speed.timeout_ms`） | 注入再生 | 2d | Phase |
| 8 | IMU バイアス step/drift | 合成データ | 2d | Phase |
| 9 | 共分散健全性 + LIO-SAM joint 伝播のモンテカルロ照合 | `test_covariance_health.cpp` | 0 / 2d / 3 | Phase |
| 10 | 実時間性（p99 出力 < 2 ms、Graph2 < 300 ms、ジッタ < 5 ms） | 30 分再生 + `processing_time_ms` | 2c | Phase |
| 11 | 二重計上 / NEES（`slam.use` / `carry_cross_covariance` / `graph2_binding`） | 解析スクリプト | 2d | Phase |
| 12 | 共分散変換の往復・解析解一致・twist の `Πᵀ` 並べ替え | `test_covariance_convert.cpp` | 0 | **必須** |
| 13 | パラメータ感度（`w_min` × `weight_function`） | 総当たり再生 | 2d | Phase |
| 14 | 自作ファクタのヤコビアン（車速 / 非ホロノミック / GNSS レバーアーム） | `test_wheel_speed_factor.cpp` / `test_gnss_lever_arm_factor.cpp` | 2b | Phase |
| 15 | 初期化シーケンス | `test_initializer.cpp` | 2a | Phase |
| 16 | 統合（3 モード + スタック起動） | 手動 / CI シナリオ | 4 | Phase |
| **17** | 時刻順序（out-of-order 測定・tick 直後の遅延測定） | `test_graph2_window.cpp` + 注入再生 | 2c | **必須** |
| **18** | replay 等価性（IMU + 車速 + 層 A 棄却の再現） | `test_replay_equivalence.cpp` | 2c | **必須** |
| 19 | 初期バイアス（重力規約） | `test_initial_bias.cpp` | 2a | Phase |
| **20** | 測定 history の永続性・非二重計上（INV-1〜5 + 例外カウンタ） | `test_measurement_history.cpp` + 参照バッチ比較 | 2c | **必須** |
| **21** | smoothing 整合性（単体 = Phase 0、走行条件 = Phase 2c） | `test_output_stage.cpp` + 注入再生 | 0 / 2c | **必須** |
| **22** | スレッド安全性（TSan・`graph1_mutex_` 保持時間・アンカー跨ぎの連続性） | `test_thread_safety.cpp` + TSan CI + 30 分再生 | 2c | **必須** |
| 23 | SLAM 相対拘束（正常系 / ループ閉じ込み / 境界跨ぎ対の分割） | `test_slam_pair_gate.cpp` | 2b / 2c | `slam.use: true` 時**必須** |
| **24** | GNSS 初期化 yaw（dual-antenna / 静止で拒否 / course / 明示 fallback） | `test_initializer.cpp` | 2a | **必須** |
| — | パラメータ検証（design §6.2 の全行） | `test_param_validator.cpp` | 0 | **必須** |

**CI に常設するジョブ**:

1. **ThreadSanitizer ビルド**（試験 17・18・22 を実行）。D36・D37 のスレッド設計は静的レビューで見落とされた種類の
   欠陥なので、機械的検出手段を常設する。
2. **debug ビルド（`assert` 有効）での試験 20**。INV-1〜5 は release ビルドでは診断 ERROR になるだけなので、
   assert が効く構成で必ず回す。
3. **要求どおりの入力構成（`slam.use: false`）での試験 1・3・11**（§5）。

## 5. 要求適合性チェックリスト（Phase 4 で実行）

`design_last.md` §0 のトレーサビリティを、実装完了時に機械的に確認する手順。

| # | 確認項目 | 手段 | 期待 |
| --- | --- | --- | --- |
| RC-1 | 要求 R2 の 3 トピックが実際に出ている（`kinematic_state` / `pose_twist_fusion_filter/pose_with_covariance` / `pose_with_covariance_no_yawbias`） | 試験 1 | 3 本とも存在し、型・frame_id が一致。`#2a/#2b/#3` の内容が完全一致 |
| RC-2 | 要求 R3 の 3 モードが `fusion_method` だけで切り替わる | 試験 16 | 3 モードとも同一 launch・同一下流構成で起動する |
| RC-3 | 要求 R5 の 4 入力だけで動作する（拡張の SLAM 無しでも成立） | **`slam.use: false` で試験 1・3・11 を再実行** | 全 green。NEES が dof の 95 % 区間内 |
| RC-4 | 要求 R6/R9 の更新周期 | `graph1_node_interval_ms` = 20 / `graph2_optimize_interval_ms` = 500 を config で確認 + 試験 10 の実測周期 | 設定どおり。Graph1 の出力が 20 ms 刻みで `StateHistory` に入る |
| RC-5 | 要求 R7 からの逸脱（Graph1 を最適化として実装しない）が妥当 | 合成データで、Graph1 の解と「同一区間を GTSAM でバッチ最適化した鎖状グラフの最新ノード周辺分布」を比較 | 位置差 < 1 mm、共分散相対差 < 1 %（線形ガウス近似の範囲で一致することを示す） |
| RC-6 | 要求 R8（Graph1 のリセット） | 試験 3・18 | アンカー更新ごとに Graph1 が再初期化され、replay 後の状態が連続伝播と一致 |
| RC-7 | 要求 R10 からの逸脱（固定ラグ窓）が妥当 | 試験 4（遅延 300 ms が tick 前後どちらでも採用される）+ 試験 20 ②（参照バッチ解と一致） | 全 green。全リセット方式では試験 4 が原理的に落ちることを注記として残す |
| RC-8 | 要求 R11（共分散補正が到着時の Graph1 分布に基づく） | 試験 5（単体・NIS 基準）+ 試験 20 ③（INV-3: tick を跨いで `d²`/`w` が不変） | 解析値と 1e-6 一致。凍結が破れない |

RC-5 と RC-7 は「要求からの逸脱が実害を持たないこと」を数値で示すための試験であり、
レビュー・受け入れ時の説明材料として残す。

## 6. 残課題（実測で確定・design_last.md §9.2 と連動）

| 項目 | 決定手段 |
| --- | --- |
| `covariance_correction.w_min` / `weight_function` | 試験 5・13 |
| `slam.covariance_scale` | 試験 11 ① の NEES |
| `slam.use` の運用既定（要求範囲外の拡張のため） | 試験 11 ①。要求どおりの構成で評価する場合は `false`（RC-3） |
| `slam.use_as` の既定 | 外部 SLAM の実出力品質 |
| `output.exact_smoothing_adjoint` の既定 | 試験 21 ⑩。**共分散側の輸送は既定で常時 ON（D40・切替なし）** |
| `slam.max_speed_mps` / `max_yaw_rate_rps` / `jump_margin_*` | 車両仕様。主判定は χ² なのでチューニング不要（D38） |
| `slam.interp_sigma_per_s` | 試験 20 の一致精度 |
| `initialization.gnss_yaw_source` の運用既定 | 試験 24 |
| `graph1_mutex_` の保持時間 | 試験 22 ②。p99 > 2 ms なら atomic swap 方式へ（D36） |
| IMU ノイズ値 | 実機 Allan 分散で置換 |
| `graph2_lag_ms` の最終値 | 試験 4・10 |
| `graph2.marginalization` の既定 | 試験 3 |
| `wheel_speed.graph2_binding` の既定 | 試験 11 ③ |
| `reset.smoothing_max_*_rate` の最終値 | 試験 21（下流の反応で決定） |
| `graph2.optimizer: isam2` の incremental 化 | **Phase 2 以降の検討事項**（現設計は毎 tick 再構築・D24） |

---

## 付録 A. 実装順序の依存グラフ（要約）

```
Phase 0 ─┬─ fusion_plugin_base（P1: API を最初に確定）
         ├─ covariance_convert ──────────────┐
         ├─ OutputStage（smoothing・輸送）────┤ 全モードが依存
         ├─ OutputPublisher ─────────────────┤
         └─ param_validator ─────────────────┘
             │
             ├─→ Phase 1（EKF）  … 互換性の数値基準を確立
             │
             └─→ Phase 2a（バッファ・初期化）
                     └→ 2b（ファクタ・ゲート）
                             └→ 2c（2 段グラフ・アンカー）  ← 最も重い。INV-1〜5 の assert を先に置く
                                     └→ 2d（異常系・数値検証）
                                             └→ Phase 3（LIO-SAM）→ Phase 4（統合・要求適合性）
```

- **Phase 0 で `OutputStage` を完成させる**のが本計画の要点である（D43）。
  smoothing はモード非依存かつ数学的に細かい（輸送作用素の取り違えが起きやすい）ので、
  偽の `InternalEstimate` を与える単体試験で先に固めておくと、Phase 2c で「アンカー更新の配線」だけに集中できる。
- **2c に入る前に INV-1〜5 の assert を書く**（P4）。二重計上・情報消失は後から探すのが極めて難しい。

## 付録 B. `implementation_plan.md`（rev.6）からの変更点

### B.1 コンポーネントの新設・移動

| # | 変更 | 理由 |
| --- | --- | --- |
| B-1 | **`smoothing_state` をプラグイン配下（`factor_graph_fusion/`）からノード直下へ移動**し、`output_stage.{hpp,cpp}` を新設。`anchor_manager` から smoothing の責務を除去 | D43（プラグインは internal のみ返す）。旧計画では smoothing が自作モードのプラグイン内にあり、(a) EKF / LIO-SAM モードで smoothing が効かない、(b) プラグイン IF の `estimate() const` と矛盾する、という 2 つの問題があった |
| B-2 | **`param_validator.{hpp,cpp}` と `test_param_validator.cpp` を新設**（Phase 0・マージ必須） | design §6.2 の検証は 25 行以上あり、うち数行は**起動失敗**（廃止パラメータ・`max_anchor_jump_*`）である。旧計画ではノード実装の一部としか書かれておらず、テスト対象になっていなかった |
| B-3 | **`test_output_stage.cpp` を新設**し、試験 21 を Phase 0（単体）と Phase 2c（走行条件）に分割 | B-1 の結果、smoothing が Phase 0 で完成・単体試験可能になった。輸送作用素の取り違え（試験 21 ⑪⑫）を最も早い段階で潰せる |
| B-4 | `vehicle_twist_buffer` に**層 A 棄却フラグの保持**を追加 | D42（車速は層 A のみ適用、棄却サンプルは Graph1・Graph2 とも不使用、replay でも同一再現） |
| B-5 | `graph1_propagator` に**車速の層 A ゲート**を追加 | 同上（D42） |
| B-6 | `covariance_convert` に **twist 用の `Πᵀ · M · Π` API** を明示 | design §5.3.5 の記述の向きが逆になっていた箇所を修正したため、実装でも別 API として分離し試験 12 で検証する |

### B.2 パラメータ・既定値の追随

| # | 変更 | 反映先 |
| --- | --- | --- |
| B-7 | `graph2.boundary_merge_yaw_rate_rps`（既定 1.0）を新設 | `config/*.param.yaml`、schema、`param_validator`、`graph2_window`（D44） |
| B-8 | `wheel_speed.timeout_ms`（既定 200）を新設 | `config/*.param.yaml`、schema、`param_validator`、Phase 2d の timeout 監視（D44） |
| B-9 | `max_anchor_jump_m/rad` の既定を **0.05/0.005 → 0.03/0.003**、`smoothing_done_m/rad` を **0.01/0.001 → 0.005/0.0005** | `config/*.param.yaml`、schema、試験 3・21（D45） |
| B-10 | §6.2 の `max_anchor_jump_*` 検証を「WARN」から**「`< 0.05` を満たさなければ起動失敗」**へ格上げ | `param_validator`、`test_param_validator`（D45） |

### B.3 試験計画の修正

| # | 変更 |
| --- | --- |
| B-11 | 試験 21 の注入量を **0.01 / 0.1 / 1 / 5 m** に変更（旧計画が参照していた design rev.6 の試験 21 は ①「0.1 m は即時適用」と ⑬「0.1 m は必ず smoothing」が矛盾していた）。⑬ は `OutputStage` の所有検証に差し替え |
| B-12 | 試験 18 に**車速の層 A 棄却判定の replay 再現**を追加（D42） |
| B-13 | 試験 20 に **INV-5 例外経路の破棄件数が診断カウンタに現れること**（⑧）を追加 |
| B-14 | 試験 12 に **twist の `[ω;ν] → [ν;ω]` 並べ替え**の成分単位検証を追加 |
| B-15 | 試験 7 の車速 timeout の根拠を `wheel_speed.timeout_ms` と明記（旧計画では根拠パラメータが存在しなかった） |
| B-16 | マージ必須の範囲を **1・2・3・12・17・18・20・21・22・24 +（`slam.use: true` 時）23** に整理。`test_param_validator` も必須に追加 |
| B-17 | **§5 要求適合性チェックリスト（RC-1〜RC-8）を新設**。特に RC-3（`slam.use: false` での回帰）・RC-5（Graph1 の解が鎖状グラフのバッチ解と一致）・RC-7（固定ラグ窓の妥当性）は、要求からの逸脱・拡張を数値で正当化するための試験である |
| B-18 | CI 常設ジョブを 3 本に明記（TSan / debug assert 付き試験 20 / `slam.use: false` 回帰） |

### B.4 削除したもの

- 冒頭の改訂履歴（rev.2〜rev.6）と、コンポーネント一覧の ★☆◇◆（版マーカ）。
  → 版を跨いだ差分は本付録に集約し、本文は D-ID 参照のみとした。
- §5「確定事項と残課題」の Q 表（Q1〜Q51 / rev.2・rev.3・rev.6 の 3 テーブル）。
  → すべて `design_last.md` §8 の D1〜D45 に反映済み。実装への影響は本書の §3 各フェーズに直接書き下した。
