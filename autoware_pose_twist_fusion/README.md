# autoware_pose_twist_fusion

Pose/twist fusion node that **replaces `autoware_ekf_localizer`** and switches the estimation
algorithm with a single launch parameter.

Design: [`docs/autoware_pose_twist_fusion/design_last.md`](../../../../docs/autoware_pose_twist_fusion/design_last.md) ·
Implementation plan: [`implementation_plan_last.md`](../../../../docs/autoware_pose_twist_fusion/implementation_plan_last.md)

| `fusion_method` | Algorithm | Origin |
| --- | --- | --- |
| `ekf` | Autoware 2D EKF | `autoware_ekf_localizer` (`ekf_module`, vendored) |
| `lio_sam_imu_preintegration` | IMU preintegration + iSAM2 | LIO-SAM `imuPreintegration.cpp` (BSD-3-Clause) |
| `factor_graph_fusion` | Two stage GTSAM factor graph | new (requirements R4-R11) |
| `passthrough` | Forwards the pose_estimator input | wiring verification only |

## Output interface (identical in all modes)

`#2a`, `#2b` and `#3` always carry the same message content.

| # | Node topic | Remapped to | Type |
| --- | --- | --- | --- |
| 1 | `kinematic_state` | `.../pose_twist_fusion_filter/kinematic_state` | `nav_msgs/Odometry` |
| 2a | `pose_with_covariance` | **`/localization/pose_with_covariance`** (absolute) | `PoseWithCovarianceStamped` |
| 2b | `ns_pose_with_covariance` | `.../pose_twist_fusion_filter/pose_with_covariance` | `PoseWithCovarianceStamped` |
| 3 | `pose_with_covariance_no_yawbias` | `.../pose_with_covariance_no_yawbias` | `PoseWithCovarianceStamped` |
| 4 | `biased_pose_with_covariance` | `.../biased_pose_with_covariance` | `PoseWithCovarianceStamped` |
| 5 | `twist_with_covariance` | `.../twist_with_covariance` | `TwistWithCovarianceStamped` |
| 6 | `pose` / `biased_pose` | `.../pose`, `.../biased_pose` | `PoseStamped` |
| 7 | `twist` | `.../twist` | `TwistStamped` |
| 8 | `estimated_yaw_bias` | `.../estimated_yaw_bias` | `Float64Stamped` |
| 9 | `debug/processing_time_ms` | `.../debug/processing_time_ms` | `Float64Stamped` |
| 10 | `debug/covariance_correction/*` | same | `Float64Stamped` |
| 11 | `/diagnostics` | same | `DiagnosticArray` |
| TF | `map -> base_link` | — | (`enable_tf`) |

Yaw bias convention (D1): `pose_with_covariance` and `pose_with_covariance_no_yawbias` are the
**bias free** pose; only `biased_pose_with_covariance` contains the yaw bias. The 6-DoF modes have
no yaw bias state and publish `estimated_yaw_bias = 0.0`.

Only this node may publish `map -> base_link`. Launch a second instance with `enable_tf:=false`
when comparing two methods side by side.

## Usage

```bash
ros2 launch autoware_pose_twist_fusion pose_twist_fusion.launch.xml \
     fusion_method:=factor_graph_fusion
```

To use it inside the stack, replace the `ekf_localizer` include of
`pose_twist_fusion_filter.launch.xml` with this launch file; the remap names are identical.

## Architecture

```
PoseTwistFusionNode                     ... the only node (MultiThreadedExecutor, 4 callback groups)
├─ OutputStage                          ... owns the anchor-jump smoothing (D37 / D43)
├─ OutputPublisher                      ... all topics of §3.1 + TF
├─ param_validator                      ... §6.2, start-up failure vs diagnostics WARN
└─ FusionPlugin (pluginlib)
   ├─ EkfFusionPlugin
   ├─ LioSamImuPreintegrationPlugin
   └─ FactorGraphFusionPlugin
      ├─ Graph1Propagator               ... 20 ms analytic propagation + wheel speed EKF update
      └─ AnchorManager (worker thread)  ... 500 ms fixed-lag window: build -> optimize ->
                                            marginalize -> prune -> anchor -> Graph1 replay
```

Three properties are load bearing and are covered by tests:

* **Plugins return the internal estimate only** (D43). Smoothing lives in `OutputStage`, which is
  why `estimate_internal()` is `const` and why gates/NIS can never see a smoothed pose.
  `PublishedEstimate` has no public constructor, so it can only be derived from an
  `InternalEstimate` plus the offset `c` (INV-4).
* **Normal reception and replay share one code path** (`Graph1::apply_imu` / `apply_wheel_speed`,
  D15). `test_replay_equivalence` compares continuous propagation against an anchor update plus
  replay, and also fails if the wheel speed replay is removed.
* **No double counting at the window boundary** (INV-1/2/5). `t_next_begin` is always a node, the
  boundary node never receives a unary measurement, the prune decides on the assigned node index,
  and a measurement that never became a factor is never pruned.

## factor_graph_fusion in one paragraph

Graph1 (20 ms) is the chain "IMU preintegration + wheel speed / non-holonomic unary factors".
For a chain the MAP marginal of the newest node equals the forward recursion, so it is implemented
as analytic propagation plus a one-step EKF update instead of a 50 Hz graph optimization
(intentional deviation from R7, design §0.1 (1)). Graph2 (500 ms) keeps an overlapping fixed-lag
window of `graph2_lag_ms`, rebuilds it from scratch every tick - which makes the result independent
of measurement arrival order - marginalizes everything older than the next boundary into one linear
factor, and hands the optimized `X*, V*, B*, Σ*` (joint 15×15) to Graph1, which resets and replays
the buffered IMU and wheel speed (intentional deviation from R10, design §0.1 (2)).

Absolute pose inputs (GNSS / NDT / SLAM) pass the covariance correction of requirement R11:
layer A is a χ² gate against the Graph1 prediction at the measurement time, layer B scales the
measurement covariance by `1/w` with `w = exp(-½ max(0, d²-dof))`, and layer C (off by default) adds
a robust kernel. `w` is an explicit heuristic (D22), it is frozen at arrival (INV-3), and wheel
speed gets layer A only (D42).

## Parameters

`config/pose_twist_fusion.param.yaml` holds every parameter with the design defaults; the node
starts and converges with those alone. `schema/pose_twist_fusion.schema.json` documents ranges and
enums. Validation (design §6.2) runs at start-up: violations throw instead of letting the node run
in a silently broken configuration, and retired parameters (`slam.max_relative_jump_m`,
`slam.max_relative_jump_rad`, `graph2.force_boundary_node`) fail the start-up rather than being
ignored.

## Tests

```bash
colcon test --packages-select autoware_pose_twist_fusion
```

| File | Design test | Subject |
| --- | --- | --- |
| `test_output_interface.launch.py` | 1 | all output topics / types / service exist |
| `test_covariance_convert.cpp` | 12 | ROS ↔ GTSAM round trip, twist `Πᵀ M Π` |
| `test_param_validator.cpp` | — | every row of §6.2 |
| `test_output_stage.cpp` | 21 | smoothing, transport operators vs Monte Carlo |
| `test_wheel_speed_factor.cpp` | 14 | wheel speed / non-holonomic Jacobians |
| `test_gnss_lever_arm_factor.cpp` | 14 | lever arm factor, `r_bg = 0` equals `GPSFactor` |
| `test_graph2_window.cpp` | 17, D30, D34 | node timeline, wheel binding, boundary merge |
| `test_measurement_history.cpp` | 20 | persistence, INV-1 / INV-3 / INV-5 |
| `test_replay_equivalence.cpp` | 18 | anchor update + replay equivalence, layer A replay |
| `test_anchor_manager.cpp` | 3, 20 | full Graph2 tick, INV-2 |
| `test_state_history.cpp` | 4 | delayed measurement interpolation |
| `test_covariance_corrector.cpp` | 5 | NIS based gating with the design's numbers |
| `test_initial_bias.cpp` | 19 | `b_a = ā - R_wbᵀ(0,0,g)` gravity convention |
| `test_initializer.cpp` | 15, 24 | initialization state machine, GNSS yaw sources |
| `test_slam_pair_gate.cpp` | 23 | SLAM relative constraints, jump detection |
| `test_covariance_health.cpp` | 9 | joint 15×15 propagation vs Monte Carlo |

Not implemented here (they need rosbags or a running stack): tests 2, 6, 7, 8, 10, 11, 13, 16 and
the TSan CI job of §11.

## Licenses

Apache-2.0, except `src/plugins/lio_sam/*` which is derived from LIO-SAM (BSD-3-Clause) and
`src/plugins/ekf/vendored/*` which is copied from `autoware_ekf_localizer` (Apache-2.0). The
vendored EKF sources differ from the originals only in the namespace, the include paths and the
header guards - the algorithm is untouched (design §5.1).
