# Shared core and native ROS wrappers — 2026-10-27

## Scope and baseline

The source baseline is `1b3c0b4ae828d4b2e31c438fb64f405ffd4de942` on the
fork's `humble` branch. The previous `main` (`7a7569e`) is its ancestor and
contained the ROS 1 implementation. The stable-window initialization and gravity
repair previously existed only in Humble. The new local `main` uses that repaired
estimator for both native adapters.

This is a structural change. Registration equations, point filters, voxel sizes,
initialization acceptance criteria and matching thresholds were retained.
Original installed programs and navigation frozen libraries were not replaced.

## Design

`fast_lio_core` owns one mapper's queues, preprocessing, IMU propagation, filter,
ikd-Tree, map insertion and optional PCD/log output. There is no ROS header or
linked ROS library in the core. The SDK exports `fast_lio::fast_lio_core`; an
installed consumer only needs its public headers, Eigen and PCL common. The filter,
IMU and ikd-Tree headers are absent from both node compilation dependency graphs.

The 1,265-line Humble node was replaced by native nodes of 170 lines (Humble) and
158 lines (Noetic). They bind parameters, convert messages, publish outputs and
handle lifecycle. The parameter catalog and message conversion helpers are shared.
ROS topic/frame names stay outside the core configuration. Both packages retain
`fast_lio` and their native launch/message interfaces; the selector prevents
workspace discovery from treating both packages as one build.

Each mapper has one serial owner. Independent instances passed an interleaved
motion test against isolated references. Callbacks are synchronous: IMU prediction
and scan correction are emitted at their original processing stages. The observer
must not re-enter the mapper or retain references to callback arguments. Returned
clouds own immutable snapshots. No global filter pointer or global thread limiter
was introduced.

The old source-root `config` path remains a symlink to `humble/config`. Its
MID360 YAML is byte-identical to the baseline, preserving existing frozen startup
commands. Native ROS 1 configuration is under `noetic/config`. The MID360 cube/blind
settings include the existing Humble fixes. Other native defaults are retained,
including Noetic Velodyne's enabled PCD saving (Humble's example disables it).

The SDK's `pcd_directory` selects an output directory. Native wrappers preserve
the original source-root `PCD` behavior. Previously unused ROS parameters
`map_file_path`, `filter_size_corner` and `mapping.fov_degree` remain inert and
are not part of the core's active configuration. PCD saving is independent of ROS
publication. The old unconditional IMU/debug file writes and oversized fixed
scratch clouds are removed; optional runtime diagnostics use a bounded asynchronous
writer. Its single file stops growing at 100,000,000 bytes across restarts.

## Verification

All builds and runtime comparisons were restricted to CPUs `0,2,4,6`. Release
uses CMake's normal optimization flags and ccache. A separate RelWithDebInfo
configuration generates `-O2 -g`, without an unconditional `-O3` or `-march=native`.
The development configuration was inspected; the runtime comparisons use Release.

- Standalone core, Humble and native Noetic each pass 3/3 CTest targets, containing
  11 cases. They cover initialization motion/gaps/units/bias, the committed filter
  state, independent moving instances, invalid-startup recovery, PCD readback and
  bounded log output across two instance lifetimes.
- Installed C++ SDK consumers configure, build and run on Jammy and Focal using
  `project(... LANGUAGES CXX)` and `find_package(fast_lio_core)`. No ROS API is used.
- Native short-motion E2E tests cover Livox CustomMsg and Velodyne, Ouster and
  MARSIM PointCloud2 layouts. Each emits 58 mapped frames and 1,180 IMU predictions.
  All four Humble cases match the baseline numerically, including all timestamps,
  covariance, TF, gravity and the eight semantic point fields. Noetic's maximum
  short-fixture position difference is below `1.2e-18 m`.
- The complete recorded-input comparison uses 171 seconds of MID360/IMU data:
  1,710 scans, 34,215 IMU samples and 12,785,462 input points. Both nodes consume all
  scans and emit 1,692 mapped frames, 33,860 IMU predictions, matching cloud/TF/
  gravity counts, and exit normally. Initialization-only frames are intentionally
  absent from the mapped output, as in the baseline.

The recordings are from the existing SpRobo continuation experiment. Only sensor
fields consumed by FAST-LIO were exported, preserving their values and record
order. Ground truth was not passed to the estimator. Exported inputs omit unused
message metadata, so this is not a claim of byte-identical ROS 1/ROS 2 wire data.
Replay uses the original MID360 calibration, gravity `9.80665`, 0.1 m surface/map
leaves, point filter 2, four iterations, cube side 500 m and detection range 100 m.
PCD and runtime logging are disabled for the resource comparison. Humble uses
Fast DDS with a 64 MiB shared-memory segment; Noetic uses native ROS 1 transport.

### Final full-route results

The two baseline repetitions produce identical numerical outputs. The final
Humble candidate also matches every recorded numerical field exactly: scan/IMU
odometry, covariance, TF, gravity and all eight body-cloud fields, without
trajectory alignment. The native Noetic candidate preserves counts and timestamps;
its maximum scan-pose difference from Humble is 1.822 mm, IMU-pose difference
2.039 mm, scan-angle difference 0.006102 degrees and body-point difference
0.3324 mm. Its strict 1 mm / 0.0001 rad cross-environment comparison **does not
pass**. This is a bounded difference relative to the baseline, not measured error
against ground truth. No runtime acceptance threshold was changed.

| Run, four available CPUs, 2× replay | Node CPU seconds | Mean cores | Peak RSS MiB | Ack p95 / max ms |
| --- | ---: | ---: | ---: | ---: |
| Original Humble, repetition 1 | 24.06 | 0.2780 | 266.54 | 18.62 / 27.29 |
| Original Humble, repetition 2 | 24.94 | 0.2882 | 268.13 | 20.15 / 28.30 |
| Final Humble | 20.13 | 0.2326 | 242.11 | 17.65 / 25.39 |
| Final native Noetic | 20.13 | 0.2326 | 162.10 | 9.17 / 30.49 |

Humble uses about 16–19% less node CPU time and 9–10% less peak RSS than the two
baseline runs on this fixture. This is not a general hardware performance claim.
Noetic has different transport, compiler and PCL/Eigen versions, so its resource
numbers are not an isolated algorithm speedup comparison. Ack latency includes
transport, the replay probe and scheduling; it is not registration duration.
The measurement excludes the separate replay/observer processes.

The native launch files retain the same MID360 point-filter/iteration/voxel
settings. Noetic's source MID360 YAML also receives Humble's existing 500 m cube
and 0.2 m blind-range settings; the launch files override the cube to 1,000 m in
both versions. The measured replay uses the explicit configuration listed above.


## Boundary and ablation findings

The first output-snapshot prototype delayed IMU messages until registration ended.
Replacing that path with the synchronous observer restores the original output
stages. A second conversion of IMU time through `double` also introduced ten 1 ns
changes in the short fixture; integer nanosecond propagation removed them.

Noetic's original CPU-dependent matching default is retained: two workers with
four visible x86 cores. A one-worker control lowered CPU/RAM but did not remove
the cross-environment trajectory difference. An isolated Eigen 3.4 control also
did not eliminate it. Neither experimental configuration replaces the native
candidate.

A frozen real point cloud demonstrates one concrete dependency difference:
PCL 1.10 sorts voxel entries with `std::sort`, while PCL 1.12 uses integer
spreadsort. Both produce 2,531 centroids, but 184 fields differ, with maximum
XYZ displacement `3.5762786865234375e-7 m`. In an isolated PCL 1.10 probe, replacing
only the sorting mechanism (and the default constructor required by that
mechanism) makes all fields identical to PCL 1.12. This proves a source of
cross-version floating-point differences; it does not prove it is the sole source
of every trajectory discrepancy. System PCL and production filtering code were
not patched or upgraded.

Noetic initially read an older installed SDK header before its source header.
Both native builds now put their own public headers first. This matters when a
Livox dependency shares an install prefix with an earlier FAST-LIO SDK. Focal's
CTest 3.16 must be run from the test directory: newer `--test-dir` syntax can yield
"No tests were found" without running the intended suite.

## Remaining limits

Hardware capture, the live Livox SDK/driver chain, ARM deployment and long-duration
resource use were not qualified here. Noetic E2E uses generated messages matching
the official ROS 1 Livox schema, not a running hardware driver. Rotating-LiDAR
checks use synthetic motion and do not replace real-device or feature-mode tests.
The comparison establishes behavior relative to the repaired frontend; it is not
an absolute 10 cm accuracy acceptance test.

The existing one-frame lag in published scan covariance and the IMU prediction's
interval-head timestamp semantics are retained. They were not silently changed
as part of this refactor. The strict cross-environment 1 mm comparison is reported
separately from successful transport/processing E2E; its gate is not relaxed to
hide a dependency difference.

Local evidence is under the workspace's `build/fast_lio/core_wrappers_20261027`:
`before.json`, `protected_after.json`, the three CTest logs, SDK consumer results,
format comparisons, full replay outputs and `pcl_sort/comparison.json`.
`handoff_manifest.json` records the committed revision, binary hashes and selected
builds. Experimental alternatives remain identified separately. Publication and
robot installation are separate actions after review.
