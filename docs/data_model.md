# データモデル / データ辞書

スクリプトが使うトピックとメッセージ定義のまとめ。**メッセージ定義（IDL）の正本は
`t2-auto/Yatagarasu` リポジトリの `src/interfaces/**` にある**。新規セッションでスキーマを確認/変更する場合はまず下記 IDL を読むこと。

## メッセージ定義（IDL）の取得手順

このリポジトリには IDL は含まれない。Claude Code セッションでは:

1. `add_repo` ツールで `t2-auto/Yatagarasu` を追加（owner=`t2-auto`, repo=`Yatagarasu`）。
2. クローン後、`src/interfaces/**/*.idl` を読む。主な所在:

| メッセージ | IDL パス（Yatagarasu 内） |
|---|---|
| `AugmentedScene` | `src/interfaces/perception_msgs/msg/AugmentedScene.idl` |
| `AugmentedObject` | `src/interfaces/perception_msgs/msg/AugmentedObject.idl` |
| `TrackedBbox` | `src/interfaces/perception_msgs/msg/TrackedBbox.idl` |
| `ObjectTrackingInfo` | `src/interfaces/perception_msgs/msg/ObjectTrackingInfo.idl` |
| `LocalMapInfo` | `src/interfaces/perception_msgs/msg/LocalMapInfo.idl` |
| `LocalLaneSegment` | `src/interfaces/perception_msgs/msg/LocalLaneSegment.idl` |
| `LocalLaneBoundary` | `src/interfaces/perception_msgs/msg/LocalLaneBoundary.idl` |
| `LocalLaneLocationContext` | `src/interfaces/perception_msgs/msg/LocalLaneLocationContext.idl` |
| 配列容量定数 | `src/interfaces/perception_msgs/msg/SequenceSizeDefine.idl` |
| `EgoPose` | `src/interfaces/odometry_msgs/msg/EgoPose.idl` |
| `Header` | `src/interfaces/common_msgs/msg/Header.idl` |
| `Point32`/`Point`/`Vector3` | `src/interfaces/geometry_msgs/msg/*.idl` |

## トピック一覧

| トピック | 型 | 使うスクリプト |
|---|---|---|
| `/t2/object_augmentor/augmented_scene` | `perception_msgs::msg::AugmentedScene` | target_object, lane_boundary_tracker, ego_lane_segments, lane_segment_traversal_time, follower_gap_tracker, speed_margin_profile |
| `/t2/odometry/ego` | `odometry_msgs::msg::EgoPose` | lane_segment_traversal_time(任意), follower_gap_tracker(任意), lateral_g_monitor, speed_margin_profile(任意) |
| `/t2/bev_detection/objects` | （legacy） | target_object / lane_boundary_tracker が width 取得に使用。track_id 方式移行で実質不要 |
| `/t2/resource_monitor/raw` | resource monitor | high_memory_processes |

## AugmentedScene（`/t2/object_augmentor/augmented_scene`）

```
AugmentedScene
├─ header                         common_msgs::msg::Header  (※ stamp 無し。下記注意)
├─ augmented_objects[]            AugmentedObject
│   ├─ bbox_info                  TrackedBbox
│   │   ├─ id (int32)             物体ID
│   │   ├─ position (Point)       世界座標
│   │   ├─ local_position (Point) 車両座標系 (x=前方, y=左, z=上)
│   │   ├─ local_theta (double)
│   │   ├─ length / width / height (double)
│   │   ├─ type (ClassType) / sub_type
│   │   └─ confidence
│   ├─ tracking_info              ObjectTrackingInfo
│   │   ├─ velocity (Vector3)                 障害物の絶対速度（世界座標）
│   │   ├─ local_relative_velocity (Vector3)  自車に対する相対速度（車両座標系, +x=接近）
│   │   ├─ local_velocity (Vector3)           障害物の絶対速度（車両座標系）
│   │   ├─ track_id (?)                        ※ 実データでは bbox_info.id を使うのが確実
│   │   ├─ is_stationary (bool) / stationary_duration
│   │   └─ acceleration[] / local_acceleration[] / tangential_acceleration
│   └─ augmentor_info             AugmentorInfo
└─ local_map_info                 LocalMapInfo
    ├─ local_lane_segments[]      LocalLaneSegment   (容量 kLocalLaneSegmentCapacity=500)
    ├─ ego_lane_segment_indices[] uint64   local_lane_segments への index（自車が乗る区間, 容量20）
    └─ local_stop_lines[]         LocalStopLine
```

### LocalLaneSegment（自車レーン解析の中心）

| フィールド | 型 | 意味 |
|---|---|---|
| `id` | string<64> | セグメントID。**HD マップ由来でフレーム間で安定**（通過時間計測の前提） |
| `road_id` | string<64> | 道路ID |
| `successor_ids[]` | string<64>×15 | 退出側（直後）セグメントID |
| `predecessor_ids[]` | string<64>×15 | 進入側（直前）セグメントID |
| `left_neighbor_forward_id[]` | string<64>×1 | 左隣（同方向） |
| `right_neighbor_forward_id[]` | string<64>×1 | 右隣（同方向） |
| `central_curve[]` | Point32×180 | 中心線（車両座標系の点列）。**弧長=SL距離、始点→終点=直線距離** |
| `length` | float | セグメント長 [m]（提供値。おおむね central_curve の弧長） |
| `left_boundary` / `right_boundary` | LocalLaneBoundary | `.curve[]`（Point32）と `.types[]`, `.is_virtual` |
| `nth_lane` | uint8 | 第何レーンか |
| `speed_limit_max` / `speed_limit_min` | float | 制限速度 **[m/s]**（`*_map_debug` / `*_vlm_debug` は内訳） |
| `target_speed_max` / `target_speed_min` | float | 目標速度 [m/s] |
| `is_target_lane` | bool | 推奨走行レーン |
| `is_tollgate` | bool | **料金所（ETC）区間** |
| `is_tunnel` / `is_no_lane` / `is_out_of_course` / `is_slower_traffic` / `is_truck_lane` / `is_merging_anticipation` / `is_blockade_lane` | bool | 各種属性 |
| `lane_classification` | enum | `LocalLaneClassificationType` |
| `split_merge_type` | enum | `LocalSplitMergeType` |
| `left_lc_permission` / `right_lc_permission` | enum | `LocalLaneChangePermissionType` |
| `location_context` | LocalLaneLocationContext | `.location`, `.context`（**ETC 区間判別に有用**） |

## EgoPose（`/t2/odometry/ego`）

| フィールド | 型 | 意味 |
|---|---|---|
| `position` | Point | VRP（車両基準点）の odometry 座標（x,y。z 未計算） |
| `linear_velocity` | Vector3 | odometry 座標系の速度 |
| `local_linear_velocity` | Vector3 | **車両座標系の速度**（x=前方）。速度は `sqrt(x²+y²)` |
| `linear_acceleration` / `local_linear_acceleration` | Vector3 | 加速度（`local_*.y`=**横加速度**, `.x`=前後） |
| `angular_velocity` / `local_angular_velocity` | Vector3 | 角速度（`.z`=**ヨーレート** [rad/s]） |
| `roll` / `pitch` / `yaw` | double | 姿勢角 [rad] |

## enum テーブル（整数値 → 名前）

`LocalLaneContext`（ETC 判別に重要）:
`0 UNKNOWN, 1 HIGHWAY, 2 ENTERING_ETC, 3 PASSING_ETC, 4 EXITING_ETC, 5 URBAN_ROADS, 6 SPECIAL1, 7 BASE, 8 DEPARTURE_SPOT, 9 ARRIVAL_SPOT`

`LocalLaneLocation`:
`0 UNKNOWN, 1 GENERIC, 2 KOBE_NISHI_IC_NOBORI_ENTRANCE, 3 KOBE_NISHI_IC_KUDARI_EXIT, 4 AYASE_SIC_KUDARI_ENTRANCE, 5 AYASE_SIC_NOBORI_EXIT, 6 NISHINOMIYA_KITA_IC_NOBORI_ENTRANCE, 7 NISHINOMIYA_KITA_IC_KUDARI_EXIT, 8 KOBE_NISHI_BASE, 9 AYASE_BASE, 10 NISHINOMIYA_KITA_BASE`

`LocalLaneClassificationType`:
`0 UNKNOWN, 1 PASSING_LANE, 2 DRIVING_LANE, 3 VEHICLES_STOP_LANE, 4 SIDE_LANE_SPLIT, 5 SIDE_LANE_MERGE, 6 SPLIT, 7 MERGE, 8 ACCELERATION_LANE, 9 DECELERATION_LANE, 10 INCREASE_OF_LANES, 11 DECREASE_OF_LANES, 12 INCREASE_OF_NO_LANES, 13 DECREASE_OF_NO_LANES, 14 NO_LANES, 15 PARKING_SLOT, 16 SIDEWALK`

`LocalSplitMergeType`: `0 UNCHANGED, 1 SPLIT, 2 MERGE, 3 INCREASE, 4 DECREASE, 5 UNKNOWN`

`LocalLaneChangePermissionType`: `0 NO_LANE, 1 ALLOWED, 2 NOT_ALLOWED_REGULATION, 3 NOT_ALLOWED_PHYSICAL, 4 NOT_ALLOWED_BOTH, 5 UNKNOWN`

`LocalLaneBoundaryType`: `0 UNSPECIFIED, 1 DOTTED_YELLOW, 2 DOTTED_WHITE, 3 SOLID_YELLOW, 4 SOLID_WHITE, 5 DOUBLE_YELLOW, 6 CURB`

> enum の順序は IDL に追従する。メンバー追加で値がずれ得るので、名前化テーブルを更新する際は必ず IDL を再確認すること。

## 重要な注意（ハマりどころ）

- **Header に `stamp`（sec/nsec）は無い**。`module_name, sequence_number, creation_timestamp, measurement_timestamp(uint64 ns), lidar/camera_timestamp, frame_id`。タイミングには `event.receiveTime` を使う。
- **`LocalLaneSegment.id` はマップ由来で安定**（`lane().id().id()`）。フレーム間で同一物理区間は同じ id。通過時間計測・latch はこれを前提にしている。
- **速度は m/s**（制限速度含む）。km/h 表示はスクリプト側で `×3.6`。
- **物体速度の3フレーム**: `velocity`=世界絶対 / `local_velocity`=車両系絶対 / `local_relative_velocity`=車両系の対自車相対（後続車の接近判定に使う）。
- **`ego_lane_segment_indices` は index**（`local_lane_segments` への添字）。空になり得る（その場合は原点最近傍でフォールバック）。
- **64bit 整数（uint64/int64）は Foxglove では `bigint` で届く**。`ego_lane_segment_indices`（uint64）が
  代表例。数値化ヘルパ `num()` は `typeof v === "bigint"` を `Number(v)` に変換して扱うこと。怠ると
  全 index が破棄され、自車セグメント特定・通過時間計測が**無言で壊れる**（実データでのみ顕在化）。
- 配列容量: `kLaneCurveCapacity=180`, `kLocalLaneSegmentCapacity=500`, `kEgoLaneSegmentCapacity=20`。
