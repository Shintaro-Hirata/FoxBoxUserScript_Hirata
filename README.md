# FoxBox User Scripts

FoxBox (Foxglove Studio ベース) 向けの User Scripts 集です。

## スクリプト一覧

### 1. `target_object.ts`

**入力トピック**: `/t2/bev_detection/objects`  
**出力トピック**: `/studio_script/target_object`

指定した世界座標に最も近い物体を毎フレーム探して情報を出力します。
トラッカー状態を持たないため Seek/Scrub でも即時に結果が得られます。

**Variables パネルで設定する変数:**

| 変数名 | 型 | 説明 |
|---|---|---|
| `target_x` | number | ターゲットの世界座標 x |
| `target_y` | number | ターゲットの世界座標 y |
| `target_z` | number | ターゲットの世界座標 z |
| `threshold_m` | number | マッチング閾値 [m]（省略時 1.0m） |

**主な出力フィールド:**

| フィールド | 説明 |
|---|---|
| `match_found` | 閾値内の物体が見つかれば true |
| `matched.*` | 閾値内で最も近い物体の全情報 |
| `nearest.*` | 閾値外でもとにかく最近傍の物体 |

---

### 2. `lane_boundary_tracker.ts`

**入力トピック**: `/t2/object_augmentor/augmented_scene`, `/t2/bev_detection/objects`  
**出力トピック**: `/studio_script/lane_boundary_tracker`

指定した世界座標のターゲット物体から、指定レーンセグメントの左境界線（白線）までの距離を算出します。

**Variables パネルで設定する変数:**

| 変数名 | 型 | 説明 |
|---|---|---|
| `target_x` | number | ターゲットの世界座標 x（`target_object` と同じ値） |
| `target_y` | number | 同 y |
| `target_z` | number | 同 z |
| `threshold_m` | number | マッチング閾値 [m]（省略時 1.0m） |
| `lane_segment_id` | string | 追跡するレーンセグメントの id（省略時は自動選択） |
| `curve_point_index` | number | `left_boundary.curve` の配列インデックス（省略可） |

**セグメント自動選択ロジック:**
1. `lane_segment_id` が設定されていれば id で完全一致検索
2. 未設定 & ターゲット取得済み → `central_curve` への最短距離で自動選択
3. 未設定 & ターゲットなし → `is_target_lane=true` を fallback

**主な出力フィールド:**

| フィールド | 説明 |
|---|---|
| `distance_to_left_boundary_m` | ターゲット ↔ 左境界 curve 全体の最短距離 [m] |
| `distance_target_edge_to_selected_y_m` | `target.local_y - selected.y - width/2`（y軸方向の端-点距離） |
| `selected_point` | `curve_point_index` で指定した curve 上の点の座標 |
| `available_segments` | 全 segment の id と距離一覧（id 確認・選択用） |

**Plot パネルでの活用例:**
```
lane_boundary_tracker.distance_to_left_boundary_m
lane_boundary_tracker.distance_target_edge_to_selected_y_m
lane_boundary_tracker.target_object_found
lane_boundary_tracker.segment_found
```

## 使用上の注意

- トークン等の認証情報をスクリプト内に記載しないでください
- FoxBox の User Scripts エディタに貼り付けて使用します
- `Output | undefined` は FoxBox が許可する唯一の union 型です（multi-input スクリプトで使用）
- Seek backward するとモジュール変数がリセットされます。Seek forward は状態が保持されます
