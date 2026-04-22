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

**主な出力フィールド（直交座標系）:**

| フィールド | 説明 |
|---|---|
| `distance_to_left_boundary_m` | ターゲット ↔ 左境界 curve 全体の最短距離 [m] |
| `distance_target_edge_to_selected_y_m` | `target.local_y - selected.y - width/2`（y軸方向の端-点距離） |
| `selected_point` | `curve_point_index` で指定した curve 上の点の座標 |
| `nearest_point_on_left_curve` | polyline 上の最短点（頂点間の補間点）|
| `available_segments` | 全 segment の id と距離一覧（id 確認・選択用） |

**主な出力フィールド（SL 座標系 / Frenet, v6〜）:**

SL の基準は選択 segment の `central_curve`。
- `S`: `central_curve[0]` から polyline 沿いに測った累積弧長 [m]
- `L`: `central_curve` 進行方向から見た符号付き横距離 [m]（左=正 / 右=負）

| フィールド | 説明 |
|---|---|
| `sl_valid` | SL 変換が成立した場合 true |
| `central_curve_total_length_m` | `central_curve` 全体の弧長 [m] |
| `target_s_m` / `target_l_m` | ターゲットの SL 座標 |
| `left_boundary_l_at_target_s_m` | ターゲットと同じ S 位置での左白線 L 値 |
| `distance_target_to_left_boundary_sl_m` | SL 空間での中心→左白線 横距離（符号付き、通常 +） |
| `distance_target_edge_to_left_boundary_sl_m` | 上記から `target.width/2` を引いた端-白線距離 |
| `right_boundary_*` | 右白線も同様（距離は `target_l - right_l` の形で計算） |
| `left_boundary_l_bracket_mode` | 補間区間種別（`interpolated` / `before_start` / `after_end` 等、デバッグ用） |

**Plot パネルでの活用例:**
```
# 直交座標系
lane_boundary_tracker.distance_to_left_boundary_m
lane_boundary_tracker.distance_target_edge_to_selected_y_m

# SL 座標系（停止車両↔白線距離はこちらが推奨）
lane_boundary_tracker.distance_target_edge_to_left_boundary_sl_m
lane_boundary_tracker.distance_target_edge_to_right_boundary_sl_m
lane_boundary_tracker.target_s_m
lane_boundary_tracker.target_l_m

lane_boundary_tracker.target_object_found
lane_boundary_tracker.segment_found
lane_boundary_tracker.sl_valid
```

**SL 変換の注意点:**
- `central_curve` はセグメント単位で定義されるため、セグメントをまたぐと S がリセットされます
- 2D（x-y 平面）で計算しています。勾配のある道路では若干の誤差が出ます
- 左/右白線の点は `central_curve` 上で S が単調増加であることを前提に線形補間しています
- `bracket_mode = before_start / after_end` の場合は target の S 位置が白線の範囲外（外挿相当）なので値の信頼性は落ちます

## 使用上の注意

- トークン等の認証情報をスクリプト内に記載しないでください
- FoxBox の User Scripts エディタに貼り付けて使用します
- `Output | undefined` は FoxBox が許可する唯一の union 型です（multi-input スクリプトで使用）
- Seek backward するとモジュール変数がリセットされます。Seek forward は状態が保持されます
