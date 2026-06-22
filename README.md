# FoxBox User Scripts

FoxBox (Foxglove Studio ベース) 向けの User Scripts 集です。

## スクリプト一覧

### 1. `target_object.ts`

**入力トピック**: `/t2/object_augmentor/augmented_scene`
**出力トピック**: `/studio_script/target_object`

`augmented_objects` から `track_id` で指定した物体を毎フレーム探して情報を出力します。`track_id` はフレーム間で安定しているため、位置ベースのマッチングと比べて誤認識のリスクがありません。

**Variables パネルで設定する変数:**

| 変数名 | 型 | 説明 |
|---|---|---|
| `track_id` | number | 追跡する物体の track_id（`bbox_info.id`）（必須） |
| `target_type` | number | 物体タイプで絞り込み（省略可） |
| `target_sub_type` | number | サブタイプで絞り込み（省略可） |
| `velocity_max_mps` | number | 速度上限 [m/s]（停止車両検索時に設定。省略時フィルタなし） |

**track_id の確認方法:**
Raw Messages で `/t2/object_augmentor/augmented_scene` → `augmented_objects` を展開し、追跡したい物体の `bbox_info.id`（または `tracking_info.track_id`）を確認。

**主な出力フィールド:**

| フィールド | 説明 |
|---|---|
| `match_found` | track_id が見つかれば true |
| `matched.track_id` | マッチした物体の track_id |
| `matched.local_position` | 車両フレーム内位置 |
| `matched.length / width / height` | 寸法 |
| `matched.velocity` | 速度ベクトル（tracking_info から） |
| `matched.velocity_norm` | 速度のノルム [m/s] |
| `matched.type / sub_type` | 物体タイプ |

---

### 2. `lane_boundary_tracker.ts`

**入力トピック**: `/t2/object_augmentor/augmented_scene`（単一トピック）
**出力トピック**: `/studio_script/lane_boundary_tracker`

`track_id` で指定したターゲット物体から、レーン境界線（白線）までの距離を、直交座標系および SL 座標系（Frenet フレーム）で算出します。

v15 で `/t2/bev_detection/objects` を廃止し、単一トピック化。ターゲットは `augmented_objects` から `track_id` で直接検索するため、**Seek / コマ送り時のターゲット消失問題が解消**されました。

SL 計算の `central_curve` は、`local_lane_segments` の `successor_ids` / `predecessor_ids` を辿って**自車レーンのセグメントを ID ベースで自動結合**し、単一セグメントの長さ制限（≈86m）を超えて **100m 以上の距離**にも対応します。

**Variables パネルで設定する変数:**

| 変数名 | 型 | 説明 |
|---|---|---|
| `track_id` | number | 追跡する物体の track_id（`target_object` と同じ値） |

**セグメント自動選択ロジック（直交座標系の距離計算用）:**
1. ターゲット取得済み → `central_curve` への最短距離で自動選択
2. ターゲットなし → `is_target_lane=true` を fallback

---

## 出力フィールド一覧（lane_boundary_tracker）

### 直交座標系（Cartesian）

結合済み boundary + 2D 距離で計算。SL とほぼ同値の参考値。

| フィールド | 説明 |
|---|---|
| `distance_to_left_boundary_m` | ターゲット中心 ↔ 左境界の最短 2D 距離 [m] |
| `distance_target_edge_to_left_boundary_m` | 上記 − `width/2`（車両端→白線距離）|
| `distance_to_right_boundary_m` | ターゲット中心 ↔ 右境界の最短 2D 距離 [m] |
| `distance_target_edge_to_right_boundary_m` | 上記 − `width/2` |
| `nearest_point_on_left_curve` | left_boundary polyline 上の最短点（頂点間の補間点） |
| `available_segments` | 全 segment の id と距離一覧（id 確認・選択用） |

### SL 座標系（Frenet フレーム）

`successor_ids` / `predecessor_ids` による ID ベース結合で長い polyline を構築。100m+ の距離に対応。

| フィールド | 説明 |
|---|---|
| `sl_valid` | SL 変換が成立した場合 true |
| `sl_central_curve_source` | 使用中のソース: `"chained_id"` / `"single_segment"` |
| `chained_segment_count` | ID ベース結合時に繋いだセグメント数 |
| `central_curve_total_length_m` | 使用中 `central_curve` 全体の弧長 [m] |
| `central_curve_point_count` | 使用中 `central_curve` の頂点数 |
| `ego_closest_curve_index` | S=0 とした `central_curve` のセグメント番号 |
| `target_s_m` | ターゲットの S 座標。S=0 は自車位置。**正=前方、負=後方** [m] |
| `target_s_chord_m` | ego投影点〜target投影点の直線距離（S と比較検証用） |
| `target_l_m` | ターゲットの L 座標（符号付き横距離）[m] |
| `target_central_projection` | ターゲットを central_curve に投影した点の座標 |
| `target_central_segment_index` | 投影先の polyline セグメント番号 |
| `left_boundary_l_at_target_s_m` | ターゲットと同じ S 位置での左白線 L 値（補間） |
| `distance_target_to_left_boundary_sl_m` | `target_l − left_l`（中心→白線の横距離）|
| `distance_target_edge_to_left_boundary_sl_m` | 上記 − `width/2`（**車両端→白線距離**）|
| `right_boundary_l_at_target_s_m` | 右白線の補間 L 値 |
| `distance_target_to_right_boundary_sl_m` | `target_l − right_l` |
| `distance_target_edge_to_right_boundary_sl_m` | 上記 − `width/2` |
| `left_boundary_l_bracket_mode` | 補間区間種別（デバッグ用） |

---

## SL 座標系（Frenet フレーム）の計算理論

### 概要

SL 座標系は、道路の中心線（`central_curve`）を基準にした**曲線座標系**です。直交座標系（x, y, z）では道路形状に依存して距離の解釈が変わりますが、SL 座標系では道路に沿った位置（S）と道路からの横ずれ（L）で表現できるため、車両と白線の距離を直感的に扱えます。

```
          L (横方向)
          ↑
          |
          |    ● ターゲット (S=+50, L=1.34)
          |
  --------+---------- central_curve (L=0)
          |
   ← S<0  |  S>0 →
          S=0
        (自車最近傍)
```

- **S (Station / Longitudinal)**: 自車（車両フレーム原点）に最も近い `central_curve` 上の**投影点**を **S=0** とし、polyline に沿って測った**符号付き弧長** [m]。正=前方、負=後方。
- **L (Lateral)**: ターゲットの **central_curve 上の投影点**からターゲットまの**符号付き垂直距離** [m]

> この S 原点規約は社内 Python 版 `vehicle_coord_to_sl` と同一です。`target_s_m ≈ 50` ならターゲットは道路沿いに約 50m 先にいます。

### ステップ 0: ターゲットの検索（v15〜 track_id 方式）

`augmented_scene` の `augmented_objects` から `track_id`（`bbox_info.id` 等）で直接検索します。

```
Variables: track_id = 4
augmented_objects[0].bbox_info.id = 7  → skip
augmented_objects[1].bbox_info.id = 4  → match! → local_position, width を取得
```

v14 以前は `/t2/bev_detection/objects` の世界座標 (position) でマッチングしていましたが、以下の問題があったため track_id 方式に移行しました:
- 類似物体（同じ type/size のトラック等）が同じ位置閾値内に存在すると誤認識
- 2 トピック間のメッセージ到着順序により Seek 後にターゲットが消失
- 座標フレーム不一致（bev_detection と augmented_scene のタイムスタンプずれ）

track_id 方式ではこれらの問題がすべて解消されます。

### ステップ 1: 複数セグメントの ID ベース結合

`local_lane_segments` の各セグメントは `central_curve` が約 86m と限られています。100m 以上離れたターゲットの SL を計算するため、`successor_ids` / `predecessor_ids` を辿って**自車レーンのセグメントを自動結合**します。

```
(後方)                 ego segment              (前方)
seg_C ← seg_B ← seg_A ← [ego (+x固定)] → seg_D → seg_E
   (predecessor_ids で辿る)        (successor_ids で辿る)
```

**結合アルゴリズム（`chainLaneBidir`）:**

1. **ego segment の特定**: `central_curve` 上の点が車両原点 (0, 0) に最も近いセグメント
2. **+x 方向に固定**: ego segment の curve を常に +x 方向（車両前方）に orient。逆方向の場合は curve を反転し、`left_boundary` と `right_boundary` を交換
3. **前方結合**: `successor_ids`（curve 反転時は `predecessor_ids`）を辿って次のセグメントの id を取得し、その segment の curve を append。最大 30 回繰り返し
4. **後方結合**: `predecessor_ids`（curve 反転時は `successor_ids`）を辿って前のセグメントを prepend
5. **ID がない場合は結合停止**: endpoint 近接による推測はせず、他車線を拾うリスクを排除
6. **近接点除去**: 結合部で 0.01m 以内の重複点のみ除去（S 精度を維持）
7. 結合された polyline を `central_curve` / `left_boundary` / `right_boundary` として SL 計算に使用

`successor_ids` / `predecessor_ids` は lane graph の接続情報であるため、他車線のセグメントを拾うリスクはありません。

**フォールバック**: `successor_ids` が存在しないセグメントでは結合が停止し、ego segment の `central_curve` のみを使用（≈86m、≈40 点）。S 範囲は狭いものの距離計算自体は正しく行えます。

> **ポイント**: ego segment の向きは**ターゲット位置に依存しない**（常に +x 方向）ため、ターゲットが前方でも後方でも S の符号が一貫します。

### ステップ 2: S 値の計算（自車基準の累積弧長）

結合済みの `central_curve` の頂点配列 `[P₀, P₁, P₂, ..., Pₙ]` に対して S を計算します。

#### 2-a. 標準累積弧長の計算

まず `curve[0]` を起点にした標準累積弧長を求めます:

```
raw[0] = 0
raw[i] = raw[i-1] + √((Pᵢ.x - Pᵢ₋₁.x)² + (Pᵢ.y - Pᵢ₋₁.y)²)
```

#### 2-b. 自車最近傍投影点の探索

車両フレーム原点 (0, 0) を `central_curve` の各セグメントに 2D 投影し、最も近い **polyline 上の点**の S を求めます（頂点間の補間点も含む）:

```
bestS = raw[i] + t × |Pᵢ₊₁ − Pᵢ|    (t は投影パラメータ)
```

#### 2-c. S 原点をシフト

```
cumS[i] = raw[i] − bestS
```

```
(後方seg)           (ego seg)                    (前方seg)
P₀ ───── P₁ ───── ◇(ego投影, S=0) ───── P₃ ───── P₄
-120m    -80m       0m               +40m      +80m     ← cumS
```

#### 2-d. ターゲットの S を求める

1. ターゲットを各セグメント `Pᵢ → Pᵢ₊₁` に投影（2D 最近傍点探索）
2. 最も近いセグメントを特定（セグメント番号 `i`、内分パラメータ `t ∈ [0, 1]`）
3. `S = cumS[i] + t × |Pᵢ₊₁ − Pᵢ|`

**t の求め方（内積）:**

```
AB = Pᵢ₊₁ − Pᵢ             (線分ベクトル)
AP = target − Pᵢ            (始点→ターゲット)

t = dot(AP, AB) / |AB|²     (AB への AP の射影比)
  = (AP.x × AB.x + AP.y × AB.y) / (AB.x² + AB.y²)

t = clamp(t, 0, 1)          (線分外にはみ出さないよう制限)
```

t の幾何学的意味:
- `t = 0` → 投影点は Pᵢ（線分始点）
- `t = 1` → 投影点は Pᵢ₊₁（線分終点）
- `0 < t < 1` → 投影点は線分上の中間（始点から t×100% の位置）
- clamp 前に `t < 0` → ターゲットが始点より手前 → Pᵢ に固定
- clamp 前に `t > 1` → ターゲットが終点より先 → Pᵢ₊₁ に固定

`S > 0` ならターゲットは自車より前方、`S < 0` なら後方です。

### ステップ 3: L 値の計算（符号付き横距離）

ステップ 2 で求めた投影セグメント `Pᵢ → Pᵢ₊₁` 上の最近傍点を `Q` とします。

**Q の求め方（内積による射影）:**

ステップ 2-d で求めた内分パラメータ `t` を使い、線分 `Pᵢ → Pᵢ₊₁` 上の点を補間します:

```
Q = Pᵢ + t × (Pᵢ₊₁ − Pᵢ)
```

```
       Pᵢ ──────────── Q ─────────── Pᵢ₊₁
       |←─── t=0.4 ───→|
                        |
                        | L = |target − Q|
                        |
                        ● target
```

Q は「ターゲットから線分 `Pᵢ → Pᵢ₊₁` に垂線を下ろした足」に相当します（t が [0, 1] 範囲内の場合）。t が範囲外にクランプされた場合は、線分の端点が Q になります。

**L の大きさ** は Q からターゲットまでの 2D ユークリッド距離:

```
|L| = √((target.x - Q.x)² + (target.y - Q.y)²)
```

**L の符号** は、セグメント方向ベクトルとオフセットベクトルの 2D 外積（z 成分）で決定します:

```
セグメント方向: d = (Pᵢ₊₁.x - Pᵢ.x,  Pᵢ₊₁.y - Pᵢ.y)
オフセット:     o = (target.x - Q.x,   target.y - Q.y)

cross = d.x × o.y − d.y × o.x

L > 0  ←  cross ≥ 0（セグメント進行方向の左側）
L < 0  ←  cross < 0（セグメント進行方向の右側）
```

> **注意**: `central_curve` の方向は道路の走行方向と一致するとは限りません。逆方向の場合、L の正負が道路の左右と逆転します。そのため、距離の計算では `target_l − boundary_l` の**差分**で符号を相殺し、結果が `central_curve` の方向に依存しないようにしています。

### ステップ 4: 白線の L 値を補間

左右の白線（結合済みの `left_boundary` / `right_boundary`）の各頂点を同様に `central_curve` に投影し、`(S, L)` の配列を作ります。

```
左白線の頂点 B₀, B₁, B₂, ... を central_curve に投影:

  B₀ → (S=2.1, L=1.83)
  B₁ → (S=5.4, L=1.81)
  B₂ → (S=9.0, L=1.84)
  ...
```

ターゲットの S 位置（例: S=7.2）における白線の L 値を、隣接する 2 点間の**線形補間**で求めます:

```
S=5.4 での L=1.81
S=9.0 での L=1.84

fraction = (7.2 - 5.4) / (9.0 - 5.4) = 0.5
L_boundary = 1.81 + 0.5 × (1.84 - 1.81) = 1.825
```

`left_boundary_l_bracket_mode` フィールドは補間の状態を示します:

| 値 | 意味 |
|---|---|
| `interpolated` | 2 点間で正常に補間 |
| `before_start` | target の S が白線の最小 S より小さい（先頭の L 値で外挿） |
| `after_end` | target の S が白線の最大 S より大きい（末尾の L 値で外挿） |
| `single` | 白線が 1 点のみ |

### ステップ 5: 車両端 ↔ 白線距離の算出

最終的な車両端から白線までの距離は:

```
distance_target_to_left_boundary_sl_m      = target_l  −  left_boundary_l
distance_target_edge_to_left_boundary_sl_m = target_l  −  left_boundary_l  −  width / 2
```

```
   left_boundary (L = left_l)
        |
        |←──── distance_target_to_left ────→|
        |                                    |
        |←── distance_edge_to_left ──→|      |
        |                             |  w/2 |
        |                        left_edge   center (target_l)
```

右白線も同一の式で計算します:

```
distance_target_to_right_boundary_sl_m      = target_l  −  right_boundary_l
distance_target_edge_to_right_boundary_sl_m = target_l  −  right_boundary_l  −  width / 2
```

> この式は直交座標系の `target.y − boundary.y − width/2` と同じ構造です。L 軸は y 軸と完全には一致しないため若干の差異がありますが、平坦で直線的な道路では非常に近い値になります。

### 直交座標系との比較

| 項目 | 直交座標系 (Cartesian) | SL 座標系 (Frenet) |
|---|---|---|
| **基準** | 車両フレームの y 軸 | `central_curve` への垂線 |
| **道路形状への対応** | 曲がった道路では距離の意味が変わる | 道路に沿った距離なので直感的 |
| **代表フィールド** | `distance_target_edge_to_left_boundary_m` | `distance_target_edge_to_left_boundary_sl_m` |
| **精度** | 結合済み boundary への 2D 最短距離 | polyline 投影 + 外積（道路に沿った距離） |
| **推奨用途** | 参考値 | **停止車両 ↔ 白線の汎用的な距離計測** |
| **central_curve のソース** | 単一セグメント | ID ベース結合（`successor_ids` / `predecessor_ids`）> 単一セグメント |
| **S の原点** | ― | 自車最近傍投影点（S=0）。社内 Python 版と統一 |

---

## Plot パネルでの活用例

```
# SL 座標系（停止車両↔白線距離の計測に推奨）
lane_boundary_tracker.distance_target_edge_to_left_boundary_sl_m
lane_boundary_tracker.distance_target_edge_to_right_boundary_sl_m
lane_boundary_tracker.target_s_m
lane_boundary_tracker.target_l_m

# 直交座標系（参考値）
lane_boundary_tracker.distance_target_edge_to_left_boundary_m
lane_boundary_tracker.distance_target_edge_to_right_boundary_m

# 状態フラグ・デバッグ
lane_boundary_tracker.target_found
lane_boundary_tracker.segment_found
lane_boundary_tracker.sl_valid
lane_boundary_tracker.sl_central_curve_source
lane_boundary_tracker.chained_segment_count
lane_boundary_tracker.central_curve_total_length_m
lane_boundary_tracker.central_curve_point_count
```

### 50m / 100m 後方からの白線距離計測

自車がターゲットに接近するログを再生すると、`target_s_m` はフレームごとに減少します:

| 状態 | `target_s_m` の目安 |
|---|---|
| ターゲットの 100m 後方 | ≈ +100 |
| ターゲットの 50m 後方 | ≈ +50 |
| ターゲットの真横 | ≈ 0 |
| ターゲットを通過 | < 0（負値が維持される） |

Plot パネルで **X 軸に `target_s_m`、Y 軸に `distance_target_edge_to_left_boundary_sl_m`** を設定すると、各距離での白線距離がプロットできます。

> `target_s_m` は直線距離ではなく**道路に沿った弧長**です。カーブがある場合、直線距離より大きくなります。弧長と直線距離の比較には `target_s_chord_m` を使用できます。

---

### 3. `high_memory_processes.ts`

**入力トピック**: `/t2/resource_monitor/raw`
**出力トピック**: `/studio_script/high_memory_processes`

`process_info` から resident_memory が閾値以上のプロセスを抽出します。

**Variables パネルで設定する変数:**

| 変数名 | 型 | 説明 |
|---|---|---|
| `memory_threshold_mb` | number | メモリ閾値 [MB]（省略時 100MB） |

**主な出力フィールド:**

| フィールド | 説明 |
|---|---|
| `filtered_count` | 閾値以上のプロセス数 |
| `filtered_processes[i].index` | `process_info` の元インデックス |
| `filtered_processes[i].name` | プロセス名 |
| `filtered_processes[i].pid` | PID |
| `filtered_processes[i].resident_memory_mb` | メモリ使用量 [MB] |

---

### 4. `ego_lane_segments.ts`

**入力トピック**: `/t2/object_augmentor/augmented_scene`
**出力トピック**: `/studio_script/ego_lane_segments`

自車が属する `lane_segment` の `id` と長さ（**直線距離=弦長** / **SL 距離=弧長**）に加え、**左右の隣接・直前/直後（predecessor / successor）のセグメント `id`** と、**取得した各 `id` の `local_lane_segments` 詳細（lane_augmentor 出力の全フィールド）**を出力します（v2）。

自車セグメントは `local_map_info.ego_lane_segment_indices`（augmentor が自車足元 ±0.1m で判定済みのインデックス列）を `local_lane_segments` のインデックスとして引いて特定します。`ego_lane_segment_indices` が空の場合のみ、`central_curve` への原点 (0,0) 最近傍セグメントにフォールバックします（`source` フィールドで判別可能）。

**Variables パネルで設定する変数:** なし

**長さの定義:**

| フィールド | 定義 |
|---|---|
| `straight_m` | `central_curve` の始点→終点の 2D 直線距離（**直線距離 / 弦長**） |
| `sl_m` | `central_curve` の 2D 累積弧長（**SL 距離**、道路に沿った長さ） |
| `provided_length_m` | `LocalLaneSegment.length`（augmentor 提供値、検証/参考用） |

> 直線区間では `straight_m ≈ sl_m`。カーブがあるほど `sl_m > straight_m` になります。

**トップレベルの主な出力フィールド:**

| フィールド | 説明 |
|---|---|
| `source` | セグメント特定方法（`ego_indices` / `projection_fallback` / `none`） |
| `ego_segment_count` / `ego_segment_ids` | 自車が属するセグメント数 / `id` 一覧 |
| `primary_segment_id` | 代表セグメント（先頭）の `id` |
| `primary_context_name` | 代表セグメントの位置コンテキスト名（`HIGHWAY` / `ENTERING_ETC` / `PASSING_ETC` / `EXITING_ETC` など） |
| `segments[]` | 自車セグメントの詳細（`relation="ego"`、下記 `SegmentDetail`） |
| `related_segments[]` | 自車セグメントの隣接/前後セグメントの詳細（`relation` で関係を表す） |
| `total_straight_m` / `total_sl_m` / `total_provided_m` | 全自車セグメントの**合計**長さ |

**`SegmentDetail`（`segments[]` / `related_segments[]` 共通）の主なフィールド:**

| フィールド | 説明 |
|---|---|
| `relation` | 自車との関係（`ego` / `predecessor` / `successor` / `left_neighbor` / `right_neighbor`） |
| `relative_to_ego_id` | この関連の起点となった自車セグメント `id`（`ego` は空） |
| `id` / `road_id` / `segment_index` | セグメント ID / 道路 ID / `local_lane_segments` 内の index |
| `straight_m` / `sl_m` / `provided_length_m` / `point_count` | 長さ・点数 |
| `ego_s_in_segment_m` | 自車原点をこのセグメントに投影した弧長（区間内のどこにいるか） |
| `successor_ids` / `predecessor_ids` | **直後 / 直前**のセグメント `id`（複数可） |
| `left_neighbor_ids` / `right_neighbor_ids` | **左 / 右**隣の同方向セグメント `id` |
| `speed_limit_max_kph` / `speed_limit_min_kph` / `target_speed_max_kph` | 制限速度・目標速度 |
| `nth_lane` / `lane_classification(_name)` / `split_merge_type` | 車線属性 |
| `left_lc_permission` / `right_lc_permission` | 車線変更可否（enum 値） |
| `is_target_lane` / `is_tollgate` / `is_tunnel` / `is_no_lane` / `is_out_of_course` / `is_slower_traffic` / `is_truck_lane` / `is_merging_anticipation` / `is_blockade_lane` | 各種フラグ |
| `location(_name)` / `context(_name)` | 位置・状況コンテキスト（ETC 区間の判別に有用） |

> **ETC 区間の特定**: `context_name`（`LocalLaneContext`）が `ENTERING_ETC` / `PASSING_ETC` / `EXITING_ETC` のセグメントが ETC 前後・通過区間です。`is_tollgate` と併せて対象区間の `id` を収集できます。
> `lane_segment_traversal_time` で通過時間を測りたい `id` は、`ego_segment_ids` / `related_segments[].id` から収集できます。

---

### 5. `lane_segment_traversal_time.ts`

**入力トピック**: `/t2/object_augmentor/augmented_scene`（必須）、`/t2/odometry/ego`（任意：現在速度の表示用）
**出力トピック**: `/studio_script/lane_segment_traversal_time`

指定した `id` の `lane_segment` を**通過するのにかかる時間**を出力します。**複数 id を同時に指定可能**です。

**Variables パネルで設定する変数:**

| 変数名 | 型 | 説明 |
|---|---|---|
| `target_segment_ids` | string | 対象セグメント `id`。カンマ/空白/セミコロン区切りで複数可（例: `"lane_001, lane_002 lane_003"`） |
| `assumed_speeds_kph` | string | 推定に使う仮定速度 [km/h]（既定 `"10,30,40"`） |
| `lateral_accel_limit_mps2` | number | 横加速度上限 [m/s²]（既定 `2.5`） |

**出力する「時間」は 2 種類:**

1. **実測通過時間 `measured_time_s`** — ログ再生中に自車が実際にそのセグメントを通過した時間。`ego_lane_segment_indices` で自車がそのセグメント上にいた区間を `event.receiveTime` で計測します（進入してから退出するまで）。**自車が実際にそのレーンを走行した場合のみ**得られます。分解能は `augmented_scene` の更新周期（約 0.1s）に依存します。
2. **推定通過時間 `est_*_s`** — `区間長(SL) ÷ 速度` による推定。制限速度 / 仮定速度 / 現在の自車速度 / 横G制限速度で算出します。**地図に出現すれば（隣接レーンなど自車が走らない区間でも）算出可能**です。

**対象セグメントごとの出力（`targets[i]`）:**

| フィールド | 説明 |
|---|---|
| `id` | 対象セグメント `id` |
| `latched` | 地図にこの `id` が出現し情報取得済みか（一度取得すると保持） |
| `found_in_map_now` | 現フレームの地図に存在するか |
| `length_sl_m` / `length_straight_m` / `length_provided_m` | SL 距離 / 直線距離 / 提供 length |
| `speed_limit_max_kph` | 制限速度上限 [km/h] |
| `is_tollgate` | 料金所（ETC）区間フラグ |
| `min_curve_radius_m` | `central_curve` の最小曲率半径 [m]（直線相当は大きな値） |
| `lateral_g_max_speed_kph` | 横G上限を満たす最大速度 `sqrt(a_lat · R)` [km/h] |
| `measured_state` | `not_seen` / `in_progress` / `completed` |
| `measured_time_s` | 実測通過時間 [s]（進行中はそこまでの経過） |
| `measured_avg_speed_kph` | 実測平均速度 [km/h]（完了時 = `length_sl / measured_time`） |
| `est_time_at_speed_limit_s` | 制限速度での推定通過時間 [s] |
| `est_time_at_lateral_g_max_s` | 横G上限速度での推定通過時間 [s] |
| `est_time_at_ego_speed_s` | 現在の自車速度での推定通過時間 [s] |
| `est_times_at_assumed[j]` | 仮定速度ごとの `{ speed_kph, time_s }` |

**自動蓄積フィールド（id 指定不要）:**

| フィールド | 説明 |
|---|---|
| `current_ego_segment_ids` | 現在の自車セグメント `id` 一覧 |
| `current_ego_elapsed_s` | 現在セグメント上での経過時間 [s] |
| `recent_traversals[i]` | 自車が通過し終えたセグメント履歴（新しい順、最大 40 件）。`{ id, time_s, length_sl_m, avg_speed_kph, speed_limit_kph }` |

> `recent_traversals` により、ログを一度再生するだけで「自車が走った各セグメントの実測通過時間と平均速度」の一覧が自動で貯まります。id を事前指定しなくても、通過区間の実測値を収集できます。

**典型的なワークフロー（ETC 前後の速度上限見直し検討）:**
1. `ego_lane_segments` を再生し、ETC 前後の対象セグメント `id` を収集する。
2. それらを `target_segment_ids` に設定する。
3. `measured_time_s` / `measured_avg_speed_kph`（実走の実態）と、`est_times_at_assumed`（例: 30→40km/h に上げた場合の所要時間）を比較する。
4. `min_curve_radius_m` / `lateral_g_max_speed_kph` で「その区間はカーブ的に何 km/h まで横G 2.5 m/s² 以内で出せるか」を確認する。

**Plot パネルでの活用例:**
```
# 現在セグメントの進捗・速度
lane_segment_traversal_time.current_ego_elapsed_s
lane_segment_traversal_time.ego_speed_kph

# 指定セグメントの実測 vs 推定（targets は配列なのでインデックス指定）
lane_segment_traversal_time.targets[0].measured_time_s
lane_segment_traversal_time.targets[0].est_time_at_speed_limit_s
lane_segment_traversal_time.targets[0].lateral_g_max_speed_kph
```

**注意:**
- 実測時間は自車が対象レーンを実走した場合のみ。隣接レーンの区間は推定時間のみ得られます。
- 実測分解能は地図更新周期（約 0.1s）に依存します。
- Seek で時刻が巻き戻ると計測状態（`recent_traversals` / 進行中の計測）をリセットします。
- 時間は SL（弧長）距離を速度で割って算出します（車両は車線に沿って走るため）。
- `min_curve_radius_m` / `lateral_g_max_speed_kph` は 2D（x-y 平面）の `central_curve` 形状から算出する目安値です。実際の許容速度は勾配・カント・制御マージン等で変わります。

---

### 6. `follower_gap_tracker.ts`

**入力トピック**: `/t2/object_augmentor/augmented_scene`（必須）、`/t2/odometry/ego`（任意：自車速度・車間時間用）
**出力トピック**: `/studio_script/follower_gap_tracker`

自車**後方・同一レーン**の追従車（follower）を追跡し、車間（SL 距離）・相対速度（接近率）・追従車の対地速度・車間時間（time headway）を出力します。ETC 前後で後続車が「**詰まる**」様子を定量化するための指標です。

自車レーンの `central_curve` を `successor_ids` / `predecessor_ids` で前後に結合して SL 座標（S=0 は自車投影点）を構築し、各 `augmented_object` を投影します。`|L| ≤ 閾値` かつ `S < 0` を後方同一レーン車とみなし、最近傍（S が 0 に最も近い負値）を follower とします。

**Variables パネルで設定する変数:**

| 変数名 | 型 | 説明 |
|---|---|---|
| `same_lane_l_threshold_m` | number | 同一レーン判定の `|L|` 閾値 [m]（既定 `1.75`） |

**主な出力フィールド:**

| フィールド | 説明 |
|---|---|
| `follower_found` / `follower_track_id` | 追従車の有無 / track_id |
| `follower_gap_s_m` | 後方車間（SL/弧長、正値） |
| `follower_gap_chord_m` | 自車原点→追従車の直線距離（参考） |
| `follower_lateral_l_m` | 追従車の横ずれ L |
| `follower_abs_speed_mps` / `_kph` | 追従車の**対地速度**（`local_velocity`） |
| `follower_relative_long_mps` | 自車に対する前後相対速度（**+で接近**、`local_relative_velocity.x`） |
| `follower_closing` | 接近中か（相対速度 > 0.1） |
| `follower_time_headway_s` | 車間時間 = `gap_s / 追従車速度`（**詰まりの主要指標**） |
| `follower_is_stationary` | 追従車が停止判定か |
| `follower_gap_ema_m` / `follower_relative_ema_mps` | 車間・相対速度の指数移動平均（同一 track_id 間で平滑化） |
| `leader_*` | 前方最近傍車（参考） |
| `behind_count` / `ahead_count` | 同一レーンの後方/前方車両数 |
| `behind_objects[]` | 後方キュー（車間が近い順、最大 10 件）。`{ track_id, gap_s_m, lateral_l_m, abs_speed_*, relative_long_mps, is_stationary, type }` |

> **詰まりの読み方**: `follower_gap_s_m` が縮小 / `follower_time_headway_s` が低下 / `follower_closing=true` が続く、または `behind_count` が増えるほど後続が詰まっています。ETC 前後区間（`ego_lane_segments.primary_context_name` 等で判別）でこれらを時系列比較してください。

---

### 7. `lateral_g_monitor.ts`

**入力トピック**: `/t2/odometry/ego`
**出力トピック**: `/studio_script/lateral_g_monitor`

自車の**実測横加速度（横G）**を出力し、横G上限と比較します。「2.5 m/s² 制約により ETC 前後を 30km/h に制限している」前提を実データで確認し、速度引き上げ余地を評価するための指標です。

**Variables パネルで設定する変数:**

| 変数名 | 型 | 説明 |
|---|---|---|
| `lateral_accel_limit_mps2` | number | 横加速度上限 [m/s²]（既定 `2.5`） |
| `ma_window` | number | 移動平均/窓ピークのサンプル数（既定 `11`） |

**横G の 2 系統**（通常ほぼ一致。相互検証用）:
- `lateral_accel_measured_mps2` … `local_linear_acceleration.y`（車両座標系の横加速度推定）
- `lateral_accel_kinematic_mps2` … `speed × yaw_rate`（旋回による求心加速度）

**主な出力フィールド:**

| フィールド | 説明 |
|---|---|
| `speed_mps` / `_kph` | 自車速度 |
| `yaw_rate_radps` / `_degps` | ヨーレート |
| `lateral_accel_measured_mps2` / `_abs_mps2` | 実測横G（符号付き / 絶対値） |
| `lateral_accel_kinematic_mps2` / `_abs_mps2` | 運動学的横G（`v·ω`） |
| `longitudinal_accel_mps2` | 前後加速度（参考） |
| `limit_mps2` / `ratio_measured` / `margin_measured_mps2` | 上限 / 比率 / 余裕 |
| `over_limit` | 実測横G が上限超過か |
| `peak_abs_mps2` | リセット以降の最大横G |
| `ma_abs_mps2` / `windowed_peak_abs_mps2` | 移動平均 / 窓内ピーク |

> **使い方**: ETC 前後カーブで `lateral_accel_measured_abs_mps2` が実際に 2.5 に達しているか、`margin_measured_mps2` にどれだけ余裕があるかを確認します。30km/h で余裕があれば速度引き上げの根拠になります。`ego_lane_segments` の `context_name` と同一タイムラインで重ねると区間対応が分かります。

---

### 8. `speed_margin_profile.ts`

**入力トピック**: `/t2/object_augmentor/augmented_scene`（必須）、`/t2/odometry/ego`（任意：現在速度）
**出力トピック**: `/studio_script/speed_margin_profile`

自車の現在レーンと**先読み（successor 連鎖）**の各区間について、**現在速度 / 制限速度 / 横G上限速度**を並べ、引き上げ余地を可視化します。「ETC 前後で制限速度を上げて良いか」を区間単位で判断する材料です。

**Variables パネルで設定する変数:**

| 変数名 | 型 | 説明 |
|---|---|---|
| `lateral_accel_limit_mps2` | number | 横加速度上限 [m/s²]（既定 `2.5`） |
| `lookahead_count` | number | 先読みするセグメント数（既定 `5`） |

**区間プロファイル（`segments[]` = 自車、`lookahead[]` = 先読み）の主なフィールド:**

| フィールド | 説明 |
|---|---|
| `id` / `relation` / `order` | セグメント ID / `ego` or `lookahead` / 先読み順 |
| `context_name` / `is_tollgate` / `is_tunnel` | 区間種別（ETC 等） |
| `sl_m` | SL 距離 |
| `speed_limit_kph` / `target_speed_max_kph` | 制限速度 / 目標速度 |
| `min_curve_radius_m` / `lateral_g_max_speed_kph` | 最小曲率半径 / 横G上限速度 `sqrt(a·R)` |
| `current_speed_kph` | 自車現在速度 |
| `margin_to_limit_kph` / `margin_to_latg_max_kph` | 制限速度・横G上限速度までの余裕 |
| `raise_ok` | **横G上限速度 ≥ 制限速度**（制限速度まで横G制約内で到達可能か） |
| `est_time_at_current_s` / `est_time_at_limit_s` / `est_time_at_latg_max_s` | 各速度での通過時間 |

> **判断の目安**: `raise_ok=true` かつ `lateral_g_max_speed_kph` が目標速度を上回る区間は、横G制約内で増速できる候補です。`context_name` が `ENTERING_ETC` / `EXITING_ETC` の区間で確認してください。

---

## 制約と注意事項

### SL 座標系

- `central_curve` のソースは `sl_central_curve_source` で確認可能（`"chained_id"` = ID ベース結合、`"single_segment"` = 単一セグメント）
- ID ベース結合は `successor_ids` / `predecessor_ids` がないセグメントで停止します（他車線を拾うリスクを排除するため、endpoint 近接フォールバックは廃止）
- SL 変換は 2D（x-y 平面）で計算しています。勾配のある道路では若干の誤差が生じます
- 左/右白線の S 値が単調増加であることを前提に線形補間しています
- `bracket_mode = before_start / after_end` の場合、target の S 位置が白線の S 範囲外（外挿相当）なので値の信頼性は低下します
- `central_curve` の方向が道路走行方向と逆の場合がありますが、距離計算は `target_l − boundary_l` の差分で符号を相殺するため、結果に影響しません
- ID ベース結合時、セグメント反転が必要な場合は `left_boundary` と `right_boundary` を自動交換します

### 一般

- トークン等の認証情報をスクリプト内に記載しないでください
- FoxBox の User Scripts エディタに貼り付けて使用します
- `target_object` と `lane_boundary_tracker` は同じ `track_id` を Variables で共有します
- Seek backward するとモジュール変数がリセットされますが、v15 以降は単一トピック構成のためターゲット消失問題は発生しません
