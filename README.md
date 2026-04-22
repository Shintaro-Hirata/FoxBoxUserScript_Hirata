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

指定した世界座標のターゲット物体から、指定レーンセグメントの左右境界線（白線）までの距離を、直交座標系および SL 座標系（Frenet フレーム）で算出します。

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

---

## 出力フィールド一覧

### 直交座標系（Cartesian）

| フィールド | 説明 |
|---|---|
| `distance_to_left_boundary_m` | ターゲット ↔ 左境界 curve 全体の最短距離 [m] |
| `distance_to_right_boundary_m` | ターゲット ↔ 右境界 curve 全体の最短距離 [m] |
| `distance_target_edge_to_selected_y_m` | `target.local_y − selected.y − width/2`（y 軸方向の端-点距離） |
| `selected_point` | `curve_point_index` で指定した curve 上の点の座標 |
| `nearest_point_on_left_curve` | left_boundary polyline 上の最短点（頂点間の補間点） |
| `available_segments` | 全 segment の id と距離一覧（id 確認・選択用） |

### SL 座標系（Frenet フレーム, v6〜）

| フィールド | 説明 |
|---|---|
| `sl_valid` | SL 変換が成立した場合 true |
| `central_curve_total_length_m` | `central_curve` 全体の弧長 [m] |
| `target_s_m` | ターゲットの S 座標（central_curve 沿い弧長）[m] |
| `target_l_m` | ターゲットの L 座標（符号付き横距離）[m] |
| `target_central_projection` | ターゲットを central_curve に投影した点の座標 |
| `target_central_segment_index` | 投影先の polyline セグメント番号 |
| `left_boundary_l_at_target_s_m` | ターゲットと同じ S 位置での左白線 L 値（補間） |
| `distance_target_to_left_boundary_sl_m` | `target_l − left_l`（中心→白線の横距離）|
| `distance_target_edge_to_left_boundary_sl_m` | 上記 − `width/2`（車両端→白線距離）|
| `right_boundary_l_at_target_s_m` | 右白線の補間 L 値 |
| `distance_target_to_right_boundary_sl_m` | `target_l − right_l` |
| `distance_target_edge_to_right_boundary_sl_m` | 上記 − `width/2` |
| `left_boundary_l_bracket_mode` | 補間区間種別（デバッグ用） |
| `target_same_frame_match` | augmented_objects から同一フレーム位置を取得できたか |

---

## SL 座標系（Frenet フレーム）の計算理論

### 概要

SL 座標系は、道路の中心線（`central_curve`）を基準にした**曲線座標系**です。直交座標系（x, y, z）では道路形状に依存して距離の解釈が変わりますが、SL 座標系では道路に沿った位置（S）と道路からの横ずれ（L）で表現できるため、車両と白線の距離を直感的に扱えます。

```
          L (横方向)
          ↑
          |
          |    ● ターゲット (S=80.5, L=1.34)
          |
  --------+---------- central_curve (L=0)
          |
          |
          +----------→ S (道路沿い方向)
```

- **S (Longitudinal)**: `central_curve` の始点 `curve[0]` から polyline に沿って測った**累積弧長** [m]
- **L (Lateral)**: `central_curve` からの**符号付き垂直距離** [m]

### ステップ 1: S 値の計算（累積弧長）

`central_curve` は頂点の配列 `[P₀, P₁, P₂, ..., Pₙ]` で定義されます。各頂点間を直線で結んだ **polyline** を道路中心線とみなします。

```
P₀ ───── P₁ ───── P₂ ───── P₃ ───── P₄
|  seg 0  |  seg 1  |  seg 2  |  seg 3  |
0m       12m       25m       40m       52m   ← 累積弧長
```

各頂点の累積弧長は、前の頂点からの 2D 距離（x-y 平面）を足し合わせて求めます:

```
cumS[0] = 0
cumS[i] = cumS[i-1] + √((Pᵢ.x - Pᵢ₋₁.x)² + (Pᵢ.y - Pᵢ₋₁.y)²)
```

ターゲット点の S を求めるには:

1. ターゲットを各セグメント `Pᵢ → Pᵢ₊₁` に投影（2D 最近傍点探索）
2. 最も近いセグメントを特定（セグメント番号 `i`、内分パラメータ `t ∈ [0, 1]`）
3. `S = cumS[i] + t × |Pᵢ₊₁ − Pᵢ|`

### ステップ 2: L 値の計算（符号付き横距離）

ステップ 1 で求めた投影セグメント `Pᵢ → Pᵢ₊₁` 上の最近傍点を `Q` とします。

```
        Pᵢ ────────── Pᵢ₊₁
                Q ← 投影点
                |
                | L (垂直距離)
                |
                ● ターゲット
```

**L の大きさ** は 2D ユークリッド距離:

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

### ステップ 3: 白線の L 値を補間

左右の白線（`left_boundary.curve`, `right_boundary.curve`）の各頂点を同様に `central_curve` に投影し、`(S, L)` の配列を作ります。

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

### ステップ 4: 車両端 ↔ 白線距離の算出

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

> この式は直交座標系の `distance_target_edge_to_selected_y_m`（= `target.y − boundary.y − width/2`）と同じ構造です。L 軸は y 軸と完全には一致しないため若干の差異がありますが、平坦で直線的な道路では非常に近い値になります。

### 直交座標系との比較

| 項目 | 直交座標系 (Cartesian) | SL 座標系 (Frenet) |
|---|---|---|
| **基準** | 車両フレームの y 軸 | `central_curve` への垂線 |
| **道路形状への対応** | 曲がった道路では距離の意味が変わる | 道路に沿った距離なので直感的 |
| **代表フィールド** | `distance_target_edge_to_selected_y_m` | `distance_target_edge_to_left_boundary_sl_m` |
| **白線の指定** | `curve_point_index` で 1 点を手動選択 | ターゲットの S 位置で自動補間 |
| **精度** | y 軸差分のみなので高速 | polyline 投影 + 外積で若干重い |
| **推奨用途** | 特定の curve 頂点との距離を確認したい場合 | 停止車両 ↔ 白線の汎用的な距離計測 |

### 座標フレームについて（v7〜）

ターゲットの位置は `/t2/bev_detection/objects` から世界座標で特定し、`raw_id` を保存します。距離・SL 計算には `/t2/object_augmentor/augmented_scene` の `augmented_objects` から同じ `raw_id` の `local_position` を取得し、`central_curve` と**同一の車両フレーム**内で計算します。

これにより、2 つのトピックのタイムスタンプずれによる座標フレーム不一致の影響を低減しています。`target_same_frame_match = true` の場合、同一フレーム位置が使用されています。`false` の場合は `bev_detection` の `local_position`（別フレームの可能性あり）にフォールバックします。

---

## Plot パネルでの活用例

```
# SL 座標系（停止車両↔白線距離の計測に推奨）
lane_boundary_tracker.distance_target_edge_to_left_boundary_sl_m
lane_boundary_tracker.distance_target_edge_to_right_boundary_sl_m
lane_boundary_tracker.target_s_m
lane_boundary_tracker.target_l_m

# 直交座標系（特定 curve 頂点との距離確認用）
lane_boundary_tracker.distance_to_left_boundary_m
lane_boundary_tracker.distance_target_edge_to_selected_y_m

# 状態フラグ
lane_boundary_tracker.target_object_found
lane_boundary_tracker.segment_found
lane_boundary_tracker.sl_valid
lane_boundary_tracker.target_same_frame_match
```

---

## 制約と注意事項

- `central_curve` はセグメント単位で定義されるため、セグメントをまたぐと S 値がリセットされます
- SL 変換は 2D（x-y 平面）で計算しています。勾配のある道路では若干の誤差が生じます
- 左/右白線の S 値が単調増加であることを前提に線形補間しています
- `bracket_mode = before_start / after_end` の場合、target の S 位置が白線の S 範囲外（外挿相当）なので値の信頼性は低下します
- `central_curve` の方向が道路走行方向と逆の場合がありますが、距離計算は `target_l − boundary_l` の差分で符号を相殺するため、結果に影響しません
- トークン等の認証情報をスクリプト内に記載しないでください
- FoxBox の User Scripts エディタに貼り付けて使用します
- `Output | undefined` は FoxBox が許可する唯一の union 型です（multi-input スクリプトで使用）
- Seek backward するとモジュール変数がリセットされます。Seek forward は状態が保持されます
