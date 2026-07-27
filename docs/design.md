# 設計判断と根拠

各スクリプトの「なぜこう作ったか」。会話履歴を持たない新規セッションが意図を復元できるように記録する。
利用者向けの使い方は `README.md`、データ定義は `docs/data_model.md`、共通ルールは
`docs/foxbox_userscript_conventions.md` を参照。

## 共通方針

- **入力の起点は `augmented_scene`**。ここに物体（`augmented_objects`）とローカル地図
  （`local_map_info`）が両方入っており、多くの解析が 1 トピックで完結する。自車速度が要るものだけ
  `/t2/odometry/ego` を従トピックとして足す。
- **時刻は `receiveTime`**、**状態はモジュール変数＋Seek リセット**（規約 4・5）。
- **出力スキーマは完全具体化**（規約 3）。

## ego_lane_segments.ts（自車レーン情報）

- **自車セグメントの特定**: `local_map_info.ego_lane_segment_indices`（augmentor が自車足元 ±0.1m で
  判定済みの index 列）を第一候補にする。原点投影で自前計算するより、augmentor の判定を信頼する方が
  正確。空のときだけ `central_curve` への原点最近傍でフォールバック（`source` で判別可能に）。
- **長さの 2 種**: 直線距離＝`central_curve` 始点→終点の弦長、SL 距離＝弧長（累積）。ユーザー要件で
  両方要求された。`length`（提供値）も併記して検証できるようにした。
- **v2 拡張（隣接・前後・全属性）**: 各自車セグメントの `successor/predecessor/left/right` の id を出し、
  それら参照先を `local_lane_segments` から解決して `related_segments` に全フィールドをダンプ。
  `relation` で自車との関係を示す。ETC 区間の判別のため `location_context.context` を名前化して出す
  （`ENTERING_ETC/PASSING_ETC/EXITING_ETC`）。

## lane_segment_traversal_time.ts（区間通過時間）

- **時間は 2 系統**:
  1. **実測** — `ego_lane_segment_indices` から「自車がその id 上にいるフレーム区間」を検出し、
     進入時刻〜退出時刻を `receiveTime` で計測。id はマップ由来で安定なので id をキーに追跡できる。
     Seek 巻き戻しでリセット。分解能は地図更新周期（~0.1s）。
     退出は `EXIT_DEBOUNCE_S` でデバウンスし、1〜数フレームの index 欠落では確定させない
     （measured 時間の途中打ち切りを防ぐ）。退出確定後の再進入は新しい通過として再計測する。
     index は `uint64`＝`bigint` で届くため `num()` で数値化する（規約 11）。
  2. **推定** — SL 長 ÷ 速度。制限速度・仮定速度（既定 10/30/40km/h＝ETC/前後/法定）・現在自車速度・
     横G上限速度で算出。自車が走らない隣接区間でも地図に出れば推定可能。
- **latch**: 一度地図に出た区間の長さ・制限速度・曲率を保持し、地図から外れても推定を出せるようにした。
- **`recent_traversals`**: 通過し終えた自車セグメントの実測時間・平均速度を自動蓄積（新しい順、上限 40）。
  一度再生するだけで区間別の実走データ表が得られる（id を事前指定しなくてよい）。
- 時間は SL（弧長）を使う（車両は車線に沿って走るため）。

## follower_gap_tracker.ts（後続車ギャップ／詰まり）

- **同一レーン判定は SL 投影**で行う（車両座標の |y| ではカーブで破綻するため）。自車レーンの
  `central_curve` を `chainEgoCentral` で前後結合し、S=0 を自車投影点として各物体を投影。
  `|L| ≤ 閾値`（既定 1.75m）かつ `S<0` を後方同一レーン車、その最近傍（S が 0 に最も近い負値）を
  follower とする。前方最近傍を leader として参考出力。
- **自車レーンの起点は `ego_lane_segment_indices`** を優先（無ければ原点最近傍頂点にフォールバック）。
  車線変更中や車線際で SL 起点に隣接レーンを掴むのを防ぐ。index は `bigint` 前提で数値化する。
- **速度**: 対地速度は `local_velocity`、接近率は `local_relative_velocity.x`（+で接近）。
  車間時間 `time_headway = gap_s / follower速度` を詰まりの主要指標にした。
- **EMA**: follower の track_id が変わったらリセットして平滑化（別車の値が混ざらないように）。
- `chainEgoCentral` は `lane_boundary_tracker` の双方向結合の central 部分を移植した簡約版
  （左右境界は不要なので省略）。

## lateral_g_monitor.ts（実測横G）

- **2 系統の横G**を出す: 実測 `local_linear_acceleration.y` と 運動学的 `speed × yaw_rate`
  （`yaw_rate` は `local_angular_velocity.z` 優先）。両者はほぼ一致するはずで、相互検証に使う。
- 上限（既定 2.5）との比・余裕・超過フラグ・ピーク保持・移動平均を出し、ETC 前後カーブで実際に
  2.5 に達しているか／余裕があるかを可視化する（＝速度引き上げ余地の根拠）。
- odometry 単独（1 トピック）。区間対応は `ego_lane_segments.context_name` と同一タイムラインで重ねる。

## speed_margin_profile.ts（速度マージン／増速余地）

- 自車セグメント＋`successor` 連鎖の先読みについて、**現在速度／制限速度／横G上限速度**を並べる。
- 横G上限速度 `= sqrt(a_lat · R_min)`（`R_min` は `central_curve` の最小曲率半径）。
  `raise_ok = 横G上限速度 ≥ 制限速度` で「制限速度まで横G内で到達可能か」を一目で示す。
- 先読みで ETC 前後区間（`context_name`）の制限を事前に把握できる。

## 曲率半径・横G上限速度の考え方（背景の核心）

- 横加速度 `a = v²/R`。上限 `a_max=2.5 m/s²` のとき許容速度 `v = sqrt(a_max·R)`。
- 例: R=50m なら v≈11.2m/s≈40km/h。つまり曲率半径が緩い区間は 40km/h でも 2.5 以内。
- これが「前後区間を上げてよいか」の定量判断の骨子。`speed_margin_profile` と `lateral_g_monitor`
  （実測）を突き合わせて報告する。

## 却下した/見送った設計

- **共有ライブラリ化**: 自己完結・個別貼り付け運用を優先し、ヘルパ重複を許容（規約 2）。
- **ego 速度を augmented_scene から取得**: `AugmentedScene` に自車速度は無い（header と物体と地図のみ）。
  速度が要るものは `odometry/ego` を従トピックで購読。
- **通過時間を header タイムスタンプで計測**: Header に sec/nsec が無く uint64 ns は JS 数値精度の懸念も
  あるため、`receiveTime` を採用。
