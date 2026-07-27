# FoxBox UserScript 作成規約・ハマりどころ

FoxBox（Foxglove Studio ベース）の User Scripts を書く際の必須ルール。新規に
スクリプトを追加/修正するときはこれに従うこと。破ると「Foxglove 上では動くのに
出力が壊れる／再生でおかしくなる」等の分かりにくい不具合になる。

## 1. スクリプトの基本形

```ts
import { Input } from "./types";

type GlobalVariables = { /* Variables パネルの変数 */ };
type Output = { /* 出力の固定スキーマ（下記ルール） */ };

export const inputs = ["/t2/..."];              // 購読トピック
export const output = "/studio_script/...";     // 出力トピック
export default function script(
  event: Input<"/t2/...">,
  globalVars: GlobalVariables,
): Output | undefined { /* ... */ }
```

- `event` は `{ topic, receiveTime: {sec,nsec}, message }`。`message` は `as unknown as {...}` でキャストして読む。
- 複数トピックを購読するときは `inputs` に並べ、`event.topic === "..."` で分岐。型は union（`Input<"a"> | Input<"b">`）。

## 2. 自己完結（最重要）

- **各スクリプトは 1 本ずつ Foxglove の User Scripts エディタへ貼り付ける運用**を前提とする。
- したがって **共有ライブラリへの切り出しをしない**。`dist2D` や SL 投影などのヘルパが複数ファイルで
  重複しているのは**意図的**。DRY より「1 ファイルで完結して貼れる」ことを優先する。
- 例外は `./types`（Foxglove がランタイムで提供する型）。リポジトリの `scripts/types.ts` は
  **ローカルの tsc/テスト専用スタブ**であり、Foxglove には貼らない。

## 3. 出力スキーマは完全に具体化する

Foxglove は最初の出力メッセージから出力トピックのスキーマを推論する。壊さないために:

- **optional（`?`）や union 型を出力型に使わない**。全フィールドを常に出力する。
- 数値/真偽/文字列は毎回同じ型で返す（`undefined` や `null` を混ぜない）。
- 配列要素は**常に同一形状**のオブジェクトにする。
- 「該当なし」は値で表現する（例: `found:false` + 数値 `0`、文字列 `""`）。optional にしない。
- 空配列でも、`Output` 型に要素型を明示しておけば tsc/推論が要素形状を把握できる。

## 4. 時刻は `event.receiveTime`

- ROS Header に `stamp`（sec/nsec）は**無い**（`docs/data_model.md` 参照）。
- タイミング（通過時間、EMA、ピーク等）は `event.receiveTime`（`{sec,nsec}`）を秒に直して使う:
  `const t = sec + nsec*1e-9;`
- 出力の `stamp` フィールドも `receiveTime` から作ると Plot のタイムラインと一致する。

## 5. 状態保持と Seek リセット

- モジュールトップレベルの `let`/`const`（Map 等）はメッセージ間で保持される。
  累積・移動平均・enter/exit 検出などに使う。
- **Seek で時刻が巻き戻ったら状態をリセットする**。定石:
  ```ts
  let lastProcTime = -1;
  // ...
  const nowSec = timeToSec(event.receiveTime);
  if (lastProcTime >= 0 && nowSec < lastProcTime) { resetState(); }
  lastProcTime = nowSec;
  ```
- リセットしないと、巻き戻し後に古い enter 時刻や EMA が混入して値が壊れる。

## 6. 複数トピック構成のパターン

- 主トピック（例: `augmented_scene`）受信時に出力を計算して返す。
- 従トピック（例: `odometry/ego`）受信時は**状態（最新速度等）を更新して `undefined` を返す**
  （出力は主トピックに集約）。lane_boundary_tracker / traversal / follower / speed_margin がこの形。

## 7. enum は整数

- IDL の enum はワイヤ上は整数で届く。名前化はスクリプト内のテーブル（配列 index→名前）で行う。
- `enumName(v, NAMES)` のように、数値/文字列どちらで来ても耐えるヘルパにしておくと安全。
- **enum の順序は IDL に追従**。メンバー追加で値がずれ得るので、テーブル更新時は IDL を再確認。

## 8. 幾何計算の約束

- 座標は車両座標系（x=前方, y=左, z=上）。自車原点は (0,0,0)。
- 距離・弧長は基本 **2D（x-y）** で計算（道路はほぼ平面、z ノイズを避ける）。既存スクリプトと統一。
- **SL 座標**（Frenet）: `central_curve` を successor/predecessor で結合し、自車投影点を S=0 とする。
  L は進行方向左が正（2D 外積で符号）。詳細は `lane_boundary_tracker.ts` と README の理論節。
- **曲率半径**: Menger 曲率 `κ = 2·|(B-A)×(C-A)| / (|AB||BC||CA|)`, `R=1/κ`。直線は sentinel（大きな値）。

## 9. TypeScript ターゲット

- `Map` / `Set` / `for...of` / スプレッド / アロー関数などは使用可（ターゲットは ES2020 相当）。
- `strict` + `noUnusedLocals` + `noUnusedParameters` + `noImplicitReturns` で書く
  （未使用引数は `_` プレフィックスで回避）。`test/run.sh` がこれで検査する。

## 10. 検証

- 変更したら必ず `bash test/run.sh`（または `npm run verify`）。
- 新スクリプトを足したら:
  - `test/tsconfig.build.json` は `scripts/**` を拾うので追加不要。
  - `test/behavior.test.cjs` に合成データの assertion を足す（`require("./out/<name>.js").default`）。
- ステートフルなロジック（enter/exit, EMA, Seek リセット, SL 投影, 曲率）は特に合成テストで固める。
- **合成テストの数値は実データと型が違う点に注意**（下記 11）。特に uint64 は number で書きがちだが実際は bigint。

## 11. 64bit 整数は `bigint`（ハマりどころ）

- ROS2 の `uint64`/`int64` フィールドは Foxglove のデコーダで **`bigint`** として届く
  （`number` ではない）。代表例は `ego_lane_segment_indices`（`sequence<uint64>`）。
- 数値化ヘルパは必ず bigint を受ける:
  ```ts
  function num(v: unknown, dflt: number): number {
    if (typeof v === "number" && isFinite(v)) { return v; }
    if (typeof v === "bigint") { return Number(v); }
    return dflt;
  }
  ```
- これを怠ると index が全部 `dflt`（-1）になり、**エラーも出さずに機能が停止**する
  （自車セグメント特定が空→フォールバック、通過時間計測が動かない等）。合成テストでは
  `[BigInt(0)]` を渡して実データの型を再現すること。

## 12. 常に空になり得る出力配列

- 出力に配列（`targets` / `recent_traversals` / `behind_objects` / `related_segments` など）がある場合、
  **最初のメッセージで空だと Foxglove がネスト要素のスキーマを推論できず**、Plot 等でその列が
  最初の非空メッセージまで現れないことがある（コード欠陥ではないが利用者向けに周知）。
- 対策: 変数（例: `target_segment_ids`）を設定してから再生する／要素が入るシーンまでシークする。
