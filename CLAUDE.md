# CLAUDE.md — FoxBoxUserScript_Hirata

> このファイルは Claude Code が新規セッション開始時に自動で読み込みます。
> **まずここを読み、次に `docs/` を参照してください。**

## このリポジトリは何か

**FoxBox**（Foxglove Studio ベースの社内ログ可視化ツール）向けの **User Scripts** 集です（TypeScript）。
T2 Yatagarasu の走行ログ（mcap）を Foxglove で再生し、レーン・物体・自車状態を解析します。
各スクリプトは 1 つ以上の ROS2 トピックを購読し、`/studio_script/*` に構造化データを出力します。
利用者はそれを Plot / Raw Messages / State Transitions パネルで可視化します。

## 新規セッションが最初にやること（チェックリスト）

1. **この `CLAUDE.md` と `docs/` を読む**。特に:
   - `docs/data_model.md` … 使用トピック・メッセージ定義・enum・IDL の所在（データ辞書）
   - `docs/foxbox_userscript_conventions.md` … FoxBox UserScript を書く上での必須ルール
   - `docs/design.md` … 各スクリプトの設計判断と根拠
   - `docs/handoff.md` … 移行の経緯・現状・TODO
2. **メッセージ定義（IDL）は別リポジトリにある**。スキーマを確認/変更するときは `add_repo` で取得:
   - `t2-auto/Yatagarasu` … `src/interfaces/**/*.idl` に全メッセージ定義（**スキーマ調査に必須**）
   - 参照経路は `docs/data_model.md` に列挙済み。
3. **変更したら必ず検証**: リポジトリ直下で `bash test/run.sh`（または `npm run verify`）。
   strict `tsc`（全スクリプト）+ 合成データの behavior テストが走ります。
4. 作業用ブランチで開発し、**こまめに `git push`** する。
   > ⚠️ 実行環境コンテナは揮発性で、セッションをまたぐと作業コピーが再クローンされることがあります。
   > 実際に一度ローカルのブランチが古いコミットへ巻き戻りました（push 済みだったため復旧できた）。
   > **push していない変更は失われる前提**で作業してください。

## リポジトリ構成

| パス | 内容 |
|---|---|
| `scripts/*.ts` | User Scripts 本体（Foxglove の User Scripts エディタへ貼り付ける） |
| `scripts/types.ts` | **ローカル用スタブ**（`./types`）。tsc/テスト専用で、**Foxglox には貼らない** |
| `test/` | 検証ハーネス（スタブ経由の tsc + 合成データ behavior テスト） |
| `docs/` | 設計・データ辞書・FoxBox 規約・引き継ぎメモ |
| `README.md` | 利用者向け（各スクリプトの使い方・出力フィールド一覧） |
| `*.pptx` | 仕様書・イメージ図（バイナリ） |

## スクリプト一覧

| スクリプト | 入力トピック | 出力トピック | 概要 |
|---|---|---|---|
| `target_object.ts` | `augmented_scene`(+`bev_detection`※legacy) | `/studio_script/target_object` | `track_id` 指定物体を毎フレーム追跡 |
| `lane_boundary_tracker.ts` | `augmented_scene`(+`bev_detection`※legacy) | `/studio_script/lane_boundary_tracker` | 物体↔白線距離（直交/SL 座標） |
| `high_memory_processes.ts` | `/t2/resource_monitor/raw` | `/studio_script/high_memory_processes` | 高メモリプロセス抽出 |
| `ego_lane_segments.ts` | `augmented_scene` | `/studio_script/ego_lane_segments` | 自車レーンの id・長さ（直線/SL）・左右/前後 id・全属性 |
| `lane_segment_traversal_time.ts` | `augmented_scene`(+`odometry/ego`) | `/studio_script/lane_segment_traversal_time` | 指定区間の通過時間（実測＋推定、複数 id） |
| `follower_gap_tracker.ts` | `augmented_scene`(+`odometry/ego`) | `/studio_script/follower_gap_tracker` | 後続車ギャップ／詰まりの定量化 |
| `lateral_g_monitor.ts` | `/t2/odometry/ego` | `/studio_script/lateral_g_monitor` | 実測横G と上限比較 |
| `speed_margin_profile.ts` | `augmented_scene`(+`odometry/ego`) | `/studio_script/speed_margin_profile` | 現在速度／制限速度／横G上限速度の比較 |

## 絶対に守る FoxBox UserScript 制約（詳細は `docs/foxbox_userscript_conventions.md`）

- **各スクリプトは自己完結**（1 本ずつエディタへ貼り付ける運用）。共有 import を増やさない。ヘルパ関数の重複は**意図的に許容**。
- 出力オブジェクトは **optional/union 禁止・全フィールド具体型・配列要素は同一形状**（Foxglove のスキーマ推論のため）。
- **時刻は `event.receiveTime`（`{sec,nsec}`）**。ROS Header に `stamp` は無い（uint64 ns の別フィールド）。
- **enum は整数**で届く（名前化はスクリプト側のテーブルで行う）。
- **状態はモジュール変数**で保持。`receiveTime` が巻き戻ったら（Seek）状態をリセットする。
- 2 トピック構成では、従トピックは状態更新のみで `undefined` を返し、出力は主トピック受信時に行う。

## 検証方法

```bash
bash test/run.sh          # ①strict tsc（scripts 全体）②テスト用ビルド ③合成データ behavior テスト
# または
npm run verify            # 上と同等（npm install 不要。環境の tsc/node を使用）
```
テストは Foxglove そのものではなく、`scripts/types.ts` スタブ経由でスクリプトの `default` 関数に
手作りメッセージを流し込み、出力を検証します（`test/behavior.test.cjs`）。

## 背景（なぜ作っているか）

Jira **VT26-1124「ETC 通過前後の周囲状況解析」**。
ETC 区間は 10km/h、前後区間は 30km/h で走行しているが後続車が詰まりやすい。前後区間は法的には
40km/h だが、社内の**横G上限 2.5 m/s²** に引っかかるため 30km/h に制限している。速度上限を上げれば
詰まりが解消するかを社内報告するため、**通過時間・後続車ギャップ・実測横G・速度マージン**を
ログから計測するのがこれらスクリプト群の目的。

## 規約

- コメント/ドキュメントは日本語可。**秘密情報（トークン等）はコミットしない**。
- コミットは簡潔な命令形サマリ。ブランチ運用・移行状況は `docs/handoff.md` を参照。
