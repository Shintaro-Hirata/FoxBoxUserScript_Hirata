# 引き継ぎメモ（Team プラン移行）

## 背景

アカウントを Team プランへ移行する。チャット履歴は引き継げるが、**この Claude Code セッションは
引き継げない**ため、新規セッションがこのリポジトリの文脈をゼロから復元できるよう整備した。
本リポジトリの目的・データ・設計は `CLAUDE.md` と `docs/` に集約してある。

## 現状サマリ（このセッションまでの成果）

Jira **VT26-1124「ETC 通過前後の周囲状況解析」**向けに、以下の User Scripts を実装・検証済み。

要件と対応:

| 依頼 | 対応スクリプト |
|---|---|
| 自車が属する lane_segment の id と長さ（直線距離・SL距離） | `ego_lane_segments.ts` |
| 指定 id の lane_segment 通過時間（複数指定可） | `lane_segment_traversal_time.ts` |
| ego_lane の左右・前後の lane_segment id | `ego_lane_segments.ts`（v2 で追加） |
| 取得 id の local_lane_segments 全情報 | `ego_lane_segments.ts`（v2 `related_segments`） |
| 追加①：詰まりの定量化 | `follower_gap_tracker.ts` |
| 追加②：横G制約の実データ検証 | `lateral_g_monitor.ts` |
| 追加③：増速余地の判断 | `speed_margin_profile.ts` |

既存スクリプト（`target_object.ts` / `lane_boundary_tracker.ts` / `high_memory_processes.ts`）は
従来どおり。今回 `lane_boundary_tracker.ts` から**デッドコードを除去**した（下記）。

整備物:
- **検証ハーネス** `test/`（`scripts/types.ts` スタブ + `tsconfig.json` + `test/behavior.test.cjs`）。
  `bash test/run.sh` で strict tsc（全 8 スクリプト）+ 合成テスト（69 assertion）が緑。
- **ドキュメント** `CLAUDE.md`（自動読込）+ `docs/data_model.md` / `design.md` /
  `foxbox_userscript_conventions.md` / 本ファイル。

## リファクタ／バグ確認の結果

- 全スクリプト strict `tsc`（`noUnusedLocals` 等）+ 合成テストを通過。機能バグは検出されていない。
- `lane_boundary_tracker.ts` の**未使用コードを削除**: `minDist`, `ptSeg`, `ptCurve`,
  `findByEndpoint`, `lateralOk`, 定数 `CONNECT_THRESH`/`LATERAL_THRESH`, 変数
  `egoRoadId`/`hasConnectivity`。いずれも「endpoint 近接フォールバック廃止」の残骸で、
  tsc が未使用と保証しており挙動不変（README 記載の ID ベース結合方針と一致）。
- 方針決定（ユーザー承認済み）: **各スクリプトは自己完結を維持**（個別コピペ運用のため共有 lib 化はしない）。
  ヘルパの重複は意図的。詳細は `docs/foxbox_userscript_conventions.md`。

## 新規セッションが最初にやること

`CLAUDE.md` 冒頭のチェックリスト参照。要点:
1. `CLAUDE.md` と `docs/` を読む。
2. スキーマ調査/変更時は `add_repo` で `t2-auto/Yatagarasu` を取得（IDL は `src/interfaces/**`）。
3. 変更後は `bash test/run.sh` で検証。
4. **こまめに push**（下記の教訓）。

## 教訓：push していない変更は失われる

このセッション中、実行環境コンテナの再クローンで**ローカルのブランチが過去のコミットへ巻き戻った**。
push 済みだったため `git fetch` + fast-forward で復旧できた。作業コピーは揮発性なので、
まとまった変更は必ず push すること。

## ブランチ運用

- 開発は作業ブランチ（例: `claude/inspiring-hypatia-u4rhnk`）で行い、`main` を正本とする。
- 引き継ぎを確実にするため、本整備は `main` へマージする（PR 経由）。
  新規セッションは `main` からそのまま作業を始められる。

## TODO / 今後の候補

- **統合ダッシュボード**: `ego_lane_segments`（区間・ETC context）／`follower_gap_tracker`（詰まり）／
  `lateral_g_monitor`（実測横G）／`speed_margin_profile`（増速余地）を同一タイムラインに並べる
  Foxglove レイアウトを用意すると報告が一気に楽になる。
- **実データでの妥当性確認**: 合成テストは通るが、実 mcap での follower 検出閾値（`same_lane_l_threshold_m`）
  や横G系統の一致度は要確認。
- **既存スクリプトのテスト追加**: `target_object` / `lane_boundary_tracker` は behavior テスト未整備。
  必要なら `test/behavior.test.cjs` に追加。
- **enum テーブルの追随**: `LocalLaneContext` 等にメンバー追加があればスクリプト内テーブルを更新。

## あなた（オーナー）側で確認/決定した方が良いこと

- **Team ワークスペースで GitHub 連携と対象リポジトリ（特に IDL のある `t2-auto/Yatagarasu`）、
  Jira/Atlassian へのアクセスが維持されるか**。維持されないと新セッションはスキーマ調査ができない。
- 本リポジトリを個人アカウントから **Org へ移管**する予定があるか（remote/リンクの張り替えが発生）。
- コネクタ／スキルの利用可否が Team プランで変わる可能性。
