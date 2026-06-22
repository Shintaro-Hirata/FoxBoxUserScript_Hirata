// ============================================================================
// Ego lane segment info  v1
//
// /t2/object_augmentor/augmented_scene を購読し、自車が属する lane_segment の
// id と長さ (直線距離 = 弦長, SL 距離 = 弧長) を出力する。
//
// 入力:   /t2/object_augmentor/augmented_scene
// 出力:   /studio_script/ego_lane_segments
//
// Variables パネルで設定する変数: なし
//
// 自車セグメントの特定:
//   local_map_info.ego_lane_segment_indices (augmentor が自車足元 ±0.1m で判定済み)
//   を local_lane_segments のインデックスとして使用。
//   空の場合は central_curve への原点 (0,0) 最近傍セグメントでフォールバック。
//
// 長さの定義:
//   straight_m        : central_curve の始点→終点の 2D 直線距離 (弦長)
//   sl_m              : central_curve の 2D 累積弧長 (SL 距離, 道路に沿った長さ)
//   provided_length_m : LocalLaneSegment.length (augmentor 提供値, 参考/検証用)
//
// 合計 (total_*) は ego_lane_segment_indices に含まれる全セグメントの和。
// 通常は 1 セグメントだが、セグメント境界をまたぐ瞬間などは複数になり得る。
// ============================================================================

import { Input } from "./types";

type Vec3  = { x: number; y: number; z: number };
type Stamp = { sec: number; nsec: number };

type GlobalVariables = Record<string, unknown>;

// --- 入力型 (必要なフィールドのみ) ------------------------------------------
type InLaneBoundary = { curve?: Vec3[] };

type InLaneSeg = {
  id?: string;
  road_id?: string;
  central_curve?: Vec3[];
  left_boundary?: InLaneBoundary;
  right_boundary?: InLaneBoundary;
  length?: number;
  nth_lane?: number;
  speed_limit_max?: number;
  speed_limit_min?: number;
  target_speed_max?: number;
  is_target_lane?: boolean;
  is_tollgate?: boolean;
  is_tunnel?: boolean;
  is_no_lane?: boolean;
  lane_classification?: number;
};

type InLocalMapInfo = {
  local_lane_segments?: InLaneSeg[];
  ego_lane_segment_indices?: number[];
};

type InAugmentedScene = {
  header?: unknown;
  local_map_info?: InLocalMapInfo;
};

// --- 出力型 (optional / union 禁止: 全フィールド固定型) ----------------------
type EgoSegInfo = {
  list_index: number;            // ego_lane_segment_indices 内の並び順
  segment_index: number;         // local_lane_segments 内のインデックス
  id: string;                    // セグメント ID (HD マップ由来, フレーム間で安定)
  straight_m: number;            // 直線距離 (弦長, 始点→終点)
  sl_m: number;                  // SL 距離 (弧長, central_curve 累積)
  provided_length_m: number;     // LocalLaneSegment.length (提供値)
  point_count: number;           // central_curve 点数
  ego_s_in_segment_m: number;    // 自車原点をこのセグメントに投影した弧長 (始点基準)
  speed_limit_max_mps: number;   // 制限速度上限 [m/s]
  speed_limit_max_kph: number;   // 制限速度上限 [km/h]
  speed_limit_min_mps: number;   // 制限速度下限 [m/s]
  nth_lane: number;              // 第何レーンか
  lane_classification: number;   // 車線属性種別 (enum 値)
  is_target_lane: boolean;       // 推奨走行レーンか
  is_tollgate: boolean;          // 料金所 (ETC) 区間か
  is_tunnel: boolean;            // トンネル区間か
  road_id: string;               // 道路 ID
};

type Output = {
  stamp: Stamp;                  // event.receiveTime (タイムライン用)
  source: string;               // "ego_indices" | "projection_fallback" | "none"
  total_segments_in_map: number; // local_lane_segments の総数
  ego_segment_count: number;     // 自車が属するセグメント数
  ego_segment_ids: string[];     // 自車セグメントの id 一覧
  primary_segment_id: string;    // 代表セグメント (先頭) の id
  segments: EgoSegInfo[];        // 自車セグメントの詳細
  total_straight_m: number;      // 直線距離の合計 (弦長の和)
  total_sl_m: number;            // SL 距離の合計 (弧長の和)
  total_provided_m: number;      // 提供 length の合計
};

const MPS_TO_KPH = 3.6;

function num(v: unknown, dflt: number): number {
  return typeof v === "number" && isFinite(v) ? v : dflt;
}

function dist2D(a: Vec3, b: Vec3): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return Math.sqrt(dx * dx + dy * dy);
}

// central_curve の 2D 累積弧長 (SL 距離)。
function arcLength2D(curve: Vec3[]): number {
  if (curve.length < 2) {
    return 0;
  }
  let s = 0;
  for (let i = 1; i < curve.length; i++) {
    s += dist2D(curve[i]!, curve[i - 1]!);
  }
  return s;
}

// 始点→終点の 2D 直線距離 (弦長)。
function chord2D(curve: Vec3[]): number {
  if (curve.length < 2) {
    return 0;
  }
  return dist2D(curve[0]!, curve[curve.length - 1]!);
}

// 点 p を線分 a-b に 2D 投影し、内分パラメータ t (0..1) と距離を返す。
function projParam(p: Vec3, a: Vec3, b: Vec3): { t: number; dist: number } {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const lenSq = abx * abx + aby * aby;
  if (lenSq === 0) {
    return { t: 0, dist: dist2D(p, a) };
  }
  let t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / lenSq;
  t = Math.max(0, Math.min(1, t));
  const px = a.x + t * abx;
  const py = a.y + t * aby;
  const dx = p.x - px;
  const dy = p.y - py;
  return { t, dist: Math.sqrt(dx * dx + dy * dy) };
}

// 自車原点 (0,0) を central_curve に投影し、始点からの弧長 (= ego の S) を返す。
// あわせて curve への最短距離も返す (フォールバックのセグメント選択に使用)。
function egoSInSegment(curve: Vec3[]): { s: number; dist: number } {
  if (curve.length === 0) {
    return { s: 0, dist: Number.POSITIVE_INFINITY };
  }
  if (curve.length === 1) {
    return { s: 0, dist: Math.sqrt(curve[0]!.x * curve[0]!.x + curve[0]!.y * curve[0]!.y) };
  }
  const origin: Vec3 = { x: 0, y: 0, z: 0 };
  let acc = 0;
  let bestS = 0;
  let bestD = Number.POSITIVE_INFINITY;
  for (let i = 0; i < curve.length - 1; i++) {
    const segLen = dist2D(curve[i]!, curve[i + 1]!);
    const r = projParam(origin, curve[i]!, curve[i + 1]!);
    if (r.dist < bestD) {
      bestD = r.dist;
      bestS = acc + r.t * segLen;
    }
    acc += segLen;
  }
  return { s: bestS, dist: bestD };
}

function buildSegInfo(seg: InLaneSeg, listIndex: number, segmentIndex: number): EgoSegInfo {
  const curve: Vec3[] = Array.isArray(seg.central_curve) ? seg.central_curve : [];
  const slMps = num(seg.speed_limit_max, 0);
  const minMps = num(seg.speed_limit_min, 0);
  const ego = egoSInSegment(curve);
  return {
    list_index: listIndex,
    segment_index: segmentIndex,
    id: typeof seg.id === "string" ? seg.id : "",
    straight_m: chord2D(curve),
    sl_m: arcLength2D(curve),
    provided_length_m: num(seg.length, 0),
    point_count: curve.length,
    ego_s_in_segment_m: ego.s,
    speed_limit_max_mps: slMps,
    speed_limit_max_kph: slMps * MPS_TO_KPH,
    speed_limit_min_mps: minMps,
    nth_lane: num(seg.nth_lane, 0),
    lane_classification: num(seg.lane_classification, -1),
    is_target_lane: seg.is_target_lane === true,
    is_tollgate: seg.is_tollgate === true,
    is_tunnel: seg.is_tunnel === true,
    road_id: typeof seg.road_id === "string" ? seg.road_id : "",
  };
}

export const inputs = ["/t2/object_augmentor/augmented_scene"];
export const output = "/studio_script/ego_lane_segments";

export default function script(
  event: Input<"/t2/object_augmentor/augmented_scene">,
  _globalVars: GlobalVariables,
): Output {
  const rt = event.receiveTime ?? { sec: 0, nsec: 0 };
  const stamp: Stamp = { sec: num(rt.sec, 0), nsec: num(rt.nsec, 0) };

  const msg = event.message as unknown as InAugmentedScene;
  const info: InLocalMapInfo = msg.local_map_info ?? {};
  const segments: InLaneSeg[] = Array.isArray(info.local_lane_segments)
    ? info.local_lane_segments : [];
  const indices: number[] = Array.isArray(info.ego_lane_segment_indices)
    ? info.ego_lane_segment_indices : [];

  const out: EgoSegInfo[] = [];
  let source = "none";

  // (A) ego_lane_segment_indices を primary に使用
  if (indices.length > 0) {
    for (let k = 0; k < indices.length; k++) {
      const idx = Math.trunc(num(indices[k], -1));
      if (idx < 0 || idx >= segments.length) {
        continue;
      }
      out.push(buildSegInfo(segments[idx]!, k, idx));
    }
    if (out.length > 0) {
      source = "ego_indices";
    }
  }

  // (B) フォールバック: 原点 (0,0) に最も近い central_curve を持つセグメント
  if (out.length === 0 && segments.length > 0) {
    let bestIdx = -1;
    let bestD = Number.POSITIVE_INFINITY;
    for (let i = 0; i < segments.length; i++) {
      const curve = segments[i]!.central_curve;
      if (!Array.isArray(curve) || curve.length === 0) {
        continue;
      }
      const d = egoSInSegment(curve).dist;
      if (d < bestD) {
        bestD = d;
        bestIdx = i;
      }
    }
    if (bestIdx >= 0) {
      out.push(buildSegInfo(segments[bestIdx]!, 0, bestIdx));
      source = "projection_fallback";
    }
  }

  let totalStraight = 0;
  let totalSl = 0;
  let totalProvided = 0;
  const ids: string[] = [];
  for (const s of out) {
    totalStraight += s.straight_m;
    totalSl += s.sl_m;
    totalProvided += s.provided_length_m;
    ids.push(s.id);
  }

  return {
    stamp,
    source,
    total_segments_in_map: segments.length,
    ego_segment_count: out.length,
    ego_segment_ids: ids,
    primary_segment_id: out.length > 0 ? out[0]!.id : "",
    segments: out,
    total_straight_m: totalStraight,
    total_sl_m: totalSl,
    total_provided_m: totalProvided,
  };
}
