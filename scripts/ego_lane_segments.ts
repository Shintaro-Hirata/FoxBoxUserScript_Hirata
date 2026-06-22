// ============================================================================
// Ego lane segment info  v2
//
// /t2/object_augmentor/augmented_scene を購読し、自車が属する lane_segment の
// id・長さ (直線距離=弦長 / SL距離=弧長) と、隣接 (左右)・前後 (predecessor /
// successor) のセグメント id、さらに取得した各 id の local_lane_segments 詳細
// (lane_augmentor 出力の全フィールド) を出力する。
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
//   straight_m        : central_curve 始点→終点の 2D 直線距離 (弦長)
//   sl_m              : central_curve の 2D 累積弧長 (SL 距離, 道路に沿った長さ)
//   provided_length_m : LocalLaneSegment.length (augmentor 提供値, 参考)
//
// 関連セグメント (related_segments):
//   各自車セグメントの successor / predecessor / left_neighbor / right_neighbor
//   の id を解決し、local_lane_segments に存在すればその詳細を出力する。
//   relation フィールドで自車との関係が分かる。
// ============================================================================

import { Input } from "./types";

type Vec3  = { x: number; y: number; z: number };
type Stamp = { sec: number; nsec: number };

type GlobalVariables = Record<string, unknown>;

// --- 入力型 ------------------------------------------------------------------
type InLaneBoundary = { curve?: Vec3[] };
type InLocationContext = { location?: number | string; context?: number | string };

type InLaneSeg = {
  id?: string;
  road_id?: string;
  successor_ids?: string[];
  predecessor_ids?: string[];
  left_neighbor_forward_id?: string[];
  right_neighbor_forward_id?: string[];
  central_curve?: Vec3[];
  left_boundary?: InLaneBoundary;
  right_boundary?: InLaneBoundary;
  length?: number;
  nth_lane?: number;
  speed_limit_max?: number;
  speed_limit_min?: number;
  target_speed_max?: number;
  target_speed_min?: number;
  is_target_lane?: boolean;
  is_merging_anticipation?: boolean;
  is_route?: boolean;
  is_no_lane?: boolean;
  is_tollgate?: boolean;
  is_out_of_course?: boolean;
  is_tunnel?: boolean;
  is_slower_traffic?: boolean;
  is_truck_lane?: boolean;
  is_blockade_lane?: boolean;
  lane_classification?: number | string;
  split_merge_type?: number | string;
  left_lc_permission?: number | string;
  right_lc_permission?: number | string;
  location_context?: InLocationContext;
};

type InLocalMapInfo = {
  local_lane_segments?: InLaneSeg[];
  ego_lane_segment_indices?: number[];
};

type InAugmentedScene = {
  local_map_info?: InLocalMapInfo;
};

// --- 出力型 (optional / union 禁止: 全フィールド固定型) ----------------------
type SegmentDetail = {
  relation: string;              // "ego" | "predecessor" | "successor" | "left_neighbor" | "right_neighbor"
  relative_to_ego_id: string;    // この関連の起点となった自車セグメント id ("" = 自車自身)
  segment_index: number;         // local_lane_segments 内のインデックス
  id: string;                    // セグメント ID (HD マップ由来, フレーム間で安定)
  road_id: string;               // 道路 ID
  // ジオメトリ
  straight_m: number;            // 直線距離 (弦長)
  sl_m: number;                  // SL 距離 (弧長)
  provided_length_m: number;     // LocalLaneSegment.length
  point_count: number;           // central_curve 点数
  ego_s_in_segment_m: number;    // 自車原点をこのセグメントに投影した弧長 (始点基準)
  // 接続・隣接
  successor_ids: string[];       // 退出側 (直後) セグメント id
  predecessor_ids: string[];     // 進入側 (直前) セグメント id
  left_neighbor_ids: string[];   // 左隣の同方向セグメント id
  right_neighbor_ids: string[];  // 右隣の同方向セグメント id
  // 速度
  speed_limit_max_kph: number;   // 制限速度上限 [km/h]
  speed_limit_min_kph: number;   // 制限速度下限 [km/h]
  speed_limit_max_mps: number;   // 制限速度上限 [m/s]
  target_speed_max_kph: number;  // 目標速度上限 [km/h]
  target_speed_min_kph: number;  // 目標速度下限 [km/h]
  // 車線属性
  nth_lane: number;              // 第何レーンか
  lane_classification: number;   // 車線属性種別 (enum 値)
  lane_classification_name: string;
  split_merge_type: number;      // 分合流区分 (enum 値)
  left_lc_permission: number;    // 左車線変更可否 (enum 値)
  right_lc_permission: number;   // 右車線変更可否 (enum 値)
  is_target_lane: boolean;       // 推奨走行レーンか
  is_merging_anticipation: boolean;
  is_route: boolean;
  is_no_lane: boolean;
  is_tollgate: boolean;          // 料金所 (ETC) 区間か
  is_out_of_course: boolean;
  is_tunnel: boolean;
  is_slower_traffic: boolean;
  is_truck_lane: boolean;
  is_blockade_lane: boolean;
  // 位置・状況コンテキスト (ETC 等)
  location: number;              // LocalLaneLocation enum 値
  location_name: string;
  context: number;               // LocalLaneContext enum 値 (HIGHWAY/ENTERING_ETC/...)
  context_name: string;
};

type Output = {
  stamp: Stamp;
  source: string;                // "ego_indices" | "projection_fallback" | "none"
  total_segments_in_map: number; // local_lane_segments の総数
  ego_segment_count: number;     // 自車が属するセグメント数
  ego_segment_ids: string[];     // 自車セグメントの id 一覧
  primary_segment_id: string;    // 代表セグメント (先頭) の id
  primary_context_name: string;  // 代表セグメントの位置コンテキスト名 (ETC 等)
  segments: SegmentDetail[];     // 自車セグメントの詳細 (relation="ego")
  related_segment_count: number;
  related_segments: SegmentDetail[]; // 隣接/前後セグメントの詳細
  total_straight_m: number;      // 直線距離の合計 (自車セグメント)
  total_sl_m: number;            // SL 距離の合計 (自車セグメント)
  total_provided_m: number;      // 提供 length の合計
};

const MPS_TO_KPH = 3.6;

// LocalLaneContext (LocalLaneLocationContext.idl)
const CONTEXT_NAMES = [
  "UNKNOWN", "HIGHWAY", "ENTERING_ETC", "PASSING_ETC", "EXITING_ETC",
  "URBAN_ROADS", "SPECIAL1", "BASE", "DEPARTURE_SPOT", "ARRIVAL_SPOT",
];
// LocalLaneLocation
const LOCATION_NAMES = [
  "UNKNOWN", "GENERIC", "KOBE_NISHI_IC_NOBORI_ENTRANCE", "KOBE_NISHI_IC_KUDARI_EXIT",
  "AYASE_SIC_KUDARI_ENTRANCE", "AYASE_SIC_NOBORI_EXIT",
  "NISHINOMIYA_KITA_IC_NOBORI_ENTRANCE", "NISHINOMIYA_KITA_IC_KUDARI_EXIT",
  "KOBE_NISHI_BASE", "AYASE_BASE", "NISHINOMIYA_KITA_BASE",
];
// LocalLaneClassificationType
const CLASSIFICATION_NAMES = [
  "UNKNOWN", "PASSING_LANE", "DRIVING_LANE", "VEHICLES_STOP_LANE", "SIDE_LANE_SPLIT",
  "SIDE_LANE_MERGE", "SPLIT", "MERGE", "ACCELERATION_LANE", "DECELERATION_LANE",
  "INCREASE_OF_LANES", "DECREASE_OF_LANES", "INCREASE_OF_NO_LANES", "DECREASE_OF_NO_LANES",
  "NO_LANES", "PARKING_SLOT", "SIDEWALK",
];

function num(v: unknown, dflt: number): number {
  return typeof v === "number" && isFinite(v) ? v : dflt;
}
// enum フィールドを数値として読む (Foxglove では整数, 念のため文字列も許容)
function enumNum(v: unknown): number {
  if (typeof v === "number" && isFinite(v)) {
    return v;
  }
  if (typeof v === "string") {
    const n = Number(v);
    if (isFinite(n)) {
      return n;
    }
  }
  return -1;
}
function enumName(v: unknown, names: string[]): string {
  if (typeof v === "string" && !isFinite(Number(v))) {
    return v;
  }
  const n = enumNum(v);
  return n >= 0 && n < names.length ? names[n]! : String(n);
}
function strArr(v: unknown): string[] {
  if (!Array.isArray(v)) {
    return [];
  }
  const out: string[] = [];
  for (const x of v) {
    if (typeof x === "string" && x.length > 0) {
      out.push(x);
    }
  }
  return out;
}

function dist2D(a: Vec3, b: Vec3): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return Math.sqrt(dx * dx + dy * dy);
}
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
function chord2D(curve: Vec3[]): number {
  if (curve.length < 2) {
    return 0;
  }
  return dist2D(curve[0]!, curve[curve.length - 1]!);
}
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

function buildDetail(
  seg: InLaneSeg, segmentIndex: number, relation: string, relativeTo: string,
): SegmentDetail {
  const curve: Vec3[] = Array.isArray(seg.central_curve) ? seg.central_curve : [];
  const slMax = num(seg.speed_limit_max, 0);
  const slMin = num(seg.speed_limit_min, 0);
  const lc = seg.location_context ?? {};
  return {
    relation,
    relative_to_ego_id: relativeTo,
    segment_index: segmentIndex,
    id: typeof seg.id === "string" ? seg.id : "",
    road_id: typeof seg.road_id === "string" ? seg.road_id : "",
    straight_m: chord2D(curve),
    sl_m: arcLength2D(curve),
    provided_length_m: num(seg.length, 0),
    point_count: curve.length,
    ego_s_in_segment_m: egoSInSegment(curve).s,
    successor_ids: strArr(seg.successor_ids),
    predecessor_ids: strArr(seg.predecessor_ids),
    left_neighbor_ids: strArr(seg.left_neighbor_forward_id),
    right_neighbor_ids: strArr(seg.right_neighbor_forward_id),
    speed_limit_max_kph: slMax * MPS_TO_KPH,
    speed_limit_min_kph: slMin * MPS_TO_KPH,
    speed_limit_max_mps: slMax,
    target_speed_max_kph: num(seg.target_speed_max, 0) * MPS_TO_KPH,
    target_speed_min_kph: num(seg.target_speed_min, 0) * MPS_TO_KPH,
    nth_lane: num(seg.nth_lane, 0),
    lane_classification: enumNum(seg.lane_classification),
    lane_classification_name: enumName(seg.lane_classification, CLASSIFICATION_NAMES),
    split_merge_type: enumNum(seg.split_merge_type),
    left_lc_permission: enumNum(seg.left_lc_permission),
    right_lc_permission: enumNum(seg.right_lc_permission),
    is_target_lane: seg.is_target_lane === true,
    is_merging_anticipation: seg.is_merging_anticipation === true,
    is_route: seg.is_route === true,
    is_no_lane: seg.is_no_lane === true,
    is_tollgate: seg.is_tollgate === true,
    is_out_of_course: seg.is_out_of_course === true,
    is_tunnel: seg.is_tunnel === true,
    is_slower_traffic: seg.is_slower_traffic === true,
    is_truck_lane: seg.is_truck_lane === true,
    is_blockade_lane: seg.is_blockade_lane === true,
    location: enumNum(lc.location),
    location_name: enumName(lc.location, LOCATION_NAMES),
    context: enumNum(lc.context),
    context_name: enumName(lc.context, CONTEXT_NAMES),
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

  // id → {seg, index} の索引
  const segById = new Map<string, { seg: InLaneSeg; index: number }>();
  for (let i = 0; i < segments.length; i++) {
    const sid = segments[i]!.id;
    if (typeof sid === "string" && sid.length > 0) {
      segById.set(sid, { seg: segments[i]!, index: i });
    }
  }

  // --- 自車セグメントの特定 ------------------------------------------------
  const egoSegs: { seg: InLaneSeg; index: number }[] = [];
  let source = "none";

  if (indices.length > 0) {
    for (let k = 0; k < indices.length; k++) {
      const idx = Math.trunc(num(indices[k], -1));
      if (idx < 0 || idx >= segments.length) {
        continue;
      }
      egoSegs.push({ seg: segments[idx]!, index: idx });
    }
    if (egoSegs.length > 0) {
      source = "ego_indices";
    }
  }

  if (egoSegs.length === 0 && segments.length > 0) {
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
      egoSegs.push({ seg: segments[bestIdx]!, index: bestIdx });
      source = "projection_fallback";
    }
  }

  // --- 自車セグメント詳細 + 合計 ------------------------------------------
  const egoDetails: SegmentDetail[] = [];
  const egoIdSet = new Set<string>();
  let totalStraight = 0;
  let totalSl = 0;
  let totalProvided = 0;
  for (const e of egoSegs) {
    const d = buildDetail(e.seg, e.index, "ego", "");
    egoDetails.push(d);
    egoIdSet.add(d.id);
    totalStraight += d.straight_m;
    totalSl += d.sl_m;
    totalProvided += d.provided_length_m;
  }

  // --- 関連セグメント (左右・前後) を解決 ----------------------------------
  // 各自車セグメントの接続/隣接 id を relation 付きで集め、local_lane_segments
  // に存在するものを詳細化する。id 重複は最初の relation を採用。
  const relatedDetails: SegmentDetail[] = [];
  const relatedSeen = new Set<string>();
  function addRelated(ids: string[], relation: string, egoId: string): void {
    for (const id of ids) {
      if (egoIdSet.has(id) || relatedSeen.has(id)) {
        continue;
      }
      const hit = segById.get(id);
      if (hit == null) {
        continue;
      }
      relatedSeen.add(id);
      relatedDetails.push(buildDetail(hit.seg, hit.index, relation, egoId));
    }
  }
  for (const d of egoDetails) {
    addRelated(d.predecessor_ids, "predecessor", d.id);
    addRelated(d.successor_ids, "successor", d.id);
    addRelated(d.left_neighbor_ids, "left_neighbor", d.id);
    addRelated(d.right_neighbor_ids, "right_neighbor", d.id);
  }

  const ids: string[] = [];
  for (const d of egoDetails) {
    ids.push(d.id);
  }

  return {
    stamp,
    source,
    total_segments_in_map: segments.length,
    ego_segment_count: egoDetails.length,
    ego_segment_ids: ids,
    primary_segment_id: egoDetails.length > 0 ? egoDetails[0]!.id : "",
    primary_context_name: egoDetails.length > 0 ? egoDetails[0]!.context_name : "",
    segments: egoDetails,
    related_segment_count: relatedDetails.length,
    related_segments: relatedDetails,
    total_straight_m: totalStraight,
    total_sl_m: totalSl,
    total_provided_m: totalProvided,
  };
}
