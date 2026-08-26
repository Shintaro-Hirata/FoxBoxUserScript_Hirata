// ============================================================================
// Ego lane speed limit + left/right neighbor lanes  v1
//
// /t2/object_augmentor/augmented_scene を購読し、
//   1) 自車が属する local_map_info.local_lane_segments の speed_limit_max を
//      毎フレーム出力する（トップレベルの speed_limit_max_kph / _mps を
//      そのまま Plot パネルへ）。
//   2) 自車セグメントの左右隣 (left_neighbor_forward_id /
//      right_neighbor_forward_id) の local_lane_segments 詳細を出力する。
//
// 入力:   /t2/object_augmentor/augmented_scene
// 出力:   /studio_script/ego_lane_speed_limit
//
// Variables パネルで設定する変数: なし
//
// speed_limit_max の意味 (perception_msgs/msg/LocalLaneSegment.idl):
//   「マップ上で定義された制限速度の上限、もしくは標識などにより更新された
//    制限速度の上限（m/s）」= そのセグメントの制限速度。単位は m/s。
//   speed_limit_max_map_debug (地図由来) / speed_limit_max_vlm_debug
//   (VLM 認識由来) も参考値として併記する。
//
// 自車セグメントの特定:
//   local_map_info.ego_lane_segment_indices (augmentor が自車足元 ±0.1m で
//   判定済み) を使用。空の場合は central_curve への原点 (0,0) 最近傍で
//   フォールバック。代表 (primary) は先頭インデックスのセグメント。
//
// 左右レーンの解決:
//   代表自車セグメントの left_neighbor_forward_id / right_neighbor_forward_id
//   (IDL 上それぞれ最大 1 要素) を id として local_lane_segments から検索。
//   代表に無ければ他の自車セグメントの隣接 id で補完する。
//   id はあるがローカルマップ内に実体が無い場合は exists=false のまま
//   id フィールドだけ埋める。
// ============================================================================

import { Input } from "./types";

type Vec3  = { x: number; y: number; z: number };
type Stamp = { sec: number; nsec: number };

type GlobalVariables = Record<string, unknown>;

// --- 入力型 ------------------------------------------------------------------
type InLocationContext = { location?: number | string; context?: number | string };

type InLaneSeg = {
  id?: string;
  road_id?: string;
  successor_ids?: string[];
  predecessor_ids?: string[];
  left_neighbor_forward_id?: string[];
  right_neighbor_forward_id?: string[];
  central_curve?: Vec3[];
  length?: number;
  nth_lane?: number;
  speed_limit_max?: number;
  speed_limit_max_map_debug?: number;
  speed_limit_max_vlm_debug?: number;
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
type LaneInfo = {
  exists: boolean;               // local_lane_segments 内に実体が見つかったか
  id: string;                    // セグメント ID (未解決でも参照 id があれば入る)
  segment_index: number;         // local_lane_segments 内のインデックス (-1 = なし)
  road_id: string;               // 道路 ID
  length_m: number;              // LocalLaneSegment.length
  nth_lane: number;              // 第何レーンか
  // 速度 (speed_limit_* は IDL 上 m/s)
  speed_limit_max_mps: number;   // 制限速度上限 [m/s]
  speed_limit_max_kph: number;   // 制限速度上限 [km/h]
  speed_limit_max_map_kph: number; // 地図由来の制限速度上限 [km/h] (debug)
  speed_limit_max_vlm_kph: number; // VLM 認識由来の制限速度上限 [km/h] (debug)
  speed_limit_min_kph: number;   // 制限速度下限 [km/h]
  target_speed_max_kph: number;  // 目標速度上限 [km/h]
  target_speed_min_kph: number;  // 目標速度下限 [km/h]
  // 車線属性
  lane_classification: number;   // 車線属性種別 (enum 値)
  lane_classification_name: string;
  split_merge_type: number;      // 分合流区分 (enum 値)
  left_lc_permission: number;    // 左車線変更可否 (enum 値)
  left_lc_permission_name: string;
  right_lc_permission: number;   // 右車線変更可否 (enum 値)
  right_lc_permission_name: string;
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
  // 接続・隣接 id
  successor_ids: string[];
  predecessor_ids: string[];
  left_neighbor_ids: string[];
  right_neighbor_ids: string[];
  // 位置・状況コンテキスト (ETC 等)
  location: number;              // LocalLaneLocation enum 値
  location_name: string;
  context: number;               // LocalLaneContext enum 値 (HIGHWAY/PASSING_ETC/...)
  context_name: string;
};

type Output = {
  stamp: Stamp;
  source: string;                // "ego_indices" | "projection_fallback" | "none"
  total_segments_in_map: number; // local_lane_segments の総数
  ego_segment_count: number;     // 自車が属するセグメント数
  ego_segment_ids: string[];     // 自車セグメントの id 一覧
  // --- Plot 向けトップレベル値 (代表自車セグメント) ---
  speed_limit_max_kph: number;   // 自車レーンの制限速度上限 [km/h]
  speed_limit_max_mps: number;   // 自車レーンの制限速度上限 [m/s]
  speed_limit_max_map_kph: number; // 地図由来 [km/h] (debug)
  speed_limit_max_vlm_kph: number; // VLM 由来 [km/h] (debug)
  left_speed_limit_max_kph: number;  // 左隣レーンの制限速度上限 [km/h] (無ければ 0)
  right_speed_limit_max_kph: number; // 右隣レーンの制限速度上限 [km/h] (無ければ 0)
  // --- 詳細 ---
  ego: LaneInfo;                 // 代表自車セグメント
  ego_segments: LaneInfo[];      // 自車が属する全セグメント
  left: LaneInfo;                // 左隣レーン (自車と同方向)
  right: LaneInfo;               // 右隣レーン (自車と同方向)
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
// LocalLaneChangePermissionType
const LC_PERMISSION_NAMES = [
  "NO_LANE", "ALLOWED", "NOT_ALLOWED_REGULATION", "NOT_ALLOWED_PHYSICAL",
  "NOT_ALLOWED_BOTH", "UNKNOWN",
];

function num(v: unknown, dflt: number): number {
  if (typeof v === "number" && isFinite(v)) {
    return v;
  }
  // ROS2 の uint64/int64 (例: ego_lane_segment_indices) は Foxglove では bigint で届く。
  if (typeof v === "bigint") {
    return Number(v);
  }
  return dflt;
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
function projDist(p: Vec3, a: Vec3, b: Vec3): number {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const lenSq = abx * abx + aby * aby;
  if (lenSq === 0) {
    return dist2D(p, a);
  }
  let t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / lenSq;
  t = Math.max(0, Math.min(1, t));
  const px = a.x + t * abx;
  const py = a.y + t * aby;
  const dx = p.x - px;
  const dy = p.y - py;
  return Math.sqrt(dx * dx + dy * dy);
}
// 自車原点 (0,0) から central_curve までの最短距離 (フォールバック判定用)
function egoDistToCurve(curve: Vec3[]): number {
  if (curve.length === 0) {
    return Number.POSITIVE_INFINITY;
  }
  if (curve.length === 1) {
    return Math.sqrt(curve[0]!.x * curve[0]!.x + curve[0]!.y * curve[0]!.y);
  }
  const origin: Vec3 = { x: 0, y: 0, z: 0 };
  let best = Number.POSITIVE_INFINITY;
  for (let i = 0; i < curve.length - 1; i++) {
    const d = projDist(origin, curve[i]!, curve[i + 1]!);
    if (d < best) {
      best = d;
    }
  }
  return best;
}

function emptyLane(): LaneInfo {
  return {
    exists: false,
    id: "",
    segment_index: -1,
    road_id: "",
    length_m: 0,
    nth_lane: 0,
    speed_limit_max_mps: 0,
    speed_limit_max_kph: 0,
    speed_limit_max_map_kph: 0,
    speed_limit_max_vlm_kph: 0,
    speed_limit_min_kph: 0,
    target_speed_max_kph: 0,
    target_speed_min_kph: 0,
    lane_classification: -1,
    lane_classification_name: "",
    split_merge_type: -1,
    left_lc_permission: -1,
    left_lc_permission_name: "",
    right_lc_permission: -1,
    right_lc_permission_name: "",
    is_target_lane: false,
    is_merging_anticipation: false,
    is_route: false,
    is_no_lane: false,
    is_tollgate: false,
    is_out_of_course: false,
    is_tunnel: false,
    is_slower_traffic: false,
    is_truck_lane: false,
    is_blockade_lane: false,
    successor_ids: [],
    predecessor_ids: [],
    left_neighbor_ids: [],
    right_neighbor_ids: [],
    location: -1,
    location_name: "",
    context: -1,
    context_name: "",
  };
}

function buildLane(seg: InLaneSeg, segmentIndex: number): LaneInfo {
  const slMax = num(seg.speed_limit_max, 0);
  const lc = seg.location_context ?? {};
  return {
    exists: true,
    id: typeof seg.id === "string" ? seg.id : "",
    segment_index: segmentIndex,
    road_id: typeof seg.road_id === "string" ? seg.road_id : "",
    length_m: num(seg.length, 0),
    nth_lane: num(seg.nth_lane, 0),
    speed_limit_max_mps: slMax,
    speed_limit_max_kph: slMax * MPS_TO_KPH,
    speed_limit_max_map_kph: num(seg.speed_limit_max_map_debug, 0) * MPS_TO_KPH,
    speed_limit_max_vlm_kph: num(seg.speed_limit_max_vlm_debug, 0) * MPS_TO_KPH,
    speed_limit_min_kph: num(seg.speed_limit_min, 0) * MPS_TO_KPH,
    target_speed_max_kph: num(seg.target_speed_max, 0) * MPS_TO_KPH,
    target_speed_min_kph: num(seg.target_speed_min, 0) * MPS_TO_KPH,
    lane_classification: enumNum(seg.lane_classification),
    lane_classification_name: enumName(seg.lane_classification, CLASSIFICATION_NAMES),
    split_merge_type: enumNum(seg.split_merge_type),
    left_lc_permission: enumNum(seg.left_lc_permission),
    left_lc_permission_name: enumName(seg.left_lc_permission, LC_PERMISSION_NAMES),
    right_lc_permission: enumNum(seg.right_lc_permission),
    right_lc_permission_name: enumName(seg.right_lc_permission, LC_PERMISSION_NAMES),
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
    successor_ids: strArr(seg.successor_ids),
    predecessor_ids: strArr(seg.predecessor_ids),
    left_neighbor_ids: strArr(seg.left_neighbor_forward_id),
    right_neighbor_ids: strArr(seg.right_neighbor_forward_id),
    location: enumNum(lc.location),
    location_name: enumName(lc.location, LOCATION_NAMES),
    context: enumNum(lc.context),
    context_name: enumName(lc.context, CONTEXT_NAMES),
  };
}

export const inputs = ["/t2/object_augmentor/augmented_scene"];
export const output = "/studio_script/ego_lane_speed_limit";

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

  // id → index の索引
  const indexById = new Map<string, number>();
  for (let i = 0; i < segments.length; i++) {
    const sid = segments[i]!.id;
    if (typeof sid === "string" && sid.length > 0) {
      indexById.set(sid, i);
    }
  }

  // --- 自車セグメントの特定 ------------------------------------------------
  const egoIdx: number[] = [];
  let source = "none";

  for (let k = 0; k < indices.length; k++) {
    const idx = Math.trunc(num(indices[k], -1));
    if (idx >= 0 && idx < segments.length) {
      egoIdx.push(idx);
    }
  }
  if (egoIdx.length > 0) {
    source = "ego_indices";
  }

  if (egoIdx.length === 0 && segments.length > 0) {
    let bestIdx = -1;
    let bestD = Number.POSITIVE_INFINITY;
    for (let i = 0; i < segments.length; i++) {
      const curve = segments[i]!.central_curve;
      if (!Array.isArray(curve) || curve.length === 0) {
        continue;
      }
      const d = egoDistToCurve(curve);
      if (d < bestD) {
        bestD = d;
        bestIdx = i;
      }
    }
    if (bestIdx >= 0) {
      egoIdx.push(bestIdx);
      source = "projection_fallback";
    }
  }

  // --- 自車セグメント詳細 ----------------------------------------------------
  const egoLanes: LaneInfo[] = [];
  const egoIds: string[] = [];
  for (const i of egoIdx) {
    const lane = buildLane(segments[i]!, i);
    egoLanes.push(lane);
    egoIds.push(lane.id);
  }
  const primary: LaneInfo = egoLanes.length > 0 ? egoLanes[0]! : emptyLane();

  // --- 左右レーンの解決 ------------------------------------------------------
  // 代表自車セグメントの隣接 id を優先し、無ければ他の自車セグメントで補完。
  function resolveNeighbor(pick: (lane: LaneInfo) => string[]): LaneInfo {
    for (const lane of egoLanes) {
      const ids = pick(lane);
      if (ids.length === 0) {
        continue;
      }
      const id = ids[0]!;
      const idx = indexById.get(id);
      if (idx != null) {
        return buildLane(segments[idx]!, idx);
      }
      // 参照 id はあるがローカルマップ範囲外: id だけ埋めて返す
      const missing = emptyLane();
      missing.id = id;
      return missing;
    }
    return emptyLane();
  }
  const left = resolveNeighbor((l) => l.left_neighbor_ids);
  const right = resolveNeighbor((l) => l.right_neighbor_ids);

  return {
    stamp,
    source,
    total_segments_in_map: segments.length,
    ego_segment_count: egoLanes.length,
    ego_segment_ids: egoIds,
    speed_limit_max_kph: primary.speed_limit_max_kph,
    speed_limit_max_mps: primary.speed_limit_max_mps,
    speed_limit_max_map_kph: primary.speed_limit_max_map_kph,
    speed_limit_max_vlm_kph: primary.speed_limit_max_vlm_kph,
    left_speed_limit_max_kph: left.speed_limit_max_kph,
    right_speed_limit_max_kph: right.speed_limit_max_kph,
    ego: primary,
    ego_segments: egoLanes,
    left,
    right,
  };
}
