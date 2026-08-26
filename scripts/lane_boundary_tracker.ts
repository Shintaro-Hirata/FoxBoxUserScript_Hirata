// ============================================================================
// Lane boundary tracker + SL distance calculator  v15
//
// /t2/object_augmentor/augmented_scene を購読し、track_id で指定した物体から
// ターゲット最近傍セグメントの左右境界線までの距離を SL 座標系で算出する。
//
// 入力:   /t2/object_augmentor/augmented_scene (単一トピック)
// 出力:   /studio_script/lane_boundary_tracker
//
// Variables パネルで設定する変数:
//   track_id : number  (追跡する物体の track_id。target_object と同じ値)
//
// ターゲット検索:
//   augmented_objects から track_id (bbox_info.id 等) で直接検索。
//   bev_detection 不要、Seek 時のターゲット消失なし。
//
// セグメント選択:
//   (A) ターゲット取得済み → central_curve への 2D 最短距離で自動選択
//   (B) ターゲットなし → is_target_lane=true を fallback
//   白線 (left/right boundary) はターゲット最近傍セグメントから取得。
//
// SL 計算:
//   central_curve は ego レーンを successor_ids/predecessor_ids で ID ベース結合。
//   S=0 は自車の polyline 投影点。正=前方, 負=後方。
//   白線の L 値はターゲットの S 位置で線形補間。
//
// 主な出力:
//   target_s_m / target_l_m              : SL 座標
//   distance_target_edge_to_left_boundary_sl_m  : 車両端↔左白線 (SL)
//   distance_target_edge_to_right_boundary_sl_m : 車両端↔右白線 (SL)
//   distance_target_edge_to_left_boundary_m     : 同上 (Cartesian 参考値)
// ============================================================================

import { Input } from "./types";

type Vec3   = { x: number; y: number; z: number };
type Stamp  = { sec: number; nsec: number };
type Header = { seq: number; stamp: Stamp; frame_id: string };

type GlobalVariables = {
  track_id?: number | string;
  target_x?: number;
  target_y?: number;
  target_z?: number;
  threshold_m?: number | string;
  ma_window?: number | string;
};

// --- 入力型 (union OK) ------------------------------------------------------
type InBBoxInfo = {
  id?: number;
  local_position?: Vec3;
  local_theta?: number;
  length?: number;
  width?: number;
  height?: number;
  type?: number;
  sub_type?: number;
  confidence?: number;
};

type InAugObj = {
  bbox_info: InBBoxInfo;
  tracking_info?: { velocity?: Vec3; track_id?: number };
  track_id?: number;
};

type InLaneSeg = {
  id: string;
  road_id?: string;
  successor_ids?: string[];
  predecessor_ids?: string[];
  central_curve: Vec3[];
  left_boundary: { curve: Vec3[] };
  right_boundary: { curve: Vec3[] };
  is_target_lane: boolean;
};

// --- 出力型 (union / optional 禁止) -----------------------------------------
type SegmentSummary = {
  id: string;
  is_target_lane: boolean;
  dist_to_target_m: number;
};

type Output = {
  header: Header;
  track_id_input: number;
  segment_found: boolean;
  segment_id: string;
  segment_is_target_lane: boolean;
  left_boundary_curve: Vec3[];
  left_boundary_point_count: number;
  right_boundary_curve: Vec3[];
  right_boundary_point_count: number;
  target_found: boolean;
  target_track_id: number;
  bev_match_distance_m: number;
  target_local_position: Vec3;
  target_width: number;
  target_width_source: string;
  distance_computed: boolean;
  distance_to_left_boundary_m: number;
  distance_target_edge_to_left_boundary_m: number;
  nearest_point_on_left_curve: Vec3;
  nearest_segment_index: number;
  distance_to_right_boundary_m: number;
  distance_target_edge_to_right_boundary_m: number;
  nearest_point_on_right_curve: Vec3;
  available_segments: SegmentSummary[];
  available_segment_count: number;
  // ---- SL 座標系 ----
  sl_valid: boolean;
  sl_central_curve_source: string;
  chained_segment_count: number;
  central_curve_total_length_m: number;
  central_curve_point_count: number;
  ego_closest_curve_index: number;
  target_s_m: number;
  target_s_chord_m: number;
  target_l_m: number;
  target_central_projection: Vec3;
  target_central_segment_index: number;
  left_boundary_l_at_target_s_m: number;
  left_boundary_l_bracket_mode: string;
  distance_target_to_left_boundary_sl_m: number;
  distance_target_edge_to_left_boundary_sl_m: number;
  right_boundary_l_at_target_s_m: number;
  right_boundary_l_bracket_mode: string;
  distance_target_to_right_boundary_sl_m: number;
  distance_target_edge_to_right_boundary_sl_m: number;
  // 全セグメント横断で最も近い白線
  nearest_boundary_distance_m: number;
  nearest_boundary_edge_distance_m: number;
  nearest_boundary_segment_id: string;
  // 移動平均
  ma_window: number;
  target_s_ma: number;
  target_l_ma: number;
  distance_target_edge_to_left_boundary_sl_ma: number;
  distance_target_edge_to_right_boundary_sl_ma: number;
  nearest_boundary_edge_distance_ma: number;
  nearest_boundary_side: string;
};

// --- ユーティリティ ---------------------------------------------------------
function zeroVec3(): Vec3 { return { x: 0, y: 0, z: 0 }; }
function copyVec3(v: Vec3): Vec3 { return { x: v.x, y: v.y, z: v.z }; }

function dist2D(a: Vec3, b: Vec3): number {
  const dx = a.x - b.x; const dy = a.y - b.y;
  return Math.sqrt(dx * dx + dy * dy);
}

function isValidVec3(v: unknown): v is Vec3 {
  if (v == null || typeof v !== "object") return false;
  const o = v as { x?: unknown; y?: unknown; z?: unknown };
  return typeof o.x === "number" && typeof o.y === "number" && typeof o.z === "number";
}

function dist3(a: Vec3, b: Vec3): number {
  const dx = a.x - b.x; const dy = a.y - b.y; const dz = a.z - b.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function ptCurve2D(p: Vec3, c: Vec3[]): { dist: number; point: Vec3; index: number } {
  if (c.length === 0) return { dist: -1, point: zeroVec3(), index: -1 };
  if (c.length === 1) return { dist: dist2D(p, c[0]!), point: copyVec3(c[0]!), index: 0 };
  let bd = Number.POSITIVE_INFINITY; let bp = zeroVec3(); let bi = 0;
  for (let i = 0; i < c.length - 1; i++) {
    const r = ptSegXYWithT(p, c[i]!, c[i + 1]!);
    if (r.dist2D < bd) { bd = r.dist2D; bp = r.point; bi = i; }
  }
  return { dist: bd, point: bp, index: bi };
}

// --- SL 座標系ユーティリティ ------------------------------------------------
// 2D (x-y 平面) で点 p を線分 a-b に投影し、パラメータ t と投影点を返す。
function ptSegXYWithT(p: Vec3, a: Vec3, b: Vec3): {
  t: number; point: Vec3; dist2D: number;
} {
  const abx = b.x - a.x; const aby = b.y - a.y;
  const lenSq = abx * abx + aby * aby;
  if (lenSq === 0) {
    const dx = p.x - a.x; const dy = p.y - a.y;
    return { t: 0, point: copyVec3(a), dist2D: Math.sqrt(dx * dx + dy * dy) };
  }
  const t = Math.max(0, Math.min(1,
    ((p.x - a.x) * abx + (p.y - a.y) * aby) / lenSq));
  const proj: Vec3 = { x: a.x + t * abx, y: a.y + t * aby, z: a.z + t * (b.z - a.z) };
  const dx = p.x - proj.x; const dy = p.y - proj.y;
  return { t, point: proj, dist2D: Math.sqrt(dx * dx + dy * dy) };
}

// central_curve 上の自車原点 (0,0) に最も近い「polyline 上の点」を S=0 とした
// 累積弧長を返す。頂点間の補間点も考慮するため、S=0 の位置が正確。
// 社内 Python 版 vehicle_coord_to_sl と同一の S 原点規約。
function cumulativeSFromEgo(c: Vec3[]): { cumS: number[]; egoClosestIdx: number } {
  if (c.length === 0) return { cumS: [], egoClosestIdx: -1 };

  // 標準累積弧長 (curve[0] = 0)
  const raw: number[] = [0];
  for (let i = 1; i < c.length; i++) {
    const dx = c[i]!.x - c[i - 1]!.x;
    const dy = c[i]!.y - c[i - 1]!.y;
    raw.push(raw[i - 1]! + Math.sqrt(dx * dx + dy * dy));
  }

  // 自車原点 (0,0) に最も近い polyline 上の点を探索 (頂点間の補間点も含む)
  const origin = zeroVec3();
  let bestD = Number.POSITIVE_INFINITY;
  let bestS = 0;
  let bestIdx = 0;

  for (let i = 0; i < c.length - 1; i++) {
    const r = ptSegXYWithT(origin, c[i]!, c[i + 1]!);
    if (r.dist2D < bestD) {
      bestD = r.dist2D;
      const segDx = c[i + 1]!.x - c[i]!.x;
      const segDy = c[i + 1]!.y - c[i]!.y;
      bestS = raw[i]! + r.t * Math.sqrt(segDx * segDx + segDy * segDy);
      bestIdx = i;
    }
  }

  // オフセットして S=0 を自車最近傍投影点に設定
  const cumS: number[] = [];
  for (let i = 0; i < raw.length; i++) {
    cumS.push(raw[i]! - bestS);
  }

  return { cumS, egoClosestIdx: bestIdx };
}

// --- セグメント結合 (双方向) ------------------------------------------------
// ego segment を +x 方向に固定し、前方・後方の両方向に lane segment を結合する。
// 結合候補は successor_ids / predecessor_ids による ID ベース接続のみ
// (他車線を拾うリスクを避けるため endpoint 近接フォールバックは廃止)。
function chainLaneBidir(
  segments: InLaneSeg[]
): { central: Vec3[]; left: Vec3[]; right: Vec3[]; count: number } {
  const empty = { central: [] as Vec3[], left: [] as Vec3[], right: [] as Vec3[], count: 0 };
  if (segments.length === 0) return empty;

  // segment を id で引くための Map
  const segById = new Map<string, InLaneSeg>();
  for (const s of segments) { segById.set(s.id, s); }

  // ego segment: origin(0,0) に最も近い central_curve 点を持つセグメント
  let egoSeg: InLaneSeg | undefined;
  let egoMinD = Number.POSITIVE_INFINITY;
  for (const s of segments) {
    for (const p of s.central_curve) {
      const d = p.x * p.x + p.y * p.y;
      if (d < egoMinD) { egoMinD = d; egoSeg = s; }
    }
  }
  if (!egoSeg || egoSeg.central_curve.length < 2) return empty;

  function ori(s: InLaneSeg, rev: boolean): { cc: Vec3[]; lb: Vec3[]; rb: Vec3[] } {
    if (!rev) return {
      cc: s.central_curve.map(copyVec3),
      lb: s.left_boundary.curve.map(copyVec3),
      rb: s.right_boundary.curve.map(copyVec3),
    };
    return {
      cc: [...s.central_curve].reverse().map(copyVec3),
      lb: [...s.right_boundary.curve].reverse().map(copyVec3),
      rb: [...s.left_boundary.curve].reverse().map(copyVec3),
    };
  }

  // ID ベースで次のセグメントを探索 (successor_ids から未使用の最初の 1 つ)
  function findByIds(ids: string[] | undefined, usedSet: Set<string>): InLaneSeg | undefined {
    if (!Array.isArray(ids)) return undefined;
    for (const sid of ids) {
      if (usedSet.has(sid)) continue;
      const found = segById.get(sid);
      if (found && found.central_curve.length >= 2) return found;
    }
    return undefined;
  }

  // ego segment を +x 方向に固定
  const ec = egoSeg.central_curve;
  const egoRev = ec[ec.length - 1]!.x < ec[0]!.x;
  const ego = ori(egoSeg, egoRev);
  const used = new Set([egoSeg.id]);

  // 前方結合
  let fCC: Vec3[] = []; let fLB: Vec3[] = []; let fRB: Vec3[] = [];
  let curSeg = egoSeg;
  let curRev = egoRev;
  for (let n = 0; n < 30; n++) {
    // (1) successor_ids で探索
    const succIds = curRev ? curSeg.predecessor_ids : curSeg.successor_ids;
    let next = findByIds(succIds, used);
    let nextRev = false;

    if (!next) break; // ID がなければ結合停止 (endpoint フォールバックなし)
    // orient: next の先頭が chain tip に近ければそのまま、末尾が近ければ反転
    const tip = fCC.length > 0 ? fCC[fCC.length - 1]! : ego.cc[ego.cc.length - 1]!;
    const nc = next.central_curve;
    nextRev = dist2D(tip, nc[nc.length - 1]!) < dist2D(tip, nc[0]!);

    const nx = ori(next, nextRev);
    fCC = fCC.concat(nx.cc); fLB = fLB.concat(nx.lb); fRB = fRB.concat(nx.rb);
    used.add(next.id);
    curSeg = next; curRev = nextRev;
  }

  // 後方結合 (prepend)
  let bCC: Vec3[] = []; let bLB: Vec3[] = []; let bRB: Vec3[] = [];
  curSeg = egoSeg; curRev = egoRev;
  for (let n = 0; n < 30; n++) {
    // (1) predecessor_ids で探索
    const predIds = curRev ? curSeg.successor_ids : curSeg.predecessor_ids;
    let next = findByIds(predIds, used);
    let nextRev = false;

    if (!next) break; // ID がなければ結合停止
    const tip = bCC.length > 0 ? bCC[0]! : ego.cc[0]!;
    const nc = next.central_curve;
    nextRev = dist2D(tip, nc[0]!) < dist2D(tip, nc[nc.length - 1]!);

    const nx = ori(next, nextRev);
    bCC = nx.cc.concat(bCC); bLB = nx.lb.concat(bLB); bRB = nx.rb.concat(bRB);
    used.add(next.id);
    curSeg = next; curRev = nextRev;
  }

  // 結合 + 近接点除去 (ほぼ完全一致の重複のみ除去, S 精度を維持)
  function dedup(arr: Vec3[]): Vec3[] {
    if (arr.length === 0) return [];
    const out = [arr[0]!];
    for (let i = 1; i < arr.length; i++) {
      if (dist2D(arr[i]!, out[out.length - 1]!) > 0.01) out.push(arr[i]!);
    }
    return out;
  }
  return {
    central: dedup(bCC.concat(ego.cc).concat(fCC)),
    left: dedup(bLB.concat(ego.lb).concat(fLB)),
    right: dedup(bRB.concat(ego.rb).concat(fRB)),
    count: used.size,
  };
}

// 点 p を central_curve に投影し、SL 座標を計算する。
// L は符号付き (2D 外積, 進行方向から見て左=正)。
function pointToSL(p: Vec3, curve: Vec3[], cumS: number[]): {
  valid: boolean;
  s: number;
  l: number;
  projection: Vec3;
  segmentIndex: number;
} {
  if (curve.length === 0) {
    return { valid: false, s: 0, l: 0, projection: zeroVec3(), segmentIndex: -1 };
  }
  if (curve.length === 1) {
    const dx = p.x - curve[0]!.x; const dy = p.y - curve[0]!.y;
    const d = Math.sqrt(dx * dx + dy * dy);
    return { valid: true, s: 0, l: d, projection: copyVec3(curve[0]!), segmentIndex: 0 };
  }

  let bestD = Number.POSITIVE_INFINITY;
  let bestIdx = 0;
  let bestT = 0;
  let bestPoint = zeroVec3();

  for (let i = 0; i < curve.length - 1; i++) {
    const r = ptSegXYWithT(p, curve[i]!, curve[i + 1]!);
    if (r.dist2D < bestD) {
      bestD = r.dist2D; bestIdx = i; bestT = r.t; bestPoint = r.point;
    }
  }

  // S 値: 累積弧長 + 当該セグメント内の部分長
  const segDx = curve[bestIdx + 1]!.x - curve[bestIdx]!.x;
  const segDy = curve[bestIdx + 1]!.y - curve[bestIdx]!.y;
  const segLen = Math.sqrt(segDx * segDx + segDy * segDy);
  const s = cumS[bestIdx]! + bestT * segLen;

  // L 値: 2D 外積で符号判定 (左=正)
  const offX = p.x - bestPoint.x;
  const offY = p.y - bestPoint.y;
  const cross = segDx * offY - segDy * offX; // segDir × off の z 成分
  const signedL = cross >= 0 ? bestD : -bestD;

  return { valid: true, s, l: signedL, projection: bestPoint, segmentIndex: bestIdx };
}

// s 値配列と l 値配列から、指定 s 位置での l を線形補間で求める。
// sArr は単調増加と仮定 (境界線が central_curve と同方向に敷かれていれば成立)。
function interpolateLAtS(targetS: number, sArr: number[], lArr: number[]): {
  valid: boolean; l: number; bracketMode: string;
} {
  if (sArr.length === 0 || lArr.length === 0) return { valid: false, l: 0, bracketMode: "empty" };
  if (sArr.length === 1) return { valid: true, l: lArr[0]!, bracketMode: "single" };

  if (targetS <= sArr[0]!) return { valid: true, l: lArr[0]!, bracketMode: "before_start" };
  const last = sArr.length - 1;
  if (targetS >= sArr[last]!) return { valid: true, l: lArr[last]!, bracketMode: "after_end" };

  for (let i = 0; i < last; i++) {
    const s0 = sArr[i]!; const s1 = sArr[i + 1]!;
    if (s0 <= targetS && targetS <= s1) {
      if (s1 === s0) return { valid: true, l: lArr[i]!, bracketMode: "zero_span" };
      const frac = (targetS - s0) / (s1 - s0);
      return { valid: true, l: lArr[i]! + frac * (lArr[i + 1]! - lArr[i]!), bracketMode: "interpolated" };
    }
  }
  return { valid: true, l: lArr[last]!, bracketMode: "fallback" };
}

// --- 移動平均バッファ (前後 N 点の centered average) -------------------------
type MaSample = {
  s: number; l: number;
  edgeLeft: number; edgeRight: number; nearestEdge: number;
};
const maBuffer: MaSample[] = [];

function clearMa(): void { maBuffer.length = 0; }

function pushMa(sample: MaSample): void {
  maBuffer.push(sample);
}

// centered average: 中心は half 個前のサンプル。前後 half 個ずつ = 計 window 個。
// バッファに window 個未満の場合は 0 を返す。
function centeredAvg(field: keyof MaSample, window: number): number {
  if (maBuffer.length < window) return 0;
  let sum = 0;
  for (let i = maBuffer.length - window; i < maBuffer.length; i++) {
    sum += maBuffer[i]![field];
  }
  return sum / window;
}

// --- モジュール変数 -----------------------------------------------------------
let storedRefLocalPos: Vec3 = { x: 0, y: 0, z: 0 };
let storedRefFound = false;
let storedBevLocalPos: Vec3 = { x: 0, y: 0, z: 0 };
let storedBevWidth = 0;
let storedBevMatchDist = -1;
let storedBevFound = false;
// augmented_scene のデータを保存 (bev_detection 到着時にも出力するため)
let storedSceneHeader: Header = { seq: 0, stamp: { sec: 0, nsec: 0 }, frame_id: "" };
let storedSceneSegments: InLaneSeg[] = [];
let storedSceneAugObjects: InAugObj[] = [];
let storedSceneReceived = false;
let storedTrackId = -1;

// ===========================================================================
export const inputs = [
  "/t2/object_augmentor/augmented_scene",
  "/t2/bev_detection/objects",
];
export const output = "/studio_script/lane_boundary_tracker";

type InputEvent =
  | Input<"/t2/object_augmentor/augmented_scene">
  | Input<"/t2/bev_detection/objects">;

export default function script(
  event: InputEvent,
  globalVars: GlobalVariables,
): Output | undefined {

  // =========================================================================
  // bev_detection 受信: ターゲットのマッチングと local_position / width の保存
  // =========================================================================
  if (event.topic === "/t2/bev_detection/objects") {
    const threshM = globalVars.threshold_m != null ? Number(globalVars.threshold_m) : 5.0;
    const wantTid = globalVars.track_id != null ? Number(globalVars.track_id) : -1;
    const hasWorldTarget =
      typeof globalVars.target_x === "number" &&
      typeof globalVars.target_y === "number" &&
      typeof globalVars.target_z === "number";

    const bmsg = event.message as unknown as {
      detected_objects: {
        id?: number; position?: Vec3; local_position?: Vec3; width?: number;
      }[];
    };

    if (wantTid >= 0 && storedRefFound) {
      // (A) track_id 方式: augmented_scene の参照座標に最も近い local_position でマッチ
      let bestD = Number.POSITIVE_INFINITY;
      for (const o of bmsg.detected_objects ?? []) {
        if (!isValidVec3(o.local_position)) continue;
        const d = dist2D(o.local_position as Vec3, storedRefLocalPos);
        if (d < bestD) {
          bestD = d;
          storedBevLocalPos = copyVec3(o.local_position as Vec3);
          storedBevWidth = typeof o.width === "number" ? o.width : 0;
        }
      }
      storedBevMatchDist = bestD < Number.POSITIVE_INFINITY ? bestD : -1;
      storedBevFound = bestD <= threshM;
    } else if (hasWorldTarget) {
      // (B) world 座標方式: target_x/y/z に最も近い position (世界座標) でマッチ
      const tw: Vec3 = {
        x: globalVars.target_x as number,
        y: globalVars.target_y as number,
        z: globalVars.target_z as number,
      };
      let bestD = Number.POSITIVE_INFINITY;
      for (const o of bmsg.detected_objects ?? []) {
        if (!isValidVec3(o.position)) continue;
        const d = dist3(o.position as Vec3, tw);
        if (d < bestD && isValidVec3(o.local_position)) {
          bestD = d;
          storedBevLocalPos = copyVec3(o.local_position as Vec3);
          storedBevWidth = typeof o.width === "number" ? o.width : 0;
          storedRefFound = true;
        }
      }
      storedBevMatchDist = bestD < Number.POSITIVE_INFINITY ? bestD : -1;
      storedBevFound = bestD <= threshM;
      storedTrackId = -1;
    }

    if (!storedSceneReceived) return undefined;
  }

  // =========================================================================
  // augmented_scene 受信: シーンデータを保存
  // =========================================================================
  if (event.topic === "/t2/object_augmentor/augmented_scene") {
    const amsg = event.message as unknown as {
      header: Header;
      augmented_objects: InAugObj[];
      local_map_info: { local_lane_segments: InLaneSeg[] };
    };
    storedSceneHeader = amsg.header;
    storedSceneSegments = amsg.local_map_info?.local_lane_segments ?? [];
    storedSceneAugObjects = amsg.augmented_objects ?? [];
    storedSceneReceived = true;

    // track_id で参照座標を更新
    const wantTid = globalVars.track_id != null ? Number(globalVars.track_id) : -1;
    if (wantTid >= 0) {
      for (const ao of storedSceneAugObjects) {
        const bi = ao.bbox_info;
        const tid = typeof ao.track_id === "number" ? ao.track_id
          : (ao.tracking_info && typeof ao.tracking_info.track_id === "number") ? ao.tracking_info.track_id
          : typeof bi.id === "number" ? bi.id : -1;
        if (tid !== wantTid) continue;
        if (!isValidVec3(bi.local_position)) continue;
        storedRefLocalPos = copyVec3(bi.local_position as Vec3);
        storedRefFound = true;
        storedTrackId = tid;
        break;
      }
    }
  }

  // =========================================================================
  // 出力計算: どちらのトピック到着でも実行
  // =========================================================================
  const segments = storedSceneSegments;

  // ターゲット検索: augmented_scene で常に計算 (コマ送り対応)
  // width のみ bev_detection が利用可能ならそちらを優先
  const wantTrackId = globalVars.track_id != null ? Number(globalVars.track_id) : -1;
  const hasWorldTarget =
    typeof globalVars.target_x === "number" &&
    typeof globalVars.target_y === "number" &&
    typeof globalVars.target_z === "number";

  let targetFound = false;
  let effectiveLocalPos = zeroVec3();
  let effectiveWidth = 0;
  let widthSource = "none";
  const targetTrackId = storedTrackId;

  if (wantTrackId >= 0) {
    // (A) track_id 方式: augmented_scene から local_position を取得
    for (const ao of storedSceneAugObjects) {
      const bi = ao.bbox_info;
      const tid = typeof ao.track_id === "number" ? ao.track_id
        : (ao.tracking_info && typeof ao.tracking_info.track_id === "number") ? ao.tracking_info.track_id
        : typeof bi.id === "number" ? bi.id : -1;
      if (tid !== wantTrackId) continue;
      if (!isValidVec3(bi.local_position)) continue;
      effectiveLocalPos = storedBevFound ? copyVec3(storedBevLocalPos) : copyVec3(bi.local_position as Vec3);
      effectiveWidth = storedBevFound ? storedBevWidth : 0;
      widthSource = storedBevFound ? "bev" : "none";
      targetFound = true;
      break;
    }
  } else if (hasWorldTarget && storedBevFound) {
    // (B) world 座標方式: bev_detection データが必要
    effectiveLocalPos = copyVec3(storedBevLocalPos);
    effectiveWidth = storedBevWidth;
    widthSource = "bev";
    targetFound = true;
  }

  // available_segments
  const availableSegments: SegmentSummary[] = [];
  for (const s of segments) {
    const d = (targetFound && s.central_curve.length > 0)
      ? ptCurve2D(effectiveLocalPos, s.central_curve).dist : -1;
    availableSegments.push({ id: s.id, is_target_lane: s.is_target_lane, dist_to_target_m: d });
  }

  // セグメント選択: ターゲットに最も近い central_curve を持つセグメント (2D)
  let foundSeg: InLaneSeg | undefined;
  if (targetFound) {
    let bestDist = Number.POSITIVE_INFINITY;
    for (const s of segments) {
      if (s.central_curve.length === 0) continue;
      const d = ptCurve2D(effectiveLocalPos, s.central_curve).dist;
      if (d < bestDist) { bestDist = d; foundSeg = s; }
    }
  } else {
    foundSeg = segments.find((s) => s.is_target_lane);
  }

  const segmentFound = foundSeg != null;
  const leftCurve: Vec3[] = segmentFound ? foundSeg!.left_boundary.curve : [];
  const rightCurve: Vec3[] = segmentFound ? foundSeg!.right_boundary.curve : [];

  // セグメント結合 (SL central_curve 用: ego レーンを結合)
  const chained = targetFound
    ? chainLaneBidir(segments)
    : { central: [] as Vec3[], left: [] as Vec3[], right: [] as Vec3[], count: 0 };
  // 白線は foundSeg (ターゲット最近傍セグメント) の境界を使用
  // ターゲットが ego レーンと別レーンにいる場合でも正しい白線で距離計算される
  const slLeftCurve: Vec3[] = leftCurve;
  const slRightCurve: Vec3[] = rightCurve;

  // 距離計算 (結合済み boundary + 2D 距離)
  const canCompute = segmentFound && targetFound;
  let distLeft = 0; let nearestLeft = zeroVec3(); let nearestLeftIdx = -1;
  let distRight = 0; let nearestRight = zeroVec3();

  if (canCompute) {
    if (slLeftCurve.length > 0) {
      const rL = ptCurve2D(effectiveLocalPos, slLeftCurve);
      distLeft = rL.dist; nearestLeft = rL.point; nearestLeftIdx = rL.index;
    }
    if (slRightCurve.length > 0) {
      const rR = ptCurve2D(effectiveLocalPos, slRightCurve);
      distRight = rR.dist; nearestRight = rR.point;
    }
  }
  const distEdgeLeft = canCompute ? distLeft - effectiveWidth * 0.5 : 0;
  const distEdgeRight = canCompute ? distRight - effectiveWidth * 0.5 : 0;

  // =========================================================================
  // SL 座標系 (Frenet) への変換 — S=0 は自車最近傍点
  // =========================================================================
  const centralCurve: Vec3[] = chained.central.length >= 2
    ? chained.central
    : (segmentFound ? foundSeg!.central_curve : []);
  const curveSource = chained.central.length >= 2 ? "chained_id" : "single_segment";
  const { cumS, egoClosestIdx } = cumulativeSFromEgo(centralCurve);
  // total length は先頭→末尾の絶対弧長 (S原点とは独立)
  const centralLen = cumS.length >= 2
    ? cumS[cumS.length - 1]! - cumS[0]!
    : 0;

  let slValid = false;
  let targetS = 0; let targetChord = 0; let targetL = 0;
  let targetCentralProj = zeroVec3();
  let targetCentralIdx = -1;
  let leftLAtS = 0; let leftBracket = "none";
  let rightLAtS = 0; let rightBracket = "none";
  let distToLeftSL = 0; let distEdgeToLeftSL = 0;
  let distToRightSL = 0; let distEdgeToRightSL = 0;

  if (canCompute && centralCurve.length >= 2) {
    const tSL = pointToSL(effectiveLocalPos, centralCurve, cumS);
    if (tSL.valid) {
      slValid = true;
      targetS = tSL.s; targetL = tSL.l;
      targetCentralProj = tSL.projection;
      targetCentralIdx = tSL.segmentIndex;

      // chord distance: ego 投影点 〜 target 投影点の直線距離 (検証用)
      const egoProj = pointToSL(zeroVec3(), centralCurve, cumS);
      const chordDist = egoProj.valid ? dist2D(egoProj.projection, tSL.projection) : 0;
      targetChord = targetS >= 0 ? chordDist : -chordDist;

      // 左境界線の各点を central_curve に投影して (s, l) 配列を作る (結合済み curve 使用)
      if (slLeftCurve.length > 0) {
        const sArr: number[] = []; const lArr: number[] = [];
        for (const pt of slLeftCurve) {
          const r = pointToSL(pt, centralCurve, cumS);
          if (r.valid) { sArr.push(r.s); lArr.push(r.l); }
        }
        const li = interpolateLAtS(targetS, sArr, lArr);
        if (li.valid) {
          leftLAtS = li.l;
          leftBracket = li.bracketMode;
          distToLeftSL = targetL - leftLAtS;
          distEdgeToLeftSL = distToLeftSL - effectiveWidth * 0.5;
        }
      }

      // 右境界線も同様
      if (slRightCurve.length > 0) {
        const sArr: number[] = []; const lArr: number[] = [];
        for (const pt of slRightCurve) {
          const r = pointToSL(pt, centralCurve, cumS);
          if (r.valid) { sArr.push(r.s); lArr.push(r.l); }
        }
        const ri = interpolateLAtS(targetS, sArr, lArr);
        if (ri.valid) {
          rightLAtS = ri.l;
          rightBracket = ri.bracketMode;
          // 右白線は通常 L < targetL なので (targetL - rightL) が正
          distToRightSL = targetL - rightLAtS;
          distEdgeToRightSL = distToRightSL - effectiveWidth * 0.5;
        }
      }
    }
  }

  // 全セグメントの全白線からターゲットに最も近い白線を探索 (2D)
  let nearestBdDist = Number.POSITIVE_INFINITY;
  let nearestBdSegId = "";
  let nearestBdSide = "";
  if (targetFound) {
    for (const s of segments) {
      if (s.left_boundary.curve.length > 0) {
        const r = ptCurve2D(effectiveLocalPos, s.left_boundary.curve);
        if (r.dist < nearestBdDist) { nearestBdDist = r.dist; nearestBdSegId = s.id; nearestBdSide = "left"; }
      }
      if (s.right_boundary.curve.length > 0) {
        const r = ptCurve2D(effectiveLocalPos, s.right_boundary.curve);
        if (r.dist < nearestBdDist) { nearestBdDist = r.dist; nearestBdSegId = s.id; nearestBdSide = "right"; }
      }
    }
  }
  const nearestBdEdgeDist = nearestBdDist < Number.POSITIVE_INFINITY
    ? nearestBdDist - effectiveWidth * 0.5 : 0;

  // 移動平均 (centered: 前後 half 点ずつ = 計 maWindow 点)
  const maWindow = globalVars.ma_window != null ? Number(globalVars.ma_window) : 11;
  if (targetFound) {
    pushMa({
      s: targetS, l: targetL,
      edgeLeft: distEdgeToLeftSL, edgeRight: distEdgeToRightSL,
      nearestEdge: nearestBdEdgeDist,
    });
    // バッファ上限: window の 2 倍程度で十分
    while (maBuffer.length > maWindow * 2) maBuffer.shift();
  } else {
    clearMa();
  }

  return {
    header: storedSceneHeader,
    track_id_input: wantTrackId,
    segment_found: segmentFound,
    segment_id: foundSeg ? foundSeg.id : "",
    segment_is_target_lane: segmentFound ? foundSeg!.is_target_lane : false,
    left_boundary_curve: leftCurve,
    left_boundary_point_count: leftCurve.length,
    right_boundary_curve: rightCurve,
    right_boundary_point_count: rightCurve.length,
    target_found: targetFound,
    target_track_id: targetTrackId,
    bev_match_distance_m: storedBevMatchDist,
    target_local_position: copyVec3(effectiveLocalPos),
    target_width: effectiveWidth,
    target_width_source: widthSource,
    distance_computed: canCompute,
    distance_to_left_boundary_m: distLeft,
    distance_target_edge_to_left_boundary_m: distEdgeLeft,
    nearest_point_on_left_curve: nearestLeft,
    nearest_segment_index: nearestLeftIdx,
    distance_to_right_boundary_m: distRight,
    distance_target_edge_to_right_boundary_m: distEdgeRight,
    nearest_point_on_right_curve: nearestRight,
    available_segments: availableSegments,
    available_segment_count: segments.length,
    sl_valid: slValid,
    sl_central_curve_source: curveSource,
    chained_segment_count: chained.count,
    central_curve_total_length_m: centralLen,
    central_curve_point_count: centralCurve.length,
    ego_closest_curve_index: egoClosestIdx,
    target_s_m: targetS,
    target_s_chord_m: targetChord,
    target_l_m: targetL,
    target_central_projection: targetCentralProj,
    target_central_segment_index: targetCentralIdx,
    left_boundary_l_at_target_s_m: leftLAtS,
    left_boundary_l_bracket_mode: leftBracket,
    distance_target_to_left_boundary_sl_m: distToLeftSL,
    distance_target_edge_to_left_boundary_sl_m: distEdgeToLeftSL,
    right_boundary_l_at_target_s_m: rightLAtS,
    right_boundary_l_bracket_mode: rightBracket,
    distance_target_to_right_boundary_sl_m: distToRightSL,
    distance_target_edge_to_right_boundary_sl_m: distEdgeToRightSL,
    nearest_boundary_distance_m: nearestBdDist < Number.POSITIVE_INFINITY ? nearestBdDist : 0,
    nearest_boundary_edge_distance_m: nearestBdEdgeDist,
    nearest_boundary_segment_id: nearestBdSegId,
    nearest_boundary_side: nearestBdSide,
    ma_window: maWindow,
    target_s_ma: centeredAvg("s", maWindow),
    target_l_ma: centeredAvg("l", maWindow),
    distance_target_edge_to_left_boundary_sl_ma: centeredAvg("edgeLeft", maWindow),
    distance_target_edge_to_right_boundary_sl_ma: centeredAvg("edgeRight", maWindow),
    nearest_boundary_edge_distance_ma: centeredAvg("nearestEdge", maWindow),
  };
}
