// ============================================================================
// Left boundary tracker + distance calculator  v9.1
//
// /t2/object_augmentor/augmented_scene と /t2/bev_detection/objects を購読し、
// 指定した世界座標に近い物体から、指定レーンセグメントの左境界線までの距離を算出する。
//
// 修正履歴:
//   v1-v2 : 基本実装
//   v3     : curve_point_index による curve 上の特定点の指定を追加
//   v4     : distance_target_edge_to_selected_y_m を追加
//            = target.local_position.y - selected_point.y - (target.width * 0.5)
//   v5     : ターゲット検索を /t2/bev_detection/objects に変更
//            augmented_objects の bbox_info は local_position のみ保持し world 座標を
//            持たないため、target_object スクリプトと同一ロジック (world 座標検索) で
//            bev_detection から local_position を取得してから距離計算する。
//   v6     : SL 座標系 (Frenet) への変換を追加
//            central_curve を基準に target/left/right boundary を s(弧長)/l(符号付き横距離)
//            に変換し、SL 空間での車両端↔白線距離を出力する。
//   v7     : 座標フレーム不一致の修正
//            bev_detection (T1) の local_position と augmented_scene (T2) の central_curve
//            は異なる時刻の車両フレームで、自車移動により SL 投影がずれる問題を修正。
//            augmented_objects から同一 raw_id の bbox_info.local_position を取得し、
//            augmented_scene と同一フレームで距離・SL 計算を行うようにした。
//   v7.1   : 左白線 SL 距離の符号修正 (leftL-targetL → targetL-leftL)
//   v8     : S 原点を自車最近傍点に変更 (社内 Python 版 vehicle_coord_to_sl と統一)
//            S=0 は自車原点 (0,0) に最も近い central_curve 頂点。
//            前方 (curve 方向) が正、後方が負。target_s_m≈50 → 目標物は約 50m 先。
//   v9     : 複数セグメント結合 (chainLaneForward)
//            ego → target 方向に endpoint 近接で lane segment を順次結合し、
//            1 本の長い central_curve / left / right boundary で SL 計算。
//            これにより central_curve 全長 86m の制限を超えて 100m+ の S に対応。
//   v9.1   : 双方向結合 (chainLaneBidir) に変更
//            ego segment を常に +x 方向に固定し、前方・後方の両方向へ結合。
//            ターゲットが後方にあるときも正しく S<0 を返す。
//
// 入力:   /t2/object_augmentor/augmented_scene
//         /t2/bev_detection/objects
// 出力:   /studio_script/lane_boundary_tracker
//
// Variables パネルで設定する変数:
//   lane_segment_id   : string  (省略可。省略時 → ターゲットに最近傍の segment を自動選択)
//   target_x          : number  (世界座標。target_object スクリプトと同じ値)
//   target_y          : number
//   target_z          : number
//   threshold_m        : number  (省略時 1.0m)
//   curve_point_index  : number  (left_boundary.curve の配列インデックス。省略可)
//
// セグメント選択ロジック:
//   (A) lane_segment_id 設定あり → id 完全一致で選択 (手動指定)
//   (B) 未設定 & ターゲット取得済み → central_curve への最短距離で自動選択
//   (C) 未設定 & ターゲットなし → is_target_lane=true を fallback
//
// 主な出力フィールド:
//   distance_to_left_boundary_m          : ターゲット ↔ 左境界 curve 全体の最短距離
//   distance_target_edge_to_selected_y_m : target.local_y - selected.y - width/2
//   selected_point                        : curve_point_index で指定した curve 上の点
//   available_segments                    : 全 segment の id と距離一覧 (id 確認用)
//   ---- SL 座標系 (v6) ----
//   target_s_m / target_l_m              : central_curve を基準にした target の SL
//   left_boundary_l_at_target_s_m        : target と同じ s 位置での左白線の L 値
//   distance_target_to_left_boundary_sl_m: targetL - leftL (Cartesian の target.y - boundary.y と同符号)
//   distance_target_edge_to_left_boundary_sl_m: 上記から target.width/2 を引いた端-白線距離
//   (right_boundary も同様)
// ----------------------------------------------------------------------------
// SL 座標系の定義:
//   S: central_curve の始点 (curve[0]) からポリライン沿いに測った累積弧長 [m]
//   L: central_curve に対する横方向距離 (2D x-y 平面, 右手系で左が正) [m]
//   L > 0 → central_curve 進行方向の左側 (左白線があるはずの側)
//   L < 0 → central_curve 進行方向の右側
// ============================================================================

import { Input } from "./types";

type Vec3   = { x: number; y: number; z: number };
type Stamp  = { sec: number; nsec: number };
type Header = { seq: number; stamp: Stamp; frame_id: string };

type GlobalVariables = {
  lane_segment_id?: string;
  target_x?: number;
  target_y?: number;
  target_z?: number;
  threshold_m?: number;
  curve_point_index?: number;
};

// --- 入力型 (union OK) ------------------------------------------------------
type InDetObj = {
  id?: number;
  position?: Vec3;
  local_position?: Vec3;
  width?: number;
};

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
  tracking_info?: { velocity?: Vec3 };
};

type InLaneSeg = {
  id: string;
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
  lane_segment_id_input: string;
  target_world: Vec3;
  threshold_m: number;
  select_mode: string;
  segment_found: boolean;
  segment_id: string;
  segment_is_target_lane: boolean;
  left_boundary_curve: Vec3[];
  left_boundary_point_count: number;
  right_boundary_curve: Vec3[];
  right_boundary_point_count: number;
  target_object_found: boolean;
  target_object_raw_id: number;
  target_object_local_position: Vec3;
  target_object_width: number;
  target_object_world_match_dist_m: number;
  target_same_frame_match: boolean;
  target_effective_local_position: Vec3;
  target_effective_width: number;
  distance_computed: boolean;
  distance_to_left_boundary_m: number;
  nearest_point_on_left_curve: Vec3;
  nearest_segment_index: number;
  distance_to_right_boundary_m: number;
  nearest_point_on_right_curve: Vec3;
  selected_point_index: number;
  selected_point_valid: boolean;
  selected_point: Vec3;
  distance_to_selected_point_m: number;
  distance_target_edge_to_selected_y_m: number;
  available_segments: SegmentSummary[];
  available_segment_count: number;
  // ---- SL 座標系 ----
  sl_valid: boolean;
  chained_segment_count: number;
  central_curve_total_length_m: number;
  ego_closest_curve_index: number;
  target_s_m: number;
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

function ptSeg(p: Vec3, a: Vec3, b: Vec3): { dist: number; point: Vec3 } {
  const abx = b.x - a.x; const aby = b.y - a.y; const abz = b.z - a.z;
  const lenSq = abx * abx + aby * aby + abz * abz;
  if (lenSq === 0) return { dist: dist3(p, a), point: copyVec3(a) };
  const t = Math.max(0, Math.min(1,
    ((p.x - a.x) * abx + (p.y - a.y) * aby + (p.z - a.z) * abz) / lenSq));
  const proj: Vec3 = { x: a.x + t * abx, y: a.y + t * aby, z: a.z + t * abz };
  return { dist: dist3(p, proj), point: proj };
}

function ptCurve(p: Vec3, c: Vec3[]): { dist: number; point: Vec3; index: number } {
  if (c.length === 0) return { dist: -1, point: zeroVec3(), index: -1 };
  if (c.length === 1) return { dist: dist3(p, c[0]!), point: copyVec3(c[0]!), index: 0 };
  let bd = Number.POSITIVE_INFINITY; let bp = zeroVec3(); let bi = 0;
  for (let i = 0; i < c.length - 1; i++) {
    const r = ptSeg(p, c[i]!, c[i + 1]!);
    if (r.dist < bd) { bd = r.dist; bp = r.point; bi = i; }
  }
  return { dist: bd, point: bp, index: bi };
}

function minDist(p: Vec3, c: Vec3[]): number {
  if (c.length === 0) return Number.POSITIVE_INFINITY;
  return ptCurve(p, c).dist;
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

// central_curve 上の各頂点までの累積弧長 (2D) を、自車原点 (0,0) に最も近い頂点を
// S=0 として返す。S=0 より前方 (curve 方向) は正、後方は負。
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

  // 自車原点 (0,0) に最も近い頂点を探索
  let egoIdx = 0;
  let egoMinD = Number.POSITIVE_INFINITY;
  for (let i = 0; i < c.length; i++) {
    const d = c[i]!.x * c[i]!.x + c[i]!.y * c[i]!.y;
    if (d < egoMinD) { egoMinD = d; egoIdx = i; }
  }

  // オフセットして S=0 を自車最近傍点に設定
  const offset = raw[egoIdx]!;
  const cumS: number[] = [];
  for (let i = 0; i < raw.length; i++) {
    cumS.push(raw[i]! - offset);
  }

  return { cumS, egoClosestIdx: egoIdx };
}

// --- セグメント結合 (双方向) ------------------------------------------------
// ego segment を +x 方向に固定し、前方・後方の両方向に endpoint 近接で lane segment
// を結合して 1 本の長い polyline を作る。S=0(自車) を中心に前方=正, 後方=負。
function chainLaneBidir(
  segments: InLaneSeg[]
): { central: Vec3[]; left: Vec3[]; right: Vec3[]; count: number } {
  const empty = { central: [] as Vec3[], left: [] as Vec3[], right: [] as Vec3[], count: 0 };
  if (segments.length === 0) return empty;
  const THRESH = 10.0;

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

  // ego segment を +x 方向に固定 (ターゲット方向ではなく道路前方)
  const ec = egoSeg.central_curve;
  const ego = ori(egoSeg, ec[ec.length - 1]!.x < ec[0]!.x);
  const used = new Set([egoSeg.id]);

  // 前方結合: ego.cc[last] から +x 方向へ
  let fCC: Vec3[] = []; let fLB: Vec3[] = []; let fRB: Vec3[] = [];
  for (let n = 0; n < 30; n++) {
    const tip = fCC.length > 0 ? fCC[fCC.length - 1]! : ego.cc[ego.cc.length - 1]!;
    let best: InLaneSeg | undefined; let bd = THRESH; let br = false;
    for (const s of segments) {
      if (used.has(s.id) || s.central_curve.length < 2) continue;
      const c = s.central_curve;
      const d0 = dist2D(tip, c[0]!); const dN = dist2D(tip, c[c.length - 1]!);
      if (d0 < bd) { bd = d0; best = s; br = false; }
      if (dN < bd) { bd = dN; best = s; br = true; }
    }
    if (!best) break;
    const nx = ori(best, br);
    fCC = fCC.concat(nx.cc); fLB = fLB.concat(nx.lb); fRB = fRB.concat(nx.rb);
    used.add(best.id);
  }

  // 後方結合: ego.cc[0] から -x 方向へ (prepend)
  let bCC: Vec3[] = []; let bLB: Vec3[] = []; let bRB: Vec3[] = [];
  for (let n = 0; n < 30; n++) {
    const tip = bCC.length > 0 ? bCC[0]! : ego.cc[0]!;
    let best: InLaneSeg | undefined; let bd = THRESH; let br = false;
    for (const s of segments) {
      if (used.has(s.id) || s.central_curve.length < 2) continue;
      const c = s.central_curve;
      const dLast = dist2D(tip, c[c.length - 1]!);
      const dFirst = dist2D(tip, c[0]!);
      if (dLast < bd) { bd = dLast; best = s; br = false; }
      if (dFirst < bd) { bd = dFirst; best = s; br = true; }
    }
    if (!best) break;
    const nx = ori(best, br);
    bCC = nx.cc.concat(bCC); bLB = nx.lb.concat(bLB); bRB = nx.rb.concat(bRB);
    used.add(best.id);
  }

  // 結合 + 近接点除去
  function dedup(arr: Vec3[]): Vec3[] {
    if (arr.length === 0) return [];
    const out = [arr[0]!];
    for (let i = 1; i < arr.length; i++) {
      if (dist2D(arr[i]!, out[out.length - 1]!) > 0.5) out.push(arr[i]!);
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

// --- モジュール変数 (bev_detection から保存) --------------------------------
let storedTargetFound    = false;
let storedTargetLocalPos: Vec3 = { x: 0, y: 0, z: 0 };
let storedTargetWidth    = 0;
let storedTargetRawId    = -1;
let storedTargetMatchDist = -1;

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
  // bev_detection 受信: ターゲットを世界座標で検索し local_position を保存
  // =========================================================================
  if (event.topic === "/t2/bev_detection/objects") {
    const msg = event.message as unknown as { detected_objects: InDetObj[] };
    const detected: InDetObj[] = msg.detected_objects ?? [];

    const hasTarget =
      typeof globalVars.target_x === "number" &&
      typeof globalVars.target_y === "number" &&
      typeof globalVars.target_z === "number";
    const thresholdM =
      typeof globalVars.threshold_m === "number" ? globalVars.threshold_m : 1.0;

    storedTargetFound = false;
    storedTargetLocalPos = zeroVec3();
    storedTargetWidth = 0;
    storedTargetRawId = -1;
    storedTargetMatchDist = -1;

    if (hasTarget) {
      const tw: Vec3 = {
        x: globalVars.target_x as number,
        y: globalVars.target_y as number,
        z: globalVars.target_z as number,
      };
      let bestD = Number.POSITIVE_INFINITY;
      for (const o of detected) {
        if (!isValidVec3(o.position)) continue;
        const d = dist3(o.position as Vec3, tw);
        if (d < bestD) {
          bestD = d;
          storedTargetRawId = typeof o.id === "number" ? o.id : -1;
          storedTargetLocalPos = isValidVec3(o.local_position)
            ? copyVec3(o.local_position as Vec3) : zeroVec3();
          storedTargetWidth = typeof o.width === "number" ? o.width : 0;
          storedTargetFound = true;
        }
      }
      storedTargetMatchDist = bestD === Number.POSITIVE_INFINITY ? -1 : bestD;
      if (bestD > thresholdM) storedTargetFound = false;
    }

    return undefined;
  }

  // =========================================================================
  // augmented_scene 受信: 保存した local_position で距離計算
  // =========================================================================
  const msg = event.message as unknown as {
    header: Header;
    augmented_objects: InAugObj[];
    local_map_info: { local_lane_segments: InLaneSeg[] };
  };

  const segments: InLaneSeg[] = msg.local_map_info?.local_lane_segments ?? [];

  const segIdInput =
    typeof globalVars.lane_segment_id === "string" ? globalVars.lane_segment_id : "";
  const hasTarget =
    typeof globalVars.target_x === "number" &&
    typeof globalVars.target_y === "number" &&
    typeof globalVars.target_z === "number";
  const targetWorld: Vec3 = hasTarget
    ? { x: globalVars.target_x as number, y: globalVars.target_y as number, z: globalVars.target_z as number }
    : zeroVec3();
  const thresholdM =
    typeof globalVars.threshold_m === "number" ? globalVars.threshold_m : 1.0;
  const wantIdx =
    typeof globalVars.curve_point_index === "number"
      ? Math.floor(globalVars.curve_point_index) : -1;

  // available_segments
  const availableSegments: SegmentSummary[] = [];
  for (const s of segments) {
    const d = (storedTargetFound && s.central_curve.length > 0)
      ? minDist(storedTargetLocalPos, s.central_curve) : -1;
    availableSegments.push({ id: s.id, is_target_lane: s.is_target_lane, dist_to_target_m: d });
  }

  // セグメント選択
  let foundSeg: InLaneSeg | undefined;
  let selectMode = "none";

  if (segIdInput.length > 0) {
    foundSeg = segments.find((s) => s.id === segIdInput);
    selectMode = "manual";
  } else if (storedTargetFound) {
    let bestDist = Number.POSITIVE_INFINITY;
    for (const s of segments) {
      if (s.central_curve.length === 0) continue;
      const d = minDist(storedTargetLocalPos, s.central_curve);
      if (d < bestDist) { bestDist = d; foundSeg = s; }
    }
    selectMode = "nearest_to_target";
  } else {
    foundSeg = segments.find((s) => s.is_target_lane);
    selectMode = "is_target_lane";
  }

  const segmentFound = foundSeg != null;
  const leftCurve: Vec3[] = segmentFound ? foundSeg!.left_boundary.curve : [];
  const rightCurve: Vec3[] = segmentFound ? foundSeg!.right_boundary.curve : [];

  // augmented_objects から同一フレームの local_position を取得 (SL 計算精度向上)
  // bev_detection の raw_id で照合し、augmented_scene と同じ座標系にする
  let effectiveLocalPos = copyVec3(storedTargetLocalPos);
  let effectiveWidth = storedTargetWidth;
  let sameFrameMatch = false;

  if (storedTargetFound && storedTargetRawId >= 0) {
    const augObjs: InAugObj[] = msg.augmented_objects ?? [];
    for (const ao of augObjs) {
      if (ao.bbox_info.id === storedTargetRawId && isValidVec3(ao.bbox_info.local_position)) {
        effectiveLocalPos = copyVec3(ao.bbox_info.local_position as Vec3);
        if (typeof ao.bbox_info.width === "number") effectiveWidth = ao.bbox_info.width;
        sameFrameMatch = true;
        break;
      }
    }
  }

  // 距離計算 (同一フレーム位置を使用)
  const canCompute = segmentFound && storedTargetFound;
  let distLeft = -1; let nearestLeft = zeroVec3(); let nearestLeftIdx = -1;
  let distRight = -1; let nearestRight = zeroVec3();

  if (canCompute) {
    if (leftCurve.length > 0) {
      const rL = ptCurve(effectiveLocalPos, leftCurve);
      distLeft = rL.dist; nearestLeft = rL.point; nearestLeftIdx = rL.index;
    }
    if (rightCurve.length > 0) {
      const rR = ptCurve(effectiveLocalPos, rightCurve);
      distRight = rR.dist; nearestRight = rR.point;
    }
  }

  // 指定 curve point
  const idxValid = wantIdx >= 0 && wantIdx < leftCurve.length;
  const selPoint = idxValid ? copyVec3(leftCurve[wantIdx]!) : zeroVec3();
  const distToSel = (idxValid && storedTargetFound) ? dist3(effectiveLocalPos, selPoint) : -1;
  const distEdgeToSelY = (idxValid && storedTargetFound)
    ? effectiveLocalPos.y - selPoint.y - (effectiveWidth * 0.5)
    : 0;

  // =========================================================================
  // SL 座標系 (Frenet) への変換 — 複数セグメント結合, S=0 は自車最近傍点
  // =========================================================================
  // ego → 前方/後方の両方向に lane segment を結合
  const chained = storedTargetFound
    ? chainLaneBidir(segments)
    : { central: [] as Vec3[], left: [] as Vec3[], right: [] as Vec3[], count: 0 };
  const centralCurve: Vec3[] = chained.central.length >= 2
    ? chained.central
    : (segmentFound ? foundSeg!.central_curve : []);
  const slLeftCurve: Vec3[] = chained.left.length > 0 ? chained.left : leftCurve;
  const slRightCurve: Vec3[] = chained.right.length > 0 ? chained.right : rightCurve;
  const { cumS, egoClosestIdx } = cumulativeSFromEgo(centralCurve);
  // total length は先頭→末尾の絶対弧長 (S原点とは独立)
  const centralLen = cumS.length >= 2
    ? cumS[cumS.length - 1]! - cumS[0]!
    : 0;

  let slValid = false;
  let targetS = 0; let targetL = 0;
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

  return {
    header: msg.header,
    lane_segment_id_input: segIdInput,
    target_world: targetWorld,
    threshold_m: thresholdM,
    select_mode: selectMode,
    segment_found: segmentFound,
    segment_id: foundSeg ? foundSeg.id : "",
    segment_is_target_lane: segmentFound ? foundSeg!.is_target_lane : false,
    left_boundary_curve: leftCurve,
    left_boundary_point_count: leftCurve.length,
    right_boundary_curve: rightCurve,
    right_boundary_point_count: rightCurve.length,
    target_object_found: storedTargetFound,
    target_object_raw_id: storedTargetRawId,
    target_object_local_position: copyVec3(storedTargetLocalPos),
    target_object_width: storedTargetWidth,
    target_object_world_match_dist_m: storedTargetMatchDist,
    target_same_frame_match: sameFrameMatch,
    target_effective_local_position: copyVec3(effectiveLocalPos),
    target_effective_width: effectiveWidth,
    distance_computed: canCompute,
    distance_to_left_boundary_m: distLeft,
    nearest_point_on_left_curve: nearestLeft,
    nearest_segment_index: nearestLeftIdx,
    distance_to_right_boundary_m: distRight,
    nearest_point_on_right_curve: nearestRight,
    selected_point_index: wantIdx,
    selected_point_valid: idxValid,
    selected_point: selPoint,
    distance_to_selected_point_m: distToSel,
    distance_target_edge_to_selected_y_m: distEdgeToSelY,
    available_segments: availableSegments,
    available_segment_count: segments.length,
    sl_valid: slValid,
    chained_segment_count: chained.count,
    central_curve_total_length_m: centralLen,
    ego_closest_curve_index: egoClosestIdx,
    target_s_m: targetS,
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
  };
}
