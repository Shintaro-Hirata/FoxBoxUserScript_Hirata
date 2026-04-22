// ============================================================================
// Left boundary tracker + distance calculator  v5
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
};

// --- ユーティリティ ---------------------------------------------------------
function zeroVec3(): Vec3 { return { x: 0, y: 0, z: 0 }; }
function copyVec3(v: Vec3): Vec3 { return { x: v.x, y: v.y, z: v.z }; }

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

  // 距離計算
  const canCompute = segmentFound && storedTargetFound;
  let distLeft = -1; let nearestLeft = zeroVec3(); let nearestLeftIdx = -1;
  let distRight = -1; let nearestRight = zeroVec3();

  if (canCompute) {
    if (leftCurve.length > 0) {
      const rL = ptCurve(storedTargetLocalPos, leftCurve);
      distLeft = rL.dist; nearestLeft = rL.point; nearestLeftIdx = rL.index;
    }
    if (rightCurve.length > 0) {
      const rR = ptCurve(storedTargetLocalPos, rightCurve);
      distRight = rR.dist; nearestRight = rR.point;
    }
  }

  // 指定 curve point
  const idxValid = wantIdx >= 0 && wantIdx < leftCurve.length;
  const selPoint = idxValid ? copyVec3(leftCurve[wantIdx]!) : zeroVec3();
  const distToSel = (idxValid && storedTargetFound) ? dist3(storedTargetLocalPos, selPoint) : -1;
  const distEdgeToSelY = (idxValid && storedTargetFound)
    ? storedTargetLocalPos.y - selPoint.y - (storedTargetWidth * 0.5)
    : -9999;

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
  };
}
