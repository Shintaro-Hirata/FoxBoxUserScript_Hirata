// ============================================================================
// Target object finder for /t2/bev_detection/objects
//
// 指定した座標 (target_x, target_y, target_z) に最も近い物体を毎フレーム探し、
// x/y/z すべてが閾値(threshold_m)以内なら「マッチ」として情報を出力する。
// トラッカー状態を持たないので Seek/Scrub しても即時に結果が出る。
//
// 入力:   /t2/bev_detection/objects
// 出力:   /studio_script/target_object
//
// ターゲット座標の指定方法 (どちらか / 両方。GlobalVariables が優先):
//   (A) Variables パネルに下記変数を作る
//         target_x    : number  -- 世界座標系での x
//         target_y    : number  -- 世界座標系での y
//         target_z    : number  -- 世界座標系での z
//         threshold_m : number  -- 各軸の一致判定に使う閾値 (省略時 1.0m)
//   (B) 下の DEFAULT_* 定数を書き換えて保存
//
// マッチ条件 (per-axis):
//   |obj.position.x - target_x| < threshold_m
//   AND |obj.position.y - target_y| < threshold_m
//   AND |obj.position.z - target_z| < threshold_m
//
// ターゲット座標の求め方:
//   Raw Messages で /t2/bev_detection/objects を開き、追跡したい物体の
//   position.x / y / z の値を一度コピー。Variables に貼り付ける。
// ============================================================================

import { Input } from "./types";

type GlobalVariables = {
  target_x?: number;
  target_y?: number;
  target_z?: number;
  threshold_m?: number;
};

// Variables が未設定のときに使うデフォルト (ここを編集して使ってもOK)
const DEFAULT_TARGET_X    = 0;
const DEFAULT_TARGET_Y    = 0;
const DEFAULT_TARGET_Z    = 0;
const DEFAULT_THRESHOLD_M = 1.0;

type Vec3   = { x: number; y: number; z: number };
type Stamp  = { sec: number; nsec: number };
type Header = { seq: number; stamp: Stamp; frame_id: string };

type InDetectedObject = {
  id?: number;
  position?: Vec3;
  local_position?: Vec3;
  theta?: number;
  local_theta?: number;
  length?: number;
  width?: number;
  height?: number;
  type?: number;
  sub_type?: number;
  confidence?: number;
  timestamp?: number;
};

type CandidateObject = {
  raw_id: number;
  position: Vec3;
  dx: number;
  dy: number;
  dz: number;
  distance_m: number;
  max_axis_diff_m: number;
  local_position: Vec3;
  local_position_valid: boolean;
  theta: number;
  length: number;
  width: number;
  height: number;
  type: number;
  sub_type: number;
  confidence: number;
  within_threshold: boolean;
};

type Output = {
  header: Header;
  target: Vec3;
  threshold_m: number;
  match_found: boolean;
  match_count: number;
  detected_count: number;
  detected_with_position_count: number;
  matched: CandidateObject;
  nearest: CandidateObject;
  candidates_within_threshold: CandidateObject[];
};

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

function emptyCandidate(): CandidateObject {
  return {
    raw_id: -1, position: zeroVec3(), dx: 0, dy: 0, dz: 0,
    distance_m: -1, max_axis_diff_m: -1, local_position: zeroVec3(),
    local_position_valid: false, theta: 0, length: 0, width: 0,
    height: 0, type: 0, sub_type: 0, confidence: 0, within_threshold: false,
  };
}

function buildCandidate(
  o: InDetectedObject & { position: Vec3 },
  target: Vec3,
  thresholdM: number,
): CandidateObject {
  const dx = o.position.x - target.x;
  const dy = o.position.y - target.y;
  const dz = o.position.z - target.z;
  const maxAxis = Math.max(Math.abs(dx), Math.abs(dy), Math.abs(dz));
  const withinThreshold =
    Math.abs(dx) < thresholdM && Math.abs(dy) < thresholdM && Math.abs(dz) < thresholdM;
  const hasLocal = isValidVec3(o.local_position);
  return {
    raw_id: typeof o.id === "number" ? o.id : -1,
    position: copyVec3(o.position), dx, dy, dz,
    distance_m: Math.sqrt(dx * dx + dy * dy + dz * dz),
    max_axis_diff_m: maxAxis,
    local_position: hasLocal ? copyVec3(o.local_position as Vec3) : zeroVec3(),
    local_position_valid: hasLocal,
    theta:      typeof o.theta      === "number" ? o.theta      : 0,
    length:     typeof o.length     === "number" ? o.length     : 0,
    width:      typeof o.width      === "number" ? o.width      : 0,
    height:     typeof o.height     === "number" ? o.height     : 0,
    type:       typeof o.type       === "number" ? o.type       : 0,
    sub_type:   typeof o.sub_type   === "number" ? o.sub_type   : 0,
    confidence: typeof o.confidence === "number" ? o.confidence : 0,
    within_threshold: withinThreshold,
  };
}

export const inputs = ["/t2/bev_detection/objects"];
export const output = "/studio_script/target_object";

export default function script(
  event: Input<"/t2/bev_detection/objects">,
  globalVars: GlobalVariables,
): Output {
  const target: Vec3 = {
    x: typeof globalVars.target_x === "number" ? globalVars.target_x : DEFAULT_TARGET_X,
    y: typeof globalVars.target_y === "number" ? globalVars.target_y : DEFAULT_TARGET_Y,
    z: typeof globalVars.target_z === "number" ? globalVars.target_z : DEFAULT_TARGET_Z,
  };
  const thresholdM =
    typeof globalVars.threshold_m === "number" ? globalVars.threshold_m : DEFAULT_THRESHOLD_M;

  const msg = event.message as unknown as {
    header: Header;
    detected_objects: InDetectedObject[];
  };
  const detected = msg.detected_objects ?? [];

  let detectedWithPosition = 0;
  let nearest: CandidateObject = emptyCandidate();
  let nearestDist = Number.POSITIVE_INFINITY;
  const withinThreshold: CandidateObject[] = [];

  for (const o of detected) {
    if (!isValidVec3(o.position)) continue;
    detectedWithPosition++;
    const c = buildCandidate(o as InDetectedObject & { position: Vec3 }, target, thresholdM);
    if (c.distance_m < nearestDist) { nearestDist = c.distance_m; nearest = c; }
    if (c.within_threshold) withinThreshold.push(c);
  }

  withinThreshold.sort((a, b) => a.distance_m - b.distance_m);
  const matched = withinThreshold.length > 0 ? withinThreshold[0]! : emptyCandidate();

  return {
    header: msg.header, target, threshold_m: thresholdM,
    match_found: withinThreshold.length > 0,
    match_count: withinThreshold.length,
    detected_count: detected.length,
    detected_with_position_count: detectedWithPosition,
    matched, nearest,
    candidates_within_threshold: withinThreshold,
  };
}
