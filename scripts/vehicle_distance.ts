// ============================================================================
// Vehicle distance tracker for /t2/object_augmentor/augmented_scene  v1
//
// track_id で指定した物体との「車間距離」を毎フレーム出力する。
// ALC 区間に依存せず、augmented_scene 単体で動作する。
//
// 入力:   /t2/object_augmentor/augmented_scene
// 出力:   /studio_script/vehicle_distance
//
// 座標系の前提:
//   bbox_info.local_position は車両座標系における「物体中心」の位置。
//   車両座標系の原点は自車のバンパー位置ではなく、車体は
//   x ∈ [-back_edge_to_center, +front_edge_to_center]、
//   y ∈ [-half_width, +half_width] の矩形を占める
//   (いすゞギガ: front 4.726 m / back 7.259 m / 半幅 1.295 m)。
//   そのため local_position の単純なノルムは接触までの距離ではない。
//   本スクリプトは自車矩形と物体矩形の最短距離 (clearance_m) を主指標とし、
//   後方の物体は自動的に「自車最後部からの距離」になる。
//
// Variables パネルで設定する変数:
//   track_id         : number -- 追跡する物体の track_id (bbox_info.id) (必須)
//   ego_front_edge_m : number -- 原点→自車最前部 [m] (省略時 4.726)
//   ego_back_edge_m  : number -- 原点→自車最後部 [m] (省略時 7.259)
//   ego_half_width_m : number -- 自車半幅 [m] (省略時 1.295)
//
// 主な出力フィールド:
//   clearance_m             : 自車矩形と物体矩形の最短 2D 距離 (接触時 0)
//   longitudinal_gap_m      : 前後方向のバンパー間ギャップ (x 方向のみ)
//   lateral_gap_m           : 横方向の車体間ギャップ (y 方向のみ)
//   distance_center_norm_m  : 原点→物体中心のノルム (参考値)
//   relative_position       : "front" / "rear" / "side" (物体中心の位置関係)
//   ttc_s                   : longitudinal_gap_m / 接近速度 (接近中のみ、非接近時 -1)
// ============================================================================

import { Input } from "./types";

type GlobalVariables = {
  track_id?: number | string;
  ego_front_edge_m?: number | string;
  ego_back_edge_m?: number | string;
  ego_half_width_m?: number | string;
};

type Vec3   = { x: number; y: number; z: number };
type Vec2   = { x: number; y: number };
type Stamp  = { sec: number; nsec: number };
type Header = { seq: number; stamp: Stamp; frame_id: string };

type InBBoxInfo = {
  id?: number;
  local_position?: Vec3;
  local_theta?: number;
  length?: number;
  width?: number;
  height?: number;
  type?: number;
  sub_type?: number;
};

type InAugObj = {
  bbox_info: InBBoxInfo;
  tracking_info?: {
    local_relative_velocity?: Vec3;
    track_id?: number;
  };
  track_id?: number;
};

type Output = {
  header: Header;
  track_id_input: number;
  match_found: boolean;
  track_id: number;
  local_position: Vec3;
  local_theta: number;
  target_length: number;
  target_width: number;
  relative_position: string;      // 物体中心が自車矩形の前 / 後 / 横のどこにあるか
  clearance_m: number;            // 自車矩形 ↔ 物体矩形の最短 2D 距離 (接触時 0)
  longitudinal_gap_m: number;     // x 方向のバンパー間ギャップ (x 範囲が重なると 0)
  lateral_gap_m: number;          // y 方向の車体間ギャップ (y 範囲が重なると 0)
  distance_center_norm_m: number; // 原点→物体中心の 2D ノルム (参考値)
  closing_speed_mps: number;      // 前後方向の接近速度 (正: 接近中)
  ttc_s: number;                  // longitudinal_gap_m / closing_speed (非接近時 -1)
};

function zeroVec3(): Vec3 { return { x: 0, y: 0, z: 0 }; }

function isValidVec3(v: unknown): v is Vec3 {
  if (v == null || typeof v !== "object") return false;
  const o = v as { x?: unknown; y?: unknown; z?: unknown };
  return typeof o.x === "number" && typeof o.y === "number" && typeof o.z === "number";
}

function getTrackId(ao: InAugObj): number {
  if (typeof ao.track_id === "number") return ao.track_id;
  if (ao.tracking_info && typeof ao.tracking_info.track_id === "number") return ao.tracking_info.track_id;
  if (typeof ao.bbox_info.id === "number") return ao.bbox_info.id;
  return -1;
}

// 中心 (cx, cy)・向き theta・寸法 length x width の矩形の 4 頂点
function rectCorners(cx: number, cy: number, theta: number, length: number, width: number): Vec2[] {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  const hl = length / 2;
  const hw = width / 2;
  const local: Vec2[] = [
    { x: +hl, y: +hw },
    { x: +hl, y: -hw },
    { x: -hl, y: -hw },
    { x: -hl, y: +hw },
  ];
  return local.map((p) => ({ x: cx + c * p.x - s * p.y, y: cy + s * p.x + c * p.y }));
}

// 点 p と線分 (a, b) の最短距離
function pointSegDist(p: Vec2, a: Vec2, b: Vec2): number {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const len2 = abx * abx + aby * aby;
  let t = len2 > 0 ? ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  const qx = a.x + t * abx;
  const qy = a.y + t * aby;
  return Math.hypot(p.x - qx, p.y - qy);
}

function segmentsIntersect(a: Vec2, b: Vec2, c: Vec2, d: Vec2): boolean {
  const cross = (o: Vec2, p: Vec2, q: Vec2) =>
    (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x);
  const d1 = cross(c, d, a);
  const d2 = cross(c, d, b);
  const d3 = cross(a, b, c);
  const d4 = cross(a, b, d);
  return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
         ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

// 凸多角形の内部判定 (境界含む)
function pointInConvexPolygon(p: Vec2, poly: Vec2[]): boolean {
  let sign = 0;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i]!;
    const b = poly[(i + 1) % poly.length]!;
    const cr = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    if (cr === 0) continue;
    const s = cr > 0 ? 1 : -1;
    if (sign === 0) { sign = s; } else if (s !== sign) { return false; }
  }
  return true;
}

// 2 つの凸多角形間の最短距離 (交差・包含時は 0)
function convexPolygonDist(polyA: Vec2[], polyB: Vec2[]): number {
  if (pointInConvexPolygon(polyA[0]!, polyB) || pointInConvexPolygon(polyB[0]!, polyA)) {
    return 0;
  }
  let best = Number.POSITIVE_INFINITY;
  for (let i = 0; i < polyA.length; i++) {
    const a1 = polyA[i]!;
    const a2 = polyA[(i + 1) % polyA.length]!;
    for (let j = 0; j < polyB.length; j++) {
      const b1 = polyB[j]!;
      const b2 = polyB[(j + 1) % polyB.length]!;
      if (segmentsIntersect(a1, a2, b1, b2)) { return 0; }
      best = Math.min(
        best,
        pointSegDist(a1, b1, b2),
        pointSegDist(a2, b1, b2),
        pointSegDist(b1, a1, a2),
        pointSegDist(b2, a1, a2),
      );
    }
  }
  return best;
}

// 向き theta の矩形が軸 (cos=c, sin=s は theta 由来) 方向に張る半幅
function halfExtentAlongAxis(theta: number, length: number, width: number, axisIsX: boolean): number {
  const c = Math.abs(Math.cos(theta));
  const s = Math.abs(Math.sin(theta));
  if (axisIsX) { return (c * length + s * width) / 2; }
  return (s * length + c * width) / 2;
}

export const inputs = ["/t2/object_augmentor/augmented_scene"];
export const output = "/studio_script/vehicle_distance";

type InputEvent = Input<"/t2/object_augmentor/augmented_scene">;

export default function script(
  event: InputEvent,
  globalVars: GlobalVariables,
): Output | undefined {
  const msg = event.message as unknown as {
    header: Header;
    augmented_objects: InAugObj[];
  };
  const augObjects: InAugObj[] = msg.augmented_objects ?? [];

  const wantTrackId = globalVars.track_id != null ? Number(globalVars.track_id) : -1;
  const egoFront = globalVars.ego_front_edge_m != null ? Number(globalVars.ego_front_edge_m) : 4.726;
  const egoBack  = globalVars.ego_back_edge_m  != null ? Number(globalVars.ego_back_edge_m)  : 7.259;
  const egoHalfW = globalVars.ego_half_width_m != null ? Number(globalVars.ego_half_width_m) : 1.295;

  const out: Output = {
    header: msg.header,
    track_id_input: wantTrackId,
    match_found: false,
    track_id: -1,
    local_position: zeroVec3(),
    local_theta: 0,
    target_length: 0,
    target_width: 0,
    relative_position: "none",
    clearance_m: -1,
    longitudinal_gap_m: -1,
    lateral_gap_m: -1,
    distance_center_norm_m: -1,
    closing_speed_mps: 0,
    ttc_s: -1,
  };

  for (const ao of augObjects) {
    const tid = getTrackId(ao);
    if (wantTrackId >= 0 && tid !== wantTrackId) { continue; }
    const bi = ao.bbox_info;
    if (!isValidVec3(bi.local_position)) { continue; }

    const pos = bi.local_position as Vec3;
    const theta = typeof bi.local_theta === "number" ? bi.local_theta : 0;
    const len = typeof bi.length === "number" ? bi.length : 0;
    const wid = typeof bi.width  === "number" ? bi.width  : 0;

    out.match_found = true;
    out.track_id = tid;
    out.local_position = { x: pos.x, y: pos.y, z: pos.z };
    out.local_theta = theta;
    out.target_length = len;
    out.target_width = wid;
    out.distance_center_norm_m = Math.hypot(pos.x, pos.y);

    // 自車矩形: x ∈ [-egoBack, +egoFront], y ∈ [-egoHalfW, +egoHalfW]
    const egoPoly: Vec2[] = [
      { x: egoFront, y: +egoHalfW },
      { x: egoFront, y: -egoHalfW },
      { x: -egoBack, y: -egoHalfW },
      { x: -egoBack, y: +egoHalfW },
    ];
    const targetPoly = rectCorners(pos.x, pos.y, theta, len, wid);
    out.clearance_m = convexPolygonDist(egoPoly, targetPoly);

    // 軸方向ギャップ: 物体矩形の x / y 方向の張り出しを考慮
    const halfX = halfExtentAlongAxis(theta, len, wid, true);
    const halfY = halfExtentAlongAxis(theta, len, wid, false);
    const gapFront = (pos.x - halfX) - egoFront;   // 物体最後端 − 自車最前部
    const gapRear  = (-egoBack) - (pos.x + halfX); // 自車最後部 − 物体最前端
    out.longitudinal_gap_m = Math.max(gapFront, gapRear, 0);
    out.lateral_gap_m = Math.max(Math.abs(pos.y) - halfY - egoHalfW, 0);

    if (pos.x > egoFront) {
      out.relative_position = "front";
    } else if (pos.x < -egoBack) {
      out.relative_position = "rear";
    } else {
      out.relative_position = "side";
    }

    // TTC: 前後方向の相対速度から算出 (正の closing_speed = 接近中)
    const ti = ao.tracking_info;
    if (ti != null && isValidVec3(ti.local_relative_velocity)) {
      const vxRel = (ti.local_relative_velocity as Vec3).x;
      const closing = out.relative_position === "rear" ? vxRel : -vxRel;
      out.closing_speed_mps = closing;
      if (closing > 0.1 && out.longitudinal_gap_m > 0) {
        out.ttc_s = out.longitudinal_gap_m / closing;
      }
    }
    break;
  }

  return out;
}
