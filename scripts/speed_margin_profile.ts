// ============================================================================
// Speed margin profile  v1
//
// 自車の現在レーンと先読み (successor 連鎖) の各区間について、
// 現在速度 / 制限速度 / 横G上限速度 を並べ、引き上げ余地を可視化する。
// 「ETC 前後で制限速度を上げて良いか」を区間単位で判断する材料。
//
// 入力:   /t2/object_augmentor/augmented_scene  (必須)
//         /t2/odometry/ego                       (任意, 現在速度)
// 出力:   /studio_script/speed_margin_profile
//
// Variables パネルで設定する変数:
//   lateral_accel_limit_mps2 : number -- 横加速度上限 [m/s^2] (既定 2.5)
//   lookahead_count           : number -- 先読みするセグメント数 (既定 5)
//
// 各区間の横G上限速度 = sqrt(a_lat * R_min)。R_min は central_curve の最小曲率半径。
// raise_ok = (横G上限速度 >= 制限速度) のとき、制限速度まで横G制約内で到達可能。
// ============================================================================

import { Input } from "./types";

type Vec3  = { x: number; y: number; z: number };
type Stamp = { sec: number; nsec: number };

type GlobalVariables = {
  lateral_accel_limit_mps2?: unknown;
  lookahead_count?: unknown;
};

type InLocationContext = { context?: number | string };
type InLaneSeg = {
  id?: string;
  successor_ids?: string[];
  central_curve?: Vec3[];
  length?: number;
  speed_limit_max?: number;
  target_speed_max?: number;
  is_tollgate?: boolean;
  is_tunnel?: boolean;
  is_target_lane?: boolean;
  is_route?: boolean;
  location_context?: InLocationContext;
};
type InScene = {
  local_map_info?: {
    local_lane_segments?: InLaneSeg[];
    ego_lane_segment_indices?: number[];
  };
};
type InEgoPose = {
  local_linear_velocity?: Vec3;
  linear_velocity?: Vec3;
};

type SegProfile = {
  id: string;
  relation: string;                  // "ego" | "lookahead"
  order: number;                     // 先読み順 (ego=0)
  sl_m: number;                      // SL 距離 (弧長)
  context_name: string;              // 位置コンテキスト (ETC 等)
  is_tollgate: boolean;
  is_tunnel: boolean;
  speed_limit_kph: number;           // 制限速度上限
  target_speed_max_kph: number;      // 目標速度上限
  min_curve_radius_m: number;        // 最小曲率半径
  lateral_g_max_speed_kph: number;   // 横G上限を満たす最大速度
  current_speed_kph: number;         // 自車現在速度
  margin_to_limit_kph: number;       // 制限速度 - 現在速度
  margin_to_latg_max_kph: number;    // 横G上限速度 - 現在速度
  raise_ok: boolean;                 // 横G上限速度 >= 制限速度
  est_time_at_current_s: number;     // SL / 現在速度
  est_time_at_limit_s: number;       // SL / 制限速度
  est_time_at_latg_max_s: number;    // SL / 横G上限速度
};

type Output = {
  stamp: Stamp;
  ego_speed_mps: number;
  ego_speed_kph: number;
  lateral_accel_limit_mps2: number;
  has_ego_segment: boolean;
  primary_segment_id: string;
  primary_context_name: string;
  segment_count: number;
  segments: SegProfile[];            // 自車セグメント (relation="ego")
  lookahead_count: number;
  lookahead: SegProfile[];           // successor 連鎖の先読み
};

const MPS_TO_KPH = 3.6;
const RADIUS_SENTINEL_M = 99999;
const VMAX_CAP_KPH = 200;
const CONTEXT_NAMES = [
  "UNKNOWN", "HIGHWAY", "ENTERING_ETC", "PASSING_ETC", "EXITING_ETC",
  "URBAN_ROADS", "SPECIAL1", "BASE", "DEPARTURE_SPOT", "ARRIVAL_SPOT",
];

function isNum(v: unknown): v is number {
  return typeof v === "number" && isFinite(v);
}
function num(v: unknown, dflt: number): number {
  if (isNum(v)) {
    return v;
  }
  // ROS2 の uint64/int64 (例: ego_lane_segment_indices) は Foxglove では bigint で届く。
  if (typeof v === "bigint") {
    return Number(v);
  }
  return dflt;
}
function isVec3(v: unknown): v is Vec3 {
  if (v == null || typeof v !== "object") {
    return false;
  }
  const o = v as { x?: unknown; y?: unknown; z?: unknown };
  return isNum(o.x) && isNum(o.y) && isNum(o.z);
}
function speedMag(v: Vec3 | undefined): number {
  if (!isVec3(v)) {
    return 0;
  }
  return Math.sqrt(v.x * v.x + v.y * v.y);
}
function timeToSec(t: { sec?: number; nsec?: number } | undefined): number {
  if (t == null) {
    return 0;
  }
  return num(t.sec, 0) + num(t.nsec, 0) * 1e-9;
}
function enumName(v: unknown, names: string[]): string {
  if (typeof v === "string" && !isFinite(Number(v))) {
    return v;
  }
  const n = isNum(v) ? v : Number(v);
  return isFinite(n) && n >= 0 && n < names.length ? names[n]! : String(v ?? "");
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
function minCurveRadius(curve: Vec3[]): number {
  let maxCurv = 0;
  for (let i = 1; i < curve.length - 1; i++) {
    const a = curve[i - 1]!;
    const b = curve[i]!;
    const c = curve[i + 1]!;
    const lab = dist2D(a, b);
    const lbc = dist2D(b, c);
    const lac = dist2D(a, c);
    if (lab < 1e-4 || lbc < 1e-4 || lac < 1e-4) {
      continue;
    }
    const area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    const curv = (2 * area2) / (lab * lbc * lac);
    if (curv > maxCurv) {
      maxCurv = curv;
    }
  }
  if (maxCurv < 1e-6) {
    return RADIUS_SENTINEL_M;
  }
  return Math.min(1 / maxCurv, RADIUS_SENTINEL_M);
}

// --- モジュール状態 (現在速度) ----------------------------------------------
let egoSpeedMps = 0;
let lastProcTime = -1;

export const inputs = [
  "/t2/object_augmentor/augmented_scene",
  "/t2/odometry/ego",
];
export const output = "/studio_script/speed_margin_profile";

type InputEvent =
  | Input<"/t2/object_augmentor/augmented_scene">
  | Input<"/t2/odometry/ego">;

function buildProfile(
  seg: InLaneSeg, relation: string, order: number, latAccel: number, egoKph: number,
): SegProfile {
  const curve: Vec3[] = Array.isArray(seg.central_curve) ? seg.central_curve : [];
  const slM = arcLength2D(curve);
  const limitKph = num(seg.speed_limit_max, 0) * MPS_TO_KPH;
  const rMin = minCurveRadius(curve);
  const vMaxKph = Math.min(Math.sqrt(latAccel * rMin) * MPS_TO_KPH, VMAX_CAP_KPH);
  const limitMps = limitKph / MPS_TO_KPH;
  const vMaxMps = vMaxKph / MPS_TO_KPH;
  const egoMps = egoKph / MPS_TO_KPH;
  return {
    id: typeof seg.id === "string" ? seg.id : "",
    relation,
    order,
    sl_m: slM,
    context_name: enumName(seg.location_context?.context, CONTEXT_NAMES),
    is_tollgate: seg.is_tollgate === true,
    is_tunnel: seg.is_tunnel === true,
    speed_limit_kph: limitKph,
    target_speed_max_kph: num(seg.target_speed_max, 0) * MPS_TO_KPH,
    min_curve_radius_m: rMin,
    lateral_g_max_speed_kph: vMaxKph,
    current_speed_kph: egoKph,
    margin_to_limit_kph: limitKph - egoKph,
    margin_to_latg_max_kph: vMaxKph - egoKph,
    raise_ok: vMaxKph >= limitKph && limitKph > 0,
    est_time_at_current_s: egoMps > 0.3 ? slM / egoMps : 0,
    est_time_at_limit_s: limitMps > 1e-6 ? slM / limitMps : 0,
    est_time_at_latg_max_s: vMaxMps > 1e-6 ? slM / vMaxMps : 0,
  };
}

export default function script(
  event: InputEvent,
  globalVars: GlobalVariables,
): Output | undefined {
  const nowSec = timeToSec(event.receiveTime);
  if (lastProcTime >= 0 && nowSec < lastProcTime) {
    egoSpeedMps = 0;
  }
  lastProcTime = nowSec;

  if (event.topic === "/t2/odometry/ego") {
    const ego = event.message as unknown as InEgoPose;
    egoSpeedMps = isVec3(ego.local_linear_velocity)
      ? speedMag(ego.local_linear_velocity)
      : speedMag(ego.linear_velocity);
    return undefined;
  }

  const msg = event.message as unknown as InScene;
  const info = msg.local_map_info ?? {};
  const segments: InLaneSeg[] = Array.isArray(info.local_lane_segments)
    ? info.local_lane_segments : [];
  const indices: number[] = Array.isArray(info.ego_lane_segment_indices)
    ? info.ego_lane_segment_indices : [];
  const stamp: Stamp = { sec: num(event.receiveTime?.sec, 0), nsec: num(event.receiveTime?.nsec, 0) };

  const latAccel = Math.max(0.1, num(globalVars.lateral_accel_limit_mps2, 2.5));
  const lookN = Math.max(0, Math.trunc(num(globalVars.lookahead_count, 5)));
  const egoKph = egoSpeedMps * MPS_TO_KPH;

  const segById = new Map<string, InLaneSeg>();
  for (const s of segments) {
    if (typeof s.id === "string" && s.id.length > 0) {
      segById.set(s.id, s);
    }
  }

  // 自車セグメント特定 (indices 優先, なければ原点最近傍)
  const egoSegs: InLaneSeg[] = [];
  for (const raw of indices) {
    const idx = Math.trunc(num(raw, -1));
    if (idx >= 0 && idx < segments.length) {
      egoSegs.push(segments[idx]!);
    }
  }
  if (egoSegs.length === 0 && segments.length > 0) {
    let bestIdx = -1;
    let bestD = Number.POSITIVE_INFINITY;
    for (let i = 0; i < segments.length; i++) {
      const cc = segments[i]!.central_curve;
      if (!Array.isArray(cc) || cc.length === 0) {
        continue;
      }
      let dMin = Number.POSITIVE_INFINITY;
      for (const p of cc) {
        const d = p.x * p.x + p.y * p.y;
        if (d < dMin) {
          dMin = d;
        }
      }
      if (dMin < bestD) {
        bestD = dMin;
        bestIdx = i;
      }
    }
    if (bestIdx >= 0) {
      egoSegs.push(segments[bestIdx]!);
    }
  }

  const segProfiles: SegProfile[] = [];
  for (const s of egoSegs) {
    segProfiles.push(buildProfile(s, "ego", 0, latAccel, egoKph));
  }

  // 先読み: primary ego セグメントから successor を辿る。
  // 分岐では is_target_lane / is_route の後継を優先し、無ければ最初に見つかった後継。
  const lookahead: SegProfile[] = [];
  const used = new Set<string>();
  for (const s of egoSegs) {
    if (typeof s.id === "string") {
      used.add(s.id);
    }
  }
  let cur: InLaneSeg | undefined = egoSegs.length > 0 ? egoSegs[0] : undefined;
  for (let n = 0; n < lookN && cur != null; n++) {
    const succ = Array.isArray(cur.successor_ids) ? cur.successor_ids : [];
    let next: InLaneSeg | undefined;
    let firstExisting: InLaneSeg | undefined;
    for (const sid of succ) {
      if (used.has(sid)) {
        continue;
      }
      const hit = segById.get(sid);
      if (hit == null) {
        continue;
      }
      if (firstExisting == null) {
        firstExisting = hit;
      }
      if (hit.is_target_lane === true || hit.is_route === true) {
        next = hit;
        break;
      }
    }
    if (next == null) {
      next = firstExisting;
    }
    if (next == null) {
      break;
    }
    used.add(typeof next.id === "string" ? next.id : "");
    lookahead.push(buildProfile(next, "lookahead", n + 1, latAccel, egoKph));
    cur = next;
  }

  return {
    stamp,
    ego_speed_mps: egoSpeedMps,
    ego_speed_kph: egoKph,
    lateral_accel_limit_mps2: latAccel,
    has_ego_segment: egoSegs.length > 0,
    primary_segment_id: segProfiles.length > 0 ? segProfiles[0]!.id : "",
    primary_context_name: segProfiles.length > 0 ? segProfiles[0]!.context_name : "",
    segment_count: segProfiles.length,
    segments: segProfiles,
    lookahead_count: lookahead.length,
    lookahead,
  };
}
