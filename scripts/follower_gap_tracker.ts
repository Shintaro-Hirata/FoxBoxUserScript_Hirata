// ============================================================================
// Follower gap / congestion tracker  v1
//
// 自車後方・同一レーンの追従車 (follower) を追跡し、車間 (SL距離)・相対速度
// (接近率)・追従車の絶対速度・車間時間 (time headway) を出力する。
// ETC 前後で後続車が「詰まる」様子を定量化するための指標。
//
// 入力:   /t2/object_augmentor/augmented_scene  (必須)
//         /t2/odometry/ego                       (任意, 自車速度 / 車間時間用)
// 出力:   /studio_script/follower_gap_tracker
//
// Variables パネルで設定する変数:
//   same_lane_l_threshold_m : number  -- 同一レーン判定の |L| 閾値 [m] (既定 1.75)
//
// 計算:
//   自車レーンの central_curve を successor_ids/predecessor_ids で前後に結合し、
//   S=0 を自車投影点とする SL 座標を構築 (lane_boundary_tracker と同方式)。
//   各 augmented_object を投影し、|L| <= 閾値 かつ S<0 を後方同一レーン車とみなす。
//   最近傍 (S が 0 に最も近い負値) を follower とする。
//
// 速度の定義 (ObjectTrackingInfo):
//   local_velocity          : 障害物の絶対速度 (車両座標系)        → 追従車の対地速度
//   local_relative_velocity : 自車に対する相対速度 (車両座標系)    → +x で接近 (車間縮小)
// ============================================================================

import { Input } from "./types";

type Vec3  = { x: number; y: number; z: number };
type Stamp = { sec: number; nsec: number };

type GlobalVariables = {
  same_lane_l_threshold_m?: unknown;
};

// --- 入力型 ------------------------------------------------------------------
type InBBoxInfo = {
  id?: number;
  local_position?: Vec3;
  length?: number;
  width?: number;
  type?: number;
};
type InTrackingInfo = {
  local_velocity?: Vec3;
  local_relative_velocity?: Vec3;
  is_stationary?: boolean;
};
type InAugObj = {
  bbox_info?: InBBoxInfo;
  tracking_info?: InTrackingInfo;
};
type InLaneSeg = {
  id?: string;
  successor_ids?: string[];
  predecessor_ids?: string[];
  central_curve?: Vec3[];
};
type InScene = {
  local_map_info?: { local_lane_segments?: InLaneSeg[]; ego_lane_segment_indices?: number[] };
  augmented_objects?: InAugObj[];
};
type InEgoPose = {
  linear_velocity?: Vec3;
  local_linear_velocity?: Vec3;
};

// --- 出力型 ------------------------------------------------------------------
type BehindObj = {
  track_id: number;
  gap_s_m: number;            // 後方車間 (SL, 正値)
  lateral_l_m: number;        // 横ずれ L
  abs_speed_mps: number;      // 対地速度
  abs_speed_kph: number;
  relative_long_mps: number;  // 自車相対の前後速度 (+で接近)
  is_stationary: boolean;
  type: number;
};

type Output = {
  stamp: Stamp;
  ego_speed_mps: number;
  ego_speed_kph: number;
  sl_valid: boolean;
  chained_segment_count: number;
  central_curve_length_m: number;
  central_point_count: number;
  same_lane_l_threshold_m: number;
  object_count: number;
  // follower (最近傍後方同一レーン車)
  follower_found: boolean;
  follower_track_id: number;
  follower_gap_s_m: number;          // 後方車間 (SL, 正値)
  follower_gap_chord_m: number;      // 自車原点→追従車の直線距離 (参考)
  follower_lateral_l_m: number;
  follower_abs_speed_mps: number;    // 対地速度 (local_velocity)
  follower_abs_speed_kph: number;
  follower_forward_speed_mps: number;// local_velocity.x
  follower_relative_long_mps: number;// local_relative_velocity.x (+で接近)
  follower_closing: boolean;         // 接近中か (relative_long > 0.1)
  follower_time_headway_s: number;   // gap_s / 追従車絶対速度
  follower_is_stationary: boolean;
  follower_type: number;
  follower_width: number;
  follower_length: number;
  follower_gap_ema_m: number;        // 車間の指数移動平均
  follower_relative_ema_mps: number; // 相対速度の指数移動平均
  // leader (最近傍前方同一レーン車) 参考
  leader_found: boolean;
  leader_track_id: number;
  leader_gap_s_m: number;
  leader_lateral_l_m: number;
  leader_abs_speed_mps: number;
  leader_relative_long_mps: number;
  // 後方キュー
  behind_count: number;
  ahead_count: number;
  behind_objects: BehindObj[];
};

const MPS_TO_KPH = 3.6;
const EMA_ALPHA = 0.3;
const BEHIND_CAP = 10;

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
function copyVec3(v: Vec3): Vec3 {
  return { x: v.x, y: v.y, z: v.z };
}
function zeroVec3(): Vec3 {
  return { x: 0, y: 0, z: 0 };
}
function dist2D(a: Vec3, b: Vec3): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return Math.sqrt(dx * dx + dy * dy);
}
function timeToSec(t: { sec?: number; nsec?: number } | undefined): number {
  if (t == null) {
    return 0;
  }
  return num(t.sec, 0) + num(t.nsec, 0) * 1e-9;
}
function speedMag(v: Vec3 | undefined): number {
  if (!isVec3(v)) {
    return 0;
  }
  return Math.sqrt(v.x * v.x + v.y * v.y);
}
// 物体 ID は TrackedBbox.id (int32)。IDL 上 AugmentedObject/ObjectTrackingInfo に
// track_id は存在しないため bbox_info.id を用いる。
function getTrackId(ao: InAugObj): number {
  return ao.bbox_info && isNum(ao.bbox_info.id) ? ao.bbox_info.id : -1;
}

// 点 p を線分 a-b に 2D 投影 (t と投影点, 距離)。
function ptSegXYWithT(p: Vec3, a: Vec3, b: Vec3): { t: number; point: Vec3; dist2D: number } {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const lenSq = abx * abx + aby * aby;
  if (lenSq === 0) {
    return { t: 0, point: copyVec3(a), dist2D: dist2D(p, a) };
  }
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * abx + (p.y - a.y) * aby) / lenSq));
  const proj: Vec3 = { x: a.x + t * abx, y: a.y + t * aby, z: a.z + t * (b.z - a.z) };
  const dx = p.x - proj.x;
  const dy = p.y - proj.y;
  return { t, point: proj, dist2D: Math.sqrt(dx * dx + dy * dy) };
}

// 自車レーンの central_curve を +x 前方に固定して前後に結合する。
// egoHint (ego_lane_segment_indices から解決したセグメント) があればそれを起点にし、
// 無ければ原点 (0,0) に最も近い central_curve 頂点を持つセグメントを起点にする。
function chainEgoCentral(
  segments: InLaneSeg[], egoHint: InLaneSeg | undefined,
): { central: Vec3[]; count: number } {
  const empty = { central: [] as Vec3[], count: 0 };
  if (segments.length === 0) {
    return empty;
  }
  const segById = new Map<string, InLaneSeg>();
  for (const s of segments) {
    if (typeof s.id === "string") {
      segById.set(s.id, s);
    }
  }
  let egoSeg: InLaneSeg | undefined =
    egoHint && Array.isArray(egoHint.central_curve) && egoHint.central_curve.length >= 2
      ? egoHint : undefined;
  if (egoSeg == null) {
    let egoMinD = Number.POSITIVE_INFINITY;
    for (const s of segments) {
      const cc = s.central_curve;
      if (!Array.isArray(cc)) {
        continue;
      }
      for (const p of cc) {
        const d = p.x * p.x + p.y * p.y;
        if (d < egoMinD) {
          egoMinD = d;
          egoSeg = s;
        }
      }
    }
  }
  if (!egoSeg || !Array.isArray(egoSeg.central_curve) || egoSeg.central_curve.length < 2) {
    return empty;
  }
  function ori(s: InLaneSeg, rev: boolean): Vec3[] {
    const cc = (s.central_curve ?? []).map(copyVec3);
    return rev ? cc.reverse() : cc;
  }
  function findByIds(ids: string[] | undefined, used: Set<string>): InLaneSeg | undefined {
    if (!Array.isArray(ids)) {
      return undefined;
    }
    for (const sid of ids) {
      if (used.has(sid)) {
        continue;
      }
      const f = segById.get(sid);
      if (f && Array.isArray(f.central_curve) && f.central_curve.length >= 2) {
        return f;
      }
    }
    return undefined;
  }
  const ec = egoSeg.central_curve;
  const egoRev = ec[ec.length - 1]!.x < ec[0]!.x;
  const ego = ori(egoSeg, egoRev);
  const used = new Set<string>([typeof egoSeg.id === "string" ? egoSeg.id : ""]);

  let fCC: Vec3[] = [];
  let curSeg = egoSeg;
  let curRev = egoRev;
  for (let n = 0; n < 30; n++) {
    const succIds = curRev ? curSeg.predecessor_ids : curSeg.successor_ids;
    const next = findByIds(succIds, used);
    if (!next) {
      break;
    }
    const tip = fCC.length > 0 ? fCC[fCC.length - 1]! : ego[ego.length - 1]!;
    const nc = next.central_curve!;
    const nextRev = dist2D(tip, nc[nc.length - 1]!) < dist2D(tip, nc[0]!);
    fCC = fCC.concat(ori(next, nextRev));
    used.add(typeof next.id === "string" ? next.id : "");
    curSeg = next;
    curRev = nextRev;
  }

  let bCC: Vec3[] = [];
  curSeg = egoSeg;
  curRev = egoRev;
  for (let n = 0; n < 30; n++) {
    const predIds = curRev ? curSeg.successor_ids : curSeg.predecessor_ids;
    const next = findByIds(predIds, used);
    if (!next) {
      break;
    }
    const tip = bCC.length > 0 ? bCC[0]! : ego[0]!;
    const nc = next.central_curve!;
    const nextRev = dist2D(tip, nc[0]!) < dist2D(tip, nc[nc.length - 1]!);
    bCC = ori(next, nextRev).concat(bCC);
    used.add(typeof next.id === "string" ? next.id : "");
    curSeg = next;
    curRev = nextRev;
  }

  const merged = bCC.concat(ego).concat(fCC);
  const dedup: Vec3[] = [];
  for (let i = 0; i < merged.length; i++) {
    if (i === 0 || dist2D(merged[i]!, dedup[dedup.length - 1]!) > 0.01) {
      dedup.push(merged[i]!);
    }
  }
  return { central: dedup, count: used.size };
}

// S=0 を自車原点投影点とする累積弧長。
function cumulativeSFromEgo(c: Vec3[]): number[] {
  if (c.length === 0) {
    return [];
  }
  const raw: number[] = [0];
  for (let i = 1; i < c.length; i++) {
    raw.push(raw[i - 1]! + dist2D(c[i]!, c[i - 1]!));
  }
  const origin = zeroVec3();
  let bestD = Number.POSITIVE_INFINITY;
  let bestS = 0;
  for (let i = 0; i < c.length - 1; i++) {
    const r = ptSegXYWithT(origin, c[i]!, c[i + 1]!);
    if (r.dist2D < bestD) {
      bestD = r.dist2D;
      bestS = raw[i]! + r.t * dist2D(c[i]!, c[i + 1]!);
    }
  }
  const cumS: number[] = [];
  for (let i = 0; i < raw.length; i++) {
    cumS.push(raw[i]! - bestS);
  }
  return cumS;
}

// 点 p の SL (S=符号付き弧長, L=符号付き横距離) を求める。投影点も返す。
function pointToSL(p: Vec3, curve: Vec3[], cumS: number[]): {
  valid: boolean; s: number; l: number; projection: Vec3;
} {
  if (curve.length < 2) {
    return { valid: false, s: 0, l: 0, projection: zeroVec3() };
  }
  let bestD = Number.POSITIVE_INFINITY;
  let bestIdx = 0;
  let bestT = 0;
  let bestPoint = zeroVec3();
  for (let i = 0; i < curve.length - 1; i++) {
    const r = ptSegXYWithT(p, curve[i]!, curve[i + 1]!);
    if (r.dist2D < bestD) {
      bestD = r.dist2D;
      bestIdx = i;
      bestT = r.t;
      bestPoint = r.point;
    }
  }
  const segDx = curve[bestIdx + 1]!.x - curve[bestIdx]!.x;
  const segDy = curve[bestIdx + 1]!.y - curve[bestIdx]!.y;
  const segLen = Math.sqrt(segDx * segDx + segDy * segDy);
  const s = cumS[bestIdx]! + bestT * segLen;
  const offX = p.x - bestPoint.x;
  const offY = p.y - bestPoint.y;
  const cross = segDx * offY - segDy * offX;
  const signedL = cross >= 0 ? bestD : -bestD;
  return { valid: true, s, l: signedL, projection: bestPoint };
}

// --- モジュール状態 ----------------------------------------------------------
let egoSpeedMps = 0;
let lastProcTime = -1;
let emaFollowerId = -1;
let emaGap = 0;
let emaRel = 0;

function resetState(): void {
  egoSpeedMps = 0;
  emaFollowerId = -1;
  emaGap = 0;
  emaRel = 0;
}

export const inputs = [
  "/t2/object_augmentor/augmented_scene",
  "/t2/odometry/ego",
];
export const output = "/studio_script/follower_gap_tracker";

type InputEvent =
  | Input<"/t2/object_augmentor/augmented_scene">
  | Input<"/t2/odometry/ego">;

export default function script(
  event: InputEvent,
  globalVars: GlobalVariables,
): Output | undefined {
  const nowSec = timeToSec(event.receiveTime);
  if (lastProcTime >= 0 && nowSec < lastProcTime) {
    resetState();
  }
  lastProcTime = nowSec;

  if (event.topic === "/t2/odometry/ego") {
    const ego = event.message as unknown as InEgoPose;
    const lv = ego.local_linear_velocity;
    const wv = ego.linear_velocity;
    egoSpeedMps = isVec3(lv) ? speedMag(lv) : (isVec3(wv) ? speedMag(wv) : 0);
    return undefined;
  }

  const msg = event.message as unknown as InScene;
  const segments: InLaneSeg[] = Array.isArray(msg.local_map_info?.local_lane_segments)
    ? msg.local_map_info!.local_lane_segments! : [];
  const indices: number[] = Array.isArray(msg.local_map_info?.ego_lane_segment_indices)
    ? msg.local_map_info!.ego_lane_segment_indices! : [];
  const objects: InAugObj[] = Array.isArray(msg.augmented_objects) ? msg.augmented_objects : [];
  const lThresh = Math.max(0.1, num(globalVars.same_lane_l_threshold_m, 1.75));
  const stamp: Stamp = { sec: num(event.receiveTime?.sec, 0), nsec: num(event.receiveTime?.nsec, 0) };

  // 自車セグメント: ego_lane_segment_indices を優先し SL フレームの起点にする
  // (最近傍頂点より確実。車線変更中に隣接レーンを掴むのを防ぐ)。
  let egoHint: InLaneSeg | undefined;
  for (const raw of indices) {
    const idx = Math.trunc(num(raw, -1));
    if (idx >= 0 && idx < segments.length) {
      egoHint = segments[idx]!;
      break;
    }
  }
  const chained = chainEgoCentral(segments, egoHint);
  const central = chained.central;
  const slValid = central.length >= 2;
  const cumS = slValid ? cumulativeSFromEgo(central) : [];
  const centralLen = cumS.length >= 2 ? cumS[cumS.length - 1]! - cumS[0]! : 0;

  // 同一レーンの後方/前方車を収集
  const behind: BehindObj[] = [];
  let aheadCount = 0;
  let follower: { obj: InAugObj; s: number; l: number; proj: Vec3 } | undefined;
  let leader: { obj: InAugObj; s: number; l: number } | undefined;

  if (slValid) {
    for (const ao of objects) {
      const bi = ao.bbox_info;
      if (bi == null || !isVec3(bi.local_position)) {
        continue;
      }
      const sl = pointToSL(bi.local_position as Vec3, central, cumS);
      if (!sl.valid || Math.abs(sl.l) > lThresh) {
        continue;
      }
      const ti = ao.tracking_info ?? {};
      if (sl.s < 0) {
        // 後方同一レーン車
        behind.push({
          track_id: getTrackId(ao),
          gap_s_m: -sl.s,
          lateral_l_m: sl.l,
          abs_speed_mps: speedMag(ti.local_velocity),
          abs_speed_kph: speedMag(ti.local_velocity) * MPS_TO_KPH,
          relative_long_mps: isVec3(ti.local_relative_velocity)
            ? (ti.local_relative_velocity as Vec3).x : 0,
          is_stationary: ti.is_stationary === true,
          type: num(bi.type, -1),
        });
        // follower = S が 0 に最も近い負値 (最近傍後方)
        if (follower == null || sl.s > follower.s) {
          follower = { obj: ao, s: sl.s, l: sl.l, proj: sl.projection };
        }
      } else if (sl.s > 0) {
        aheadCount++;
        if (leader == null || sl.s < leader.s) {
          leader = { obj: ao, s: sl.s, l: sl.l };
        }
      }
    }
  }

  // 後方キューを車間昇順 (近い順) に整列
  behind.sort((a, b) => a.gap_s_m - b.gap_s_m);
  const behindOut = behind.slice(0, BEHIND_CAP);

  // follower 詳細
  let fFound = false;
  let fTid = -1;
  let fGapS = 0;
  let fGapChord = 0;
  let fL = 0;
  let fAbs = 0;
  let fFwd = 0;
  let fRel = 0;
  let fHeadway = 0;
  let fStationary = false;
  let fType = -1;
  let fWidth = 0;
  let fLength = 0;
  if (follower != null) {
    fFound = true;
    fTid = getTrackId(follower.obj);
    fGapS = -follower.s;
    const fpos = follower.obj.bbox_info!.local_position as Vec3;
    fGapChord = Math.sqrt(fpos.x * fpos.x + fpos.y * fpos.y); // 自車原点→追従車の直線距離
    fL = follower.l;
    const ti = follower.obj.tracking_info ?? {};
    fAbs = speedMag(ti.local_velocity);
    fFwd = isVec3(ti.local_velocity) ? (ti.local_velocity as Vec3).x : 0;
    fRel = isVec3(ti.local_relative_velocity) ? (ti.local_relative_velocity as Vec3).x : 0;
    fStationary = ti.is_stationary === true;
    fHeadway = fAbs > 0.3 ? fGapS / fAbs : 0;
    const bi = follower.obj.bbox_info!;
    fType = num(bi.type, -1);
    fWidth = num(bi.width, 0);
    fLength = num(bi.length, 0);
  }

  // follower の EMA (id が変わったらリセット)
  if (fFound) {
    if (emaFollowerId !== fTid) {
      emaFollowerId = fTid;
      emaGap = fGapS;
      emaRel = fRel;
    } else {
      emaGap = EMA_ALPHA * fGapS + (1 - EMA_ALPHA) * emaGap;
      emaRel = EMA_ALPHA * fRel + (1 - EMA_ALPHA) * emaRel;
    }
  } else {
    emaFollowerId = -1;
    emaGap = 0;
    emaRel = 0;
  }

  // leader 詳細
  let lFound = false;
  let lTid = -1;
  let lGapS = 0;
  let lL = 0;
  let lAbs = 0;
  let lRel = 0;
  if (leader != null) {
    lFound = true;
    lTid = getTrackId(leader.obj);
    lGapS = leader.s;
    lL = leader.l;
    const ti = leader.obj.tracking_info ?? {};
    lAbs = speedMag(ti.local_velocity);
    lRel = isVec3(ti.local_relative_velocity) ? (ti.local_relative_velocity as Vec3).x : 0;
  }

  return {
    stamp,
    ego_speed_mps: egoSpeedMps,
    ego_speed_kph: egoSpeedMps * MPS_TO_KPH,
    sl_valid: slValid,
    chained_segment_count: chained.count,
    central_curve_length_m: centralLen,
    central_point_count: central.length,
    same_lane_l_threshold_m: lThresh,
    object_count: objects.length,
    follower_found: fFound,
    follower_track_id: fTid,
    follower_gap_s_m: fGapS,
    follower_gap_chord_m: fGapChord,
    follower_lateral_l_m: fL,
    follower_abs_speed_mps: fAbs,
    follower_abs_speed_kph: fAbs * MPS_TO_KPH,
    follower_forward_speed_mps: fFwd,
    follower_relative_long_mps: fRel,
    follower_closing: fRel > 0.1,
    follower_time_headway_s: fHeadway,
    follower_is_stationary: fStationary,
    follower_type: fType,
    follower_width: fWidth,
    follower_length: fLength,
    follower_gap_ema_m: emaGap,
    follower_relative_ema_mps: emaRel,
    leader_found: lFound,
    leader_track_id: lTid,
    leader_gap_s_m: lGapS,
    leader_lateral_l_m: lL,
    leader_abs_speed_mps: lAbs,
    leader_relative_long_mps: lRel,
    behind_count: behind.length,
    ahead_count: aheadCount,
    behind_objects: behindOut,
  };
}
