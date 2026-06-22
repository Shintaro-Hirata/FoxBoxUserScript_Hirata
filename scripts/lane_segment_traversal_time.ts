// ============================================================================
// Lane segment traversal time  v1
//
// 指定した id の lane_segment を「通過するのにかかる時間」を出力する。
// 複数 id を同時に指定可能。
//
// 入力:   /t2/object_augmentor/augmented_scene  (必須)
//         /t2/odometry/ego                       (任意, 現在速度の表示用)
// 出力:   /studio_script/lane_segment_traversal_time
//
// Variables パネルで設定する変数:
//   target_segment_ids        : string  -- 対象セグメント id。カンマ/空白区切りで複数可
//                                          (例: "lane_001, lane_002 lane_003")
//   assumed_speeds_kph         : string  -- 推定に使う仮定速度 [km/h] (既定 "10,30,40")
//   lateral_accel_limit_mps2   : number  -- 横加速度上限 [m/s^2] (既定 2.5)
//
// 出力する「時間」は 2 種類:
//   (1) measured_time_s : ログ再生中に自車が実際に通過した時間 (実測)
//        ego_lane_segment_indices で自車がそのセグメント上にいた区間を
//        event.receiveTime で計測。完了 = 進入してから退出するまで。
//        ※ 自車が実際にそのレーンを走行した場合のみ得られる。
//   (2) est_*_s : 区間長 (SL/弧長) ÷ 速度 による推定時間。
//        制限速度 / 仮定速度 / 現在の自車速度 / 横G制限速度 で算出。
//        ※ 隣接レーンなど自車が走らない区間でも、地図に出現すれば算出可能。
//
// 背景 (ETC 前後の速度上限見直し検討) 向け補助指標:
//   - min_curve_radius_m       : central_curve の最小曲率半径
//   - lateral_g_max_speed_kph  : 横G上限を満たす最大速度 sqrt(a_lat * R)
//   これにより「この区間はカーブ的に何 km/h まで出せるか」を確認できる。
//
// 注意:
//   - 実測時間の分解能は augmented_scene の更新周期 (~0.1s) に依存。
//   - Seek で時刻が巻き戻ると計測状態をリセットする。
// ============================================================================

import { Input } from "./types";

type Vec3  = { x: number; y: number; z: number };
type Stamp = { sec: number; nsec: number };

type GlobalVariables = {
  target_segment_ids?: unknown;
  assumed_speeds_kph?: unknown;
  lateral_accel_limit_mps2?: unknown;
};

// --- 入力型 ------------------------------------------------------------------
type InLaneSeg = {
  id?: string;
  central_curve?: Vec3[];
  length?: number;
  speed_limit_max?: number;
  is_tollgate?: boolean;
};

type InLocalMapInfo = {
  local_lane_segments?: InLaneSeg[];
  ego_lane_segment_indices?: number[];
};

type InAugmentedScene = {
  local_map_info?: InLocalMapInfo;
};

type InEgoPose = {
  linear_velocity?: Vec3;
  local_linear_velocity?: Vec3;
};

// --- 出力型 (optional / union 禁止) -----------------------------------------
type AssumedTime = {
  speed_kph: number;   // 仮定速度 [km/h]
  time_s: number;      // 区間長 / 仮定速度 [s]
};

type TargetOut = {
  id: string;                       // 対象セグメント id
  latched: boolean;                 // 地図にこの id が出現し情報を取得済みか
  found_in_map_now: boolean;        // 現フレームの地図に存在するか
  length_sl_m: number;              // SL 距離 (弧長, central_curve 累積)
  length_straight_m: number;        // 直線距離 (弦長, 始点→終点)
  length_provided_m: number;        // LocalLaneSegment.length 提供値
  speed_limit_max_kph: number;      // 制限速度上限 [km/h]
  is_tollgate: boolean;             // 料金所 (ETC) 区間か
  min_curve_radius_m: number;       // 最小曲率半径 [m] (直線相当は大きな値)
  lateral_g_max_speed_kph: number;  // 横G上限を満たす最大速度 [km/h]
  // 実測通過時間
  measured_state: string;           // "not_seen" | "in_progress" | "completed"
  measured_time_s: number;          // 実測通過時間 [s] (進行中はそこまでの経過)
  measured_avg_speed_kph: number;   // 実測平均速度 [km/h] (完了時)
  // 推定通過時間 (区間長 SL ÷ 速度)
  est_time_at_speed_limit_s: number;   // 制限速度での推定 [s]
  est_time_at_lateral_g_max_s: number; // 横G上限速度での推定 [s]
  est_time_at_ego_speed_s: number;     // 現在の自車速度での推定 [s]
  est_times_at_assumed: AssumedTime[]; // 仮定速度ごとの推定 [s]
};

type Traversal = {
  id: string;                  // 通過したセグメント id
  time_s: number;              // 実測通過時間 [s]
  length_sl_m: number;         // SL 距離 [m]
  avg_speed_kph: number;       // 平均速度 [km/h]
  speed_limit_kph: number;     // 制限速度上限 [km/h]
};

type Output = {
  stamp: Stamp;
  ego_speed_mps: number;             // 現在の自車速度 [m/s]
  ego_speed_kph: number;             // 現在の自車速度 [km/h]
  lateral_accel_limit_mps2: number;  // 横加速度上限 [m/s^2]
  assumed_speeds_kph: number[];      // 推定に使った仮定速度 [km/h]
  // 指定 id の結果
  target_count: number;
  targets: TargetOut[];
  // 現在の自車セグメント (進行中)
  current_ego_segment_ids: string[];
  current_ego_elapsed_s: number;     // 現在セグメント上での経過時間 [s]
  // 自動蓄積: 自車が通過し終えたセグメントの履歴 (新しい順, 上限あり)
  recent_traversal_count: number;
  recent_traversals: Traversal[];
};

const MPS_TO_KPH = 3.6;
const RADIUS_SENTINEL_M = 99999;   // 直線相当 (曲率 ~ 0) の半径表示上限
const VMAX_CAP_KPH = 200;          // 横G上限速度の表示上限 (カーブが緩い区間用)
const RECENT_CAP = 40;             // recent_traversals の保持上限
const TRACK_CAP = 600;             // tracks Map の保持上限 (超過時に古い完了分を整理)

// --- ユーティリティ ---------------------------------------------------------
function isNum(v: unknown): v is number {
  return typeof v === "number" && isFinite(v);
}
function num(v: unknown, dflt: number): number {
  return isNum(v) ? v : dflt;
}
function timeToSec(t: { sec?: number; nsec?: number } | undefined): number {
  if (t == null) {
    return 0;
  }
  return num(t.sec, 0) + num(t.nsec, 0) * 1e-9;
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

// central_curve の最小曲率半径を Menger 曲率で求める。
// 直線に近い区間は曲率 ~ 0 となり、半径は RADIUS_SENTINEL_M を返す。
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
    // 三角形 ABC の面積の 2 倍 = |(B-A) x (C-A)|
    const area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    // Menger 曲率 kappa = 2 * area2 / (lab * lbc * lac) ... area2 は 2*Area なので
    // kappa = 4*Area/(lab*lbc*lac) = 2*area2/(lab*lbc*lac)
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

function parseIds(v: unknown): string[] {
  let raw: string[] = [];
  if (Array.isArray(v)) {
    raw = v.map((x) => String(x));
  } else if (typeof v === "string") {
    raw = v.split(/[\s,;]+/);
  } else if (typeof v === "number") {
    raw = [String(v)];
  }
  const out: string[] = [];
  for (const s of raw) {
    const t = s.trim();
    if (t.length > 0 && out.indexOf(t) < 0) {
      out.push(t);
    }
  }
  return out;
}

function parseSpeeds(v: unknown, dflt: number[]): number[] {
  let arr: number[] = [];
  if (Array.isArray(v)) {
    arr = v.map((x) => Number(x));
  } else if (typeof v === "string") {
    arr = v.split(/[\s,;]+/).map((x) => Number(x));
  } else if (typeof v === "number") {
    arr = [v];
  }
  arr = arr.filter((n) => isFinite(n) && n > 0);
  return arr.length > 0 ? arr : dflt;
}

// --- 計測状態 (モジュール変数) ----------------------------------------------
type Track = {
  id: string;
  // 直近に地図で観測したジオメトリ/属性 (latch)
  latched: boolean;
  lengthSl: number;
  lengthStraight: number;
  lengthProvided: number;
  speedLimitMps: number;
  isTollgate: boolean;
  minRadiusM: number;
  // 実測通過 (自車がこのセグメント上にいた区間)
  everEgo: boolean;
  enterTime: number;
  lastInTime: number;
  exitTime: number;
  presentNow: boolean;
  completed: boolean;
};

const tracks = new Map<string, Track>();
let recentTraversals: Traversal[] = [];
let egoSpeedMps = 0;
let lastProcTime = -1;

function newTrack(id: string): Track {
  return {
    id,
    latched: false,
    lengthSl: 0,
    lengthStraight: 0,
    lengthProvided: 0,
    speedLimitMps: 0,
    isTollgate: false,
    minRadiusM: RADIUS_SENTINEL_M,
    everEgo: false,
    enterTime: 0,
    lastInTime: 0,
    exitTime: 0,
    presentNow: false,
    completed: false,
  };
}

function getOrCreate(id: string): Track {
  let t = tracks.get(id);
  if (t == null) {
    t = newTrack(id);
    tracks.set(id, t);
  }
  return t;
}

function resetState(): void {
  tracks.clear();
  recentTraversals = [];
  egoSpeedMps = 0;
}

// tracks が増えすぎたら、現在の対象でない完了済みトラックから整理する。
function pruneTracks(targetSet: Set<string>): void {
  if (tracks.size <= TRACK_CAP) {
    return;
  }
  const deletable: string[] = [];
  tracks.forEach((t, id) => {
    if (t.completed && !targetSet.has(id)) {
      deletable.push(id);
    }
  });
  let toRemove = tracks.size - (TRACK_CAP - 100);
  for (let i = 0; i < deletable.length && toRemove > 0; i++) {
    tracks.delete(deletable[i]!);
    toRemove--;
  }
}

export const inputs = [
  "/t2/object_augmentor/augmented_scene",
  "/t2/odometry/ego",
];
export const output = "/studio_script/lane_segment_traversal_time";

type InputEvent =
  | Input<"/t2/object_augmentor/augmented_scene">
  | Input<"/t2/odometry/ego">;

export default function script(
  event: InputEvent,
  globalVars: GlobalVariables,
): Output | undefined {
  const nowSec = timeToSec(event.receiveTime);

  // Seek backward 検出: 時刻が巻き戻ったら計測状態をリセット
  if (lastProcTime >= 0 && nowSec < lastProcTime) {
    resetState();
  }
  lastProcTime = nowSec;

  // --- odometry: 現在速度を更新して終了 (出力は scene 受信時のみ) ----------
  if (event.topic === "/t2/odometry/ego") {
    const ego = event.message as unknown as InEgoPose;
    const lv = ego.local_linear_velocity;
    const wv = ego.linear_velocity;
    let v = 0;
    if (lv != null && (isNum(lv.x) || isNum(lv.y))) {
      v = Math.sqrt(num(lv.x, 0) ** 2 + num(lv.y, 0) ** 2);
    } else if (wv != null && (isNum(wv.x) || isNum(wv.y))) {
      v = Math.sqrt(num(wv.x, 0) ** 2 + num(wv.y, 0) ** 2);
    }
    egoSpeedMps = v;
    return undefined;
  }

  // --- augmented_scene 受信 ------------------------------------------------
  const msg = event.message as unknown as InAugmentedScene;
  const info: InLocalMapInfo = msg.local_map_info ?? {};
  const segments: InLaneSeg[] = Array.isArray(info.local_lane_segments)
    ? info.local_lane_segments : [];
  const indices: number[] = Array.isArray(info.ego_lane_segment_indices)
    ? info.ego_lane_segment_indices : [];

  const targetIds = parseIds(globalVars.target_segment_ids);
  const targetSet = new Set(targetIds);
  const assumedKph = parseSpeeds(globalVars.assumed_speeds_kph, [10, 30, 40]);
  const latAccel = Math.max(0.1, num(globalVars.lateral_accel_limit_mps2, 2.5));

  // 現フレームに存在する id の集合と id→セグメント参照
  const segById = new Map<string, InLaneSeg>();
  for (const s of segments) {
    if (typeof s.id === "string" && s.id.length > 0) {
      segById.set(s.id, s);
    }
  }

  // 自車セグメント id の集合 (ego_lane_segment_indices ベース)
  const egoIds: string[] = [];
  const egoIdSet = new Set<string>();
  for (const raw of indices) {
    const idx = Math.trunc(num(raw, -1));
    if (idx < 0 || idx >= segments.length) {
      continue;
    }
    const sid = segments[idx]!.id;
    if (typeof sid === "string" && sid.length > 0 && !egoIdSet.has(sid)) {
      egoIdSet.add(sid);
      egoIds.push(sid);
    }
  }

  // latch 対象: (a) 全ての自車セグメント, (b) 指定された対象 id
  const toLatch = new Set<string>(egoIds);
  for (const id of targetIds) {
    toLatch.add(id);
  }
  Array.from(toLatch).forEach((id) => {
    const seg = segById.get(id);
    if (seg == null) {
      return;
    }
    const curve: Vec3[] = Array.isArray(seg.central_curve) ? seg.central_curve : [];
    const t = getOrCreate(id);
    t.latched = true;
    t.lengthSl = arcLength2D(curve);
    t.lengthStraight = chord2D(curve);
    t.lengthProvided = num(seg.length, 0);
    t.speedLimitMps = num(seg.speed_limit_max, 0);
    t.isTollgate = seg.is_tollgate === true;
    t.minRadiusM = minCurveRadius(curve);
  });

  // 実測: 進入の検出 (現フレームで自車セグメントになっている id)
  for (const id of egoIds) {
    const t = getOrCreate(id);
    if (!t.everEgo) {
      t.everEgo = true;
      t.enterTime = nowSec;
      t.completed = false;
    }
    t.lastInTime = nowSec;
    t.presentNow = true;
  }

  // 実測: 退出の検出 (前フレームまで自車セグメントだったが今はいない id)
  tracks.forEach((t, id) => {
    if (t.presentNow && !egoIdSet.has(id)) {
      t.presentNow = false;
      if (!t.completed) {
        t.exitTime = nowSec;
        t.completed = true;
        const dur = t.exitTime - t.enterTime;
        const avg = dur > 1e-6 ? t.lengthSl / dur : 0;
        recentTraversals.unshift({
          id,
          time_s: dur,
          length_sl_m: t.lengthSl,
          avg_speed_kph: avg * MPS_TO_KPH,
          speed_limit_kph: t.speedLimitMps * MPS_TO_KPH,
        });
        if (recentTraversals.length > RECENT_CAP) {
          recentTraversals = recentTraversals.slice(0, RECENT_CAP);
        }
      }
    }
  });

  pruneTracks(targetSet);

  // --- 出力の組み立て ------------------------------------------------------
  const stamp: Stamp = { sec: num(event.receiveTime?.sec, 0), nsec: num(event.receiveTime?.nsec, 0) };

  const targets: TargetOut[] = [];
  for (const id of targetIds) {
    const t = tracks.get(id);
    const found = segById.has(id);
    if (t == null || !t.latched) {
      // まだ地図に出現していない id
      targets.push({
        id,
        latched: false,
        found_in_map_now: found,
        length_sl_m: 0,
        length_straight_m: 0,
        length_provided_m: 0,
        speed_limit_max_kph: 0,
        is_tollgate: false,
        min_curve_radius_m: 0,
        lateral_g_max_speed_kph: 0,
        measured_state: "not_seen",
        measured_time_s: 0,
        measured_avg_speed_kph: 0,
        est_time_at_speed_limit_s: 0,
        est_time_at_lateral_g_max_s: 0,
        est_time_at_ego_speed_s: 0,
        est_times_at_assumed: assumedKph.map((kph) => ({ speed_kph: kph, time_s: 0 })),
      });
      continue;
    }

    const lenSl = t.lengthSl;
    // 横G上限を満たす最大速度 v = sqrt(a_lat * R)
    const vMaxRaw = Math.sqrt(latAccel * t.minRadiusM);
    const vMaxKph = Math.min(vMaxRaw * MPS_TO_KPH, VMAX_CAP_KPH);
    const vMaxMps = vMaxKph / MPS_TO_KPH;

    // 実測状態
    let mState = "not_seen";
    let mTime = 0;
    let mAvgKph = 0;
    if (t.completed) {
      mState = "completed";
      mTime = t.exitTime - t.enterTime;
      mAvgKph = mTime > 1e-6 ? (lenSl / mTime) * MPS_TO_KPH : 0;
    } else if (t.everEgo && t.presentNow) {
      mState = "in_progress";
      mTime = t.lastInTime - t.enterTime;
    } else if (t.everEgo) {
      // 過去に乗ったが完了フラグ未設定 (通常は起きない)
      mState = "completed";
      mTime = t.lastInTime - t.enterTime;
      mAvgKph = mTime > 1e-6 ? (lenSl / mTime) * MPS_TO_KPH : 0;
    }

    targets.push({
      id,
      latched: true,
      found_in_map_now: found,
      length_sl_m: lenSl,
      length_straight_m: t.lengthStraight,
      length_provided_m: t.lengthProvided,
      speed_limit_max_kph: t.speedLimitMps * MPS_TO_KPH,
      is_tollgate: t.isTollgate,
      min_curve_radius_m: t.minRadiusM,
      lateral_g_max_speed_kph: vMaxKph,
      measured_state: mState,
      measured_time_s: mTime,
      measured_avg_speed_kph: mAvgKph,
      est_time_at_speed_limit_s: t.speedLimitMps > 1e-6 ? lenSl / t.speedLimitMps : 0,
      est_time_at_lateral_g_max_s: vMaxMps > 1e-6 ? lenSl / vMaxMps : 0,
      est_time_at_ego_speed_s: egoSpeedMps > 1e-6 ? lenSl / egoSpeedMps : 0,
      est_times_at_assumed: assumedKph.map((kph) => ({
        speed_kph: kph,
        time_s: lenSl / (kph / MPS_TO_KPH),
      })),
    });
  }

  // 現在セグメント上での経過時間 (代表 = 最も経過の長いもの)
  let curElapsed = 0;
  for (const id of egoIds) {
    const t = tracks.get(id);
    if (t != null && t.everEgo) {
      const e = t.lastInTime - t.enterTime;
      if (e > curElapsed) {
        curElapsed = e;
      }
    }
  }

  return {
    stamp,
    ego_speed_mps: egoSpeedMps,
    ego_speed_kph: egoSpeedMps * MPS_TO_KPH,
    lateral_accel_limit_mps2: latAccel,
    assumed_speeds_kph: assumedKph,
    target_count: targetIds.length,
    targets,
    current_ego_segment_ids: egoIds,
    current_ego_elapsed_s: curElapsed,
    recent_traversal_count: recentTraversals.length,
    recent_traversals: recentTraversals,
  };
}
