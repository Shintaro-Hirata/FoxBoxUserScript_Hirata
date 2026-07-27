// ============================================================================
// Lateral-G monitor  v1
//
// /t2/odometry/ego から自車の実測横加速度 (横G) を出力し、横G上限と比較する。
// 「2.5 m/s^2 制約により ETC 前後を 30km/h に制限している」前提を実データで
// 確認し、速度引き上げ余地を評価するための指標。
//
// 入力:   /t2/odometry/ego
// 出力:   /studio_script/lateral_g_monitor
//
// Variables パネルで設定する変数:
//   lateral_accel_limit_mps2 : number -- 横加速度上限 [m/s^2] (既定 2.5)
//   ma_window                 : number -- 移動平均/窓ピークのサンプル数 (既定 11)
//
// 横G の 2 系統:
//   measured  : local_linear_acceleration.y (車両座標系の横加速度推定)
//   kinematic : speed * yaw_rate (v*omega, 旋回による求心加速度)
// 通常はほぼ一致する。両方出すことで相互検証できる。
// ============================================================================

import { Input } from "./types";

type Vec3  = { x: number; y: number; z: number };
type Stamp = { sec: number; nsec: number };

type GlobalVariables = {
  lateral_accel_limit_mps2?: unknown;
  ma_window?: unknown;
};

type InEgoPose = {
  local_linear_velocity?: Vec3;
  linear_velocity?: Vec3;
  local_linear_acceleration?: Vec3;
  angular_velocity?: Vec3;
  local_angular_velocity?: Vec3;
};

type Output = {
  stamp: Stamp;
  speed_mps: number;
  speed_kph: number;
  yaw_rate_radps: number;
  yaw_rate_degps: number;
  lateral_accel_measured_mps2: number;     // local_linear_acceleration.y (符号付き)
  lateral_accel_measured_abs_mps2: number;
  lateral_accel_kinematic_mps2: number;    // speed * yaw_rate (符号付き)
  lateral_accel_kinematic_abs_mps2: number;
  longitudinal_accel_mps2: number;         // local_linear_acceleration.x
  limit_mps2: number;
  ratio_measured: number;                  // |measured| / limit
  ratio_kinematic: number;                 // |kinematic| / limit
  margin_measured_mps2: number;            // limit - |measured|
  over_limit: boolean;                     // |measured| > limit
  peak_abs_mps2: number;                   // リセット以降の最大 |measured|
  ma_window: number;
  ma_abs_mps2: number;                     // |measured| の移動平均
  windowed_peak_abs_mps2: number;          // 窓内の最大 |measured|
};

const MPS_TO_KPH = 3.6;
const RAD_TO_DEG = 180 / Math.PI;

function isNum(v: unknown): v is number {
  return typeof v === "number" && isFinite(v);
}
function num(v: unknown, dflt: number): number {
  return isNum(v) ? v : dflt;
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

// --- モジュール状態 ----------------------------------------------------------
const absBuffer: number[] = [];
let peakAbs = 0;
let lastProcTime = -1;

function resetState(): void {
  absBuffer.length = 0;
  peakAbs = 0;
}

export const inputs = ["/t2/odometry/ego"];
export const output = "/studio_script/lateral_g_monitor";

export default function script(
  event: Input<"/t2/odometry/ego">,
  globalVars: GlobalVariables,
): Output {
  const nowSec = timeToSec(event.receiveTime);
  if (lastProcTime >= 0 && nowSec < lastProcTime) {
    resetState();
  }
  lastProcTime = nowSec;

  const ego = event.message as unknown as InEgoPose;
  const stamp: Stamp = { sec: num(event.receiveTime?.sec, 0), nsec: num(event.receiveTime?.nsec, 0) };

  const limit = Math.max(0.1, num(globalVars.lateral_accel_limit_mps2, 2.5));
  const maWindow = Math.max(1, Math.trunc(num(globalVars.ma_window, 11)));

  const speed = isVec3(ego.local_linear_velocity)
    ? speedMag(ego.local_linear_velocity)
    : speedMag(ego.linear_velocity);

  // ヨーレート: 車両座標系を優先 (z 軸まわりは座標系不変)
  const yawRate = isVec3(ego.local_angular_velocity)
    ? (ego.local_angular_velocity as Vec3).z
    : (isVec3(ego.angular_velocity) ? (ego.angular_velocity as Vec3).z : 0);

  const latMeasured = isVec3(ego.local_linear_acceleration)
    ? (ego.local_linear_acceleration as Vec3).y : 0;
  const lonMeasured = isVec3(ego.local_linear_acceleration)
    ? (ego.local_linear_acceleration as Vec3).x : 0;
  const latKinematic = speed * yawRate;

  const latAbs = Math.abs(latMeasured);

  // 移動平均 / 窓ピーク
  absBuffer.push(latAbs);
  while (absBuffer.length > maWindow) {
    absBuffer.shift();
  }
  let sum = 0;
  let wPeak = 0;
  for (const v of absBuffer) {
    sum += v;
    if (v > wPeak) {
      wPeak = v;
    }
  }
  const maAbs = absBuffer.length > 0 ? sum / absBuffer.length : 0;

  if (latAbs > peakAbs) {
    peakAbs = latAbs;
  }

  return {
    stamp,
    speed_mps: speed,
    speed_kph: speed * MPS_TO_KPH,
    yaw_rate_radps: yawRate,
    yaw_rate_degps: yawRate * RAD_TO_DEG,
    lateral_accel_measured_mps2: latMeasured,
    lateral_accel_measured_abs_mps2: latAbs,
    lateral_accel_kinematic_mps2: latKinematic,
    lateral_accel_kinematic_abs_mps2: Math.abs(latKinematic),
    longitudinal_accel_mps2: lonMeasured,
    limit_mps2: limit,
    ratio_measured: latAbs / limit,
    ratio_kinematic: Math.abs(latKinematic) / limit,
    margin_measured_mps2: limit - latAbs,
    over_limit: latAbs > limit,
    peak_abs_mps2: peakAbs,
    ma_window: maWindow,
    ma_abs_mps2: maAbs,
    windowed_peak_abs_mps2: wPeak,
  };
}
