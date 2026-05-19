// ============================================================================
// Target object finder for /t2/object_augmentor/augmented_scene  v2
//
// augmented_scene の augmented_objects から、指定した track_id の物体を毎フレーム
// 探して情報を出力する。track_id はシーンをまたいで安定するため、
// bev_detection の position マッチングより確実にターゲットを追跡できる。
//
// 入力:   /t2/object_augmentor/augmented_scene
// 出力:   /studio_script/target_object
//
// Variables パネルで設定する変数:
//   track_id        : number  -- 追跡する物体の track_id (bbox_info.id)
//   target_type     : number  -- 物体タイプで絞り込み (省略可)
//   target_sub_type : number  -- サブタイプで絞り込み (省略可)
//
// track_id の確認方法:
//   Raw Messages で /t2/object_augmentor/augmented_scene の augmented_objects を
//   展開し、追跡したい物体の bbox_info.id (または tracking_info.track_id) を確認。
// ============================================================================

import { Input } from "./types";

type GlobalVariables = {
  track_id?: number | string;
  target_type?: number | string;
  target_sub_type?: number | string;
  velocity_max_mps?: number | string;
};

type Vec3   = { x: number; y: number; z: number };
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
  confidence?: number;
};

type InAugObj = {
  bbox_info: InBBoxInfo;
  tracking_info?: {
    velocity?: Vec3;
    track_id?: number;
  };
  track_id?: number;
};

type MatchedObject = {
  track_id: number;
  bbox_id: number;
  local_position: Vec3;
  local_position_valid: boolean;
  local_theta: number;
  length: number;
  width: number;
  height: number;
  type: number;
  sub_type: number;
  confidence: number;
  velocity: Vec3;
  velocity_norm: number;
  velocity_valid: boolean;
};

type Output = {
  header: Header;
  track_id_input: number;
  velocity_max_mps_input: number;
  match_found: boolean;
  augmented_object_count: number;
  matched_index: number;
  matched: MatchedObject;
};

function zeroVec3(): Vec3 { return { x: 0, y: 0, z: 0 }; }
function copyVec3(v: Vec3): Vec3 { return { x: v.x, y: v.y, z: v.z }; }

function isValidVec3(v: unknown): v is Vec3 {
  if (v == null || typeof v !== "object") return false;
  const o = v as { x?: unknown; y?: unknown; z?: unknown };
  return typeof o.x === "number" && typeof o.y === "number" && typeof o.z === "number";
}

function emptyMatched(): MatchedObject {
  return {
    track_id: -1, bbox_id: -1,
    local_position: zeroVec3(), local_position_valid: false,
    local_theta: 0, length: 0, width: 0, height: 0,
    type: 0, sub_type: 0, confidence: 0,
    velocity: zeroVec3(), velocity_norm: 0, velocity_valid: false,
  };
}

function getTrackId(ao: InAugObj): number {
  if (typeof ao.track_id === "number") return ao.track_id;
  if (ao.tracking_info && typeof ao.tracking_info.track_id === "number") return ao.tracking_info.track_id;
  if (typeof ao.bbox_info.id === "number") return ao.bbox_info.id;
  return -1;
}

// augmented_scene の track_id で特定 → bev_detection から local_position 最近傍で
// 正しい width 等を取得する。
let storedRefLocalPos: Vec3 = { x: 0, y: 0, z: 0 };
let storedRefFound = false;
let storedBevWidth = 0;
let storedBevFound = false;

export const inputs = [
  "/t2/object_augmentor/augmented_scene",
  "/t2/bev_detection/objects",
];
export const output = "/studio_script/target_object";

type InputEvent =
  | Input<"/t2/object_augmentor/augmented_scene">
  | Input<"/t2/bev_detection/objects">;

export default function script(
  event: InputEvent,
  globalVars: GlobalVariables,
): Output | undefined {

  if (event.topic === "/t2/bev_detection/objects") {
    if (storedRefFound) {
      const bmsg = event.message as unknown as {
        detected_objects: { id?: number; local_position?: Vec3; width?: number }[];
      };
      let bestD = Number.POSITIVE_INFINITY;
      for (const o of bmsg.detected_objects ?? []) {
        if (!isValidVec3(o.local_position)) continue;
        const dx = (o.local_position as Vec3).x - storedRefLocalPos.x;
        const dy = (o.local_position as Vec3).y - storedRefLocalPos.y;
        const d = Math.sqrt(dx * dx + dy * dy);
        if (d < bestD) {
          bestD = d;
          storedBevWidth = typeof o.width === "number" ? o.width : 0;
          storedBevFound = true;
        }
      }
    }
    return undefined;
  }

  const msg = event.message as unknown as {
    header: Header;
    augmented_objects: InAugObj[];
  };
  const augObjects: InAugObj[] = msg.augmented_objects ?? [];

  const wantTrackId = globalVars.track_id != null ? Number(globalVars.track_id) : -1;
  const wantType    = globalVars.target_type != null ? Number(globalVars.target_type) : -1;
  const wantSubType = globalVars.target_sub_type != null ? Number(globalVars.target_sub_type) : -1;
  const velMaxMps   = globalVars.velocity_max_mps != null ? Number(globalVars.velocity_max_mps) : -1;

  let matchFound = false;
  let matchedIdx = -1;
  let matched = emptyMatched();

  for (let i = 0; i < augObjects.length; i++) {
    const ao = augObjects[i]!;
    const bi = ao.bbox_info;
    const tid = getTrackId(ao);

    // track_id フィルタ
    if (wantTrackId >= 0 && tid !== wantTrackId) continue;

    // type / sub_type フィルタ (設定時のみ)
    const objType = typeof bi.type === "number" ? bi.type : -1;
    const objSubType = typeof bi.sub_type === "number" ? bi.sub_type : -1;
    if (wantType >= 0 && objType !== wantType) continue;
    if (wantSubType >= 0 && objSubType !== wantSubType) continue;

    const hasLocal = isValidVec3(bi.local_position);
    const hasVel = ao.tracking_info != null && isValidVec3(ao.tracking_info.velocity);
    const vel = hasVel ? ao.tracking_info!.velocity as Vec3 : zeroVec3();
    const velNorm = hasVel ? Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z) : 0;

    // velocity フィルタ (設定時のみ): norm が閾値以下のみマッチ
    if (velMaxMps >= 0 && velNorm > velMaxMps) continue;

    // 参照座標を保存 (bev_detection マッチング用)
    if (hasLocal) {
      storedRefLocalPos = copyVec3(bi.local_position as Vec3);
      storedRefFound = true;
    }

    matched = {
      track_id: tid,
      bbox_id: typeof bi.id === "number" ? bi.id : -1,
      local_position: hasLocal ? copyVec3(bi.local_position as Vec3) : zeroVec3(),
      local_position_valid: hasLocal,
      local_theta: typeof bi.local_theta === "number" ? bi.local_theta : 0,
      length:     typeof bi.length     === "number" ? bi.length     : 0,
      width:      storedBevFound ? storedBevWidth
                    : typeof bi.width === "number" ? bi.width : 0,
      height:     typeof bi.height     === "number" ? bi.height     : 0,
      type:       objType,
      sub_type:   objSubType,
      confidence: typeof bi.confidence === "number" ? bi.confidence : 0,
      velocity: hasVel ? copyVec3(vel) : zeroVec3(),
      velocity_norm: velNorm,
      velocity_valid: hasVel,
    };
    matchFound = true;
    matchedIdx = i;
    break;
  }

  return {
    header: msg.header,
    track_id_input: wantTrackId,
    velocity_max_mps_input: velMaxMps,
    match_found: matchFound,
    augmented_object_count: augObjects.length,
    matched_index: matchedIdx,
    matched,
  };
}
