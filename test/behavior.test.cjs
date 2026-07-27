// ============================================================================
// Synthetic-data behavior tests for the FoxBox User Scripts.
//
// Run with:  npm test        (compiles scripts to test/out/, then runs this)
//        or:  bash test/run.sh
//
// These are NOT unit tests of Foxglove itself. They feed hand-built messages
// (shaped like the real ROS2 topics) into each script's default export and
// assert on the output object, covering the tricky stateful logic: ego-segment
// resolution, related-segment linking, the traversal-time state machine,
// follower detection on the SL frame, lateral-G math, and speed margins.
// ============================================================================

const ego  = require("./out/ego_lane_segments.js").default;
const trav = require("./out/lane_segment_traversal_time.js").default;
const foll = require("./out/follower_gap_tracker.js").default;
const latg = require("./out/lateral_g_monitor.js").default;
const smp  = require("./out/speed_margin_profile.js").default;

let pass = 0;
let fail = 0;
function ok(cond, msg) {
  if (cond) { pass++; } else { fail++; console.log("  FAIL:", msg); }
}
function approx(a, b, t, msg) { ok(Math.abs(a - b) <= t, `${msg} (got ${a}, want ~${b})`); }
function T(sec) { return { sec: Math.floor(sec), nsec: Math.round((sec - Math.floor(sec)) * 1e9) }; }
function straight(x0, x1, n) {
  const p = [];
  for (let i = 0; i < n; i++) { const x = x0 + (x1 - x0) * i / (n - 1); p.push({ x, y: 0, z: 0 }); }
  return p;
}
function seg(id, curve, length, splim, toll) {
  return { id, central_curve: curve, length, speed_limit_max: splim, is_tollgate: !!toll };
}
function scene(segs, egoIdx) {
  return {
    topic: "/t2/object_augmentor/augmented_scene", receiveTime: null,
    message: { local_map_info: { local_lane_segments: segs, ego_lane_segment_indices: egoIdx } },
  };
}

console.log("--- TEST 1: ego_lane_segments basic lengths ---");
{
  const segs = [
    seg("A", straight(-30, -10, 3), 20, 8.33, false),
    seg("B", straight(-10, 40, 6), 50, 8.33, false),  // ego origin projects to s=10
    seg("C", straight(40, 80, 5), 40, 11.1, false),
  ];
  const ev = scene(segs, [1]); ev.receiveTime = T(0);
  const out = ego(ev, {});
  ok(out.source === "ego_indices", "source=ego_indices");
  ok(out.ego_segment_count === 1, "ego_segment_count=1");
  ok(out.primary_segment_id === "B", "primary=B");
  approx(out.segments[0].straight_m, 50, 1e-6, "B straight=50");
  approx(out.segments[0].sl_m, 50, 1e-6, "B sl=50");
  approx(out.segments[0].provided_length_m, 50, 1e-6, "B provided=50");
  approx(out.segments[0].ego_s_in_segment_m, 10, 1e-6, "B ego_s=10");
  approx(out.segments[0].speed_limit_max_kph, 29.988, 0.01, "B splim kph");
  approx(out.total_sl_m, 50, 1e-6, "total_sl=50");
}

console.log("--- TEST 2: ego_lane_segments fallback (no indices) ---");
{
  const segs = [seg("A", straight(-10, 40, 6), 50, 8.33, false)];
  const ev = scene(segs, []); ev.receiveTime = T(0);
  const out = ego(ev, {});
  ok(out.source === "projection_fallback", "fallback used");
  ok(out.primary_segment_id === "A", "fallback primary=A");
}

console.log("--- TEST 3: ego_lane_segments multi-segment total ---");
{
  const segs = [seg("A", straight(-30, 0, 4), 30, 8.33, false), seg("B", straight(0, 20, 3), 20, 8.33, false)];
  const ev = scene(segs, [0, 1]); ev.receiveTime = T(0);
  const out = ego(ev, {});
  ok(out.ego_segment_count === 2, "two ego segs");
  approx(out.total_sl_m, 50, 1e-6, "total sl 30+20=50");
  approx(out.total_straight_m, 50, 1e-6, "total straight 50");
}

console.log("--- TEST 4: ego_lane_segments v2 related links + ETC context ---");
{
  const A = {
    id: "A", central_curve: straight(-10, 40, 6), length: 50, speed_limit_max: 8.333,
    successor_ids: ["B"], predecessor_ids: ["Z"], left_neighbor_forward_id: ["L"],
    right_neighbor_forward_id: ["R"], is_tollgate: false, location_context: { location: 1, context: 3 },
  };
  const B = { id: "B", central_curve: straight(40, 80, 5), length: 40, speed_limit_max: 11.11, location_context: { context: 4 } };
  const Z = { id: "Z", central_curve: straight(-40, -10, 4), length: 30, location_context: { context: 2 } };
  const L = { id: "L", central_curve: straight(-10, 40, 6), length: 50 };
  const R = { id: "R", central_curve: straight(-10, 40, 6), length: 50 };
  const ev = scene([A, B, Z, L, R], [0]); ev.receiveTime = T(0);
  const o = ego(ev, {});
  ok(o.primary_segment_id === "A", "primary A");
  ok(o.primary_context_name === "PASSING_ETC", "context PASSING_ETC");
  ok(JSON.stringify(o.segments[0].successor_ids) === '["B"]', "succ ids");
  ok(JSON.stringify(o.segments[0].left_neighbor_ids) === '["L"]', "left neighbor");
  ok(JSON.stringify(o.segments[0].right_neighbor_ids) === '["R"]', "right neighbor");
  ok(o.related_segment_count === 4, "4 related");
  const rel = {}; o.related_segments.forEach((r) => { rel[r.id] = r.relation; });
  ok(rel["B"] === "successor", "B=successor");
  ok(rel["Z"] === "predecessor", "Z=predecessor");
  ok(rel["L"] === "left_neighbor", "L=left_neighbor");
  ok(rel["R"] === "right_neighbor", "R=right_neighbor");
  const Bdet = o.related_segments.find((r) => r.id === "B");
  ok(Bdet.context_name === "EXITING_ETC", "B context EXITING_ETC");
  approx(Bdet.speed_limit_max_kph, 40, 0.1, "B 40kph");
}

console.log("--- TEST 5: traversal measured time A->B->C ---");
{
  const A = seg("A", straight(0, 30, 4), 30, 8.33, false);
  const B = seg("B", straight(0, 40, 5), 40, 8.33, false);
  const C = seg("C", straight(0, 50, 6), 50, 11.1, false);
  const vars = { target_segment_ids: "A, B C", assumed_speeds_kph: "10,30,40" };
  let last;
  for (let t = 0.0; t <= 3.0 + 1e-9; t += 0.1) { const ev = scene([A, B, C], [0]); ev.receiveTime = T(t); last = trav(ev, vars); }
  ok(last.targets.find((x) => x.id === "A").measured_state === "in_progress", "A in_progress before exit");
  for (let t = 3.1; t <= 7.0 + 1e-9; t += 0.1) { const ev = scene([A, B, C], [1]); ev.receiveTime = T(t); last = trav(ev, vars); }
  for (let t = 7.1; t <= 8.0 + 1e-9; t += 0.1) { const ev = scene([A, B, C], [2]); ev.receiveTime = T(t); last = trav(ev, vars); }
  const a = last.targets.find((x) => x.id === "A");
  const b = last.targets.find((x) => x.id === "B");
  const c = last.targets.find((x) => x.id === "C");
  ok(a.measured_state === "completed", "A completed");
  approx(a.measured_time_s, 3.0, 0.12, "A measured ~3.0s (present span)");
  approx(a.measured_avg_speed_kph, (30 / a.measured_time_s) * 3.6, 1e-6, "A avg speed = len/measured");
  ok(b.measured_state === "completed", "B completed");
  approx(b.measured_time_s, 3.9, 0.12, "B measured ~3.9s (present span)");
  ok(c.measured_state === "in_progress", "C in_progress");
  const c40 = c.est_times_at_assumed.find((e) => e.speed_kph === 40);
  approx(c40.time_s, 50 / (40 / 3.6), 1e-6, "C est@40kph");
  approx(c.est_time_at_speed_limit_s, 50 / 11.1, 1e-3, "C est@speedlimit");
  ok(last.recent_traversals.some((r) => r.id === "A") && last.recent_traversals.some((r) => r.id === "B"), "recent has A,B");
  ok(last.recent_traversals[0].id === "B", "most recent completed is B");
}

console.log("--- TEST 6: traversal seek-backward resets state ---");
{
  const A = seg("A", straight(0, 30, 4), 30, 8.33, false);
  const vars = { target_segment_ids: "A" };
  let ev = scene([A], [0]); ev.receiveTime = T(200); trav(ev, vars);
  ev = scene([A], [0]); ev.receiveTime = T(201); trav(ev, vars);
  ev = scene([A], [0]); ev.receiveTime = T(50); const last = trav(ev, vars);
  ok(last.recent_traversals.length === 0, "recent cleared after seek-back");
  ok(last.targets[0].measured_state === "in_progress", "A re-entered in_progress after seek");
}

console.log("--- TEST 7: follower_gap_tracker ---");
{
  const E = { id: "E", central_curve: straight(-50, 50, 11), length: 100, successor_ids: [], predecessor_ids: [] };
  function obj(tid, x, y, vx, rel, stat) {
    return {
      track_id: tid,
      bbox_info: { id: tid, local_position: { x, y, z: 0 }, width: 1.8, length: 4.5, type: 1 },
      tracking_info: { local_velocity: { x: vx, y: 0, z: 0 }, local_relative_velocity: { x: rel, y: 0, z: 0 }, is_stationary: !!stat },
    };
  }
  const objs = [obj(11, -20, 0, 8, 0.5), obj(22, 30, 0, 9, -0.2), obj(33, -10, 5, 7, 0), obj(44, -40, 0.5, 8, 0.1)];
  const ev = {
    topic: "/t2/object_augmentor/augmented_scene", receiveTime: T(0),
    message: { local_map_info: { local_lane_segments: [E] }, augmented_objects: objs },
  };
  const o = foll(ev, {});
  ok(o.sl_valid, "sl valid");
  approx(o.central_curve_length_m, 100, 1, "central len 100");
  ok(o.follower_found, "follower found");
  ok(o.follower_track_id === 11, "follower=11 (nearest behind)");
  approx(o.follower_gap_s_m, 20, 0.5, "follower gap 20");
  ok(o.follower_closing === true, "follower closing");
  approx(o.follower_abs_speed_mps, 8, 1e-6, "follower abs 8");
  ok(o.behind_count === 2, "behind_count=2 (11,44; not adjacent 33)");
  ok(o.ahead_count === 1, "ahead_count=1");
  ok(o.leader_found && o.leader_track_id === 22, "leader=22");
  approx(o.leader_gap_s_m, 30, 0.5, "leader gap 30");
  ok(o.behind_objects[0].track_id === 11 && o.behind_objects[1].track_id === 44, "behind sorted near-first");
  approx(o.follower_time_headway_s, 20 / 8, 0.05, "time headway 2.5s");
}

console.log("--- TEST 8: lateral_g_monitor ---");
{
  function pose(speed, lat, yaw) {
    return {
      topic: "/t2/odometry/ego", receiveTime: T(0),
      message: {
        local_linear_velocity: { x: speed, y: 0, z: 0 },
        local_linear_acceleration: { x: 0.1, y: lat, z: 0 },
        local_angular_velocity: { x: 0, y: 0, z: yaw },
      },
    };
  }
  let o = latg(pose(10, 2.0, 0.2), {});
  approx(o.speed_mps, 10, 1e-6, "speed 10");
  approx(o.lateral_accel_measured_mps2, 2.0, 1e-6, "measured 2.0");
  approx(o.lateral_accel_kinematic_mps2, 2.0, 1e-6, "kinematic v*w=2.0");
  approx(o.ratio_measured, 0.8, 1e-6, "ratio 0.8");
  ok(o.over_limit === false, "not over 2.5");
  const p2 = {
    topic: "/t2/odometry/ego", receiveTime: T(0.1),
    message: { local_linear_velocity: { x: 10, y: 0, z: 0 }, local_linear_acceleration: { x: 0, y: 3.0, z: 0 }, local_angular_velocity: { z: 0.3 } },
  };
  o = latg(p2, {});
  ok(o.over_limit === true, "over limit at 3.0");
  approx(o.peak_abs_mps2, 3.0, 1e-6, "peak 3.0");
}

console.log("--- TEST 9: speed_margin_profile ---");
{
  const P = {
    id: "P", central_curve: straight(-10, 40, 6), length: 50, speed_limit_max: 8.333, target_speed_max: 8.333,
    successor_ids: ["Q"], is_tollgate: false, location_context: { context: 3 },
  };
  const Q = { id: "Q", central_curve: straight(40, 90, 6), length: 50, speed_limit_max: 11.11, successor_ids: [], location_context: { context: 4 } };
  smp({ topic: "/t2/odometry/ego", receiveTime: T(0), message: { local_linear_velocity: { x: 8.333, y: 0, z: 0 } } }, {});
  const ev = scene([P, Q], [0]); ev.topic = "/t2/object_augmentor/augmented_scene"; ev.receiveTime = T(0.1);
  const o = smp(ev, {});
  ok(o.has_ego_segment, "has ego seg");
  ok(o.primary_segment_id === "P", "primary P");
  approx(o.ego_speed_kph, 30, 0.05, "ego 30kph");
  approx(o.segments[0].speed_limit_kph, 30, 0.05, "P limit 30");
  approx(o.segments[0].margin_to_limit_kph, 0, 0.05, "margin to limit ~0");
  ok(o.segments[0].raise_ok === true, "raise ok (straight)");
  ok(o.lookahead_count === 1, "lookahead 1 (Q)");
  ok(o.lookahead[0].id === "Q", "lookahead Q");
  approx(o.lookahead[0].speed_limit_kph, 40, 0.1, "Q 40kph");
  ok(o.lookahead[0].context_name === "EXITING_ETC", "Q EXITING_ETC");
}

console.log("--- TEST 10: bigint ego_lane_segment_indices (uint64) accepted ---");
{
  // Foxglove delivers uint64 indices as bigint. Scripts must coerce, not skip.
  const segs = [seg("A", straight(-30, -10, 3), 20, 8.33, false), seg("B", straight(-10, 40, 6), 50, 8.33, false)];
  const ev = {
    topic: "/t2/object_augmentor/augmented_scene", receiveTime: T(0),
    message: { local_map_info: { local_lane_segments: segs, ego_lane_segment_indices: [BigInt(1)] } },
  };
  const o = ego(ev, {});
  ok(o.source === "ego_indices", "bigint index accepted (source=ego_indices, not fallback)");
  ok(o.primary_segment_id === "B", "bigint index resolves to B");
}

console.log("--- TEST 11: traversal flicker debounce + re-entry re-measure ---");
{
  const F = seg("F11", straight(0, 40, 5), 40, 8.33, false);
  const G = seg("G11", straight(0, 40, 5), 40, 8.33, false);
  const vars = { target_segment_ids: "F11" };
  const fr = (t, idxs) => ({
    topic: "/t2/object_augmentor/augmented_scene", receiveTime: T(t),
    message: { local_map_info: { local_lane_segments: [F, G], ego_lane_segment_indices: idxs } },
  });
  let last;
  // On F for 1000.0..1002.0 with a single-frame dropout at i=10 (t=1001.0)
  for (let i = 0; i <= 20; i++) {
    const idxs = i === 10 ? [] : [BigInt(0)];
    last = trav(fr(1000.0 + i * 0.1, idxs), vars);
  }
  ok(last.targets[0].measured_state === "in_progress", "flicker: F still in_progress (not truncated)");
  approx(last.targets[0].measured_time_s, 2.0, 0.15, "flicker: measured spans full ~2.0s");
  // Leave F for good (>0.5s) -> completes with the full span
  for (let i = 21; i <= 30; i++) { last = trav(fr(1000.0 + i * 0.1, [BigInt(1)]), vars); }
  ok(last.targets[0].measured_state === "completed", "F completed after real exit");
  approx(last.targets[0].measured_time_s, 2.0, 0.15, "F measured ~2.0s (present span)");
  // Genuine re-entry after completion -> re-measures from a new enter
  for (let i = 31; i <= 35; i++) { last = trav(fr(1000.0 + i * 0.1, [BigInt(0)]), vars); }
  ok(last.targets[0].measured_state === "in_progress", "F re-entry -> in_progress again");
  ok(last.targets[0].measured_time_s < 1.0, "F re-entry measured restarts small");
}

console.log("--- TEST 12: follower uses ego_lane_segment_indices (bigint) as SL anchor ---");
{
  const E = { id: "E12", central_curve: straight(-50, 50, 11), length: 100, successor_ids: [], predecessor_ids: [] };
  const obj = {
    bbox_info: { id: 77, local_position: { x: -15, y: 0, z: 0 }, width: 1.8, length: 4.5, type: 1 },
    tracking_info: { local_velocity: { x: 7, y: 0, z: 0 }, local_relative_velocity: { x: 0.3, y: 0, z: 0 }, is_stationary: false },
  };
  const ev = {
    topic: "/t2/object_augmentor/augmented_scene", receiveTime: T(2000),
    message: { local_map_info: { local_lane_segments: [E], ego_lane_segment_indices: [BigInt(0)] }, augmented_objects: [obj] },
  };
  const o = foll(ev, {});
  ok(o.follower_found && o.follower_track_id === 77, "follower found via bigint ego index");
  approx(o.follower_gap_s_m, 15, 0.5, "follower gap 15");
}

console.log(`\nRESULT: ${pass} passed, ${fail} failed`);
process.exit(fail > 0 ? 1 : 0);
