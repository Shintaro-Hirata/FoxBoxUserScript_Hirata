// ============================================================================
// Resource monitor: high memory process filter + ECU totals + per-process CPU
//
// /t2/resource_monitor/raw から以下を出力する。
//   1. process_info のうち resident_memory が閾値 (デフォルト 100MB) 以上の
//      プロセス一覧
//   2. 全プロセスの resident_memory 合計 (total_resident_memory_mb)
//   3. システム全体のメモリ使用量 (memory_info: used = total - free)
//   4. ECU 全体の CPU 使用率 [%] (cpu_info の累積カウンタから
//      resource_error_monitor_node と同じ「時間窓内の Δactive/Δtotal」で算出)
//   5. プロセスごとの CPU 使用率 [%] と CPU 使用率上位プロセス一覧
//      (utime + stime + cutime + cstime の増分 ÷ cpu_total_time の増分。
//       ECU 全体 = 全コア合計に対する割合)
//
// 入力:   /t2/resource_monitor/raw
// 出力:   /studio_script/high_memory_processes
//
// Variables パネルで設定する変数:
//   memory_threshold_mb : number  (省略時 100MB)
//   cpu_window_ms       : number  (省略時 2000ms; CPU使用率の計算窓)
//   cpu_top_n           : number  (省略時 10; top_cpu_processes の件数)
//
// Plot パネル例:
//   /studio_script/high_memory_processes.cpu_usage_percent
//   /studio_script/high_memory_processes.system_used_ram_mb
//   /studio_script/high_memory_processes.top_cpu_processes[0].cpu_percent
//   /studio_script/high_memory_processes.top_cpu_processes[:]{name=="planning"}.cpu_percent
// ============================================================================

import { Input } from "./types";

type GlobalVariables = {
  memory_threshold_mb?: number;
  cpu_window_ms?: number;
  cpu_top_n?: number;
};

type InProcessInfo = {
  name?: string;
  process_name?: string;
  pid?: number;
  resident_memory?: number | bigint;
  utime?: number | bigint; // ユーザーモード累積時間 [ns]
  stime?: number | bigint; // カーネルモード累積時間 [ns]
  cutime?: number | bigint; // wait 済み子プロセスのユーザーモード累積時間 [ns]
  cstime?: number | bigint; // wait 済み子プロセスのカーネルモード累積時間 [ns]
};

// apex_ecu_monitor_msgs/Duration: ナノ秒の累積値
type InDuration = { duration?: number | bigint };

type InCpuInfo = {
  num_cpu_cores?: number | bigint;
  cpu_uptime?: InDuration;
  cpu_idle_time?: InDuration;
  cpu_active_time?: InDuration;
  cpu_total_time?: InDuration;
  cpu_temp?: number;
};

// apex_ecu_monitor_msgs/MemoryInfo: /proc/meminfo 由来のため単位は KB
type InMemoryInfo = {
  free_ram_mem?: number | bigint;
  free_swap_mem?: number | bigint;
  total_ram_mem?: number | bigint;
  total_swap_mem?: number | bigint;
};

type InHeader = {
  creation_timestamp?: number | bigint;
};

type ProcessSummary = {
  index: number;
  name: string;
  pid: number;
  resident_memory_mb: number;
  cpu_percent: number; // ECU 全体 (全コア合計) に対する割合
  cpu_used_cores: number; // コア数換算 (num_cpu_cores × cpu_percent / 100)
};

type Output = {
  threshold_mb: number;
  total_process_count: number;
  filtered_count: number;
  // メモリ合計 (process_info の resident_memory の総和)
  total_resident_memory_mb: number;
  filtered_resident_memory_mb: number;
  // システム全体メモリ (memory_info[0])
  system_memory_valid: boolean;
  system_total_ram_mb: number;
  system_free_ram_mb: number;
  system_used_ram_mb: number;
  system_used_ram_percent: number;
  // ECU 全体 CPU (cpu_info[0])
  cpu_valid: boolean;
  cpu_usage_percent: number;
  cpu_used_cores: number;
  num_cpu_cores: number;
  // CPU 使用率の高い順 (cpu_top_n 件)
  top_cpu_processes: ProcessSummary[];
  // resident_memory が閾値以上 (メモリ降順)
  filtered_processes: ProcessSummary[];
};

const BYTES_TO_MB = 1048576;
const KB_TO_MB = 1024;
const DEFAULT_CPU_WINDOW_MS = 2000; // resource_monitor 設定の cpu_usage_history_time_ms と同値
const DEFAULT_CPU_TOP_N = 10;
const MAX_CPU_SAMPLES = 200;

export const inputs = ["/t2/resource_monitor/raw"];
export const output = "/studio_script/high_memory_processes";

// uint64 フィールドは bigint で届くことがあるため数値化を一元化する
function toNum(v: number | bigint | undefined): number {
  if (typeof v === "number") {
    return v;
  }
  if (typeof v === "bigint") {
    return Number(v);
  }
  const n = Number(v ?? 0);
  return Number.isFinite(n) ? n : 0;
}

function toBig(v: number | bigint | undefined): bigint {
  if (typeof v === "bigint") {
    return v;
  }
  const n = Number(v ?? 0);
  if (Number.isFinite(n)) {
    return BigInt(Math.round(n));
  }
  return BigInt(0);
}

function round2(v: number): number {
  return Math.round(v * 100) / 100;
}

// CPU 使用率計算用のサンプル履歴 (モジュール変数のため再生セッション内で保持。
// Seek backward 時は Foxglove がランタイムごとリセットする)
type CpuSample = {
  t_ns: bigint; // header.creation_timestamp (無効時は receiveTime)
  active_ns: bigint; // アクティブ時間の累積値
  total_ns: bigint; // cpu_total_time の累積値
};

// 時間窓内の最古・最新サンプル間の Δactive/Δtotal から使用率 [%] を求める。
// samples は呼び出し側が保持する履歴配列で、この関数が追加・間引きを行う。
function updateUsageWindow(
  samples: CpuSample[],
  sampleTimeNs: bigint,
  activeNs: bigint,
  totalNs: bigint,
  windowMs: number,
): number {
  // 時刻の巻き戻り (ループ再生等) を検出したら履歴を破棄する
  const last = samples[samples.length - 1];
  if (last !== undefined && sampleTimeNs < last.t_ns) {
    samples.length = 0;
  }

  samples.push({ t_ns: sampleTimeNs, active_ns: activeNs, total_ns: totalNs });

  // 時間窓と件数上限で古いサンプルを間引く
  const cutoff = sampleTimeNs - BigInt(Math.max(1, Math.round(windowMs))) * BigInt(1000000);
  while (samples.length > 0 && samples[0]!.t_ns < cutoff) {
    samples.shift();
  }
  while (samples.length > MAX_CPU_SAMPLES) {
    samples.shift();
  }

  if (samples.length < 2) {
    return 0;
  }

  const oldest = samples[0]!;
  const newest = samples[samples.length - 1]!;
  const totalDiff = Number(newest.total_ns - oldest.total_ns);
  if (totalDiff <= 0) {
    return 0;
  }
  const activeDiff = Number(newest.active_ns - oldest.active_ns);
  const usage = (activeDiff / totalDiff) * 100;
  return Math.min(100, Math.max(0, usage));
}

// ECU 全体用の履歴
const ecuCpuSamples: CpuSample[] = [];
// プロセス別の履歴。キーは "pid:プロセス名" (再起動で pid が変われば別履歴になる)
const processCpuSamples = new Map<string, CpuSample[]>();

// 消えたプロセス (または巻き戻りで無効になった) 履歴を破棄する
function cleanupProcessSamples(nowNs: bigint, windowMs: number): void {
  const staleNs = BigInt(Math.max(10000, Math.round(windowMs) * 5)) * BigInt(1000000);
  for (const [key, samples] of processCpuSamples) {
    const newest = samples[samples.length - 1];
    if (newest === undefined) {
      processCpuSamples.delete(key);
      continue;
    }
    const age = nowNs > newest.t_ns ? nowNs - newest.t_ns : newest.t_ns - nowNs;
    if (age > staleNs) {
      processCpuSamples.delete(key);
    }
  }
}

export default function script(
  event: Input<"/t2/resource_monitor/raw">,
  globalVars: GlobalVariables,
): Output {
  const msg = event.message as unknown as {
    header?: InHeader;
    cpu_info?: InCpuInfo[];
    memory_info?: InMemoryInfo[];
    process_info?: InProcessInfo[];
  };

  const processes: InProcessInfo[] = msg.process_info ?? [];
  const thresholdMB =
    typeof globalVars.memory_threshold_mb === "number"
      ? globalVars.memory_threshold_mb : 100;
  const cpuWindowMs =
    typeof globalVars.cpu_window_ms === "number" && globalVars.cpu_window_ms > 0
      ? globalVars.cpu_window_ms : DEFAULT_CPU_WINDOW_MS;
  const cpuTopN =
    typeof globalVars.cpu_top_n === "number" && globalVars.cpu_top_n >= 0
      ? Math.floor(globalVars.cpu_top_n) : DEFAULT_CPU_TOP_N;

  // --- サンプル時刻 (header 優先、無効時は receiveTime) ---
  const headerTimeNs = toBig(msg.header?.creation_timestamp);
  const sampleTimeNs =
    headerTimeNs > BigInt(0)
      ? headerTimeNs
      : BigInt(event.receiveTime.sec) * BigInt(1000000000) +
        BigInt(event.receiveTime.nsec);

  // --- ECU 全体 CPU 使用率 ---
  const cpuInfo = (msg.cpu_info ?? [])[0];
  const numCpuCores = cpuInfo !== undefined ? toNum(cpuInfo.num_cpu_cores) : 0;
  const ecuTotalNs = cpuInfo !== undefined ? toBig(cpuInfo.cpu_total_time?.duration) : BigInt(0);
  let cpuValid = false;
  let cpuUsagePercent = 0;
  if (cpuInfo !== undefined && ecuTotalNs > BigInt(0)) {
    const activeNs = toBig(cpuInfo.cpu_active_time?.duration);
    cpuUsagePercent = updateUsageWindow(
      ecuCpuSamples, sampleTimeNs, activeNs, ecuTotalNs, cpuWindowMs);
    cpuValid = true;
  }

  // --- プロセスごとのメモリと CPU 使用率 ---
  let totalResidentMB = 0;
  let filteredResidentMB = 0;
  const stats: ProcessSummary[] = [];
  for (let i = 0; i < processes.length; i++) {
    const p = processes[i]!;
    const mb = toNum(p.resident_memory) / BYTES_TO_MB;
    totalResidentMB += mb;

    const name =
      typeof p.process_name === "string" ? p.process_name
        : typeof p.name === "string" ? p.name : "";
    const pid = typeof p.pid === "number" ? p.pid : 0;

    // ECU の cpu_total_time を分母に使う (resource_error_monitor_node と同方式)
    let procCpuPercent = 0;
    if (ecuTotalNs > BigInt(0)) {
      const procActiveNs =
        toBig(p.utime) + toBig(p.stime) + toBig(p.cutime) + toBig(p.cstime);
      const key = `${pid}:${name}`;
      let samples = processCpuSamples.get(key);
      if (samples === undefined) {
        samples = [];
        processCpuSamples.set(key, samples);
      }
      procCpuPercent = updateUsageWindow(
        samples, sampleTimeNs, procActiveNs, ecuTotalNs, cpuWindowMs);
    }

    stats.push({
      index: i,
      name,
      pid,
      resident_memory_mb: round2(mb),
      cpu_percent: round2(procCpuPercent),
      cpu_used_cores: round2(numCpuCores * (procCpuPercent / 100)),
    });
  }

  cleanupProcessSamples(sampleTimeNs, cpuWindowMs);

  // CPU 使用率の高い順 (同率はメモリ降順)
  const topCpu = [...stats]
    .sort((a, b) =>
      b.cpu_percent !== a.cpu_percent
        ? b.cpu_percent - a.cpu_percent
        : b.resident_memory_mb - a.resident_memory_mb)
    .slice(0, cpuTopN);

  // メモリ閾値フィルタ (メモリ降順)
  const filtered = stats
    .filter((s) => s.resident_memory_mb >= thresholdMB)
    .sort((a, b) => b.resident_memory_mb - a.resident_memory_mb);
  for (const s of filtered) {
    filteredResidentMB += s.resident_memory_mb;
  }

  // --- システム全体メモリ (memory_info は KB 単位) ---
  const memInfo = (msg.memory_info ?? [])[0];
  const systemMemoryValid = memInfo !== undefined;
  const systemTotalMB = memInfo !== undefined ? toNum(memInfo.total_ram_mem) / KB_TO_MB : 0;
  const systemFreeMB = memInfo !== undefined ? toNum(memInfo.free_ram_mem) / KB_TO_MB : 0;
  const systemUsedMB = Math.max(0, systemTotalMB - systemFreeMB);
  const systemUsedPercent = systemTotalMB > 0 ? (systemUsedMB / systemTotalMB) * 100 : 0;

  return {
    threshold_mb: thresholdMB,
    total_process_count: processes.length,
    filtered_count: filtered.length,
    total_resident_memory_mb: round2(totalResidentMB),
    filtered_resident_memory_mb: round2(filteredResidentMB),
    system_memory_valid: systemMemoryValid,
    system_total_ram_mb: round2(systemTotalMB),
    system_free_ram_mb: round2(systemFreeMB),
    system_used_ram_mb: round2(systemUsedMB),
    system_used_ram_percent: round2(systemUsedPercent),
    cpu_valid: cpuValid,
    cpu_usage_percent: round2(cpuUsagePercent),
    cpu_used_cores: round2(numCpuCores * (cpuUsagePercent / 100)),
    num_cpu_cores: numCpuCores,
    top_cpu_processes: topCpu,
    filtered_processes: filtered,
  };
}
