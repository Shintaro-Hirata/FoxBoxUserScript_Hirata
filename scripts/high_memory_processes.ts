// ============================================================================
// High memory process filter
//
// /t2/resource_monitor/raw の process_info から resident_memory が
// 閾値 (デフォルト 100MB) 以上のプロセスを抽出する。
//
// 入力:   /t2/resource_monitor/raw
// 出力:   /studio_script/high_memory_processes
//
// Variables パネルで設定する変数:
//   memory_threshold_mb : number  (省略時 100MB)
// ============================================================================

import { Input } from "./types";

type GlobalVariables = {
  memory_threshold_mb?: number;
};

type InProcessInfo = {
  name?: string;
  pid?: number;
  resident_memory?: number;
  virtual_memory?: number;
  cpu_percent?: number;
  command?: string;
};

type ProcessSummary = {
  name: string;
  pid: number;
  resident_memory_mb: number;
};

type Output = {
  threshold_mb: number;
  total_process_count: number;
  filtered_count: number;
  filtered_processes: ProcessSummary[];
};

const BYTES_TO_MB = 1048576;

export const inputs = ["/t2/resource_monitor/raw"];
export const output = "/studio_script/high_memory_processes";

export default function script(
  event: Input<"/t2/resource_monitor/raw">,
  globalVars: GlobalVariables,
): Output {
  const msg = event.message as unknown as {
    process_info: InProcessInfo[];
  };

  const processes: InProcessInfo[] = msg.process_info ?? [];
  const thresholdMB =
    typeof globalVars.memory_threshold_mb === "number"
      ? globalVars.memory_threshold_mb : 100;

  const filtered: ProcessSummary[] = [];
  for (const p of processes) {
    const bytes = typeof p.resident_memory === "number" ? p.resident_memory : 0;
    const mb = bytes / BYTES_TO_MB;
    if (mb >= thresholdMB) {
      filtered.push({
        name: typeof p.name === "string" ? p.name : "",
        pid: typeof p.pid === "number" ? p.pid : 0,
        resident_memory_mb: Math.round(mb * 100) / 100,
      });
    }
  }

  filtered.sort((a, b) => b.resident_memory_mb - a.resident_memory_mb);

  return {
    threshold_mb: thresholdMB,
    total_process_count: processes.length,
    filtered_count: filtered.length,
    filtered_processes: filtered,
  };
}
