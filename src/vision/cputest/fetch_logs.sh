#!/usr/bin/env bash
# Download cputest logs from jecs to a ROS-free analysis machine.

set -euo pipefail

remote_host="${1:-jecs@192.168.0.100}"
local_dir="${2:-./perf_logs}"
mkdir -p "$local_dir"

if command -v rsync >/dev/null 2>&1; then
  rsync -av "${remote_host}:~/perf_logs/" "${local_dir}/"
else
  scp -r "${remote_host}:~/perf_logs/." "${local_dir}/"
fi

file_count=$(find "$local_dir" -type f | wc -l | tr -d ' ')
latest_file=$(find "$local_dir" -type f -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2- || true)
if [[ -z "$latest_file" ]]; then
  latest_file=$(ls -1t "$local_dir" 2>/dev/null | head -n 1 || true)
fi

echo "Downloaded files: ${file_count}"
echo "Latest file: ${latest_file:-none}"
