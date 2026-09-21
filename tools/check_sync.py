"""核对 sync.csv 诊断文件完整性（2B 落盘链路验收用）。

用法:
    python tools/check_sync.py <sync.csv 路径>

只做只读统计和展示，不做硬判定；由使用者对照验收标准自行判断。
"""
from __future__ import annotations

import argparse
import csv
from collections import Counter


EXPECTED_COLUMNS = [
    "unix_ms", "pps",
    "sample_count", "interrupt_count", "interrupt_overruns",
    "cc2_overcapture", "dt_gap_count", "i2c_errors",
    "d_interrupt_overruns", "d_cc2_overcapture", "d_dt_gap_count", "d_i2c_errors",
    "backlog",
]
DELTA_FIELDS = ["d_interrupt_overruns", "d_cc2_overcapture",
                "d_dt_gap_count", "d_i2c_errors"]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    args = ap.parse_args()

    with open(args.path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames
        rows = list(reader)

    print("=== 1. 表头字段 ===")
    print("OK" if header == EXPECTED_COLUMNS else f"不匹配\n  实际: {header}")

    print("\n=== 2. 行数与时间跨度 ===")
    print(f"数据行数: {len(rows)}")
    if len(rows) >= 2:
        span = int(rows[-1]["unix_ms"]) - int(rows[0]["unix_ms"])
        print(f"时间跨度: {span / 1000:.1f} s  (约 {len(rows) - 1} 个间隔)")
        gaps = [int(rows[i]["unix_ms"]) - int(rows[i - 1]["unix_ms"])
                for i in range(1, len(rows))]
        med = sorted(gaps)[len(gaps) // 2]
        print(f"unix_ms 间隔(ms): 中位 {med}, 范围 {min(gaps)}~{max(gaps)}")

    print("\n=== 3. 四个异常增量 d_* ===")
    for field in DELTA_FIELDS:
        vals = [int(r[field]) for r in rows]
        nonzero = sorted({v for v in vals if v != 0})
        n = sum(1 for v in vals if v != 0)
        extra = f"  非零值: {nonzero}" if nonzero else ""
        print(f"{field}: 非零 {n} 次{extra}")

    print("\n=== 4. backlog 分布 (interrupt_count - sample_count) ===")
    dist = Counter(int(r["backlog"]) for r in rows)
    print(dict(sorted(dist.items())))

    print("\n=== 5. 累计计数器首尾 ===")
    if rows:
        first, last = rows[0], rows[-1]
        print(f"首行: pps={first['pps']} sample={first['sample_count']} "
              f"interrupt={first['interrupt_count']}")
        print(f"尾行: pps={last['pps']} sample={last['sample_count']} "
              f"interrupt={last['interrupt_count']}")

    print("\n=== 提示 ===")
    print("静止正常: 四个 d_* 应全为 0；backlog 应只在 0/1 小范围波动、无持续增长；")
    print("尾行 sample/interrupt 应随 pps 正常递增（若尾行明显早于预期，说明停采时未 flush 完整）。")


if __name__ == "__main__":
    main()
