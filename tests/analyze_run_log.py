"""텔레옵 실행 기록(logs/*.csv)에서 끊김 원인 후보를 숫자로 본다. 로봇·마우스는 쓰지 않는다.

    python tests/analyze_run_log.py                  # logs/의 가장 최근 기록
    python tests/analyze_run_log.py logs/teleop_....csv

- 제어 주기 간격: 늦게 돈 주기가 많으면 명령이 끊긴다.
- 입력 시간 제한: 마우스를 밀고 있는데 새 입력이 0.1초 넘게 안 와 0으로 처리된 주기. 이때 손을 뗀 것으로 보고 멈춘다.
- 마우스 값이 바뀌는 간격: 마우스 보고 주기의 근사값. command_minimum_time_s는 이보다 길게 둔다.
"""

import argparse
import csv
from pathlib import Path

import numpy as np

RAW_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")
PUSH_THRESHOLD = 0.1  # 이보다 크게 밀린 주기를 "밀고 있음"으로 본다
LATE_MS = 20.0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", nargs="?", type=Path, help="실행 기록 CSV (기본: logs/의 가장 최근 파일)")
    args = parser.parse_args()

    path = args.path or max(Path(__file__).resolve().parents[1].joinpath("logs").glob("teleop_*.csv"))
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    t = np.array([float(row["time_s"]) for row in rows])
    interval = np.diff(t) * 1000
    print(f"{path} ({len(rows)}주기)")
    print(
        f"제어 주기 간격 [ms]: 중앙값 {np.median(interval):.1f}, 95% {np.percentile(interval, 95):.1f}, "
        f"최대 {interval.max():.1f}, {LATE_MS:.0f} ms 넘은 주기 {(interval > LATE_MS).sum()}"
    )

    for side in ("right", "left"):
        raw = np.array([[float(row[f"{side}_raw_{key}"]) for key in RAW_KEYS] for row in rows])
        fresh = np.array([int(row[f"{side}_raw_fresh"]) for row in rows])
        pushing = np.abs(raw).max(axis=1) > PUSH_THRESHOLD
        stale = pushing & (fresh == 0)
        print(
            f"{side}: 밀고 있는 주기 {pushing.sum()}, 그중 입력 시간 제한으로 0 처리 {stale.sum()} "
            f"({100 * stale.sum() / max(1, pushing.sum()):.1f}%)"
        )

        changed = np.flatnonzero(np.any(np.diff(raw, axis=0) != 0, axis=1) & pushing[1:])

        if len(changed) > 2:
            gaps = np.diff(t[1:][changed]) * 1000
            gaps = gaps[gaps < 1000]  # 손을 뗐다 다시 민 간격은 뺀다
            print(
                f"  밀고 있을 때 마우스 값이 바뀌는 간격 [ms]: 중앙값 {np.median(gaps):.0f}, "
                f"95% {np.percentile(gaps, 95):.0f}, 최대 {gaps.max():.0f}"
            )


if __name__ == "__main__":
    main()
