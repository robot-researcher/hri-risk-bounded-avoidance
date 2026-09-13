"""Check archived paired timing and guard evidence without robot hardware."""
import json
import math
from pathlib import Path
from statistics import median

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    summary = json.loads((ROOT / 'results/paired_jetson_paper_geometry.json').read_text())
    rows = [json.loads(line) for line in (ROOT / 'results/paired_jetson_paper_geometry.jsonl').read_text().splitlines() if line.strip()]
    require(len(rows) == summary['paired_cases'] == 105, 'Paired case count differs from the reported experiment')
    mismatches = 0
    for row in rows:
        a, b = row['decisions']['lookup'], row['decisions']['online_recompute']
        risk_equal = a[1] is None and b[1] is None
        if a[1] is not None and b[1] is not None:
            risk_equal = math.isclose(a[1], b[1], rel_tol=0, abs_tol=1e-12)
        equal = a[0] == b[0] and risk_equal
        require(equal == row['equal'], 'Recorded agreement flag disagrees with decisions')
        mismatches += not equal
    require(mismatches == summary['decision_mismatches'] == 0, 'Paired decision mismatch')
    for method in ('lookup', 'online_recompute'):
        values = [row['latency_us'][method] for row in rows]
        require(all(math.isfinite(v) and v >= 0 for v in values), 'Invalid timing')
        actual = median(values)
        require(math.isclose(actual, summary['timings'][method]['median_us'], rel_tol=1e-9), 'Median differs from summary')
        print(f'{method}: median {actual / 1000:.4f} ms ({len(values)} cases)')
    replay = json.loads((ROOT / 'results/measured_uncertainty_jetson_paper_geometry.json').read_text())
    for case in ('stale_hold', 'wrong_start_hold'):
        records = [r for value in replay.values() if isinstance(value, list) for r in value if isinstance(r, dict) and r.get('case') == case]
        require(len(records) == 360 and all(r['passed'] for r in records), f'{case}: expected 360 passing records')
        require(sum(s['guards'][case] for s in replay['summaries']) == len(records), 'Guard summary differs from records')
        print(f'{case}: {len(records)}/{len(records)} passed')
    print('PASS: archived paired decisions, median timings and replay counts agree.')
    print('This checks recorded evidence; it does not rerun hardware or establish physical safety.')


if __name__ == '__main__':
    main()
