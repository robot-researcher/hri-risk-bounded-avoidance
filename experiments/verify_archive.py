"""Validate the imported archive's integrity and key statistical totals."""
from pathlib import Path
import hashlib
import json
import math
from risk_bounded_collision.validation import wilson_upper_bound

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(relative):
    return json.loads((ROOT / relative).read_text(encoding='utf-8-sig'))


def main():
    ledger = read('results/archive/PROVENANCE.json')
    for entry in ledger['files']:
        payload = (ROOT / entry['path']).read_bytes()
        require(hashlib.sha256(payload).hexdigest() == entry['sha256'], 'Archive checksum mismatch: ' + entry['path'])
    prefix = 'results/archive/artifacts/'
    total = sum(read(prefix + name)['total_trials'] for name in ('calibration_results.json', 'heldout_validation_results.json', 'risk_level_validation_results.json', 'stress_validation_results.json'))
    require(total == 1690000, 'Historical trial total changed')
    stress = read(prefix + 'stress_validation_results.json')
    require(sum(r['trials'] for r in stress['results']) == 600000, 'Stress denominator differs')
    covered = 0
    for row in stress['results']:
        upper = wilson_upper_bound(row['collisions'], row['trials'])
        require(math.isclose(upper, row['wilson_upper_95'], rel_tol=1e-9), 'Wilson upper limit differs: ' + row['scenario'])
        require(upper < row['configured_delta'], 'Stress threshold exceeded')
        covered += row['guarded_risk'] >= upper
    require(len(stress['results']) == 24 and covered == 23, 'Stress surrogate coverage differs')
    swept = read(prefix + 'swept_validation_results.json')
    require(swept['scene_count'] == 2500, 'Swept scene count differs')
    require(swept['summary']['hidden_collisions_detected'] == swept['summary']['hidden_between_sample_collisions'] == 658, 'Swept detection summary differs')
    print(f'PASS: {len(ledger["files"])} archive hashes; {total:,} historical trials; 24 stress thresholds, 23/24 surrogate coverage; 658/658 swept detections.')
    print('Checks validate archived consistency, not a fresh full simulation or physical safety.')


if __name__ == '__main__':
    main()
