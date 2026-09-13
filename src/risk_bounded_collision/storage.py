"""Portable JSON persistence for certificate libraries."""

import json
from pathlib import Path
from typing import Union

from .models import CertificateLibrary, MotionPrimitive


def save_library(library: CertificateLibrary, path: Union[Path, str]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "distance_bins_m": library.distance_bins_m,
        "primitives": [
            {
                "primitive_id": primitive.primitive_id,
                "duration_s": primitive.duration_s,
                "positions": primitive.positions,
            }
            for primitive in library.primitives
        ],
        "risks": library.risks,
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_library(path: Union[Path, str]) -> CertificateLibrary:
    path = Path(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported certificate-library schema")
    primitives = tuple(
        MotionPrimitive(
            item["primitive_id"],
            float(item["duration_s"]),
            tuple(tuple(float(value) for value in position) for position in item["positions"]),
        )
        for item in payload["primitives"]
    )
    return CertificateLibrary(
        tuple(float(value) for value in payload["distance_bins_m"]),
        primitives,
        tuple(tuple(float(value) for value in row) for row in payload["risks"]),
    )
