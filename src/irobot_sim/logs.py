"""실행 로그: 사람용 텍스트(.log) + 구조화 이벤트(.jsonl). 실행마다 타임스탬프 파일."""

import json
import sys
import time
from datetime import datetime

import numpy as np


def _jsonable(v):
    if isinstance(v, np.ndarray):
        return [_jsonable(x) for x in v.tolist()]
    if isinstance(v, (np.floating, float)):
        return round(float(v), 6)
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, dict):
        return {k: _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    return v


class RunLogger:
    def __init__(self, log_dir, quiet=False):
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = log_dir / f"run_{stamp}"
        n = 1
        while base.with_suffix(".jsonl").exists():
            n += 1
            base = log_dir / f"run_{stamp}_{n}"
        self.text_path = base.with_suffix(".log")
        self.jsonl_path = base.with_suffix(".jsonl")
        self._text = open(self.text_path, "w", encoding="utf-8")
        self._jsonl = open(self.jsonl_path, "w", encoding="utf-8")
        self.quiet = quiet
        self.t0 = time.time()
        self.sim_time = 0.0

    def info(self, msg):
        line = f"[{self.sim_time:9.3f}s] {msg}"
        self._text.write(line + "\n")
        if not self.quiet:
            print(line, flush=True)

    def event(self, kind, **fields):
        rec = {"kind": kind, "t": round(self.sim_time, 4), "wall": round(time.time() - self.t0, 3)}
        rec.update(_jsonable(fields))
        self._jsonl.write(json.dumps(rec, ensure_ascii=False) + "\n")

    def block(self, title, lines):
        """여러 줄 보고(실패 보고·요약). 콘솔은 stderr로도 내보낸다."""
        text = "\n".join([f"===== {title} ====="] + [f"  {ln}" for ln in lines] + ["=" * (len(title) + 12)])
        self._text.write(text + "\n")
        print(text, file=sys.stderr if title.startswith("실패") else sys.stdout, flush=True)

    def flush(self):
        self._text.flush()
        self._jsonl.flush()

    def close(self):
        self.flush()
        self._text.close()
        self._jsonl.close()
