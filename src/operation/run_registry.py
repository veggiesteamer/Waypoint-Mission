import json
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple


def start_run(base_dir: str = "experiments/missions") -> Tuple[str, Path]:
    """
    Create a fresh, timestamp-named run folder under base_dir to hold one
    mission run's config/results, and return its id and path.
    """
    run_id = datetime.now().strftime("run_%Y%m%d_%H%M%S")
    run_dir = Path(base_dir) / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_id, run_dir


def save_run(base_dir: str, run_id: str, run_dir: Path, config: dict, results: dict,
             summary: Optional[dict] = None) -> None:
    """
    Persist this run's full config and results under run_dir, and append a
    short pointer entry to base_dir/index.json -- so "what parameters
    produced file X" can be answered by scanning one small index instead of
    every run folder.
    """
    run_dir = Path(run_dir)
    (run_dir / "config.json").write_text(json.dumps(config, indent=2, default=str))
    (run_dir / "results.json").write_text(json.dumps(results, indent=2, default=str))

    index_path = Path(base_dir) / "index.json"
    index = json.loads(index_path.read_text()) if index_path.exists() else []
    index.append({
        "run_id": run_id,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "run_dir": str(run_dir),
        "summary": summary or {},
    })
    index_path.write_text(json.dumps(index, indent=2))
