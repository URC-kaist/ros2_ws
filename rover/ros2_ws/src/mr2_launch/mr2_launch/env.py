import os
from pathlib import Path


def _iter_candidate_dirs():
    seen = set()
    for start in (Path.cwd(), Path(__file__).resolve()):
        for path in (start if start.is_dir() else start.parent, *start.parents):
            if path in seen:
                continue
            seen.add(path)
            yield path


def _load_env_file(path):
    for line in path.read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if not text or text.startswith("#") or "=" not in text:
            continue
        name, value = text.split("=", 1)
        name = name.strip()
        value = value.strip().strip('"').strip("'")
        if name and name not in os.environ:
            os.environ[name] = value


def load_mr2_env(required=()):
    for directory in _iter_candidate_dirs():
        env_path = directory / ".env"
        if env_path.is_file():
            _load_env_file(env_path)
            break

    missing = [name for name in required if not os.environ.get(name)]
    if missing:
        joined = ", ".join(missing)
        raise RuntimeError(
            f"Missing required MR2 environment variable(s): {joined}. "
            "Define them in the shell or in the repository root .env file."
        )
