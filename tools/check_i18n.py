#!/usr/bin/env python3
"""i18n check (softwarespezifikation.md 13.4): every key of web_ui/i18n/en.json exists in de.json and it.json,
values are non-empty strings and every {placeholder} of the English text also appears in the translation.
Usage: check_i18n.py [I18N_DIR]. The check of all error.E###.* keys comes when the error texts are added."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

PLACEHOLDER = re.compile(r"\{(\w+)\}")


def check(i18n_dir: Path) -> list[str]:
    files = {lang: json.loads((i18n_dir / f"{lang}.json").read_text(encoding="utf-8")) for lang in ("en", "de", "it")}
    errors: list[str] = []
    for lang, data in files.items():
        for key, value in data.items():
            if not isinstance(value, str) or not value.strip():
                errors.append(f"{lang}: {key} is empty or not a string")
    for lang in ("de", "it"):
        for key, en_text in files["en"].items():
            if key not in files[lang]:
                errors.append(f"{lang}: missing key {key}")
                continue
            missing = set(PLACEHOLDER.findall(en_text)) - set(PLACEHOLDER.findall(files[lang][key]))
            if missing:
                errors.append(f"{lang}: {key} lacks placeholder(s) {sorted(missing)}")
        for key in files[lang].keys() - files["en"].keys():
            errors.append(f"{lang}: key {key} is not in en.json")
    return errors


def main() -> int:
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / "web_ui" / "i18n"
    errors = check(root)
    for e in errors:
        print("FAIL  " + e)
    print("i18n: OK" if not errors else f"i18n: {len(errors)} error(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
