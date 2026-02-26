#!/usr/bin/env python3
import argparse
import getpass
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Iterable, List, Set

DEFAULT_TOPIC_ROOT = os.environ.get("TOPIC_ROOT", "voda/septik")
DEFAULT_HOST = os.environ.get("MQTT_HOST", "mqtt.home.arpa")
DEFAULT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
DEFAULT_USER = os.environ.get("MQTT_USER", "ha")
DEFAULT_QOS = int(os.environ.get("MQTT_QOS", "1"))
DEFAULT_PASS_FILE = os.environ.get("MQTT_PASS_FILE", str(Path.home() / ".zalevaci-nadrz" / "mqtt_password"))
DEFAULT_SCAN_ROOTS = ["main", "components", "managed_components"]

TAG_PATTERNS = [
    re.compile(r"static\s+const\s+char\s*\*\s*TAG\s*=\s*\"([^\"]+)\""),
    re.compile(r"#define\s+TAG\s+\"([^\"]+)\""),
    re.compile(r"ESP_(?:EARLY_|DRAM_)?LOG[EWIDV]\s*\(\s*\"([^\"]+)\""),
]

VALID_LEVELS = {
    "NONE": "NONE",
    "0": "NONE",
    "ERROR": "ERROR",
    "ERR": "ERROR",
    "1": "ERROR",
    "WARN": "WARN",
    "WARNING": "WARN",
    "2": "WARN",
    "INFO": "INFO",
    "3": "INFO",
    "DEBUG": "DEBUG",
    "4": "DEBUG",
    "VERBOSE": "VERBOSE",
    "TRACE": "VERBOSE",
    "5": "VERBOSE",
}

SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp"}


def normalize_level(level: str) -> str:
    key = level.strip().upper()
    if key not in VALID_LEVELS:
        allowed = "NONE, ERROR, WARN, INFO, DEBUG, VERBOSE (nebo 0..5)"
        raise ValueError(f"Neplatna log uroven '{level}'. Povolené: {allowed}")
    return VALID_LEVELS[key]


def iter_source_files(roots: Iterable[Path]) -> Iterable[Path]:
    for root in roots:
        if not root.exists() or not root.is_dir():
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.suffix.lower() in SOURCE_SUFFIXES:
                yield path


def discover_tags(roots: Iterable[Path]) -> List[str]:
    tags: Set[str] = set()

    for source in iter_source_files(roots):
        try:
            content = source.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue

        for pattern in TAG_PATTERNS:
            for match in pattern.finditer(content):
                tag = match.group(1).strip()
                if tag:
                    tags.add(tag)

    return sorted(tags)


def ensure_password(pass_file: Path) -> str:
    pass_file.parent.mkdir(parents=True, exist_ok=True)

    if pass_file.exists() and pass_file.stat().st_size > 0:
        return pass_file.read_text(encoding="utf-8", errors="ignore").strip()

    password = getpass.getpass(f"Zadej MQTT heslo ({pass_file}): ").strip()
    if not password:
        raise RuntimeError("Heslo nesmi byt prazdne.")

    pass_file.write_text(password, encoding="utf-8")
    os.chmod(pass_file, 0o600)
    print(f"Heslo ulozeno do: {pass_file}")
    return password


def publish_log_level(host: str,
                      port: int,
                      user: str,
                      password: str,
                      qos: int,
                      topic_root: str,
                      tag: str,
                      level: str,
                      dry_run: bool) -> None:
    topic = f"{topic_root}/cmd/log/level"
    payload = f"{tag}={level}"

    cmd = [
        "mosquitto_pub",
        "-d",
        "-h", host,
        "-p", str(port),
        "-u", user,
        "-P", password,
        "-q", str(qos),
        "-t", topic,
        "-m", payload,
    ]

    print(f"Publikuji: topic={topic} payload={payload}")
    if dry_run:
        print("Dry-run: publish preskocen")
        return

    subprocess.run(cmd, check=True)


def pick_tag_interactive(tags: List[str]) -> str:
    if not tags:
        raise RuntimeError("Nebyly nalezeny zadne tagy. Zkus --tag '*' nebo uprav --scan-root.")

    print("Nalezene log tagy:")
    for index, tag in enumerate(tags, start=1):
        print(f"  {index:>3}) {tag}")

    while True:
        value = input("Vyber cislo tagu nebo napis presny tag: ").strip()
        if not value:
            continue
        if value.isdigit():
            idx = int(value)
            if 1 <= idx <= len(tags):
                return tags[idx - 1]
            print("Neplatne cislo.")
            continue
        return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Nastavi ESP-IDF log level pres MQTT a umi automaticky najit tagy ve zdrojacich."
    )
    parser.add_argument("--list", action="store_true", help="Jen vypis nalezene tagy a skonci")
    parser.add_argument("--tag", help="Log tag (napr. mqtt_cmd nebo * pro vse)")
    parser.add_argument("--all", action="store_true", help="Zkratka pro --tag '*' ")
    parser.add_argument("--level", help="Uroven: NONE|ERROR|WARN|INFO|DEBUG|VERBOSE nebo 0..5")
    parser.add_argument("--scan-root", action="append", default=[], help="Dalsi root pro scan (lze opakovat)")

    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--user", default=DEFAULT_USER)
    parser.add_argument("--qos", type=int, default=DEFAULT_QOS)
    parser.add_argument("--topic-root", default=DEFAULT_TOPIC_ROOT)
    parser.add_argument("--pass-file", default=DEFAULT_PASS_FILE)
    parser.add_argument("--dry-run", action="store_true", help="Nevola mosquitto_pub")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if shutil_which("mosquitto_pub") is None and not args.list and not args.dry_run:
        print("Chyba: chybi 'mosquitto_pub'.", file=sys.stderr)
        return 1

    project_root = Path.cwd()
    roots = [project_root / root for root in DEFAULT_SCAN_ROOTS]
    roots.extend(project_root / root for root in args.scan_root)
    tags = discover_tags(roots)

    if args.list:
        for tag in tags:
            print(tag)
        return 0

    tag = "*" if args.all else (args.tag.strip() if args.tag else "")
    if not tag:
        tag = pick_tag_interactive(tags)

    level_input = args.level
    if not level_input:
        level_input = input("Uroven [NONE|ERROR|WARN|INFO|DEBUG|VERBOSE]: ").strip()

    try:
        normalized_level = normalize_level(level_input)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    try:
        password = ensure_password(Path(args.pass_file))
        publish_log_level(args.host,
                          args.port,
                          args.user,
                          password,
                          args.qos,
                          args.topic_root,
                          tag,
                          normalized_level,
                          args.dry_run)
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        print(f"Chyba: {exc}", file=sys.stderr)
        return 1

    return 0


def shutil_which(cmd: str):
    for dir_path in os.environ.get("PATH", "").split(os.pathsep):
        candidate = Path(dir_path) / cmd
        if candidate.exists() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


if __name__ == "__main__":
    sys.exit(main())
