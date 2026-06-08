#!/usr/bin/env python3
"""
d4 fuzzer — entry point.

Usage:
  python run.py                         # TUI with default.json
  python run.py myconfig.json           # TUI with custom config
  python run.py --no-ui myconfig.json   # headless (stdout only)
  python run.py --list myconfig.json    # list suites and exit
"""

from __future__ import annotations
import argparse
import os
import sys
import time
import signal
import threading
from pathlib import Path

# Make imports work when run from any cwd.
sys.path.insert(0, str(Path(__file__).parent))

from schema import Config
from runner import SuiteRunner, Status


def build_runners(config: Config, repo_root: str,
                  config_path: str = "") -> list[SuiteRunner]:
    runners = []
    for s in config.suites:
        if not s.enabled:
            continue
        for ev in s.evaluated:
            runners.append(SuiteRunner(
                suite=s, evaluated=ev,
                scripts_dir=config.globals.scripts_dir,
                fuzz_dir=config.globals.fuzz_dir,
                save_dir=config.globals.save_dir,
                config_path=config_path,
                cwd=repo_root,
            ))
    return runners


def run_headless(runners: list[SuiteRunner], workers: int) -> None:
    stop = threading.Event()

    def _sigint(_s, _f):
        print("\nStopping…", flush=True)
        stop.set()

    signal.signal(signal.SIGINT, _sigint)

    # Start up to `workers` runners; remaining wait until a slot frees.
    active: list[SuiteRunner] = []
    for r in runners[:workers]:
        r.start()
        r.resume()
        active.append(r)
        print(f"[START] {r.suite.name}", flush=True)

    reported_bugs: set[tuple[str, int]] = set()

    try:
        while not stop.is_set():
            time.sleep(1)
            parts = []
            all_done = True
            for r in active:
                snap = r.state.snapshot()
                parts.append(
                    f"{r.suite.name}: tested={snap['tested']} "
                    f"bugs={snap['bug_count']} [{snap['status'].name.lower()}]"
                )
                if snap["status"] in (Status.RUNNING, Status.PAUSED):
                    all_done = False
                with r.state.lock:
                    for bug in r.state.bugs:
                        key = (r.suite.name, bug.index)
                        if key not in reported_bugs:
                            reported_bugs.add(key)
                            print(
                                f"[BUG] {r.suite.name} #{bug.index} "
                                f"— {bug.reason}  →  {bug.instance_path}",
                                flush=True,
                            )
            print("  |  ".join(parts), flush=True)
            if all_done:
                break
    finally:
        for r in active:
            r.stop()


def run_tui(runners: list[SuiteRunner]) -> None:
    from app import FuzzApp
    FuzzApp(runners).run()  # on_mount starts runners


def main() -> None:
    parser = argparse.ArgumentParser(description="d4 fuzzer")
    parser.add_argument("config", nargs="?", help="JSON config file (default: configs/default.json)")
    parser.add_argument("--no-ui", action="store_true", help="Headless mode (no TUI)")
    parser.add_argument("--list", action="store_true", help="List suites and exit")
    parser.add_argument("--suite", help="Run only the named suite")
    args = parser.parse_args()

    if args.config:
        config = Config.load(args.config)
    else:
        config = Config.load_default()

    # Resolve relative paths against the repo root (two levels up from tools/fuzz/).
    repo_root = Path(__file__).parent.parent.parent
    if not Path(config.globals.scripts_dir).is_absolute():
        config.globals.scripts_dir = str(repo_root / config.globals.scripts_dir)
    if not Path(config.globals.fuzz_dir).is_absolute():
        config.globals.fuzz_dir = str(repo_root / config.globals.fuzz_dir)

    # Absolute path for hot-reload.
    config_path = str(Path(args.config).resolve()) if args.config else \
        str(Path(__file__).parent / "configs" / "default.json")

    if args.list:
        for s in config.suites:
            marker = "✓" if s.enabled else "✗"
            print(f"  {marker} {s.name:30s} {s.description}")
        return

    runners = build_runners(config, str(repo_root), config_path)
    if args.suite:
        runners = [r for r in runners if r.suite.name == args.suite]
        if not runners:
            print(f"No suite named {args.suite!r}")
            sys.exit(1)

    if not runners:
        print("No enabled suites found.")
        sys.exit(1)

    if args.no_ui:
        run_headless(runners, config.globals.workers)
    else:
        run_tui(runners)


if __name__ == "__main__":
    main()
