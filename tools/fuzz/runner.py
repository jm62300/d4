"""
SuiteRunner — runs one (suite, evaluated-tool) pair in a background thread.

One runner is created per evaluated tool per suite, so multiple tools can be
compared against the same oracle concurrently.

Design notes:
  - All runners start PAUSED; press 'p' in the TUI (or call resume()) to begin.
  - Config hot-reload: re-reads the JSON at the start of every iteration so
    edits to max_vars, commands, patterns, or timeout take effect immediately.
  - Bug files are saved BEFORE temp files are deleted.
"""

from __future__ import annotations
import os
import shutil
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import Optional

from generators import generate
from minimize import minimize_instance
from oracles import run_oracle
from schema import EvaluatedConfig, SuiteConfig


class Status(Enum):
    WAITING  = auto()
    RUNNING  = auto()
    PAUSED   = auto()
    STOPPED  = auto()
    MAX_BUGS = auto()
    ERROR    = auto()


@dataclass
class Bug:
    index: int
    instance_path: str
    queries_path: Optional[str]
    reason: str
    timestamp: float = field(default_factory=time.time)


@dataclass
class SuiteState:
    """Shared mutable state — always access under lock."""
    lock: threading.Lock = field(default_factory=threading.Lock)
    status: Status = Status.WAITING
    tested: int = 0
    timeouts: int = 0
    no_answer: int = 0
    bugs: list[Bug] = field(default_factory=list)
    last_error: str = ""

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "status":     self.status,
                "tested":     self.tested,
                "timeouts":   self.timeouts,
                "no_answer":  self.no_answer,
                "bug_count":  len(self.bugs),
                "last_bug":   self.bugs[-1].reason if self.bugs else "",
                "last_error": self.last_error,
            }


class SuiteRunner:
    def __init__(self, suite: SuiteConfig, evaluated: EvaluatedConfig,
                 scripts_dir: str, fuzz_dir: str, save_dir: str,
                 config_path: Optional[str] = None, cwd: Optional[str] = None):
        self.suite          = suite
        self.evaluated      = evaluated
        self._suite_name    = suite.name
        self._evaluated_name = evaluated.name
        self.scripts_dir    = scripts_dir
        self.fuzz_dir       = fuzz_dir
        # save bugs under save_dir/suite_name/evaluated_name/
        self.save_dir       = Path(save_dir) / suite.name / evaluated.name
        self.cwd            = cwd
        self._config_path   = config_path
        self.state          = SuiteState()
        self._thread: Optional[threading.Thread] = None
        self._stop_event    = threading.Event()
        self._pause_event   = threading.Event()
        # Starts paused — call resume() to begin.

    @property
    def display_name(self) -> str:
        return f"{self._suite_name} / {self._evaluated_name}"

    def start(self) -> None:
        self.save_dir.mkdir(parents=True, exist_ok=True)
        with self.state.lock:
            self.state.status = Status.PAUSED
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def pause(self) -> None:
        self._pause_event.clear()
        with self.state.lock:
            if self.state.status == Status.RUNNING:
                self.state.status = Status.PAUSED

    def resume(self) -> None:
        self._pause_event.set()
        with self.state.lock:
            if self.state.status == Status.PAUSED:
                self.state.status = Status.RUNNING

    def stop(self) -> None:
        self._stop_event.set()
        self._pause_event.set()

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------
    # Config hot-reload
    # ------------------------------------------------------------------

    def _reload_suite(self) -> None:
        if not self._config_path:
            return
        try:
            from schema import Config
            config = Config.load(self._config_path)
            for s in config.suites:
                if s.name != self._suite_name:
                    continue
                self.suite = s
                for ev in s.evaluated:
                    if ev.name == self._evaluated_name:
                        self.evaluated = ev
                        return
        except Exception:
            pass

    # ------------------------------------------------------------------

    def _loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                self._pause_event.wait()
                if self._stop_event.is_set():
                    break

                self._reload_suite()

                with self.state.lock:
                    bug_count = len(self.state.bugs)
                if bug_count >= self.suite.max_bugs:
                    with self.state.lock:
                        self.state.status = Status.MAX_BUGS
                    break

                instance_path, queries_path = None, None
                result = None
                try:
                    instance_path, queries_path = generate(
                        self.suite.generator, self.scripts_dir, self.fuzz_dir
                    )
                    result = run_oracle(
                        instance_path, queries_path,
                        self.suite, self.evaluated,
                        self.scripts_dir, cwd=self.cwd,
                    )
                except Exception as exc:
                    with self.state.lock:
                        self.state.last_error = str(exc)
                    _delete_temps(instance_path, queries_path,
                                  self.suite.generator.type)
                    continue

                # Save original then minimized bug files BEFORE deleting temp files.
                saved_instance = instance_path
                saved_queries  = queries_path
                if result is not None and not result.passed:
                    # Original (un-minimized) copy.
                    saved_instance, saved_queries = self._copy_bug(
                        instance_path, queries_path
                    )
                    # Minimized copy — written alongside as bug_NNNN_minimized.cnf.
                    min_path = minimize_instance(
                        instance_path, self.suite, self.evaluated, cwd=self.cwd
                    )
                    if min_path != instance_path:
                        self._copy_bug_minimized(min_path, saved_instance)
                        try:
                            os.unlink(min_path)
                        except OSError:
                            pass

                _delete_temps(instance_path, queries_path,
                              self.suite.generator.type)

                if result is None:
                    continue

                with self.state.lock:
                    if result.timed_out:
                        self.state.timeouts += 1
                    else:
                        self.state.tested += 1
                    if result.no_answer:
                        self.state.no_answer += 1
                    if not result.passed:
                        idx = len(self.state.bugs) + 1
                        self.state.bugs.append(Bug(
                            index=idx,
                            instance_path=saved_instance or "?",
                            queries_path=saved_queries,
                            reason=result.reason,
                        ))

        except Exception as exc:
            with self.state.lock:
                self.state.status = Status.ERROR
                self.state.last_error = str(exc)
            return

        with self.state.lock:
            if self.state.status == Status.RUNNING:
                self.state.status = Status.STOPPED

    def _copy_bug_minimized(self, min_path: str, original_saved: str) -> None:
        """Save the minimized instance alongside the original as *_minimized.ext."""
        p = Path(original_saved)
        dest = str(p.parent / (p.stem + "_minimized" + p.suffix))
        try:
            shutil.copy(min_path, dest)
        except Exception:
            pass

    def _copy_bug(self, instance_path: Optional[str],
                  queries_path: Optional[str]) -> tuple[str, Optional[str]]:
        with self.state.lock:
            idx = len(self.state.bugs) + 1
        ext   = Path(instance_path or "").suffix or ".cnf"
        saved = str(self.save_dir / f"bug_{idx:04d}{ext}")
        try:
            shutil.copy(instance_path or "", saved)
        except Exception:
            saved = instance_path or "?"
        saved_q = None
        if queries_path:
            saved_q = str(self.save_dir / f"bug_{idx:04d}.queries")
            try:
                shutil.copy(queries_path, saved_q)
            except Exception:
                saved_q = queries_path
        return saved, saved_q


def _delete_temps(instance_path: Optional[str], queries_path: Optional[str],
                  gen_type: str) -> None:
    if instance_path and gen_type != "circuit_files":
        try:
            os.unlink(instance_path)
        except OSError:
            pass
    if queries_path:
        try:
            os.unlink(queries_path)
        except OSError:
            pass
