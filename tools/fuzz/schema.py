"""
Configuration schema for the d4 fuzzing tool.
Load with Config.load(path) or Config.load_default().
"""

from __future__ import annotations
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class GeneratorConfig:
    type: str                          # cnf_plain | cnf_projected | cnf_weighted |
                                       # cnf_complex | cnf_maxsharpsat | cnf_queries |
                                       # circuit_gen | circuit_files
    max_vars: int = 100
    n_queries: int = 20
    files: list[str] = field(default_factory=list)   # for circuit_files type
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class OracleConfig:
    type: str                          # correctness | crash
    reference: str = ""               # ground-truth command (e.g. d4_static)
    result_pattern: str = r"^s (.+)"  # regex to extract the answer from the oracle output
    tolerance: float = 0.0            # >0 → approximate comparison (for float/complex results)


@dataclass
class EvaluatedConfig:
    name: str                          # display name (e.g. "counter", "new_method")
    command: str                       # binary + flags, input path is appended
    result_pattern: str = ""           # regex for this tool's output; falls back to oracle.result_pattern
    timeout: float = 10.0


@dataclass
class SuiteConfig:
    name: str
    generator: GeneratorConfig
    oracle: OracleConfig
    evaluated: list[EvaluatedConfig]   # one or more tools to compare against the oracle
    max_bugs: int = 10
    enabled: bool = True
    description: str = ""


@dataclass
class GlobalConfig:
    scripts_dir: str = "scripts"
    fuzz_dir: str = "tools/fuzz"       # directory that holds d4_static, minisat_static, cnfuzz
    save_dir: str = "/tmp/fuzz_bugs"
    workers: int = 1                   # parallel suites


@dataclass
class Config:
    globals: GlobalConfig
    suites: list[SuiteConfig]

    @staticmethod
    def load(path: str | Path) -> "Config":
        with open(path) as f:
            raw = json.load(f)
        g = raw.get("global", {})
        suites = []
        for s in raw.get("suites", []):
            gen_raw = s["generator"]
            gen = GeneratorConfig(
                type=gen_raw["type"],
                max_vars=gen_raw.get("max_vars", 100),
                n_queries=gen_raw.get("n_queries", 20),
                files=gen_raw.get("files", []),
                extra=gen_raw.get("extra", {}),
            )
            ora_raw = s["oracle"]
            ora = OracleConfig(
                type=ora_raw["type"],
                reference=ora_raw.get("reference", ""),
                result_pattern=ora_raw.get("result_pattern", r"^s (.+)"),
                tolerance=ora_raw.get("tolerance", 0.0),
            )
            evaluated = [
                EvaluatedConfig(
                    name=ev["name"],
                    command=ev["command"],
                    result_pattern=ev.get("result_pattern", ""),
                    timeout=ev.get("timeout", 10.0),
                )
                for ev in s.get("evaluated", [])
            ]
            suites.append(SuiteConfig(
                name=s["name"],
                generator=gen,
                oracle=ora,
                evaluated=evaluated,
                max_bugs=s.get("max_bugs", 10),
                enabled=s.get("enabled", True),
                description=s.get("description", ""),
            ))
        return Config(
            globals=GlobalConfig(
                scripts_dir=g.get("scripts_dir", "scripts"),
                fuzz_dir=g.get("fuzz_dir", "tools/fuzz"),
                save_dir=g.get("save_dir", "/tmp/fuzz_bugs"),
                workers=g.get("workers", 1),
            ),
            suites=suites,
        )

    @staticmethod
    def load_default() -> "Config":
        here = Path(__file__).parent
        return Config.load(here / "configs" / "default.json")
