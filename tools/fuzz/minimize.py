"""
Bug instance minimizer — ddmin + greedy clause removal.

Integrated into the fuzzer: uses run_oracle as the oracle (no subprocess),
preserves weight/type annotation lines from the original formula so that
weighted and complex counting bugs remain reproducible after minimization.

Variable renumbering is intentionally skipped: renaming variable N in the
weight lines would require rewriting all `c p weight N ...` lines too.
"""

from __future__ import annotations
import os
import tempfile
from typing import Optional


# ---------------------------------------------------------------------------
# Parse / write — weight-aware
# ---------------------------------------------------------------------------

def _parse(path: str) -> tuple[list[str], list[list[int]], list[int]]:
    """Return (preamble_lines, clauses, show_vars).

    preamble_lines: all 'c ...' comment lines except 'c p show'
    clauses:        list of int lists (no trailing 0)
    show_vars:      ints from 'c p show' line (empty if absent)
    """
    preamble: list[str] = []
    show_vars: list[int] = []
    clauses: list[list[int]] = []
    pending: list[int] = []

    with open(path) as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("p "):
                continue
            if line.startswith("c p show"):
                tokens = line.split()
                show_vars = [int(t) for t in tokens[3:] if t != "0"]
            elif line.startswith("c"):
                preamble.append(line)
            else:
                for tok in line.split():
                    n = int(tok)
                    if n == 0:
                        if pending:
                            clauses.append(pending)
                            pending = []
                    else:
                        pending.append(n)

    if pending:
        clauses.append(pending)
    return preamble, clauses, show_vars


def _write(path: str, preamble: list[str],
           clauses: list[list[int]], show_vars: list[int]) -> None:
    all_vars = {abs(l) for c in clauses for l in c}
    all_vars.update(show_vars)
    n_vars = max(all_vars) if all_vars else 0
    with open(path, "w") as fh:
        fh.write(f"p cnf {n_vars} {len(clauses)}\n")
        for line in preamble:
            fh.write(line + "\n")
        if show_vars:
            fh.write("c p show " + " ".join(map(str, show_vars)) + " 0\n")
        for clause in clauses:
            fh.write(" ".join(map(str, clause)) + " 0\n")


# ---------------------------------------------------------------------------
# ddmin + greedy (operate on clauses; preamble and show_vars are fixed)
# ---------------------------------------------------------------------------

def _ddmin(clauses: list[list[int]], still_bugs) -> list[list[int]]:
    current = list(clauses)
    n = 2
    while len(current) >= 2:
        chunk_size = max(1, len(current) // n)
        chunks = [current[i:i + chunk_size]
                  for i in range(0, len(current), chunk_size)]
        n = len(chunks)
        reduced = False

        for i in range(n):
            complement = [c for j, cs in enumerate(chunks) for c in cs if j != i]
            if still_bugs(complement):
                current = complement
                n = max(n - 1, 2)
                reduced = True
                break

        if reduced:
            continue

        for chunk in chunks:
            if still_bugs(chunk):
                current = chunk
                n = 2
                reduced = True
                break

        if reduced:
            continue

        if n >= len(current):
            break
        n = min(n * 2, len(current))

    return current


def _greedy(clauses: list[list[int]], still_bugs) -> list[list[int]]:
    current = list(clauses)
    i = 0
    while i < len(current):
        candidate = current[:i] + current[i + 1:]
        if still_bugs(candidate):
            current = candidate
        else:
            i += 1
    return current


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def minimize_instance(
    instance_path: str,
    suite,      # SuiteConfig
    evaluated,  # EvaluatedConfig
    cwd: Optional[str] = None,
) -> str:
    """Minimize a failing CNF instance via ddmin + greedy clause removal.

    Returns the path to a new temp file with the minimized instance.
    The caller is responsible for deleting it.
    Returns the original path unchanged if minimization does not apply
    (crash suites, non-CNF files, or if the bug cannot be reproduced).
    """
    if suite.oracle.type != "correctness":
        return instance_path
    if not instance_path.endswith(".cnf"):
        return instance_path

    from oracles import run_oracle

    def is_bug(path: str) -> bool:
        r = run_oracle(path, None, suite, evaluated, scripts_dir="", cwd=cwd)
        return not r.passed and not r.no_answer and not r.timed_out

    try:
        preamble, clauses, show_vars = _parse(instance_path)
    except Exception:
        return instance_path

    tmp_fd, tmp_path = tempfile.mkstemp(suffix=".cnf", prefix="fuzz_min_")
    os.close(tmp_fd)

    # Verify the bug is still reproducible before investing in minimization.
    if not is_bug(instance_path):
        os.unlink(tmp_path)
        return instance_path

    def still_bugs(cls: list[list[int]]) -> bool:
        _write(tmp_path, preamble, cls, show_vars)
        return is_bug(tmp_path)

    clauses = _ddmin(clauses, still_bugs)
    clauses = _greedy(clauses, still_bugs)

    _write(tmp_path, preamble, clauses, show_vars)
    return tmp_path
