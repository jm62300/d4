"""
Bug instance minimizer — ddmin + greedy removal.

CNF: operates on clause lists (ddmin + greedy + literal reduction).
Circuit (BC-S1.2): operates on gate lines (ddmin + greedy + literal reduction).
  Dropped gate names that are still referenced by surviving gates become primary
  inputs automatically — the BC parser promotes any unseen name to a fresh
  variable, so no reference-fixing is needed.

Variable renumbering is intentionally skipped for CNF: renaming variable N in
the weight lines would require rewriting all `c p weight N ...` lines too.
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
# Circuit (BC-S1.2) parse / write
# ---------------------------------------------------------------------------

def _parse_circuit(path: str) -> tuple[list[str], list[str]]:
    """Split a BC-S1.2 file into header lines (c/T/I) and gate lines (G ...)."""
    header: list[str] = []
    gate_lines: list[str] = []
    with open(path) as fh:
        for raw in fh:
            line = raw.rstrip()
            if not line:
                continue
            if line.startswith("G "):
                gate_lines.append(line)
            else:
                header.append(line)
    return header, gate_lines


def _write_circuit(path: str, header: list[str], gate_lines: list[str]) -> None:
    with open(path, "w") as fh:
        fh.write("\n".join(header + gate_lines) + "\n")


# ---------------------------------------------------------------------------
# Circuit gate-arity reduction
# For each gate line "G name := TYPE lit1 lit2 ... litn",
# try removing one input literal at a time (keeping >= 2, or >= 1 for I gates).
# ---------------------------------------------------------------------------

def _reduce_gate_arity(
    gate_lines: list[str],
    still_bugs,           # callable(list[str]) -> bool
) -> list[str]:
    current = list(gate_lines)
    changed = True
    while changed:
        changed = False
        for gi in range(len(current)):
            parts = current[gi].split()
            # parts: G <name> := <TYPE> <lit1> ...
            if len(parts) < 5:
                continue
            gtype = parts[3]
            min_inputs = 1 if gtype == "I" else 2
            lits = parts[4:]
            if len(lits) <= min_inputs:
                continue
            for li in range(len(lits)):
                reduced_lits = lits[:li] + lits[li + 1:]
                new_line = " ".join(parts[:4] + reduced_lits)
                candidate = current[:gi] + [new_line] + current[gi + 1:]
                if still_bugs(candidate):
                    current = candidate
                    parts = new_line.split()
                    lits = parts[4:]
                    changed = True
                    if len(lits) <= min_inputs:
                        break
    return current


# ---------------------------------------------------------------------------
# Circuit minimizer: ddmin + greedy on gate lines, then arity reduction
# ---------------------------------------------------------------------------

def _minimize_circuit(
    instance_path: str,
    suite,
    evaluated,
    cwd: Optional[str],
) -> str:
    from oracles import run_oracle

    oracle_types = ("correctness", "circuit_cnf_crosscheck")
    if suite.oracle.type not in oracle_types:
        return instance_path

    def is_bug(path: str) -> bool:
        r = run_oracle(path, None, suite, evaluated, scripts_dir="", cwd=cwd)
        return not r.passed and not r.no_answer and not r.timed_out

    try:
        header, gate_lines = _parse_circuit(instance_path)
    except Exception:
        return instance_path

    tmp_fd, tmp_path = tempfile.mkstemp(suffix=".bc", prefix="fuzz_min_")
    os.close(tmp_fd)

    if not is_bug(instance_path):
        os.unlink(tmp_path)
        return instance_path

    def still_bugs(gates: list[str]) -> bool:
        _write_circuit(tmp_path, header, gates)
        return is_bug(tmp_path)

    gate_lines = _ddmin(gate_lines, still_bugs)
    gate_lines = _greedy(gate_lines, still_bugs)
    gate_lines = _reduce_gate_arity(gate_lines, still_bugs)

    _write_circuit(tmp_path, header, gate_lines)
    return tmp_path


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def minimize_instance(
    instance_path: str,
    suite,      # SuiteConfig
    evaluated,  # EvaluatedConfig
    cwd: Optional[str] = None,
) -> str:
    """Minimize a failing instance via ddmin + greedy removal.

    Returns the path to a new temp file with the minimized instance.
    The caller is responsible for deleting it.
    Returns the original path unchanged if minimization does not apply
    (crash suites, or if the bug cannot be reproduced).
    """
    if instance_path.endswith(".bc"):
        return _minimize_circuit(instance_path, suite, evaluated, cwd)

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
