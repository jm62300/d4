"""
Oracle implementations.

An oracle takes (instance_path, queries_path, suite_config, evaluated_config,
scripts_dir) and returns OracleResult.
"""

from __future__ import annotations
import os
import re
import subprocess
import tempfile
from dataclasses import dataclass
from fractions import Fraction
from typing import Optional

from generators import bcs_file_to_cnf


@dataclass
class OracleResult:
    passed: bool
    reason: str = ""          # human description of the failure
    timed_out: bool = False
    no_answer: bool = False   # result_pattern matched nothing in at least one output


def _run(cmd: str, input_path: str, queries_path: Optional[str],
         timeout: float, cwd: Optional[str] = None) -> tuple[str, int, bool]:
    """Run a command string with the input path appended; return (stdout, exit_code, timed_out)."""
    full = cmd.split() + [input_path]
    if queries_path:
        full += [queries_path]
    try:
        r = subprocess.run(full, capture_output=True, text=True, timeout=timeout, cwd=cwd)
        return r.stdout, r.returncode, False
    except subprocess.TimeoutExpired:
        return "", -1, True


def _extract(output: str, pattern: str) -> str:
    for line in output.splitlines():
        m = re.match(pattern, line.strip())
        if m:
            return m.group(1).strip()
    return ""


def _parse_complex(s: str) -> complex:
    """Parse a complex number string like '3.02e-01 + 4.64e-01i' or '0.5'."""
    s = s.strip()
    m = re.match(r'^([+-]?\S+)\s*([+-])\s*(\S+)i$', s)
    if m:
        r = float(m.group(1))
        i = float(m.group(3)) * (1 if m.group(2) == '+' else -1)
        return complex(r, i)
    return complex(float(s), 0)


def _approx_equal(a: str, b: str, tol: float) -> bool:
    try:
        ca, cb = _parse_complex(a), _parse_complex(b)
        # relative tolerance, so tiny weighted counts are still compared
        # meaningfully (with an absolute floor for counts that are zero).
        return abs(ca - cb) <= tol * max(abs(ca), abs(cb), 1e-300)
    except ValueError:
        return a == b


def _brute_force_wmc(path: str, max_brute_vars: int = 16) -> Optional[Fraction]:
    """Exact weighted model count by full enumeration (independent of d4).

    Weights default to 1 per literal. Returns None when the instance has too
    many variables to enumerate. Exact rational arithmetic, so negative and
    zero weights are handled with no rounding.
    """
    n_vars = 0
    clauses: list[list[int]] = []
    weights: dict[int, Fraction] = {}

    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("p cnf"):
                n_vars = int(line.split()[2])
            elif line.startswith("c p weight"):
                toks = line.split()
                weights[int(toks[3])] = Fraction(toks[4])
            elif line.startswith(("c", "p")):
                continue
            else:
                cur: list[int] = []
                for tok in line.split():
                    v = int(tok)
                    if v == 0:
                        if cur:
                            clauses.append(cur)
                            cur = []
                    else:
                        cur.append(v)
                if cur:
                    clauses.append(cur)

    if n_vars > max_brute_vars:
        return None

    masks = []
    for cl in clauses:
        pos = neg = 0
        for l in cl:
            if l > 0:
                pos |= 1 << (l - 1)
            else:
                neg |= 1 << (-l - 1)
        masks.append((pos, neg))

    w_pos = [weights.get(i, Fraction(1)) for i in range(1, n_vars + 1)]
    w_neg = [weights.get(-i, Fraction(1)) for i in range(1, n_vars + 1)]

    total = Fraction(0)
    full = (1 << n_vars) - 1
    for m in range(1 << n_vars):
        inv = full & ~m
        if any(not (m & pos) and not (inv & neg) for pos, neg in masks):
            continue
        prod = Fraction(1)
        for i in range(n_vars):
            prod *= w_pos[i] if (m >> i) & 1 else w_neg[i]
        total += prod
    return total


def run_oracle(
    instance_path: str,
    queries_path: Optional[str],
    suite,       # SuiteConfig
    evaluated,   # EvaluatedConfig
    scripts_dir: str,
    cwd: Optional[str] = None,
) -> OracleResult:
    ora     = suite.oracle
    timeout = evaluated.timeout
    tst_pat = evaluated.result_pattern

    if ora.type == "crash":
        _, code, timed_out = _run(evaluated.command, instance_path, queries_path,
                                  timeout, cwd)
        if timed_out:
            return OracleResult(passed=True, timed_out=True)
        if code != 0:
            return OracleResult(passed=False, reason=f"non-zero exit {code}")
        return OracleResult(passed=True)

    if ora.type == "correctness":
        # Use the oracle's own timeout for the reference run.
        ref_out, ref_code, ref_to = _run(ora.reference, instance_path, queries_path,
                                         timeout, cwd)
        if ref_to or ref_code not in (0, 10, 20):
            return OracleResult(passed=True, timed_out=ref_to,
                                reason="reference inconclusive")

        tst_out, tst_code, tst_to = _run(evaluated.command, instance_path, queries_path,
                                          timeout, cwd)
        if tst_to:
            return OracleResult(passed=True, timed_out=True)
        if tst_code != 0:
            return OracleResult(passed=False, reason=f"crash (exit {tst_code})")

        ref_ans = _extract(ref_out, ora.result_pattern)
        tst_ans = _extract(tst_out, tst_pat)

        no_answer = ref_ans == "" or tst_ans == ""
        if no_answer:
            missing = []
            if ref_ans == "":
                missing.append(f"oracle (pattern={ora.result_pattern!r})")
            if tst_ans == "":
                missing.append(f"evaluated (pattern={tst_pat!r})")
            return OracleResult(passed=True, no_answer=True,
                                reason=f"no answer line in {', '.join(missing)}")

        tol = ora.tolerance
        match = _approx_equal(ref_ans, tst_ans, tol) if tol > 0 else (ref_ans == tst_ans)
        if not match:
            return OracleResult(
                passed=False,
                reason=f"mismatch: oracle={ref_ans!r} evaluated={tst_ans!r}",
            )
        return OracleResult(passed=True)

    if ora.type == "brute_force_wmc":
        # Ground truth computed in Python by full enumeration — independent of
        # any d4 binary, so a bug shared by counter and d4_static still shows.
        tst_out, tst_code, tst_to = _run(evaluated.command, instance_path,
                                         queries_path, timeout, cwd)
        if tst_to:
            return OracleResult(passed=True, timed_out=True)
        if tst_code != 0:
            return OracleResult(passed=False, reason=f"crash (exit {tst_code})")

        try:
            ref = _brute_force_wmc(instance_path)
        except Exception as e:
            return OracleResult(passed=True, no_answer=True,
                                reason=f"brute force failed: {e}")
        if ref is None:
            return OracleResult(passed=True, no_answer=True,
                                reason="too many variables for brute force")

        tst_ans = _extract(tst_out, tst_pat)
        if tst_ans == "":
            return OracleResult(passed=True, no_answer=True,
                                reason=f"no answer line in evaluated (pattern={tst_pat!r})")

        ref_ans = (str(ref.numerator) if ref.denominator == 1
                   else repr(float(ref)))
        tol = ora.tolerance if ora.tolerance > 0 else 1e-9
        if not _approx_equal(ref_ans, tst_ans, tol):
            return OracleResult(
                passed=False,
                reason=f"mismatch: brute-force={ref_ans!r} evaluated={tst_ans!r}",
            )
        return OracleResult(passed=True)

    if ora.type == "circuit_cnf_crosscheck":
        # Translate the circuit to CNF, run the reference counter on the CNF,
        # and compare against the circuit counter result.
        try:
            cnf_text = bcs_file_to_cnf(instance_path)
        except Exception as e:
            return OracleResult(passed=True, reason=f"CNF translation failed: {e}")

        fd, cnf_path = tempfile.mkstemp(suffix=".cnf", prefix="fuzz_xcheck_")
        try:
            os.write(fd, cnf_text.encode())
            os.close(fd)

            ref_out, ref_code, ref_to = _run(ora.reference, cnf_path, None,
                                             timeout, cwd)
            if ref_to or ref_code not in (0, 10, 20):
                return OracleResult(passed=True, timed_out=ref_to,
                                    reason="reference inconclusive")

            tst_out, tst_code, tst_to = _run(evaluated.command, instance_path,
                                             None, timeout, cwd)
            if tst_to:
                return OracleResult(passed=True, timed_out=True)
            if tst_code != 0:
                return OracleResult(passed=False, reason=f"crash (exit {tst_code})")

            ref_ans = _extract(ref_out, ora.result_pattern)
            tst_ans = _extract(tst_out, tst_pat or ora.result_pattern)

            no_answer = ref_ans == "" or tst_ans == ""
            if no_answer:
                missing = []
                if ref_ans == "":
                    missing.append(f"oracle/CNF (pattern={ora.result_pattern!r})")
                if tst_ans == "":
                    missing.append(f"evaluated/circuit (pattern={tst_pat!r})")
                return OracleResult(passed=True, no_answer=True,
                                    reason=f"no answer line in {', '.join(missing)}")

            tol = ora.tolerance
            match = _approx_equal(ref_ans, tst_ans, tol) if tol > 0 else (ref_ans == tst_ans)
            if not match:
                return OracleResult(
                    passed=False,
                    reason=f"mismatch: oracle(CNF)={ref_ans!r} evaluated(circuit)={tst_ans!r}",
                )
            return OracleResult(passed=True)
        finally:
            try:
                os.unlink(cnf_path)
            except OSError:
                pass

    raise ValueError(f"Unknown oracle type: {ora.type!r}")
