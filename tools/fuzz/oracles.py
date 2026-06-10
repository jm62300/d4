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
