#!/usr/bin/env python3
"""
minimize_cnf.py — Reduce a failing CNF instance to the smallest sub-instance
that still triggers a bug between two model counters.

ALGORITHM
  1. Three TIME-BOXED elimination modes are cycled, each limited to
     --phase-budget seconds (default 10) and announced on stderr when it
     starts:
       a. clause elimination     — ddmin (Zeller 1999), drops chunks of clauses
       b. literal elimination    — try shrinking each remaining clause
                                   (--no-reduce-literals to disable)
       c. projection elimination — try dropping each 'c p show' variable
                                   (--no-reduce-show to disable)
     A mode interrupted by its budget keeps what it removed and resumes on
     the next cycle (oracle results are cached, so re-scanning is cheap).
     The cycle repeats while it makes progress.
  2. When a full cycle removes NOTHING, an exhaustive clause-by-clause
     removal pass runs (no time budget).  If it removes something the cycle
     in step 1 starts again; otherwise a fixpoint is reached and the loop
     stops.
  3. Variables are renumbered contiguously.

BEST-SO-FAR CHECKPOINT (safe to kill the job)
  Every time the oracle confirms the bug on a smaller candidate, that formula
  is immediately written to the checkpoint file: the -o FILE if given,
  otherwise <instance>.best.cnf.  If you kill the job (Ctrl-C, SIGTERM) you
  keep the smallest failing formula found so far; the script also traps the
  signal, kills any running solvers, and tells you where the file is.
  (The checkpoint is not renumbered; renumbering only happens on normal
  completion.)

RESOURCE CONTROL
  - On timeout or interrupt the WHOLE process group of the oracle is killed
    (the oracle script and every solver it spawned) — no runaway counters.
  - Unless --timeout is given, the per-call timeout is derived from a measured
    baseline run on the original instance (3x baseline, clamped to [5, 120]s).
  - Oracle results are cached by CNF content, so re-tested subsets are free.
  - --mem-mb can cap the address space of the oracle and its children.
    CAUTION: an OOM-killed counter produces no output, which the oracle may
    report as a mismatch — the minimizer could then chase OOM crashes instead
    of the original bug.  Off by default.

PROGRESS REPORTING (all on stderr, never mixed with the CNF on stdout)
  - Pass-level progress (sizes before/after, oracle runs used) is always shown.
  - A live status line refreshes every 2 seconds: current phase, clause and
    literal counts, oracle runs, cache hits, timeouts, elapsed time.
  - -v additionally logs every successful removal.
  - A final [stats] summary reports total oracle runs, time spent in the
    oracle, cache hits and timeouts.

ORACLE CONTRACT
  The oracle command must:
    - Accept a CNF file path as its last argument
    - Exit with code 0  when the instance is CORRECT (no bug)
    - Exit with code != 0 when the bug IS present

  The default oracle is the sibling script testProjModelCounting.sh (expected
  to be in the same directory), which compares projmc against d4 and exits 1
  when their counts differ.

EXAMPLES
  # Run from the scripts/ directory (default oracle = bash testProjModelCounting.sh):
  python3 minimize_cnf.py /tmp/fail_1.cnf

  # Save the result (and the live best-so-far) to a file:
  python3 minimize_cnf.py -v -o /tmp/minimal.cnf /tmp/fail_1.cnf

  # Use a custom oracle (any command that exits non-zero on the bug):
  python3 minimize_cnf.py --oracle "bash my_check.sh" /tmp/fail_1.cnf

  # Clause removal only (skip literal shrinking):
  python3 minimize_cnf.py --no-reduce-literals /tmp/fail_1.cnf

  # Force a fixed per-call timeout instead of the adaptive one:
  python3 minimize_cnf.py --timeout 60 /tmp/fail_1.cnf
"""

import argparse
import hashlib
import os
import shlex
import signal
import subprocess
import sys
import tempfile
import time


# ---------------------------------------------------------------------------
# CNF I/O
# ---------------------------------------------------------------------------

def parse_cnf(path: str):
    """Parse a DIMACS CNF file.

    Returns:
        clauses   — list of lists of ints (no trailing 0)
        show_vars — list of ints from the 'c p show' line (empty if absent)
    """
    clauses = []
    show_vars = []

    with open(path) as fh:
        pending = []
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            if line.startswith('c p show'):
                tokens = line.split()
                show_vars = [int(t) for t in tokens[3:] if t != '0']
            elif line.startswith('c') or line.startswith('p'):
                continue
            else:
                tokens = line.split()
                for tok in tokens:
                    n = int(tok)
                    if n == 0:
                        if pending:
                            clauses.append(pending)
                            pending = []
                    else:
                        pending.append(n)
        if pending:          # malformed last clause without trailing 0
            clauses.append(pending)

    return clauses, show_vars


def cnf_to_str(clauses: list, show_vars: list) -> str:
    """Return CNF as a string."""
    all_vars = {abs(lit) for clause in clauses for lit in clause}
    all_vars.update(show_vars)
    num_vars = max(all_vars) if all_vars else 0
    lines = [f'p cnf {num_vars} {len(clauses)}']
    if show_vars:
        lines.append('c p show ' + ' '.join(str(v) for v in show_vars) + ' 0')
    for clause in clauses:
        lines.append(' '.join(str(lit) for lit in clause) + ' 0')
    return '\n'.join(lines)


def write_cnf(path: str, clauses: list, show_vars: list):
    """Write a DIMACS CNF file."""
    with open(path, 'w') as fh:
        fh.write(cnf_to_str(clauses, show_vars) + '\n')


# ---------------------------------------------------------------------------
# Oracle
# ---------------------------------------------------------------------------

def log(msg: str):
    print(msg, file=sys.stderr, flush=True)


class Oracle:
    """Runs the oracle command with caching, group-kill timeouts, stats and a
    best-so-far checkpoint."""

    def __init__(self, cmd: list, timeout: float, mem_mb: int, verbose: bool,
                 best_path: str = None):
        self.cmd = cmd
        self.timeout = timeout
        self.mem_mb = mem_mb
        self.verbose = verbose
        self.best_path = best_path
        self.best_desc = None
        self.phase = 'init'
        self.runs = 0            # actual subprocess executions
        self.cache_hits = 0
        self.timeouts = 0
        self.bugs = 0
        self.oracle_time = 0.0   # wall time spent waiting on the oracle
        self.start = time.monotonic()
        self._cache = {}
        self._last_status = 0.0
        self._status_dirty = False
        fd, self.tmp_path = tempfile.mkstemp(suffix='.cnf')
        os.close(fd)

    def close(self):
        self._end_status_line()
        try:
            os.unlink(self.tmp_path)
        except OSError:
            pass

    def check(self, clauses: list, show_vars: list) -> bool:
        """Return True if the oracle reports a bug on this candidate.

        Every buggy candidate is checkpointed to best_path: candidates are
        always subsets of the current formula, so the last buggy one is the
        smallest failing formula seen so far.
        """
        text = cnf_to_str(clauses, show_vars) + '\n'
        key = hashlib.md5(text.encode()).digest()
        if key in self._cache:
            self.cache_hits += 1
            result = self._cache[key]
        else:
            with open(self.tmp_path, 'w') as fh:
                fh.write(text)
            result = self._run()
            self._cache[key] = result
        if result:
            self.bugs += 1
            if self.best_path:
                with open(self.best_path, 'w') as fh:
                    fh.write(text)
                self.best_desc = (f'{len(clauses)} clauses, '
                                  f'{sum(len(c) for c in clauses)} literals, '
                                  f'{len(show_vars)} show vars')
        self._status(len(clauses), sum(len(c) for c in clauses))
        return result

    def _run(self) -> bool:
        self.runs += 1
        t0 = time.monotonic()

        preexec = None
        if self.mem_mb:
            import resource
            limit = self.mem_mb * 1024 * 1024

            def preexec():
                resource.setrlimit(resource.RLIMIT_AS, (limit, limit))

        # start_new_session puts the oracle and every solver it spawns into
        # their own process group, so we can kill all of them at once.
        proc = subprocess.Popen(
            self.cmd + [self.tmp_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            preexec_fn=preexec,
        )
        try:
            rc = proc.wait(timeout=self.timeout)
        except subprocess.TimeoutExpired:
            self._kill_group(proc)
            self.timeouts += 1
            self.oracle_time += time.monotonic() - t0
            # Treat timeout as "no bug" so we don't keep timed-out instances.
            return False
        except BaseException:
            # Interrupted (Ctrl-C / SIGTERM): the oracle runs in its own
            # session and would not receive the terminal's signal, kill it.
            self._kill_group(proc)
            raise

        self.oracle_time += time.monotonic() - t0

        if rc == 127:
            self._end_status_line()
            cmd_str = ' '.join(self.cmd)
            log(
                f'ERROR: oracle command not found (exit 127): {cmd_str}\n'
                'Make sure the oracle script path is correct for the current '
                'working directory, or pass --oracle with an explicit path.'
            )
            sys.exit(1)
        return rc != 0

    @staticmethod
    def _kill_group(proc):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()

    # --- progress -----------------------------------------------------------

    def _status(self, n_clauses: int, n_lits: int):
        now = time.monotonic()
        if now - self._last_status < 2.0:
            return
        self._last_status = now
        elapsed = now - self.start
        line = (f'[{self.phase}] {n_clauses} clauses / {n_lits} lits | '
                f'{self.runs} oracle runs, {self.cache_hits} cached, '
                f'{self.timeouts} timeouts | {elapsed:.0f}s elapsed')
        if sys.stderr.isatty() and not self.verbose:
            print('\r\x1b[K' + line, file=sys.stderr, end='', flush=True)
            self._status_dirty = True
        else:
            log(line)

    def _end_status_line(self):
        if self._status_dirty:
            print(file=sys.stderr, flush=True)
            self._status_dirty = False

    def pass_report(self, msg: str):
        """Pass-level progress, always shown."""
        self._end_status_line()
        log(msg)

    def summary(self) -> str:
        elapsed = time.monotonic() - self.start
        return (f'[stats] {self.runs} oracle runs ({self.oracle_time:.1f}s in '
                f'oracle, {elapsed:.1f}s wall), {self.cache_hits} cache hits, '
                f'{self.timeouts} timeouts, {self.bugs} buggy candidates')


# ---------------------------------------------------------------------------
# Minimization
# ---------------------------------------------------------------------------

def _expired(deadline) -> bool:
    return deadline is not None and time.monotonic() >= deadline


def ddmin_clauses(clauses: list, show_vars: list, oracle: Oracle,
                  deadline: float = None) -> list:
    """Apply ddmin to find a minimal failing clause subset.

    Classic ddmin (Zeller 1999):
      - Partition the current set into n chunks.
      - Try removing each chunk (complement test).
      - Try keeping each chunk alone (reduction test).
      - If neither works, double n (finer granularity).
      - Stop when n exceeds the set size.
    """
    oracle.phase = 'ddmin'
    current = list(clauses)
    n = 2

    while len(current) >= 2 and not _expired(deadline):
        chunk_size = max(1, len(current) // n)
        chunks = [current[i:i + chunk_size] for i in range(0, len(current), chunk_size)]
        n = len(chunks)  # actual number after ceiling division

        reduced = False

        # --- complement test: remove one chunk ---
        for i, chunk in enumerate(chunks):
            if _expired(deadline):
                return current
            complement = [c for j, cs in enumerate(chunks) for c in cs if j != i]
            if oracle.check(complement, show_vars):
                removed = len(current) - len(complement)
                current = complement
                n = max(n - 1, 2)
                reduced = True
                if oracle.verbose:
                    log(f'[ddmin] removed chunk of {removed} → {len(current)} clauses')
                break

        if reduced:
            continue

        # --- reduction test: keep one chunk alone ---
        for chunk in chunks:
            if _expired(deadline):
                return current
            if oracle.check(chunk, show_vars):
                if oracle.verbose:
                    log(f'[ddmin] reduced to chunk of {len(chunk)} clauses')
                current = chunk
                n = 2
                reduced = True
                break

        if reduced:
            continue

        # --- increase granularity ---
        if n >= len(current):
            break
        n = min(n * 2, len(current))

    return current


def remove_clauses(clauses: list, show_vars: list, oracle: Oracle,
                   deadline: float = None):
    """Try dropping each clause one by one.

    Returns (new_clauses, changed).
    """
    oracle.phase = 'clauses'
    current = list(clauses)
    changed = False
    i = 0
    while i < len(current):
        if _expired(deadline):
            break
        candidate = current[:i] + current[i + 1:]
        if oracle.check(candidate, show_vars):
            current = candidate
            changed = True
            if oracle.verbose:
                log(f'[clauses] removed clause → {len(current)} remain')
        else:
            i += 1
    return current, changed


def remove_literals(clauses: list, show_vars: list, oracle: Oracle,
                    deadline: float = None):
    """Try removing individual literals from each clause.

    Unit clauses (length 1) are left intact — removing their only literal
    would produce an empty clause that most parsers reject.

    Returns (new_clauses, changed).
    """
    oracle.phase = 'literals'
    current = [list(c) for c in clauses]
    changed = False

    for i in range(len(current)):
        j = 0
        while len(current[i]) > 1 and j < len(current[i]):
            if _expired(deadline):
                return current, changed
            candidate = (current[:i]
                         + [current[i][:j] + current[i][j + 1:]]
                         + current[i + 1:])
            if oracle.check(candidate, show_vars):
                removed_lit = current[i][j]
                current[i] = candidate[i]
                changed = True
                if oracle.verbose:
                    log(f'[literals] removed lit {removed_lit} from clause {i} '
                        f'→ {len(current[i])} lits remain')
                # don't advance j — the next literal slid into position j
            else:
                j += 1

    return current, changed


def minimize_show_vars(clauses: list, show_vars: list, oracle: Oracle,
                       deadline: float = None):
    """Greedily remove variables from the projection set.

    Returns (new_show_vars, changed).
    """
    oracle.phase = 'show'
    current = list(show_vars)
    changed = False
    i = 0
    while i < len(current):
        if _expired(deadline):
            break
        candidate = current[:i] + current[i + 1:]
        if oracle.check(clauses, candidate):
            if oracle.verbose:
                log(f'[show] removed var {current[i]} → {len(candidate)} show vars')
            current = candidate
            changed = True
        else:
            i += 1
    return current, changed


def renumber(clauses: list, show_vars: list):
    """Renumber variables to be contiguous starting from 1.

    Show variables are mapped first (preserving their order), then remaining
    variables in order of first appearance.
    """
    mapping = {}
    counter = [1]

    def next_id(v):
        if v not in mapping:
            mapping[v] = counter[0]
            counter[0] += 1
        return mapping[v]

    new_show = [next_id(v) for v in show_vars]
    new_clauses = []
    for clause in clauses:
        new_clauses.append([next_id(abs(lit)) * (1 if lit > 0 else -1) for lit in clause])

    return new_clauses, new_show


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='minimize_cnf.py',
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        'instance',
        help='Path to the failing CNF instance (DIMACS format).',
    )
    p.add_argument(
        '--oracle',
        default='bash testProjModelCounting.sh',
        metavar='CMD',
        help=(
            'Shell command used to detect the bug. The CNF file path is '
            'appended as the last argument. Must exit non-zero when the bug '
            'is present and zero when it is absent. '
            'Default: "bash testProjModelCounting.sh" (relative to cwd).'
        ),
    )
    p.add_argument(
        '-o', '--output',
        default=None,
        metavar='FILE',
        help=(
            'Write the minimized CNF to FILE instead of stdout. FILE is also '
            'used as the live best-so-far checkpoint during the run.'
        ),
    )
    p.add_argument(
        '--no-reduce-show',
        action='store_true',
        help=(
            'Skip trying to remove variables from the projection set '
            '(c p show line). By default the show set is also minimized '
            'after clauses are reduced.'
        ),
    )
    p.add_argument(
        '--no-renumber',
        action='store_true',
        help='Skip variable renumbering. Output keeps the original variable IDs.',
    )
    p.add_argument(
        '--no-greedy',
        action='store_true',
        help=(
            'Skip the interleaved shrink passes after ddmin. Faster but the '
            'result is usually far from 1-minimal.'
        ),
    )
    p.add_argument(
        '--reduce-literals',
        dest='reduce_literals',
        action='store_true',
        default=True,
        help=argparse.SUPPRESS,  # kept for backward compatibility (now the default)
    )
    p.add_argument(
        '--no-reduce-literals',
        dest='reduce_literals',
        action='store_false',
        help=(
            'Do not try to remove individual literals from clauses during the '
            'shrink passes (literal shrinking is interleaved with clause '
            'removal and enabled by default).'
        ),
    )
    p.add_argument(
        '--phase-budget',
        type=float,
        default=10.0,
        metavar='SEC',
        help=(
            'Time budget for each elimination mode (clause / literal / '
            'projection) before switching to the next one. An interrupted '
            'mode resumes on the next cycle. Default: 10 seconds.'
        ),
    )
    p.add_argument(
        '--timeout',
        type=float,
        default=None,
        metavar='SEC',
        help=(
            'Per-call timeout in seconds for the oracle. Default: adaptive — '
            '3x the time of a baseline run on the original instance, clamped '
            'to [5, 120] seconds.'
        ),
    )
    p.add_argument(
        '--mem-mb',
        type=int,
        default=None,
        metavar='MB',
        help=(
            'Cap the address space (RLIMIT_AS) of the oracle and its solvers. '
            'CAUTION: an OOM-killed counter may look like a mismatch to the '
            'oracle, steering the minimizer toward OOM crashes instead of the '
            'original bug. Off by default.'
        ),
    )
    p.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Also log every successful removal (pass-level progress is always shown).',
    )
    return p


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    # Turn SIGTERM into KeyboardInterrupt so `kill <pid>` also leaves a clean
    # checkpoint and no orphan solvers.
    def _sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sigterm)

    best_path = args.output if args.output else args.instance + '.best.cnf'

    oracle = Oracle(
        cmd=shlex.split(args.oracle),
        timeout=args.timeout if args.timeout is not None else 120.0,
        mem_mb=args.mem_mb,
        verbose=args.verbose,
        best_path=best_path,
    )

    try:
        # --- Parse ---
        clauses, show_vars = parse_cnf(args.instance)
        n_clauses0 = len(clauses)
        n_show0 = len(show_vars)
        n_lits0 = sum(len(c) for c in clauses)
        log(f'[init] {n_clauses0} clauses, {n_lits0} literals, {n_show0} show vars')
        log(f'[init] best-so-far failing formula is kept in {best_path} '
            '(safe to kill the job at any time)')

        # --- Verify bug exists (and measure a baseline for the timeout) ---
        t0 = time.monotonic()
        if not oracle.check(clauses, show_vars):
            log(
                'ERROR: oracle did not report a bug on the original instance.\n'
                'Check that the oracle command is correct and exits non-zero on failure.'
            )
            sys.exit(1)
        baseline = time.monotonic() - t0
        if args.timeout is None:
            oracle.timeout = min(max(5.0, 3.0 * baseline), 120.0)
        log(f'[init] bug confirmed in {baseline:.2f}s → per-call timeout {oracle.timeout:.1f}s')

        # --- time-boxed elimination modes ---
        budget = args.phase_budget

        if not args.no_greedy:
            rnd = 0
            while True:
                # Cycle the time-boxed modes while they make progress.
                cycle_changed = True
                while cycle_changed:
                    cycle_changed = False
                    rnd += 1

                    oracle.pass_report(
                        f'[cycle {rnd}] >>> mode: clause elimination '
                        f'(ddmin, {budget:.0f}s budget)')
                    runs_before = oracle.runs
                    n_before = len(clauses)
                    clauses = ddmin_clauses(clauses, show_vars, oracle,
                                            time.monotonic() + budget)
                    cycle_changed |= len(clauses) < n_before
                    oracle.pass_report(
                        f'[cycle {rnd}] {len(clauses)} clauses remain '
                        f'({oracle.runs - runs_before} oracle runs)')

                    if args.reduce_literals:
                        oracle.pass_report(
                            f'[cycle {rnd}] >>> mode: literal elimination '
                            f'({budget:.0f}s budget)')
                        runs_before = oracle.runs
                        clauses, ch = remove_literals(clauses, show_vars, oracle,
                                                      time.monotonic() + budget)
                        cycle_changed |= ch
                        oracle.pass_report(
                            f'[cycle {rnd}] {sum(len(c) for c in clauses)} literals remain '
                            f'({oracle.runs - runs_before} oracle runs)')

                    if not args.no_reduce_show and show_vars:
                        oracle.pass_report(
                            f'[cycle {rnd}] >>> mode: projected variable elimination '
                            f'({budget:.0f}s budget)')
                        runs_before = oracle.runs
                        show_vars, ch = minimize_show_vars(clauses, show_vars, oracle,
                                                           time.monotonic() + budget)
                        cycle_changed |= ch
                        oracle.pass_report(
                            f'[cycle {rnd}] {len(show_vars)} show vars remain '
                            f'({oracle.runs - runs_before} oracle runs)')

                # Cycle removed nothing: exhaustive clause-by-clause pass.
                oracle.pass_report(
                    '[final] >>> mode: clause-by-clause elimination (exhaustive)')
                runs_before = oracle.runs
                clauses, ch = remove_clauses(clauses, show_vars, oracle)
                oracle.pass_report(
                    f'[final] {len(clauses)} clauses remain '
                    f'({oracle.runs - runs_before} oracle runs)')
                if not ch:
                    oracle.pass_report('[final] fixpoint reached — minimization done')
                    break
                oracle.pass_report('[final] progress made — restarting mode cycle')
        else:
            # --no-greedy: one full ddmin pass, then reduce the projection set
            oracle.pass_report('>>> mode: clause elimination (ddmin, no budget)')
            runs_before = oracle.runs
            clauses = ddmin_clauses(clauses, show_vars, oracle)
            oracle.pass_report(
                f'[ddmin] {len(clauses)} clauses remain '
                f'({oracle.runs - runs_before} oracle runs)')
            if not args.no_reduce_show and show_vars:
                oracle.pass_report('>>> mode: projected variable elimination')
                runs_before = oracle.runs
                show_vars, _ = minimize_show_vars(clauses, show_vars, oracle)
                oracle.pass_report(
                    f'[show] {len(show_vars)} show vars remain '
                    f'({oracle.runs - runs_before} oracle runs)')

        # --- renumber ---
        if not args.no_renumber:
            clauses, show_vars = renumber(clauses, show_vars)

        # --- output ---
        oracle.pass_report(
            f'[done] clauses: {n_clauses0} → {len(clauses)}, '
            f'literals: {n_lits0} → {sum(len(c) for c in clauses)}, '
            f'show vars: {n_show0} → {len(show_vars)}')
        log(oracle.summary())

        # Refresh the checkpoint with the final (renumbered) formula.
        write_cnf(best_path, clauses, show_vars)
        if args.output:
            log(f'[done] written to {args.output}')
        else:
            log(f'[done] also written to {best_path}')
            print(cnf_to_str(clauses, show_vars))

    except KeyboardInterrupt:
        oracle.pass_report('[interrupted] job stopped by user')
        if oracle.best_desc:
            log(f'[interrupted] smallest failing formula so far '
                f'({oracle.best_desc}) is in {best_path}')
        else:
            log('[interrupted] no failing formula confirmed yet — '
                f'{best_path} was not written')
        log(oracle.summary())
        sys.exit(130)
    finally:
        oracle.close()


if __name__ == '__main__':
    main()
