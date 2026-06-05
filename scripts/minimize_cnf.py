#!/usr/bin/env python3
"""
minimize_cnf.py — Reduce a failing CNF instance to the smallest sub-instance
that still triggers a bug between two model counters.

The script uses the ddmin algorithm (Zeller 1999) to remove clauses, followed
by a greedy one-by-one pass for fine-grained cleanup.  It then tries to remove
variables from the projection set (c p show) and renumbers all variables to
produce a clean minimal output.  Use --no-reduce-show to skip the show-set
reduction.

With --reduce-literals the script also tries to remove individual literals
from clauses.  Whenever a literal is successfully removed the clause-removal
passes are re-run, and this continues until no further progress is possible
(fixpoint).

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

  # Save the result to a file and watch progress:
  python3 minimize_cnf.py -v -o /tmp/minimal.cnf /tmp/fail_1.cnf

  # Use a custom oracle (any command that exits non-zero on the bug):
  python3 minimize_cnf.py --oracle "bash my_check.sh" /tmp/fail_1.cnf

  # Skip shrinking the projection set (c p show variables):
  python3 minimize_cnf.py --no-reduce-show /tmp/fail_1.cnf

  # Keep original variable numbers (skip renumbering):
  python3 minimize_cnf.py --no-renumber /tmp/fail_1.cnf

  # Also shrink individual clauses by removing literals (fixpoint with clause removal):
  python3 minimize_cnf.py --reduce-literals /tmp/fail_1.cnf

  # Increase per-call timeout for slow solvers:
  python3 minimize_cnf.py --timeout 60 /tmp/fail_1.cnf
"""

import argparse
import os
import subprocess
import sys
import tempfile


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


def write_cnf(path: str, clauses: list, show_vars: list):
    """Write a DIMACS CNF file."""
    all_vars = {abs(lit) for clause in clauses for lit in clause}
    all_vars.update(show_vars)
    num_vars = max(all_vars) if all_vars else 0

    with open(path, 'w') as fh:
        fh.write(f'p cnf {num_vars} {len(clauses)}\n')
        if show_vars:
            fh.write('c p show ' + ' '.join(str(v) for v in show_vars) + ' 0\n')
        for clause in clauses:
            fh.write(' '.join(str(lit) for lit in clause) + ' 0\n')


def cnf_to_str(clauses: list, show_vars: list) -> str:
    """Return CNF as a string (for stdout output)."""
    all_vars = {abs(lit) for clause in clauses for lit in clause}
    all_vars.update(show_vars)
    num_vars = max(all_vars) if all_vars else 0
    lines = [f'p cnf {num_vars} {len(clauses)}']
    if show_vars:
        lines.append('c p show ' + ' '.join(str(v) for v in show_vars) + ' 0')
    for clause in clauses:
        lines.append(' '.join(str(lit) for lit in clause) + ' 0')
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Oracle
# ---------------------------------------------------------------------------

def check_bug(oracle_cmd: list, cnf_path: str, timeout: int) -> bool:
    """Return True if the oracle reports a bug (non-zero exit) on cnf_path."""
    try:
        result = subprocess.run(
            oracle_cmd + [cnf_path],
            capture_output=True,
            timeout=timeout,
        )
        if result.returncode == 127:
            # 127 = command not found; abort rather than treating as "bug present"
            cmd_str = ' '.join(oracle_cmd)
            print(
                f'ERROR: oracle command not found (exit 127): {cmd_str}\n'
                'Make sure the oracle script path is correct for the current '
                'working directory, or pass --oracle with an explicit path.',
                file=sys.stderr,
            )
            sys.exit(1)
        return result.returncode != 0
    except subprocess.TimeoutExpired:
        # Treat timeout as "no bug" so we don't keep timed-out instances.
        return False
    except Exception as exc:
        print(f'[oracle error] {exc}', file=sys.stderr)
        return False


# ---------------------------------------------------------------------------
# Minimization
# ---------------------------------------------------------------------------

def _still_bugs(clauses, show_vars, oracle_cmd, tmp_path, timeout):
    write_cnf(tmp_path, clauses, show_vars)
    return check_bug(oracle_cmd, tmp_path, timeout)


def ddmin_clauses(clauses: list, show_vars: list, oracle_cmd: list,
                  tmp_path: str, timeout: int, verbose: bool) -> list:
    """Apply ddmin to find a minimal failing clause subset.

    Classic ddmin (Zeller 1999):
      - Partition the current set into n chunks.
      - Try removing each chunk (complement test).
      - Try keeping each chunk alone (reduction test).
      - If neither works, double n (finer granularity).
      - Stop when n exceeds the set size.
    """
    current = list(clauses)
    n = 2

    if verbose:
        print(f'[ddmin] starting with {len(current)} clauses')

    while len(current) >= 2:
        chunk_size = max(1, len(current) // n)
        chunks = [current[i:i + chunk_size] for i in range(0, len(current), chunk_size)]
        n = len(chunks)  # actual number after ceiling division

        reduced = False

        # --- complement test: remove one chunk ---
        for i, chunk in enumerate(chunks):
            complement = [c for j, cs in enumerate(chunks) for c in cs if j != i]
            if _still_bugs(complement, show_vars, oracle_cmd, tmp_path, timeout):
                removed = len(current) - len(complement)
                current = complement
                n = max(n - 1, 2)
                reduced = True
                if verbose:
                    print(f'[ddmin] removed chunk of {removed} → {len(current)} clauses')
                break

        if reduced:
            continue

        # --- reduction test: keep one chunk alone ---
        for chunk in chunks:
            if _still_bugs(chunk, show_vars, oracle_cmd, tmp_path, timeout):
                if verbose:
                    print(f'[ddmin] reduced to chunk of {len(chunk)} clauses')
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


def greedy_clauses(clauses: list, show_vars: list, oracle_cmd: list,
                   tmp_path: str, timeout: int, verbose: bool) -> list:
    """One-by-one greedy pass: try removing each clause individually."""
    if verbose:
        print(f'[greedy] fine-grained pass on {len(clauses)} clauses')

    current = list(clauses)
    i = 0
    while i < len(current):
        candidate = current[:i] + current[i + 1:]
        if _still_bugs(candidate, show_vars, oracle_cmd, tmp_path, timeout):
            current = candidate
            if verbose:
                print(f'[greedy] removed clause → {len(current)} remain')
        else:
            i += 1
    return current


def minimize_show_vars(clauses: list, show_vars: list, oracle_cmd: list,
                       tmp_path: str, timeout: int, verbose: bool) -> list:
    """Greedily remove variables from the projection set."""
    if verbose:
        print(f'[show] trying to reduce {len(show_vars)} show vars')

    current = list(show_vars)
    i = 0
    while i < len(current):
        candidate = current[:i] + current[i + 1:]
        if _still_bugs(clauses, candidate, oracle_cmd, tmp_path, timeout):
            if verbose:
                print(f'[show] removed var {current[i]} → {len(candidate)} show vars')
            current = candidate
        else:
            i += 1
    return current


def greedy_literals(clauses: list, show_vars: list, oracle_cmd: list,
                    tmp_path: str, timeout: int, verbose: bool):
    """Try removing individual literals from each clause.

    Unit clauses (length 1) are left intact — removing their only literal would
    produce an empty clause that most parsers reject.

    Returns (new_clauses, changed) where changed is True if at least one
    literal was removed.
    """
    if verbose:
        total = sum(len(c) for c in clauses)
        print(f'[literals] trying to shrink literals across {len(clauses)} clauses ({total} total lits)')

    current = [list(c) for c in clauses]
    changed = False

    for i in range(len(current)):
        j = 0
        while j < len(current[i]):
            if len(current[i]) <= 1:
                break  # never empty a clause
            candidate = current[:i] + [current[i][:j] + current[i][j + 1:]] + current[i + 1:]
            if _still_bugs(candidate, show_vars, oracle_cmd, tmp_path, timeout):
                removed_lit = current[i][j]
                current[i] = candidate[i]
                changed = True
                if verbose:
                    print(f'[literals] removed lit {removed_lit} from clause {i} '
                          f'→ {len(current[i])} lits remain')
                # don't advance j — the next literal slid into position j
            else:
                j += 1

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
        help='Write the minimized CNF to FILE instead of stdout.',
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
            'Skip the final greedy one-by-one pass. ddmin alone is faster but '
            'may not reach a 1-minimal result.'
        ),
    )
    p.add_argument(
        '--reduce-literals',
        action='store_true',
        help=(
            'After clause removal, also try removing individual literals from '
            'each clause. Whenever a literal is removed the clause-removal '
            'passes restart; this repeats until no further progress is made '
            '(fixpoint). Disabled by default because it is significantly slower.'
        ),
    )
    p.add_argument(
        '--timeout',
        type=int,
        default=30,
        metavar='SEC',
        help='Per-call timeout in seconds for the oracle (default: 30).',
    )
    p.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Print progress information to stderr.',
    )
    return p


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    oracle_cmd = args.oracle.split()
    tmp_fd, tmp_path = tempfile.mkstemp(suffix='.cnf')
    os.close(tmp_fd)

    try:
        # --- Parse ---
        clauses, show_vars = parse_cnf(args.instance)
        if args.verbose:
            print(f'[init] {len(clauses)} clauses, {len(show_vars)} show vars', file=sys.stderr)

        # --- Verify bug exists ---
        if not _still_bugs(clauses, show_vars, oracle_cmd, tmp_path, args.timeout):
            print(
                'ERROR: oracle did not report a bug on the original instance.\n'
                'Check that the oracle command is correct and exits non-zero on failure.',
                file=sys.stderr,
            )
            sys.exit(1)
        if args.verbose:
            print('[init] bug confirmed on original instance', file=sys.stderr)

        # --- fixpoint loop: clause removal ↔ literal removal ---
        iteration = 0
        while True:
            iteration += 1
            if args.verbose and args.reduce_literals:
                print(f'[fixpoint] iteration {iteration}', file=sys.stderr)

            clauses = ddmin_clauses(clauses, show_vars, oracle_cmd, tmp_path, args.timeout, args.verbose)

            if not args.no_greedy:
                clauses = greedy_clauses(clauses, show_vars, oracle_cmd, tmp_path, args.timeout, args.verbose)

            if not args.reduce_literals:
                break

            clauses, lits_changed = greedy_literals(
                clauses, show_vars, oracle_cmd, tmp_path, args.timeout, args.verbose)
            if not lits_changed:
                break  # fixpoint reached
            if args.verbose:
                print('[fixpoint] literals removed — restarting clause reduction', file=sys.stderr)

        # --- shrink projection set ---
        if not args.no_reduce_show and show_vars:
            show_vars = minimize_show_vars(clauses, show_vars, oracle_cmd, tmp_path, args.timeout, args.verbose)

        # --- renumber ---
        if not args.no_renumber:
            clauses, show_vars = renumber(clauses, show_vars)

        # --- output ---
        if args.verbose:
            print(
                f'[done] {len(clauses)} clauses, {len(show_vars)} show vars',
                file=sys.stderr,
            )

        if args.output:
            write_cnf(args.output, clauses, show_vars)
            if args.verbose:
                print(f'[done] written to {args.output}', file=sys.stderr)
        else:
            print(cnf_to_str(clauses, show_vars))

    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass


if __name__ == '__main__':
    main()
