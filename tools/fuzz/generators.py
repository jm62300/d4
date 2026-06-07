"""
Instance generators for the d4 fuzzing tool.
All generators are self-contained — no external tools required.
Each returns (instance_path, queries_path | None).
"""

from __future__ import annotations
import math
import os
import random
import tempfile
from typing import Optional


# ---------------------------------------------------------------------------
# Temp file helpers
# ---------------------------------------------------------------------------

def _write_tmp(text: str, suffix: str = ".cnf") -> str:
    fd, path = tempfile.mkstemp(suffix=suffix, prefix="fuzz_")
    os.write(fd, text.encode())
    os.close(fd)
    return path


# ---------------------------------------------------------------------------
# Structured CNF generation via planted assignment
#
# Why NOT full Tseitin + forced root:
#   The biconditional Tseitin encoding (out ↔ gate(inputs)) combined with a
#   unit clause forcing the root to TRUE lets BCP cascade through every gate
#   variable in linear time.  Any preprocessor solves the formula before the
#   solver even begins.
#
# What we do instead — one-directional gate-shaped clauses:
#   AND-shape:  ¬x ∨ ¬y ∨ z     (x ∧ y → z, one clause per gate)
#   OR-shape:   x ∨ y ∨ ¬z      (z → x ∨ y, one clause per gate)
#   XOR-shape:  (la ∨ lb) ∧ (¬la ∨ ¬lb)   parity over two literals
#   RAND:       random 2-4 literal clause
#
#   These have circuit-like structure but do NOT fully fix any variable
#   given its neighbours — the model count is non-trivial and BCP cannot
#   cascade.  A planted assignment guarantees satisfiability with no unit
#   clause and no forced output.
# ---------------------------------------------------------------------------

def _gen_cnf(max_vars: int = 100,
             and_prob:  float = 0.30,
             or_prob:   float = 0.30,
             xor_prob:  float = 0.20,
             rnd_prob:  float = 0.10,
             large_prob: float = 0.10) -> tuple[str, int]:
    """
    Satisfiable CNF with structured gate-shaped clauses and planted assignment.

    Clause shapes:
      AND:   ¬x ∨ ¬y ∨ z           — one-directional AND implication (3 lits)
      OR:    x ∨ y ∨ ¬z            — one-directional OR implication (3 lits)
      XOR:   (la ∨ lb) ∧ (¬la ∨ ¬lb) — parity constraint (hard for DPLL)
      RAND:  random k-clause (k ∈ {2, 3, 4})
      LARGE: random long clause (k ∈ {5 … max_vars//3})

    No forced output, no unit clause, no BCP cascade.
    """
    n_vars   = random.randint(max(3, max_vars // 4), max_vars)
    n_groups = random.randint(2 * n_vars, 5 * n_vars)

    # Planted assignment: index 0 unused.
    asgn: list[bool] = [random.random() < 0.5 for _ in range(n_vars + 1)]

    def val(lit: int) -> bool:
        return asgn[abs(lit)] if lit > 0 else not asgn[abs(lit)]

    def fix(clause: list[int]) -> list[int]:
        """Flip one random literal to satisfy a clause falsified by asgn."""
        if not any(val(l) for l in clause):
            i = random.randrange(len(clause))
            clause[i] = -clause[i]
        return clause

    def rlit(v: int) -> int:
        return v if random.random() < 0.5 else -v

    clauses: list[list[int]] = []

    for _ in range(n_groups):
        gate = random.choices(
            ["AND", "OR", "XOR", "RAND", "LARGE"],
            weights=[and_prob, or_prob, xor_prob, rnd_prob, large_prob], k=1
        )[0]

        if gate == "XOR":
            # Parity: (la ∨ lb) ∧ (¬la ∨ ¬lb)  ≡  la ≠ lb.
            # Resample until the planted assignment has val(la) ≠ val(lb).
            for _ in range(20):
                a, b = random.sample(range(1, n_vars + 1), 2)
                la, lb = rlit(a), rlit(b)
                if val(la) != val(lb):
                    clauses.append([la,  lb])
                    clauses.append([-la, -lb])
                    break

        elif gate == "AND":
            # ¬x ∨ ¬y ∨ z  (x ∧ y → z)
            k = min(3, n_vars)
            vs = random.sample(range(1, n_vars + 1), k)
            lits = [rlit(v) for v in vs]
            clauses.append(fix([-lits[0], -lits[1], lits[2]]))

        elif gate == "OR":
            # x ∨ y ∨ ¬z  (z → x ∨ y)
            k = min(3, n_vars)
            vs = random.sample(range(1, n_vars + 1), k)
            lits = [rlit(v) for v in vs]
            clauses.append(fix([lits[0], lits[1], -lits[2]]))

        elif gate == "RAND":
            k = random.choices([2, 3, 3, 4], k=1)[0]
            k = min(k, n_vars)
            vs = random.sample(range(1, n_vars + 1), k)
            clauses.append(fix([rlit(v) for v in vs]))

        else:  # LARGE
            k = random.randint(5, max(5, n_vars // 3))
            k = min(k, n_vars)
            vs = random.sample(range(1, n_vars + 1), k)
            clauses.append(fix([rlit(v) for v in vs]))

    lines = [f"p cnf {n_vars} {len(clauses)}"]
    for clause in clauses:
        lines.append(" ".join(map(str, clause)) + " 0")
    return "\n".join(lines) + "\n", n_vars


# ---------------------------------------------------------------------------
# Native circuit generation (BC-S1.2 format)
# Ported and unified from scripts/circuit_fuzzer/{gate_list,gates_rnd1,to_bcs,to_dimacs}.py
# ---------------------------------------------------------------------------

_NEG = "-"


def _cab(name: str) -> str:
    """Absolute (positive) circuit name."""
    return name[1:] if name.startswith(_NEG) else name


def _cneg(name: str) -> str:
    """Negate a circuit name; double-negation cancels."""
    return name[1:] if name.startswith(_NEG) else _NEG + name


def _gen_circuit_structure(
    max_inputs: int = 20,
    max_gates: int = 100,
    polarity_prob: float = 0.5,
    or_prob: float = 0.3,
    gate_inputs_sigma: float = 2.5,
) -> tuple[list[tuple[str, str, list[str]]], list[str], list[str]]:
    """
    Generate a random circuit with a forced-true output.

    Returns (gates, primary_inputs, true_names) where:
      gates         — list of (output_name, gtype "A"|"O", [input_literals])
      primary_inputs — names of leaf (non-gate) inputs
      true_names    — names asserted to be true (circuit outputs)
    """
    n_inputs = random.randint(3, max(3, max_inputs))
    n_gates  = random.randint(2, max(2, max_gates))

    prim_inputs = [f"i{j}" for j in range(n_inputs)]
    available   = list(prim_inputs)
    gates: list[tuple[str, str, list[str]]] = []

    for j in range(n_gates):
        gname = f"g{j}"
        k = 2 + abs(math.floor(random.gauss(0.0, gate_inputs_sigma)))
        k = min(k, len(available))
        chosen = random.sample(available, k)
        inputs = [n if random.random() < polarity_prob else _cneg(n) for n in chosen]
        gtype  = "O" if random.random() < or_prob else "A"
        gates.append((gname, gtype, inputs))
        available.append(gname)

    # Roots: names not consumed as any gate input.
    used_as_input: set[str] = set()
    for _, _, inps in gates:
        used_as_input.update(_cab(n) for n in inps)
    roots = [n for n in available if n not in used_as_input]
    assert roots

    if len(roots) == 1:
        true_names = list(roots)
    else:
        gates.append(("root", "O", list(roots)))
        true_names = ["root"]

    # Primary inputs: appear in gate inputs but are not gate outputs.
    defined = {_cab(g[0]) for g in gates}
    actual_inputs: set[str] = set()
    for _, _, inps in gates:
        for n in inps:
            abs_n = _cab(n)
            if abs_n not in defined:
                actual_inputs.add(abs_n)
    for t in true_names:
        abs_t = _cab(t)
        if abs_t not in defined:
            actual_inputs.add(abs_t)

    return gates, sorted(actual_inputs), true_names


def _circuit_to_bcs(gates, inputs, true_names) -> str:
    """Serialize a circuit structure to BC-S1.2 text."""
    lines = ["c BC-S1.2"]
    for t in true_names:
        lines.append(f"T {t}")
    for inp in inputs:
        lines.append(f"I {inp}")
    for output, gtype, inps in gates:
        lines.append(f"G {output} := {gtype} {' '.join(inps)}")
    return "\n".join(lines) + "\n"


def _circuit_to_cnf(gates, inputs, true_names) -> str:  # reserved for future circuit-vs-CNF cross-checking
    """Tseitin encoding of a circuit to DIMACS CNF."""
    name_to_id: dict[str, int] = {}

    def gid(name: str) -> int:
        abs_n = _cab(name)
        if abs_n not in name_to_id:
            name_to_id[abs_n] = len(name_to_id) + 1
        lid = name_to_id[abs_n]
        return -lid if name.startswith(_NEG) else lid

    for n in inputs:
        gid(n)
    for output, _, inps in gates:
        gid(output)
        for n in inps:
            gid(n)

    clause_lines: list[str] = []
    for output, gtype, inps in gates:
        o  = gid(output)
        xs = [gid(n) for n in inps]
        if gtype == "A":
            clause_lines.append(f"{o} {' '.join(str(-x) for x in xs)} 0")
            for x in xs:
                clause_lines.append(f"{-o} {x} 0")
        else:
            clause_lines.append(f"{-o} {' '.join(str(x) for x in xs)} 0")
            for x in xs:
                clause_lines.append(f"{o} {-x} 0")
    for t in true_names:
        clause_lines.append(f"{gid(t)} 0")

    n_vars    = len(name_to_id)
    n_clauses = len(clause_lines)
    return f"p cnf {n_vars} {n_clauses}\n" + "\n".join(clause_lines) + "\n"


# ---------------------------------------------------------------------------
# Public generators
# ---------------------------------------------------------------------------

def gen_plain_cnf(max_vars: int = 100) -> tuple[str, None]:
    text, _ = _gen_cnf(max_vars)
    return _write_tmp(text), None


def gen_projected_cnf(max_vars: int = 50) -> tuple[str, None]:
    text, n_vars = _gen_cnf(max_vars)
    lines = text.splitlines()
    # Project over a random subset of all variables.
    size = max(1, n_vars // random.choice([7, 5, 3, 2]))
    projected = random.sample(range(1, n_vars + 1), size)
    header = next(l for l in lines if l.startswith("p cnf"))
    new_lines = [header, f"c p show {' '.join(map(str, projected))} 0"]
    new_lines += [l for l in lines if not l.startswith("p ")]
    return _write_tmp("\n".join(new_lines) + "\n"), None


def gen_weighted_cnf(max_vars: int = 20) -> tuple[str, None]:
    text, n_vars = _gen_cnf(max_vars)
    lines = text.splitlines()
    header = lines[0]          # p cnf N M — must come first
    clauses = lines[1:]
    weight_lines = ["c t wmc"]
    for i in range(1, n_vars + 1):
        p = random.random()
        weight_lines.append(f"c p weight {i} {p:.6f} 0")
        weight_lines.append(f"c p weight -{i} {1-p:.6f} 0")
    return _write_tmp("\n".join([header] + weight_lines + clauses) + "\n"), None


def gen_complex_cnf(max_vars: int = 200) -> tuple[str, None]:
    text, n_vars = _gen_cnf(max_vars)
    lines = text.splitlines()
    header = lines[0]
    clauses = lines[1:]
    weight_lines = ["c t wmc"]
    for i in range(1, n_vars + 1):
        if random.random() < 0.5:
            weight_lines.append(f"c p weight {i} 1.00 0.00 0")
            weight_lines.append(f"c p weight -{i} 1.00 0.00 0")
        else:
            pw1, pw2 = random.random(), random.random()
            nw1, nw2 = random.random(), random.random()
            weight_lines.append(f"c p weight {i} {pw1:.4f} {pw2:.4f} 0")
            weight_lines.append(f"c p weight -{i} {nw1:.4f} {nw2:.4f} 0")
    return _write_tmp("\n".join([header] + weight_lines + clauses) + "\n"), None


def gen_maxsharpsat_cnf(max_vars: int = 30) -> tuple[str, None]:
    text, n_vars = _gen_cnf(max_vars)
    lines = text.splitlines()
    header = lines[0]
    clauses = lines[1:]
    shuffled = list(range(1, n_vars + 1))
    random.shuffle(shuffled)
    r = max(1, n_vars // 3)
    max_part = " ".join(map(str, shuffled[:r]))
    ind_part  = " ".join(map(str, shuffled[r:]))
    annot_lines = [f"c max {max_part} 0", f"c ind {ind_part} 0"]
    for i in range(1, n_vars + 1):
        seed = random.random()
        annot_lines.append(f"c p weight -{i} {seed:.4f} 0")
        annot_lines.append(f"c p weight {i} {1-seed:.4f} 0")
    return _write_tmp("\n".join([header] + annot_lines + clauses) + "\n"), None


def gen_cnf_with_queries(max_vars: int = 50, n_queries: int = 20) -> tuple[str, str]:
    text, n_vars = _gen_cnf(max_vars)
    query_lines = []
    for _ in range(n_queries):
        kind  = random.choice(["m", "d"])
        ratio = max(1, n_vars * random.randint(1, 10) // 100)
        picked = random.sample(range(1, n_vars + 1), ratio)
        lits  = [(-v if random.random() < 0.5 else v) for v in picked]
        query_lines.append(f"{kind} {' '.join(map(str, lits))} 0")
    path  = _write_tmp(text)
    qpath = _write_tmp("\n".join(query_lines) + "\n", suffix=".queries")
    return path, qpath


def gen_circuit(
    max_inputs: int = 20,
    max_gates: int = 100,
    polarity_prob: float = 0.5,
    or_prob: float = 0.3,
) -> tuple[str, None]:
    """Generate a random circuit in BC-S1.2 format."""
    gates, inputs, true_names = _gen_circuit_structure(
        max_inputs=max_inputs, max_gates=max_gates,
        polarity_prob=polarity_prob, or_prob=or_prob,
    )
    return _write_tmp(_circuit_to_bcs(gates, inputs, true_names), suffix=".bc"), None


def gen_circuit_files(files: list[str]) -> tuple[str, None]:
    """Pick a random file from the list of fixed instances."""
    return random.choice(files), None


# ---------------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------------

def generate(gen_cfg, _scripts_dir: str = "", _fuzz_dir: str = "") -> tuple[str, Optional[str]]:
    t  = gen_cfg.type
    mv = gen_cfg.max_vars
    ex = gen_cfg.extra
    if t == "cnf_plain":
        return gen_plain_cnf(mv)
    if t == "cnf_projected":
        return gen_projected_cnf(mv)
    if t == "cnf_weighted":
        return gen_weighted_cnf(mv)
    if t == "cnf_complex":
        return gen_complex_cnf(mv)
    if t == "cnf_maxsharpsat":
        return gen_maxsharpsat_cnf(mv)
    if t == "cnf_queries":
        return gen_cnf_with_queries(mv, gen_cfg.n_queries)
    if t == "circuit_gen":
        return gen_circuit(
            max_inputs=ex.get("max_inputs", 20),
            max_gates=ex.get("max_gates", 100),
            polarity_prob=ex.get("polarity_prob", 0.5),
            or_prob=ex.get("or_prob", 0.3),
        )
    if t == "circuit_files":
        return gen_circuit_files(gen_cfg.files)
    raise ValueError(f"Unknown generator type: {t!r}")
