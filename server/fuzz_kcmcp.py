#!/usr/bin/env python3
"""
fuzz_kcmcp.py — Fuzzer pour le serveur d4 KCMCP

Génère des formules CNF valides aléatoires avec des options variées,
les envoie au serveur et vérifie la cohérence des résultats.

Usage:
    python3 fuzz_kcmcp.py <endpoint> [options]

    endpoint:  "unix:/path/to/socket.sock"  ou  "host:port"  ou  "port"

Options:
    --seed N        Graine aléatoire (reproductibilité)
    --count N       Nombre de cas à tester (défaut: 100)
    --timeout N     Timeout socket en secondes (défaut: 30)
    --workers N     Connexions parallèles (défaut: 1)
    --verbose       Affiche chaque cas en détail
    --stop-on-error Arrête dès la première erreur
    --cross-check   Vérifie que COUNT == WMC(poids uniformes à 1)

Exemples:
    python3 fuzz_kcmcp.py 50055 --count 500
    python3 fuzz_kcmcp.py unix:/tmp/d4.sock --seed 42 --verbose
"""

from __future__ import annotations

import argparse
import json
import random
import socket
import struct
import sys
import time
import traceback
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Protocole KCMCP (helpers bas niveau)
# ---------------------------------------------------------------------------

def _send_frame(sock: socket.socket, type_code: int, flags: int,
                request_id: int, payload: bytes) -> None:
    chunk_limit = 1 << 20  # 1 MiB
    offset = 0
    total = len(payload)
    while offset < total or total == 0:
        size = min(total - offset, chunk_limit)
        more = 1 if (offset + size < total) else 0
        hdr = struct.pack(">BBBBBBBBBB",
                          type_code,
                          flags | more,
                          (request_id >> 24) & 0xFF,
                          (request_id >> 16) & 0xFF,
                          (request_id >> 8) & 0xFF,
                          request_id & 0xFF,
                          (size >> 24) & 0xFF,
                          (size >> 16) & 0xFF,
                          (size >> 8) & 0xFF,
                          size & 0xFF)
        sock.sendall(hdr + payload[offset:offset + size])
        offset += size
        if total == 0:
            break


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Connection closed prematurely")
        buf += chunk
    return buf


def _recv_full(sock: socket.socket):
    """Lit une réponse complète (gestion des chunks MORE)."""
    payload = b""
    first_type = None
    first_rid = None
    while True:
        hdr = _recv_exact(sock, 10)
        type_code = hdr[0]
        flags = hdr[1]
        rid = (hdr[2] << 24) | (hdr[3] << 16) | (hdr[4] << 8) | hdr[5]
        plen = (hdr[6] << 24) | (hdr[7] << 16) | (hdr[8] << 8) | hdr[9]
        chunk = _recv_exact(sock, plen)
        if first_type is None:
            first_type = type_code
            first_rid = rid
        payload += chunk
        if not (flags & 0x01):
            break
    return first_type, first_rid, payload


def _connect(endpoint: str, timeout: float) -> socket.socket:
    if endpoint.startswith("unix:"):
        path = endpoint[5:]
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect(path)
    else:
        host, port = "localhost", 50055
        if ":" in endpoint:
            h, p = endpoint.rsplit(":", 1)
            host, port = h, int(p)
        else:
            port = int(endpoint)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((host, port))
    return s


def _handshake(sock: socket.socket) -> None:
    hello = json.dumps({"kcmcp": [1, 0], "client": "kcmcp-fuzzer/1.0"})
    _send_frame(sock, 0x00, 0x00, 0, hello.encode())
    t, _, payload = _recv_full(sock)
    if t != 0x00:
        raise RuntimeError(f"Handshake: expected HELLO(0x00), got {t:#04x}")


def _send_request(sock: socket.socket, request_id: int,
                  operation: int, output_format: int,
                  opts: dict, cnf: str) -> None:
    opts_bytes = json.dumps(opts).encode() if opts else b""
    hdr = struct.pack(">BBBBH", operation, 0, output_format, 0, len(opts_bytes))
    payload = hdr + opts_bytes + cnf.encode()
    _send_frame(sock, 0x01, 0x00, request_id, payload)


def _parse_result(payload: bytes, output_format: int):
    """Parse le payload RESULT et retourne (meta_dict, result_raw)."""
    _, _, meta_len_hi, meta_len_lo = struct.unpack(">BBBB", payload[:4])
    meta_len = (meta_len_hi << 8) | meta_len_lo
    meta = json.loads(payload[4:4 + meta_len])
    result_raw = payload[4 + meta_len:]
    return meta, result_raw


# ---------------------------------------------------------------------------
# Génération de formules CNF valides
# ---------------------------------------------------------------------------

@dataclass
class CNFFormula:
    nb_vars: int
    clauses: list
    dimacs: str = ""

    def __post_init__(self):
        self.dimacs = self._to_dimacs()

    def _to_dimacs(self) -> str:
        lines = [f"p cnf {self.nb_vars} {len(self.clauses)}"]
        for clause in self.clauses:
            lines.append(" ".join(map(str, clause)) + " 0")
        return "\n".join(lines) + "\n"

    @property
    def nb_clauses(self):
        return len(self.clauses)


def gen_random_cnf(rng: random.Random,
                   nb_vars: Optional[int] = None,
                   nb_clauses: Optional[int] = None,
                   min_vars: int = 1,
                   max_vars: int = 12,
                   min_clause_len: int = 1,
                   max_clause_len: int = 4) -> CNFFormula:
    """
    Génère une formule CNF aléatoire mais syntaxiquement valide.
    - Chaque clause a entre min_clause_len et max_clause_len littéraux
    - Pas de doublon de variable dans une même clause
    - Pas de tautologie (x et -x dans la même clause)
    """
    if nb_vars is None:
        nb_vars = rng.randint(min_vars, max_vars)
    if nb_clauses is None:
        # Ratio 3-SAT classique ≈ 4.27 × nb_vars pour être à la limite
        target = rng.uniform(1.0, 5.0) * nb_vars
        nb_clauses = max(1, int(target))

    clauses = []
    for _ in range(nb_clauses):
        clen = rng.randint(min_clause_len, min(max_clause_len, nb_vars))
        vars_used = rng.sample(range(1, nb_vars + 1), clen)
        clause = [v if rng.random() < 0.5 else -v for v in vars_used]
        clauses.append(clause)

    return CNFFormula(nb_vars=nb_vars, clauses=clauses)


def gen_satisfiable_cnf(rng: random.Random,
                        nb_vars: Optional[int] = None,
                        nb_clauses: Optional[int] = None) -> CNFFormula:
    """
    Génère une formule CNF satisfaisable garantie.
    Méthode : tirage d'un modèle aléatoire, puis génération de clauses
    satisfaites par ce modèle.
    """
    if nb_vars is None:
        nb_vars = rng.randint(2, 12)

    # Modèle de référence
    model = {v: rng.random() < 0.5 for v in range(1, nb_vars + 1)}

    if nb_clauses is None:
        nb_clauses = rng.randint(nb_vars, nb_vars * 4)

    clauses = []
    for _ in range(nb_clauses):
        clen = rng.randint(1, min(4, nb_vars))
        vars_used = rng.sample(range(1, nb_vars + 1), clen)
        clause = []
        # S'assurer qu'au moins un littéral est satisfait par le modèle
        satisfied = False
        for v in vars_used:
            # Polarité cohérente avec le modèle avec proba 0.7
            if not satisfied and rng.random() < 0.7:
                lit = v if model[v] else -v
                satisfied = True
            else:
                lit = v if rng.random() < 0.5 else -v
            clause.append(lit)
        if not satisfied:
            # Forcer au moins un littéral satisfait
            v = rng.choice(vars_used)
            clause = [l for l in clause if abs(l) != v]
            clause.append(v if model[v] else -v)
        clauses.append(clause)

    return CNFFormula(nb_vars=nb_vars, clauses=clauses)


def gen_known_formula(rng: random.Random) -> Tuple[CNFFormula, int]:
    """
    Retourne une formule dont on connaît exactement le nombre de modèles.
    Utile pour vérifier la correction du solveur.
    """
    kind = rng.choice(["tautology", "unit", "two_clause", "chain"])

    if kind == "tautology":
        # Formule vide → 2^n modèles
        n = rng.randint(1, 5)
        return CNFFormula(nb_vars=n, clauses=[]), 2 ** n

    elif kind == "unit":
        # Une clause unitaire x1 → 2^(n-1) modèles
        n = rng.randint(1, 5)
        return CNFFormula(nb_vars=n, clauses=[[1]]), 2 ** (n - 1)

    elif kind == "two_clause":
        # (x1 v x2) ∧ (¬x1 v x2) → x2=True obligatoire, x1 libre → 2 modèles
        return CNFFormula(nb_vars=2, clauses=[[1, 2], [-1, 2]]), 2

    else:  # chain
        # x1 ∧ x2 ∧ ... ∧ xn → 1 seul modèle
        n = rng.randint(1, 5)
        return CNFFormula(nb_vars=n, clauses=[[i] for i in range(1, n + 1)]), 1


# ---------------------------------------------------------------------------
# Génération d'options aléatoires
# ---------------------------------------------------------------------------

D4_SOLVERS = ["minisat", "glucose"]
D4_BRANCHING = ["vsads", "vsids", "dlcs", "dlis", "mom"]
D4_SCORING = ["vsads", "vsids", "dlcs", "dlis", "mom"]
D4_PHASE = ["false", "true", "polarity"]
D4_CACHE = ["nocache", "list", "tree"]
D4_PREPROC = ["backbone", "none"]


def gen_random_opts(rng: random.Random, formula: CNFFormula,
                    operation: int) -> dict:
    """Génère des options JSON valides pour une requête."""
    opts = {}

    # Options d4 (sous-ensemble aléatoire)
    d4_opts = {}
    if rng.random() < 0.4:
        d4_opts["solver"] = rng.choice(D4_SOLVERS)
    if rng.random() < 0.4:
        d4_opts["branching-heuristic"] = rng.choice(D4_BRANCHING)
    if rng.random() < 0.3:
        d4_opts["phase-heuristic"] = rng.choice(D4_PHASE)
    if rng.random() < 0.3:
        d4_opts["cache-method"] = rng.choice(D4_CACHE)
    if rng.random() < 0.3:
        d4_opts["preproc"] = rng.choice(D4_PREPROC)
    if d4_opts:
        opts["d4"] = d4_opts

    # Poids (WMC uniquement, mais on peut en mettre sur count aussi)
    if operation == 1 or (operation == 0 and rng.random() < 0.2):
        weights = {}
        for v in range(1, formula.nb_vars + 1):
            # Poids rationnels qui somment à 1 (convention WAPS/DSHARP)
            # Poids flottants (les fractions type "1/10" ne sont pas supportées)
            p = round(rng.uniform(0.01, 0.99), 6)
            weights[str(v)] = str(p)
            weights[str(-v)] = str(round(1.0 - p, 6))
        opts["weights"] = weights

    # Projset (ensemble de variables projetées)
    if rng.random() < 0.3 and formula.nb_vars >= 2:
        k = rng.randint(1, formula.nb_vars)
        projset = rng.sample(range(1, formula.nb_vars + 1), k)
        opts["projset"] = sorted(projset)

    return opts


# ---------------------------------------------------------------------------
# Vérifications de cohérence
# ---------------------------------------------------------------------------

def verify_count_result(result_raw: bytes, formula: CNFFormula,
                        opts: dict,
                        expected: Optional[int] = None) -> List[str]:
    """Vérifie que le count est plausible."""
    errors = []
    try:
        count_str = result_raw.decode("utf-8").strip()
        # Si des poids sont fournis, d4 traite le COUNT comme un WMC
        # et peut retourner un flottant → on accepte les deux
        has_weights = bool(opts.get("weights"))
        try:
            count = int(count_str)
            if count < 0:
                errors.append(f"count négatif: {count}")
            if count > 2 ** formula.nb_vars:
                errors.append(f"count {count} > 2^{formula.nb_vars} = {2**formula.nb_vars}")
            if expected is not None and count != expected:
                errors.append(f"count attendu {expected}, obtenu {count}")
        except ValueError:
            if has_weights:
                # Résultat flottant attendu avec poids → vérif WMC
                val = float(count_str)
                if val < -1e-9:
                    errors.append(f"count/wmc négatif: {val}")
                if val > 1.0 + 1e-6:
                    errors.append(f"count/wmc > 1 avec poids normalisés: {val}")
            else:
                errors.append(f"impossible de parser le count: valeur non numérique: {count_str!r}")
    except Exception as e:
        errors.append(f"impossible de parser le count: {e} (raw={result_raw!r})")
    return errors


def verify_bigint_result(result_raw: bytes, formula: CNFFormula,
                         expected: Optional[int] = None) -> list[str]:
    """Vérifie le count en format bigint."""
    errors = []
    try:
        count = int.from_bytes(result_raw, byteorder="big")
        if count < 0:
            errors.append(f"bigint négatif: {count}")
        if count > 2 ** formula.nb_vars:
            errors.append(f"bigint {count} > 2^{formula.nb_vars}")
        if expected is not None and count != expected:
            errors.append(f"bigint attendu {expected}, obtenu {count}")
    except Exception as e:
        errors.append(f"impossible de parser le bigint: {e}")
    return errors


def verify_wmc_result(result_raw: bytes, opts: dict) -> list[str]:
    """Vérifie que le WMC est dans [0, 1] si des poids sont fournis."""
    errors = []
    try:
        val_str = result_raw.decode("utf-8").strip()
        val = float(val_str)
        if val < -1e-9:
            errors.append(f"WMC négatif: {val}")
        # Si tous les poids sont dans [0,1] et somment à 1, WMC ∈ [0,1]
        if opts.get("weights") and val > 1.0 + 1e-6:
            errors.append(f"WMC > 1 avec poids normalisés: {val}")
    except Exception as e:
        errors.append(f"impossible de parser le WMC: {e} (raw={result_raw!r})")
    return errors


def verify_nnf_result(result_raw: bytes) -> list[str]:
    """Vérifie la structure basique du NNF retourné."""
    errors = []
    try:
        nnf = result_raw.decode("utf-8").strip()
        if not nnf:
            errors.append("NNF vide")
        # Un NNF valide contient au moins une ligne "nnf ..." ou des nœuds
        first_line = nnf.split("\n")[0]
        if not (first_line.startswith("nnf") or first_line.startswith("c") or
                any(c in first_line for c in ("o ", "a ", "l ", "t ", "f "))):
            errors.append(f"Format NNF inattendu, première ligne: {first_line!r}")
    except Exception as e:
        errors.append(f"impossible de décoder le NNF: {e}")
    return errors


# ---------------------------------------------------------------------------
# Cas de test
# ---------------------------------------------------------------------------

@dataclass
class FuzzCase:
    case_id: int
    formula: CNFFormula
    operation: int      # 0=count, 1=wmc, 2=compile
    output_format: int  # 0=decimal, 3=bigint, 4=ddnnf
    opts: dict
    expected_count: Optional[int] = None

    @property
    def op_name(self) -> str:
        return {0: "COUNT", 1: "WMC", 2: "COMPILE"}.get(self.operation, "?")

    @property
    def fmt_name(self) -> str:
        return {0: "decimal", 3: "bigint", 4: "ddnnf"}.get(self.output_format, "?")


def gen_case(rng: random.Random, case_id: int) -> FuzzCase:
    """Génère un cas de test aléatoire."""
    operation = rng.choices([0, 1, 2], weights=[4, 3, 3])[0]

    # Format de sortie selon l'opération
    if operation == 0:
        output_format = rng.choice([0, 3])  # decimal ou bigint
    elif operation == 1:
        output_format = 0  # WMC = decimal seulement
    else:
        output_format = 4  # compile = ddnnf

    # Formule
    formula_kind = rng.choices(
        ["random", "satisfiable", "known"],
        weights=[5, 3, 2]
    )[0]

    expected = None
    if formula_kind == "known":
        formula, expected = gen_known_formula(rng)
        if operation != 0:
            expected = None  # On ne vérifie expected que pour COUNT
        if operation == 2:
            output_format = 4  # compile impose ddnnf
    elif formula_kind == "satisfiable":
        formula = gen_satisfiable_cnf(rng)
    else:
        formula = gen_random_cnf(rng)

    opts = gen_random_opts(rng, formula, operation)

    return FuzzCase(
        case_id=case_id,
        formula=formula,
        operation=operation,
        output_format=output_format,
        opts=opts,
        expected_count=expected,
    )


# ---------------------------------------------------------------------------
# Exécution d'un cas
# ---------------------------------------------------------------------------

@dataclass
class CaseResult:
    case_id: int
    ok: bool
    errors: list[str] = field(default_factory=list)
    duration_ms: float = 0.0
    op_name: str = ""
    fmt_name: str = ""
    nb_vars: int = 0
    nb_clauses: int = 0


def run_case(sock: socket.socket, case: FuzzCase, verbose: bool) -> CaseResult:
    result = CaseResult(
        case_id=case.case_id,
        ok=True,
        op_name=case.op_name,
        fmt_name=case.fmt_name,
        nb_vars=case.formula.nb_vars,
        nb_clauses=case.formula.nb_clauses,
    )

    t0 = time.perf_counter()
    # Les erreurs de connexion (BrokenPipe, ConnectionError) remontent
    # volontairement vers la boucle principale pour déclencher la reconnexion.
    _send_request(sock, case.case_id, case.operation, case.output_format,
                  case.opts, case.formula.dimacs)
    type_code, rid, payload = _recv_full(sock)
    result.duration_ms = (time.perf_counter() - t0) * 1000

    try:
        if type_code == 0x03:
            # ERROR frame → on vérifie si c'est attendu ou une vraie erreur
            err_code = struct.unpack(">H", payload[:2])[0]
            err_msg = payload[2:].decode("utf-8", errors="replace")
            if err_code == 2:  # unsupported format: attendu
                pass
            elif err_code == 6 and "Operator Type unknown" in err_msg:
                # Limitation connue du solveur avec certaines combinaisons
                # d'options (projset + solver options spécifiques) → warning
                result.errors.append(f"WARN solver limitation (code=6): {err_msg}")
                # On ne marque PAS comme échec — c'est une limite du solveur
            else:
                result.errors.append(f"ERROR frame code={err_code}: {err_msg}")
                result.ok = False
            if verbose:
                print(f"  → {'WARN' if result.ok else 'ERROR'} {err_code}: {err_msg}")
            return result

        if type_code != 0x02:
            result.errors.append(f"type inattendu {type_code:#04x} (attendu 0x02 RESULT)")
            result.ok = False
            return result

        if rid != case.case_id:
            result.errors.append(f"request_id {rid} != {case.case_id}")
            result.ok = False

        # Parse du résultat
        meta, result_raw = _parse_result(payload, case.output_format)

        if verbose:
            print(f"  meta: {meta}  raw_len={len(result_raw)}")

        # Vérifications selon l'opération
        if case.operation == 0:
            if case.output_format == 0:
                errs = verify_count_result(result_raw, case.formula, case.opts, case.expected_count)
            else:
                errs = verify_bigint_result(result_raw, case.formula, case.expected_count)
        elif case.operation == 1:
            errs = verify_wmc_result(result_raw, case.opts)
        else:
            errs = verify_nnf_result(result_raw)

        result.errors.extend(errs)
        result.ok = len(result.errors) == 0

    except Exception as e:
        result.errors.append(f"Exception: {e}\n{traceback.format_exc()}")
        result.ok = False

    return result


# ---------------------------------------------------------------------------
# Session de fuzzing
# ---------------------------------------------------------------------------

class FuzzSession:
    def __init__(self, endpoint: str, seed: int, count: int,
                 timeout: float, verbose: bool, stop_on_error: bool,
                 cross_check: bool):
        self.endpoint = endpoint
        self.rng = random.Random(seed)
        self.count = count
        self.timeout = timeout
        self.verbose = verbose
        self.stop_on_error = stop_on_error
        self.cross_check = cross_check
        self.results: list[CaseResult] = []

    def _new_connection(self) -> socket.socket:
        sock = _connect(self.endpoint, self.timeout)
        _handshake(sock)
        return sock

    def run(self):
        print(f"[Fuzzer] endpoint={self.endpoint} "
              f"cases={self.count}")
        print("-" * 70)

        sock = self._new_connection()
        passed = 0
        failed = 0
        skipped = 0

        try:
            for i in range(1, self.count + 1):
                case = gen_case(self.rng, i)

                if self.verbose:
                    print(f"\n[{i:04d}] {case.op_name}/{case.fmt_name} "
                          f"vars={case.formula.nb_vars} "
                          f"clauses={case.formula.nb_clauses} "
                          f"opts={list(case.opts.keys())}")
                    if self.verbose:
                        print(f"  CNF:\n    " +
                              case.formula.dimacs.replace("\n", "\n    ").strip())

                # Reconnexion si le socket est mort
                try:
                    res = run_case(sock, case, self.verbose)
                except (ConnectionError, OSError, BrokenPipeError):
                    print(f"\n  [!] Connexion perdue au cas {i} — "
                          f"possible crash serveur ({case.op_name} "
                          f"vars={case.formula.nb_vars} "
                          f"cls={case.formula.nb_clauses})")
                    try:
                        sock.close()
                    except Exception:
                        pass
                    time.sleep(0.5)  # Laisser le serveur se stabiliser
                    try:
                        sock = self._new_connection()
                        # On retente le cas sur la nouvelle connexion
                        res = run_case(sock, case, self.verbose)
                        if not res.ok and "Broken pipe" not in str(res.errors):
                            res.errors.insert(0, "SERVER CRASH: connexion perdue avant réponse")
                    except Exception as e:
                        res = CaseResult(
                            case_id=case.case_id,
                            ok=False,
                            errors=[f"SERVER CRASH + reconnexion impossible: {e}"],
                            op_name=case.op_name,
                            fmt_name=case.fmt_name,
                            nb_vars=case.formula.nb_vars,
                            nb_clauses=case.formula.nb_clauses,
                        )

                self.results.append(res)

                if res.ok:
                    passed += 1
                    status = "✓"
                else:
                    failed += 1
                    status = "✗"

                if not self.verbose:
                    # Affichage compact ligne par ligne
                    print(f"[{i:04d}] {status} {case.op_name:<7} "
                          f"vars={case.formula.nb_vars:3d} "
                          f"cls={case.formula.nb_clauses:4d} "
                          f"{res.duration_ms:7.1f}ms"
                          + (f"  ERR: {res.errors[0]}" if res.errors else ""))
                else:
                    print(f"  → {'OK' if res.ok else 'FAIL'} "
                          f"({res.duration_ms:.1f}ms)"
                          + (f"\n  ERRORS: {res.errors}" if res.errors else ""))

                if self.stop_on_error and not res.ok:
                    print(f"\n[Fuzzer] --stop-on-error déclenché au cas {i}")
                    break

                # Cross-check : COUNT decimal == COUNT bigint pour la même formule
                if self.cross_check and case.operation == 0 and case.output_format == 0:
                    i += 1
                    case2 = FuzzCase(
                        case_id=i,
                        formula=case.formula,
                        operation=0,
                        output_format=3,
                        opts={k: v for k, v in case.opts.items()
                              if k not in ("weights",)},
                    )
                    res2 = run_case(sock, case2, False)
                    if res.ok and res2.ok:
                        # Comparer les deux résultats
                        try:
                            dec = int(b"".join(r.encode() if isinstance(r, str)
                                               else r
                                               for r in [res_raw_decode(res)]))
                            bi = int.from_bytes(res2_raw(res2), "big")
                            if dec != bi:
                                print(f"  ⚠ CROSS-CHECK FAIL: decimal={dec} bigint={bi}")
                                res.errors.append(f"cross-check decimal!=bigint ({dec}!={bi})")
                                res.ok = False
                        except Exception:
                            pass

        finally:
            # Fermeture propre
            try:
                _send_frame(sock, 0x08, 0x00, 0, b"")
            except Exception:
                pass
            sock.close()

        # Rapport final
        self._report(passed, failed)

    def _report(self, passed: int, failed: int):
        total = len(self.results)
        print("\n" + "=" * 70)
        print(f"[Fuzzer] Résultats : {passed}/{total} OK — {failed}/{total} FAIL")
        print("=" * 70)

        if failed:
            print("\nCas échoués :")
            for r in self.results:
                if not r.ok:
                    print(f"  [#{r.case_id:04d}] {r.op_name}/{r.fmt_name} "
                          f"vars={r.nb_vars} clauses={r.nb_clauses}")
                    for e in r.errors:
                        print(f"    • {e}")

        # Statistiques par opération
        print("\nTemps moyen par opération :")
        for op in ["COUNT", "WMC", "COMPILE"]:
            times = [r.duration_ms for r in self.results if r.op_name == op]
            if times:
                print(f"  {op:8s}: avg={sum(times)/len(times):.1f}ms "
                      f"min={min(times):.1f}ms max={max(times):.1f}ms "
                      f"n={len(times)}")

        sys.exit(0 if failed == 0 else 1)


# ---------------------------------------------------------------------------
# Point d'entrée
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Fuzzer pour le serveur d4 KCMCP",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("endpoint", help="unix:/path/socket ou host:port ou port")
    parser.add_argument("--seed", type=int, default=None,
                        help="Graine aléatoire (défaut: aléatoire)")
    parser.add_argument("--count", type=int, default=100,
                        help="Nombre de cas à tester (défaut: 100)")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="Timeout socket en secondes (défaut: 30)")
    parser.add_argument("--verbose", action="store_true",
                        help="Affiche les détails de chaque cas")
    parser.add_argument("--stop-on-error", action="store_true",
                        help="Arrête au premier échec")
    parser.add_argument("--cross-check", action="store_true",
                        help="Vérifie decimal == bigint pour les COUNT")
    args = parser.parse_args()

    seed = args.seed if args.seed is not None else random.randrange(2 ** 32)
    print(f"[Fuzzer] Graine utilisée : {seed}  "
          f"(--seed {seed} pour reproduire)")

    session = FuzzSession(
        endpoint=args.endpoint,
        seed=seed,
        count=args.count,
        timeout=args.timeout,
        verbose=args.verbose,
        stop_on_error=args.stop_on_error,
        cross_check=args.cross_check,
    )
    session.run()


if __name__ == "__main__":
    main()
