# Work in Progress

This file is updated at the end of every session so the next session can resume cleanly.
Check `git log` for what was actually committed; this file tracks what is *next*.

---

## Active thread: Part 1 — Fuzzing tool (GUI + JSON)

### Goal
Replace the `scripts/searchBad*.sh` family with a single tool that:
- Is configured via a JSON file (one entry per test variant / method)
- Shows a live TUI while tests run (pass/fail/timeout counts per variant)
- Supports both CNF and circuit inputs (circuit support needed before Part 2/3 work)
- Can be run headless (CI / no terminal) by omitting the TUI flag

### Status
**Core working.** Headless mode tested and counting (~9 tests/5 s for plain_counting).

### What was built (`tools/fuzz/`)
- `schema.py` — JSON config dataclasses
- `generators.py` — Python ports of all lib.sh generator functions
- `oracles.py` — crash + correctness oracles
- `runner.py` — `SuiteRunner` thread with pause/stop/state snapshot
- `app.py` — textual TUI (DataTable + bug log + key bindings)
- `run.py` — CLI entry point (`--no-ui`, `--list`, `--suite`)
- `configs/default.json` — all 7 test suites (plain/projected/weighted/complex/maxsharpsat/crash/circuit)

### How to run
```bash
cd /home/lagniez/Works/Softs/d4
python3 tools/fuzz/run.py                        # TUI, default config
python3 tools/fuzz/run.py --no-ui                # headless
python3 tools/fuzz/run.py --suite plain_counting # single suite
python3 tools/fuzz/run.py --list                 # list suites
```

### Remaining work (Part 1)
- [ ] TUI: **test the textual UI interactively** (needs a real terminal, not a subprocess)
- [ ] TUI: `DataTable.update_cell` API may differ in textual 8.x — verify column key syntax
- [ ] Circuit random generation: currently uses fixed `.bc` files; add random circuit generator
  once bipe's `CnfToCircuit` exists (Part 2)
- [ ] `workers > 1` in headless mode: currently only starts `workers` runners, no queuing for
  suites beyond that limit. Simple to fix when needed.
- [ ] Commit to git once TUI is verified.

---

## Pending threads (not started)

### Part 2 — bipe CnfToCircuit
Context document drafted: `doc/bipe_cnf_to_circuit.md`.
Work happens in the bipe repo (`../bipe/`).
New session needed; share the doc to bootstrap.

### Part 3 — CircuitWithCnfManager gate-native CC + cache
User disagrees with the FormulaStore plan — revisit after Part 2 is clearer.
Files involved: `src/formulaManager/circuit/CircuitWithCnfManager.cpp`,
`src/formulaManager/cnf/CnfManager.cpp`, `src/problem/ProblemTypes.hpp`.

---

## Open questions / disagreements to resolve
- Part 3: what exactly should replace the `m_formulaStore` attribute?
  User wants CnfManager to expose its representation directly rather than delegate.
  Details TBD.
