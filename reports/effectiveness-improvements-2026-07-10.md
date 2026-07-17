# Improving the effectiveness of d4 — discussion report

*Date: 2026-07-10 — companion to `code-review-2026-07-10.md`. Nothing here is implemented;
each item is a proposal to discuss, with expected gain / effort / risk. All the analysis
targets the **default configuration**, which is the one actually used:*

| Component | Default |
|---|---|
| SAT solver | Glucose (`solverName = GLUCOSE_CNF`), CaDiCaL fallback on budget |
| Scoring | VSADS, decay every 95 calls |
| Phase | polarity (last model) |
| Branching | `BRANCHING_HYBRID_PARTIAL_CLASSIC` (VSADS + tree-decomposition partial order) |
| Formula manager | `CnfManagerDyn` (`SPEC_DYNAMIC`, `needFastNotSatisfied = false`) |
| Cache | `CACHE_LIST`, cleaning `CACHE_EXPECTATION` |
| Key encoding | `CACHE_CLAUSE` (`FormulaStoreCnfCl`) with `modeStore = CACHE_NT` (untouched) |

## 0. Prerequisite: fix the correctness/memory items first

Benchmarking on the current default path is unreliable until **H1** (one-byte overflow in
the `FormulaStoreCnfCl` bitmap, triggered precisely in NT mode with untouched components)
and **H2/H3** (cache cleaning that becomes a no-op while the memory limit inflates by
5 GiB per page) are fixed. H2/H3 are themselves effectiveness items: with them fixed, the
cache genuinely adapts to memory pressure instead of drifting toward OOM on hard instances.

---

## 1. Incremental construction of the cache entries (main proposal)

### 1.1 What a cache query costs today (default NT path)

Every `searchInCache` call, i.e. **every component at every search node**, does the
following from scratch (`FormulaStoreCnfCl::storeFormula`):

1. `collectDistrib`: for each unassigned variable of the component, for both literals,
   traverse the whole not-binary occurrence list (`getVecIdxClause(l, CACHE_NT)`), test
   `isKeptClause` (`nbUnsat > 0`), and run the bucket-sort machinery that assigns
   canonical clause numbers. Cost: **O(Σ |occ(l)|)** over the component — even though in
   NT mode the encoded clauses are only the *touched* ones (usually a small fraction).
2. `computeNeededBytes` + `storeVariables` + `storeClauses`: bit-pack the variable list
   and the touched-clause residuals (`addElementInData` works bit by bit).
3. rapidhash over the bytes, then `memcmp` on candidate hits.

So the dominant cost is step 1's full occurrence-list sweep, paid to discover a touched
set that the formula manager *already knows incrementally*.

### 1.2 The information is already maintained incrementally

`CnfManagerDyn::propagateFalseInNotBin` pushes into `m_savedStateClauses` exactly the
clauses whose `nbUnsat` changes at each level (first touch per level is recorded, and
`postUpdate` restores in stack order). In other words, the *delta of the touched set* per
decision level already exists; it is simply thrown away instead of being exploited by the
cache-key construction.

### 1.3 Option A — maintain a global "touched clauses" list (low risk, incremental sweep)

Maintain, next to `m_notSatifiedClauses`, a compact list/set of clauses with
`nbUnsat > 0 && !isSat`, updated in `preUpdate`/`postUpdate` from the deltas above (the
same swap-with-last + stamp technique used by `removeSatisfiedClauses` works; restoration
is O(delta) because updates are stack-shaped).

`storeFormula` then iterates **only the touched clauses of the component** (filter by the
watched variable's membership, as `getCurrentClausesNotBin` already does with
`m_inCurrentComponent`) instead of sweeping all occurrence lists.

- Consequence for the key: clauses are discovered in list order, not in the canonical
  per-literal order, so either (a) keep the bucket-sort canonicalization but feed it only
  touched clauses (same keys as today, pure speedup), or (b) emit residuals sorted by a
  cheap canonical criterion. Variant (a) is the safe first step: identical keys, strictly
  less work.
- Expected gain: turns the per-query cost from O(Σ occ) into
  O(#touched-in-component × residual size). On instances with long occurrence lists
  (large arity, many clauses per variable) this should be a large constant-factor win.
- Effort: moderate (one new incremental structure + a filter loop).
- Risk: low — the touched-set invariant is easy to assert against the current sweep in
  debug mode (run both, compare).

### 1.4 Option B — incremental (Zobrist-style) hashing, lazy byte materialization

Go one step further: maintain the **hash** of the state incrementally instead of
recomputing bytes to hash them.

- Assign each (clause, residual-size) — or (clause, set-of-falsified-literal-slots) — a
  precomputed random 64-bit code, and each variable a random code. Maintain per-clause
  current codes updated when `nbUnsat` changes (O(delta) in `preUpdate`/`postUpdate`,
  again piggybacking on `m_savedStateClauses`).
- A component's lookup hash = combination (XOR/sum) of its variables' codes and its
  touched clauses' current codes: computable in **O(|component| + #touched)** with no
  occurrence-list traversal and no byte writing at all.
- Bytes are then needed only (a) to `memcmp`-confirm a candidate hit and (b) to insert on
  a miss. Since most queries are misses on hard instances, keep exact behaviour by
  building the byte representation *only on insert and on hash-hit confirmation* —
  everything else becomes hash-only.
- The radical version (what sharpSAT/ganak do): drop the stored bytes entirely and rely on
  a wide hash (e.g. two independent 64-bit hashes) — probabilistic exactness, but the
  cache memory per entry collapses to ~16 bytes + value, and H-style memory pressure
  problems mostly disappear. This changes the guarantee of the tool, so it would have to
  be an explicit option (`--cache-probabilistic`), not the default.
- Effort: significant; risk: moderate (correct undo of hash deltas is the tricky part, but
  it is exactly stack-shaped like everything else in `CnfManagerDyn`).

### 1.5 Option C — reuse across sibling branches

`computeDecisionNode` explores `l` then `~l` on the same component; the sub-components on
the two sides often overlap heavily. A DataBucket built for a component could be kept
attached to the search node and *patched* (variables removed, clauses touched) rather than
rebuilt. This is the most intrusive variant and probably only worth discussing after A/B:
the same benefit is largely obtained by making the per-query cost proportional to the
delta (Option B).

### 1.6 Suggested order

A first (pure speedup, identical keys, easy to validate), then B's "incremental hash +
lazy bytes" (same exact semantics), and keep B-radical (probabilistic) as an optional
mode. Measure between each step (see §5).

---

## 2. Cache: hit-rate and memory levers (default stack)

- **Cleaning policy**: after fixing H2/H3, revisit `CacheCleaningExpectation`: the
  positive/negative statistics are only aggregated per component size (`m_statVar`), and
  the threshold only ever increases (`m_threshold += 0.1` per call, never decreases), so
  the policy becomes monotonically more aggressive regardless of observed behaviour.
  An LRU-flavoured criterion (per-entry last-hit timestamp already fits in
  `CachedBucket`) or resetting the threshold when the hit ratio recovers are cheap
  experiments.
- **Entry layout (review M7)**: dropping the vptr from `DataInfo` shrinks every
  `DataBucket`/`CachedBucket` by 8 bytes and makes `CollisionNode` tighter — more entries
  per GiB and better locality on the `memcmp`/probe path.
- **First page (review M3)**: with a 4 GiB first page the allocator never signals
  `getComsumedMemory()` until 4 GiB are used, so cleaning statistics start very late.
  Smaller pages (e.g. 256 MiB) give the cleaning policy earlier feedback; worth measuring
  rather than assuming.
- **Word-level bit-packing**: `addElementInData` writes bit by bit through a byte pointer;
  packing into a local `uint64_t` accumulator and flushing whole words is a small,
  self-contained speedup of `storeClauses` (relevant even after §1.3, less relevant after
  §1.4 since bytes become rare).

## 3. Sym/Combi representations (opt-in today, but cheap wins available)

Not on the default path, but two small changes would make them worth benchmarking as
alternatives:

- **Fix the Sym canonicalization (review L1)**: `FormulaStoreCnfSym` computes a canonical
  literal order and then doesn't use it for the renaming (`m_mapVar` is filled in
  component order). One line to change; without it the sym cache almost never catches the
  isomorphisms it is designed for.
- With that fixed, try `CACHE_COMBI` with a raised `limitVarSym` (default 20 is very
  conservative): small components are exactly where isomorphic repetition is frequent, and
  they dominate the number of cache queries.

## 4. Search-side levers

- **Warm start is unmeasured (review L5)**: `warmStart` runs 29 × 500-conflict random
  queries but its success counter is never updated, so there is no data on whether it
  helps. Instrument it (and make iterations/size options) before tuning anything else on
  the solver side.
- **CaDiCaL fallback tuning**: `initBudget = 500` conflicts and `minLimitVar = 50` decide
  when Glucose gives up and CaDiCaL takes over; `cadicalRedundantFactor` decides when
  CaDiCaL is rebuilt from scratch. These three interact and were presumably hand-picked;
  a small grid on the benchmark set would tell whether the fallback fires too
  early/late. Adding counters (how often the fallback triggers, how often it flips
  UNDEF→SAT) costs nothing.
- **Partial-order weight**: in the default hybrid branching the score is
  `VSADS(v) + partialOrder(v)` with `scaleFactor = 0`; the balance between the two terms
  is fixed once at startup. Exposing the mix (or decaying the partial-order term as the
  decomposition ages relative to the residual formula) is an easy experiment.
- **Connected components**: the adaptive on/off switch (stop after 11 stale checks every
  100k calls, re-check every 500 calls) is coarse-grained and instance-global. A
  component-size-conditioned policy (always decompose above a size threshold, skip below)
  would be more predictable; the statistics to decide are already collected (`m_nbSplit`).

## 5. Measurement infrastructure (needed to decide any of the above)

1. A benchmark harness: fixed instance set (e.g. `instancesTest/cnfs` plus a selection of
   competition instances), fixed seed, reporting time, #recursive calls, #pos/#neg hits,
   cache memory, and peak RSS per instance — so any change above gets a before/after table
   instead of an impression.
2. A profiling build run (`./build.sh -p`) on 3–4 representative hard instances to confirm
   the §1.1 cost breakdown before investing in Option B.
3. An ASan/UBSan run of `scripts/searchBadExitQuick.sh` in CI — it would have caught H1
   and H2 mechanically, and it protects every experiment above.

---

## Summary table

| Idea | Expected gain | Effort | Risk |
|---|---|---|---|
| Fix H1–H3 first | correctness + memory adaptation | small | low |
| §1.3 touched-list sweep (A) | large constant factor on cache queries | moderate | low |
| §1.4 incremental hash + lazy bytes (B) | removes encoding from the hot path | high | moderate |
| §1.4 probabilistic mode (opt-in) | ~10× less cache memory | high | changes guarantee |
| §2 cleaning policy rework | fewer wrong evictions under pressure | moderate | low |
| §2 entry layout / M7 | memory + locality | small | low |
| §3 Sym fix + Combi benchmark | hit rate on symmetric instances | small | low |
| §4 solver/branching tuning | instance-dependent | small | low |
| §5 harness + sanitizer CI | enables all of the above | small | none |
