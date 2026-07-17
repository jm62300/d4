# Code review report — d4 library (`src/`)

*Date: 2026-07-10 — reviewed at commit 190e18f1 (branch `test-enumImplicant`). No code was
modified; every item below is a finding to be checked and decided on individually.*

## Context

Full review of the d4 library core requested by the user. Excluded per instructions: the
parts of `src/methods/` still relying on old structures (MaxSharpSAT, MinSharpSAT, MaxT,
QbfCounter, ExistRandomExist, ProjMCMethod). `DpllStyleMethod` was reviewed since it is the
current engine. Vendored code (`src/solvers/cnf/minisat/`, glucose, patoh) was not reviewed.
Areas reviewed in depth: caching (cache managers, bucket allocator, cleaning, formula
stores), formulaManager (CnfManager, CnfManagerDyn, Pure/BlockedCl), solver wrappers,
branching/scoring heuristics, utils, options defaults, partitioner (skim).

Things checked and found **sound**: the `CnfManagerDyn` preUpdate/postUpdate undo mechanism
vs. `DataOccurrence` pointer arithmetic; the union-find connected-component computation; the
Cl/Sym/Index cache-key encodings (including the implicit zero-separator that makes the Cl
encoding unambiguous, and the Sym renaming being count-preserving); pure-literal and
blocked-clause elimination correctly restricted to non-decision variables; the decision-node
branching loop (exhaustive, disjoint, early-break on UNSAT prefix is sound); cache cleaning
mid-recursion does not invalidate pending `TmpEntry` allocations.

---

## High severity

### H1. Off-by-one heap overflow in the bitmap variable encoding (default config)
`src/formulaManager/cnf/FormulaStoreCnfCl.cpp:192` computes the bitmap size as
`1 + ((component.back() - 1) >> 3)`, but `storeVariables` (line 246) sets bit `v` via
`p[v >> 3] |= 1 << (v & 7)`. When the max var of the component is a multiple of 8, bit
`back` lands in byte `back/8`, one **past** the computed size (needs `1 + (back >> 3)`).
- With the default `modeStore = CACHE_NT` and a component with no touched clauses,
  `nbByteStoreFormula == 0`, so the stray `|=` writes one byte past the allocation returned
  by `BucketAllocator::getArray` — corrupting an adjacent cached bucket key or free block.
  This can silently produce wrong counts (a later lookup can match a corrupted key).
- When a formula section follows, the stray bit is cleared by `storeClauses`' memset, so the
  key merely omits the max var's bit — by luck not a collision source (the max var value is
  pinned by `szData`), but fragile.
Fix: `unsigned nbByteModeArray = 1 + (component.back() >> 3);`

### H2. Unsigned wrap in `CacheCleaningExpectation::reduceCache`
`src/caching/cleaning/CacheCleaningExpectation.hpp:112-130`. The loop
`for (unsigned i = limit; i < m_statVar.size(); i--)` decrements `limit` once per iteration;
if no size class satisfies `ratio > m_threshold` (typical early on: no positive hits yet),
`limit` is decremented `limit+1` times and wraps to `0xFFFFFFFF`. Then `limit *= 1.1`
overflows (double→unsigned conversion above `UINT_MAX` is UB) and the `removeEntry`
predicate `nbVar() >= limit` removes nothing: the cleaning pass is a no-op exactly when the
cache is under memory pressure. Combined with H3 this defeats the memory limit entirely.

### H3. `reinitComsumedMemory` raises the memory limit by ~5 GiB on every trigger
`src/caching/bucket/BucketAllocator.hpp:68-71`. `CacheManager::searchInCache` calls
`reduceCache()` + `reinitComsumedMemory()` whenever `getComsumedMemory()` is true — i.e.
every time a new page is allocated — and each call does `m_memoryLimit += 5000 MiB`.
After a handful of pages the RSS check in `isMemoryLimitReached()` can never fire again;
the process drifts toward OOM on hard instances. The increment should probably only happen
when the cleaning was actually triggered by the RSS limit (and/or be far smaller).

### H4. `ListLitAllocator` page overflow with large clauses
`src/heuristics/branchingHeuristic/BranchingHeuristic.hpp:80-104`. `askMemory` only
`assert`s `size < SIZE_PAGE_LIST_LIT` (1024). `BranchingHeuristicLargeArity::selectLitSet`
passes the literal set of the largest unsatisfied clause; a clause with >1024 literals
overflows a freshly allocated 1024-`Lit` page in release builds (and the follow-up
`SIZE_PAGE_LIST_LIT - m_posPage` wraps, poisoning the free list). Needs a real guard
(allocate oversized blocks separately, or clamp).

---

## Medium severity

### M1. Rotted dead code: `CacheList::createAndStoreBucket`
`src/caching/CacheList.hpp:125-130` calls `m_bucketManager->collectBuckect(varConnected)`
(method does not exist — `collectBucket` has a different signature) and passes a
`CachedBucket` to `computeHash(DataBucket&)`. It only compiles because the template member
is never instantiated. Remove it.

### M2. `BufferRead` open-failure check is wrong
`src/utils/BufferRead.hpp:57`: `open()` returns -1 on failure, not 0 — `if (!m_fd)` never
catches the error (and misfires for fd 0). The failure is only caught later by `read()`
with a confusing message. Use `if (m_fd < 0)`. Also: `nextDouble` can throw an uncaught
`std::invalid_argument` from `std::stod` on malformed input.

### M3. Huge up-front allocations from defaults
- `src/options/cache/OptionBucketManager.hpp:44`: `sizeFirstPage` defaults to `1UL << 32` —
  the cache allocates a single 4 GiB `new char[]` at construction. Fine for competition
  machines, hostile for a library default (`libd4.a` consumers, rust binding).
- `src/caching/CacheList.hpp:145`: `nodePool.reserve(SIZE_HASH)` reserves 16.7M
  `CollisionNode`s up front (~0.7–1 GiB depending on `T`).
- `src/caching/CacheNoCollision.hpp:33`: `SIZE_HASH = 220419970` slots →
  `hashTable.resize()` of several GiB at construction (opt-in, but undocumented).

### M4. Cache statistics drift
`CacheNoCollision::pushInHashTable` overwrites an occupied slot without decrementing
`m_nbEntry`; `CacheList::removeEntry` / `CacheNoCollision::removeEntry` never decrement it
either (`decrementNbEntry()` exists but is unused). `reduceCache` prints `#entries` from
this counter, so the reported numbers are wrong.

### M5. Uninitialized / dead members in `DpllStyleMethod`
`src/methods/DpllStyleMethod.hpp:56-57,66,74-78`: `optDomConst`, `optReversePolarity`,
`m_freqDecay` are never initialized (UB if ever read); `m_clauses`, `m_signLit`,
`computePrioritySubSet` are unused; there is a leftover `#if 0` block with a `static T`
local (lines 407-413). Cleanup recommended.

### M6. Library writes directly to stdout, bypassing the configured stream
- `src/caching/bucket/BucketAllocator.cpp:100`: `printf("c Allocate a new page ...")`.
- `src/caching/cleaning/CacheCleaningExpectation.hpp:139`: unconditional `std::cout` stats
  line on every reduce.
- `CnfManagerDynPure.cpp:32`, `CnfManagerDynBlockedCl.cpp:31`: banner to `std::cout`.
These matter for server mode and the rust binding (which had to silence a cout leak) and
ignore verbosity settings. Route through the `out` stream / verbosity flags.

### M7. `DataInfo` is polymorphic but treated as POD
`src/caching/DataInfo.hpp`: the class has a virtual destructor, so `getInfo()`'s
`(unsigned*)this` points at the vptr, not `info1` (currently dead code — only
`DataBucket::getInfo()` is used — but a trap). The vptr also doubles the size of every
`DataBucket`/`CachedBucket`. Nothing inherits from `DataInfo`: drop the `virtual` and
delete `getInfo`/`getSizeInfo`/`printData`.

---

## Low severity / notes

- **L1.** `FormulaStoreCnfSym::storeClauses` builds `m_mapVar` from *component order*
  (`FormulaStoreCnfSym.cpp:223`) instead of the canonical sorted-literal order it computes —
  soundness unaffected (still an injective renaming), but it forfeits most of the symmetry
  hit rate the encoder was designed for. The `storeVariables` template there is dead code.
- **L2.** `FormulaManager::makeFormulaManager` (`FormulaManager.cpp:41-63`): missing `break`
  after the `PB_CIRC` inner switch — a new enum value would silently fall through and build
  a CNF manager for a circuit problem. Same pattern worth a `default:` in other factories.
- **L3.** `BranchingHeuristic` constructor uses `quantification.back()` for decision vars
  while the factory and `DpllStyleMethod` use `quantification[0]` — inconsistent if a
  multi-block problem ever reaches it.
- **L4.** `DataInfo.nbVar` is a 22-bit field: a component with >4.19M variables silently
  truncates in release (assert-only guard) → possible header collisions. Document the limit
  or check at parse time.
- **L5.** `WrapperSolver::warmStart` never updates `nbSAT`; the "Warm start (n): 0/29" log
  is always zero. Also uses unseeded `rand()`.
- **L6.** `getPolarity`/`getModelVar` rely on the numeric coincidence `l_True == 0`
  (`ProblemTypes.hpp:32`) when converting `lbool`→`bool`/int across minisat and d4 types.
  Correct today, fragile under any re-encoding.
- **L7.** The per-level stamp counters in `CnfManagerDyn`
  (`m_currentMarkedLitStackIndex`, `m_currentMarkedLitRemoveIndex`, `m_stampNotSatClauses`)
  are 32-bit and, unlike `m_stampMarkView`, have no wrap handling. Only reachable after 2^32
  preUpdates — worth a comment or uint64.
- **L8.** `FormulaStoreCnfCombi` allocates a second `FormulaStoreCnfCl`
  (`clBucketManagerBis`, `FormulaStoreCnfCombi.cpp:29`) that is never used — each instance
  carries `O(nbClause)` arrays.
- **L9.** Dead guard `if (m_cache->MIN_NBVAR_NOTCACHED == 0) return;` in `reduceCache`
  (the constant is 100). `MAX_NBVAR_CACHED`/`MIN_NBVAR_NOTCACHED` naming vs. use is
  confusing.
- **L10.** `BucketAllocator::releaseMemory` computes `&m_data[m_posInData - size]` even when
  `size > m_posInData` (unsigned wrap → out-of-bounds pointer arithmetic, UB; benign in
  practice). Reorder the check.

---

## Proposed follow-up (if fixes are wanted)

1. Fix H1 (`1 + (component.back() >> 3)`) — one-line, then run
   `cd scripts && bash searchBadExitQuick.sh` after rebuilding `c++/counter`.
2. Fix H2 (guard the loop: stop at `i == 0` / recompute limit without wrap) and H3 (only
   raise `m_memoryLimit` when the RSS limit triggered the cleaning).
3. Fix H4 with a real size check in `askMemory` (oversized blocks via `new`).
4. Sweep M1–M7 (mechanical cleanups).
5. Verification: rebuild library + counter/compiler demos, run
   `scripts/testModelCounter.sh` on `instancesTest/cnfs/*` and the quick regression suite;
   for H1 specifically, a CNF whose component max var is a multiple of 8 with untouched
   clauses under `-DBUILD_MODE=1` + ASan would demonstrate the overflow before/after.
