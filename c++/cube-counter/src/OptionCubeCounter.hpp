#pragma once

#include <iostream>
#include <string>

#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"

namespace d4 {

/**
 * @brief Options specific to the cube-counter demo.
 */
class OptionCubeCounter : public OptionGroup {
 public:
  OptionCubeCounter(const std::string& name = "cube-couter",
                    const std::string& description = "Counter specific options")
      : OptionGroup(name, description) {}

  /**
   * @brief Clause selection strategy for cube-and-count.
   *
   * "primal-cut"           — single primal hypergraph bisection; F_easy
   *                          = cut clauses (those spanning both partitions).
   * "iterative-primal-cut" — recursive bisection up to maxDepth levels;
   *                          F_easy = union of cut clauses at every level.
   */
  Option<std::string> selectorStrategy{
      "selectorStrategy",
      "Clause selection strategy for F_easy "
      "(primal-cut|iterative-primal-cut|high-degree)",
      "high-degree"};

  /**
   * @brief Maximum recursion depth for iterative-primal-cut.
   */
  Option<int> maxDepth{
      "maxDepth", "Recursion depth for iterative-primal-cut (default 3)", 3};

  /**
   * @brief Fraction of variables to target for high-degree-variable strategy.
   */
  Option<double> targetRatio{"targetRatio",
                             "Fraction of variables in V_target for "
                             "high-degree-variable (default 0.1)",
                             0.10};

  /**
   * @brief Minimum model-count reduction (in bits) a variable must contribute
   *        to be added to V_easy by the high-degree selector, beyond the first
   *        seeded clause. A variable adds one free dimension (+1 bit) to
   * F_easy; 1.0 keeps the estimated cube count non-increasing, lower values
   * trade more clauses (better decomposition) for more cubes.
   */
  Option<double> hdMinBits{
      "hdMinBits",
      "Min model-count reduction (bits) to add a variable in high-degree "
      "(default 1.0)",
      1.0};

  /**
   * @brief Maximum number of cubes (partial models of F_easy) before falling
   *        back to a direct count of the full formula. The cube count is
   *        measured by a fast polynomial DNNF traversal immediately after
   *        F_easy is compiled. Raising this allows more parallelism at the
   *        cost of more per-cube solves; lowering it reduces overhead but may
   *        skip decomposition opportunities. Default 1000.
   */
  Option<int> maxCubes{
      "maxCubes",
      "Max F_easy cube count before falling back to direct counting (default "
      "1000)",
      1000};

  /**
   * @brief After the selector runs, absorb every remaining clause whose
   *        variables are all already in vars(F_easy). On by default.
   */
  Option<bool> extendEasy{
      "extendEasy",
      "Absorb clauses that do not grow vars(F_easy) (default true)", true};

  /**
   * @brief Strengthen F_easy before compilation by deriving new implied
   *        clauses that stay within vars(F_easy).
   *
   * "none"       — no strengthening (default).
   * "resolution" — single-step resolution: for each clause in F with exactly
   *                one variable outside V_easy, resolve with clauses containing
   *                its negation; keep resolvents entirely within V_easy.
   * "sat"        — enumerate models of F_easy with CaDiCaL; for each model
   *                that is UNSAT w.r.t. F, extract the minimal failing core
   *                and add its negation as a new clause (implied by F).
   *                Stops after --strengthenTime seconds.
   */
  Option<std::string> strengthen{
      "strengthen",
      "Derive implied clauses over vars(F_easy) before compilation "
      "(none|resolution|sat)",
      "none"};

  /**
   * @brief Wall-clock time limit (seconds) for the sat strengthening phase.
   */
  Option<double> strengthenTime{
      "strengthenTime",
      "Time limit for sat strengthening in seconds (default 30)", 30.0};

  /**
   * @brief Maximum resolution steps per derivation path for the resolution
   *        strengthening strategy.
   */
  Option<int> strengthenSteps{
      "strengthenSteps",
      "Max resolution steps per derivation for resolution strengthening "
      "(default 10)",
      10};

  /**
   * @brief Maximum number of new V_easy clauses to collect before stopping
   *        the resolution strengthening search.
   */
  Option<int> strengthenMaxClauses{
      "strengthenMaxClauses",
      "Max new clauses to derive via resolution strengthening (default 100)",
      100};

  /**
   * @brief Maximum |P|×|N| product allowed when eliminating an outside
   *        variable in the elimination strengthening strategy.  Variables
   *        whose elimination would produce more resolvents than this are
   *        skipped.  Pure variables (one polarity absent) are always free.
   */
  Option<int> strengthenMaxProduct{
      "strengthenMaxProduct",
      "Max |P|x|N| resolvent budget per variable for elimination strengthening "
      "(default 20)",
      20};

  /**
   * @brief Before compiling F_easy, enumerate spurious models (models of
   *        F_easy that are UNSAT w.r.t. F), extract the UNSAT core via
   *        failed() with simplify=0, and add the negated core as a new clause
   *        to F_easy. Repeats up to this many iterations. 0 = disabled.
   */
  Option<int> preStrengthen{
      "preStrengthen",
      "Iterations of SAT-based pre-strengthening before DNNF compilation "
      "(0 = disabled, default 0)",
      0};

  /**
   * @brief Path to an external SAT solver binary for per-cube checks instead
   *        of the built-in CaDiCaL.  The binary must accept a DIMACS file path
   *        as its last argument, suppress output to stdout, and exit 10 (SAT)
   *        or 20 (UNSAT).  DIMACS is piped via /dev/stdin.
   *        Empty string = use CaDiCaL (default).
   */
  Option<std::string> externSolver{
      "externSolver",
      "Path to external SAT solver binary for per-cube UNSAT checks "
      "(empty = use CaDiCaL)",
      ""};

  std::vector<OptionBase*> getAllOptions() override {
    return {&selectorStrategy,
            &maxDepth,
            &targetRatio,
            &hdMinBits,
            &maxCubes,
            &extendEasy,
            &strengthen,
            &strengthenTime,
            &strengthenSteps,
            &strengthenMaxClauses,
            &strengthenMaxProduct,
            &preStrengthen,
            &externSolver};
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionCubeCounter& dt) {
    out << " Option CubeCounter: selectorStrategy(" << dt.selectorStrategy.get()
        << ") maxDepth(" << dt.maxDepth.get() << ")\n";
    return out;
  }
};

}  // namespace d4
