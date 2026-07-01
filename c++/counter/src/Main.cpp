/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#include <gmp.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>

#include "Counter.hpp"
#include "OptionCounter.hpp"
#include "ParserCircuit.hpp"
#include "ParserDimacs.hpp"
#include "argparse.hpp"
#include "arjun.h"
#include "config.h"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/preproc/PreprocManager.hpp"

namespace fs = std::filesystem;
d4::MethodManager* methodRun = nullptr;

// Run BiPe preprocessing on the formula in place.
static void runPreproc(parser::Formula& formula,
                       const bipe::OptionPreproc& optionPreproc) {
  bipe::PreprocManager preprocManager;

  // quantifications[0] holds the projection scope when doing projected
  // counting; fall back to all variables for standard model counting.
  std::vector<int> projected;
  if (formula.quantifications[0].size())
    projected = formula.quantifications[0];
  else
    for (unsigned i = 1; i <= formula.nbVar; i++) projected.push_back(i);

  // Variables whose positive and negative literal weights differ must not be
  // eliminated: removing them would change the weighted count.
  std::vector<int> varProtected;
  for (int i = 1; i <= formula.nbVar; i++) {
    std::string w1 = formula.weightMap.find(i) != formula.weightMap.end()
                         ? formula.weightMap[i]
                         : "";
    std::string w2 = formula.weightMap.find(-i) != formula.weightMap.end()
                         ? formula.weightMap[-i]
                         : "";
    if (w1 != w2) varProtected.push_back(i);
  }

  preprocManager.run(formula.nbVar, formula.clauses, projected, varProtected,
                     optionPreproc);

  // Reflect the (possibly narrowed) projected set back into the formula so the
  // counter performs projected model counting whenever preprocessing shrank the
  // scope below all variables (e.g. --preproc.project-on-input narrows it to
  // the bipartition input set). When the scope still covers every variable this
  // clears the quantification so standard model counting is used.
  if (formula.quantifications.empty()) formula.quantifications.emplace_back();
  if (projected.size() < formula.nbVar)
    formula.quantifications[0] = projected;
  else
    formula.quantifications[0].clear();
}  // runPreproc

static bool isOptionPassed(int argc, char** argv,
                           const std::string& optionName) {
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg.find(optionName) == 0) {
      return true;
    }
  }
  return false;
}

ArjunInt::Config conf;
using std::cout;
using std::endl;
using std::string;
using std::unique_ptr;
using std::vector;
using namespace CMSat;

static int fc_int(const std::string& s) {
  int val = 0;
  auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), val);
  if (ec != std::errc{}) throw std::invalid_argument("not an integer: " + s);
  if (ptr != s.data() + s.size())
    throw std::invalid_argument("trailing characters in integer: " + s);
  return val;
}
static double fc_double(const std::string& s) {
  size_t pos = 0;
  double val;
  try {
    val = std::stod(s, &pos);
  } catch (const std::exception&) {
    throw std::invalid_argument("not a double: " + s);
  }
  if (pos != s.size())
    throw std::invalid_argument("trailing characters in double: " + s);
  return val;
}
static const std::string& fc_string(const std::string& s) { return s; }

std::unique_ptr<ArjunNS::Arjun> arjun;
ArjunNS::SimpConf simp_conf;
ArjunNS::Arjun::ElimToFileConf etof_conf;
ArjunNS::Arjun::ManthanConf mconf;
int do_gates = 1;
int redundant_cls = true;
int simptofile = true;
int sampl_start_at_zero = false;
int do_synth_bve = true;
int do_pre_backbone = 0;
std::string mstrategy =
    "const(max_repairs=400),const(max_repairs=400,inv_learnt=1),bve";

int synthesis = false;
int use_brute_force_synth = 1;
int do_unate_def = true;
int do_revbce = false;
int do_backward = true;
std::string debug_minim;
double cms_glob_mult = -1.0;
int mode = 0;
std::unique_ptr<FieldGen> fg = nullptr;

argparse::ArgumentParser program =
    argparse::ArgumentParser("arjun", ArjunNS::Arjun::get_version_sha1(),
                             argparse::default_arguments::help);

template <typename T, typename F>
void myopt(const char* name, T& var, F fun, const char* hhelp) {
  using r = std::decay_t<std::invoke_result_t<F, const std::string&>>;
  static_assert(std::is_floating_point_v<r> == std::is_floating_point_v<T>,
                "Floating-point mismatch: use fc_double for floating-point "
                "vars, fc_int for integral vars");
  static_assert(std::is_integral_v<r> == std::is_integral_v<T>,
                "Integral/string mismatch: use fc_int for integral vars, "
                "fc_string for string vars");
  program.add_argument(name)
      .action([&var, fun](const auto& a) { var = fun(a); })
      .default_value(var)
      .help(hhelp);
}
template <typename T, typename F>
void myopt2(const char* name1, const char* name2, T& var, F fun,
            const char* hhelp) {
  using r = std::decay_t<std::invoke_result_t<F, const std::string&>>;
  static_assert(std::is_floating_point_v<r> == std::is_floating_point_v<T>,
                "Floating-point mismatch: use fc_double for floating-point "
                "vars, fc_int for integral vars");
  static_assert(std::is_integral_v<r> == std::is_integral_v<T>,
                "Integral/string mismatch: use fc_int for integral vars, "
                "fc_string for string vars");
  program.add_argument(name1, name2)
      .action([&var, fun](const auto& a) { var = fun(a); })
      .default_value(var)
      .help(hhelp);
}
template <typename T>
void myflag(const char* name, T& var, const char* hhelp) {
  static_assert(std::is_same_v<T, int>, "myflag var must be int");
  program.add_argument(name)
      .action([&var](const auto&) { var = 1; })
      .default_value(var)
      .flag()
      .help(hhelp);
}

void add_arjun_options() {
  myopt2("-v", "--verb", conf.verb, fc_int, "Verbosity");
  program.add_argument("--version")
      .action([&](const auto&) { exit(0); })
      .flag()
      .help("Print version and exit");

  myopt("--mode", mode, fc_int, "0=counting, 1=weightd counting");
  myopt("--allindep", etof_conf.all_indep, fc_int,
        "All variables can be made part of the indepedent support. Indep "
        "support is given ONLY to help the solver.");
  myopt("--maxc", conf.backw_max_confl, fc_int,
        "Maximum conflicts per variable in backward mode");
  myopt("--revbce", do_revbce, fc_int, "Perform reverse BCE");
  myopt("--sbva", etof_conf.num_sbva_steps, fc_int,
        "SBVA timeout in K steps. 0 = no sbva");
  myopt("--prebackbone", do_pre_backbone, fc_int,
        "Perform backbone before other things");
  myopt("--seed", conf.seed, fc_int, "Random seed");

  // synth main
  myflag("--synth", synthesis, "Run synthesis");
  myflag("--synthmore", synthesis,
         "Run synthesis, with more aggressive BVE options");
  myopt("--bruteforcesynth", use_brute_force_synth, fc_int,
        "Try brute-force synthesis for the final synthesis "
        "step before Manthan. Viable when |orig_sampl_cnf| ≤ "
        "--bruteforcesynththresh (after the minim pre-pass); above that it "
        "declines and Manthan takes over. 0=Manthan only, 1=try brute-force "
        "first (default).");
  myopt("--maxsat", mconf.maxsat_better_ctx, fc_int,
        "Use maxsat to find better counterexamples during Manthan");
  myopt("--synthbve", do_synth_bve, fc_int, "Perform BVE for synthesis");
  myopt("--extend", etof_conf.do_extend_indep, fc_int,
        "Extend independent set just before CNF dumping");
  myopt("--minimconfl", mconf.minimize_conflict, fc_int,
        "Minimize conflict size when repairing");
  myopt("--unatedef", do_unate_def, fc_int,
        "Perform definition-aware unate analysis");
  myopt("--unatedefmaxconfl", conf.unate_def_max_confl, fc_int,
        "Conflict budget per SAT call in the standard unate_def probe");
  myopt("--unatedefeq", conf.unate_def_eq, fc_int,
        "In unate_def, also detect equiv defs of the form t = L or t = ~L for "
        "some literal L");
  myopt("--unatedefeqmax", conf.unate_def_eq_max_per_var, fc_int,
        "Max equiv candidates to test per to-define variable in unate_def");
  myopt("--unatedefeqconfl", conf.unate_def_eq_max_confl, fc_int,
        "Conflict budget per SAT call inside the equiv unate_def search");
  myopt("--unatedefeqdry", conf.unate_def_eq_dry_streak, fc_int,
        "Disable equiv unate_def probe after this many consecutive misses with "
        "zero hits so far (very low = bail aggressively, very high = "
        "effectively never disable)");
  myopt("--unatedefeqnoninp", conf.unate_def_eq_noninput, fc_int,
        "Allow non-input vars (to-define + already-tested) as the candidate L "
        "in t = L. Inputs are still tried first; non-inputs only after the "
        "input list is exhausted. 0 = inputs only");
  myopt("--autarky", etof_conf.do_autarky, fc_int, "Perform autarky analysis");
  myopt("--moneperloop", mconf.one_repair_per_loop, fc_int,
        "One repair per CEX loop");
  myopt("--minvertlearn", mconf.inv_learnt, fc_int, "Invert learnt functions");

  // repairing on vars
  myopt("--bwequal", mconf.force_bw_equal, fc_int,
        "Force BW vars' indicators to be TRUE -- prevents repairing with them, "
        "but faster to repair");
  // Strategy
  myopt("--mstrategy", mstrategy, fc_string,
        "Comma-separated synthesis strategy list, e.g. "
        "\"learn(samples=1,max_repairs=100),learn(max_repairs=800),bve\". "
        "Each non-last strategy runs for 20*max_repairs tries; the last runs "
        "unlimited. "
        "Params: max_repairs, samples, samples_ccnr, min_gain_split, "
        "max_depth, sampler_fixed_conflicts, and other ManthanConf fields.");
  // Order
  myopt("--morder", mconf.manthan_order, fc_int,
        "Order vars: incidence (0), BVE (2)");
  myopt("--maxsatorder", mconf.maxsat_order, fc_int,
        "Which order to use to try to fix vars? 0 = norm, 1 = rev");
  // solver config
  myopt("--ctxsolver", mconf.ctx_solver_type, fc_int,
        "Context solver type. 0 = CryptoMiniSat, 1 = CaDiCaL");
  myopt("--repairsolver", mconf.repair_solver_type, fc_int,
        "Repair solver type. 0 = CryptoMiniSat, 1 = CaDiCaL");
  myopt("--repaircache", mconf.repair_cache_size, fc_int,
        "Repair cache size. 0 = no cache");
  // synth -- sampling
  myopt("--samples", mconf.samples, fc_int, "Number of samples");
  myopt("--samplesccnr", mconf.samples_ccnr, fc_int,
        "Number of samples from CCNR");
  myopt("--uniqsamp", mconf.do_unique_input_samples, fc_int,
        "Unique samples on input vars");
  myopt("--filtersamples", mconf.filter_samples, fc_int,
        "Filter samples from useless ones");
  myopt("--fixedconf", mconf.sampler_fixed_conflicts, fc_int,
        "Restart conflict limit in CMSGen");
  // synth -- decision tree
  myopt("--maxdepth", mconf.max_depth, fc_int,
        "Maximum depth of decision tree");
  myopt("--minleaf", mconf.min_leaf_size, fc_int,
        "Minimum leaf size in decision tree");
  myopt("--mingainsplit", mconf.min_gain_split, fc_double,
        "Minimum gain for a split in decision tree");
  myopt("--learnuseall", mconf.use_all_vars_as_feats, fc_int,
        "Use all variables as features in decision tree learning. 0 = only "
        "inputs");
  // synth -- cutoff/tuning constants
  myopt("--constvotesamples", mconf.const_vote_samples, fc_int,
        "Majority voting samples for const_functions");
  myopt("--statsevery", mconf.stats_every, fc_int,
        "Print stats every N repair loops");
  myopt("--detailedstatsevery", mconf.detailed_stats_every, fc_int,
        "Print detailed stats every N repair loops");
  myopt("--confldropy", mconf.conflict_drop_y_max, fc_int,
        "Max conflict size to try dropping y-vars");
  myopt("--conflcapkeep", mconf.conflict_cap_keep, fc_int,
        "Keep this many literals when capping conflicts");
  myopt("--batchminimmin", mconf.batch_minim_min, fc_int,
        "Min conflict size for batch minimization");
  myopt("--minimbudgetthresh", mconf.minim_budget_threshold, fc_int,
        "Conflict size above which minim budget is capped");
  myopt("--minimbudgetmax", mconf.minim_budget_max, fc_int,
        "Max minimization solver calls");
  myopt("--minimbudgetmult", mconf.minim_budget_mult, fc_int,
        "Minim budget = conflict.size * mult (up to max)");
  myopt("--ccnrmemspersample", mconf.ccnr_mems_per_sample, fc_int,
        "CCNR total memory budget per sample");
  myopt("--ccnrpercalllimit", mconf.ccnr_per_call_limit, fc_int,
        "CCNR per-call step limit for local_search");
  myopt("--czhighratio", mconf.cz_high_ratio, fc_int,
        "cost_zero > tot_repaired * this triggers tightest cz_threshold");
  myopt("--czlowratio", mconf.cz_low_ratio, fc_int,
        "cost_zero > tot_repaired * this triggers medium cz_threshold");
  myopt("--czthreshhigh", mconf.cz_threshold_high, fc_int,
        "Consecutive cost-zero break count when high cz ratio");
  myopt("--czthreshmid", mconf.cz_threshold_mid, fc_int,
        "Consecutive cost-zero break count when medium cz ratio");
  myopt("--czthreshlow", mconf.cz_threshold_low, fc_int,
        "Consecutive cost-zero break count when low cz ratio");
  // synth -- debug
  myopt("--manthancnf", mconf.write_manthan_cnf, fc_string,
        "Write Manthan CNF to this file");
  myopt("--debugsynth", conf.debug_synth, fc_string,
        "Debug synthesis, prefix with this fname");
  myopt("--interprebuildevery", conf.interp_rebuild_every, fc_int,
        "Rebuild the doubled-CNF interpolation solver every N interpolants "
        "(bounds tracer maps; smaller = exercise the rebuild path more).");
  myflag("--checkrepair", mconf.check_repair,
         "Check that error formula count decreases monotonically after each "
         "repair iteration (uses ganak)");
  myopt("--ganakbin", mconf.ganak_binary, fc_string,
        "Path to ganak binary (for --checkrepair)");
  // Craig-interpolant repair options.
  myopt("--interprepair", mconf.interp_repair, fc_int,
        "Craig-interpolant repair branch in compose_or/and. "
        "0=off, 1=on for every repair, 2=on only when conflict size >= "
        "--interprepairmincl");
  myopt("--interprepairmincl", mconf.interp_repair_min_conflict, fc_int,
        "(--interprepair=2 only) Minimum conflict size to attempt "
        "interpolation.");
  myopt("--interprepairmaxnodes", mconf.interp_repair_max_aig_nodes, fc_int,
        "Cap interpolant AIG size; if bigger, fall back to conflict-clause "
        "path. 0=no cap.");
  myopt("--interprepairb1rewrite", mconf.interp_repair_b1_rewrite, fc_int,
        "Independent: AIG rewrite of the guard=NOT(I) AND y_other_matches "
        "before Tseitin-encoding. simplify_aig is always on; this controls the "
        "heavier rewrite_aig pass. 0=off, 1=on.");
  myopt("--interprepairgroupcse", mconf.interp_repair_group_cse, fc_int,
        "Pass --group-cse to AIGToCNF when encoding the interp guard. Dedups "
        "Tseitin helpers for structurally identical sub-AIGs. 0=off, 1=on.");
  myopt("--interprepairmaxconfl", mconf.interp_repair_max_conflicts, fc_int,
        "Per-call cadical conflict budget for the interpolation solve. 0=no "
        "limit (default). Try 50000 to cap interp call cost; budget-exhausted "
        "calls fall back to the conflict-clause path.");
  myopt("--interprepairadaptive", mconf.interp_repair_adaptive_gate, fc_int,
        "Adaptive per-var gating: blacklist a var from interp when its running "
        "mean interp/conflict ratio exceeds --interprepairratioskip. 0=off, "
        "1=on.");
  myopt("--interprepairratioskip", mconf.interp_repair_adaptive_ratio_skip,
        fc_double,
        "Mean interp/conflict ratio above which the adaptive gate blacklists a "
        "var.");
  myopt("--interprepairskipwindow", mconf.interp_repair_adaptive_skip_window,
        fc_int,
        "How many tot_repaired ticks the adaptive blacklist persists before "
        "the var gets another chance.");
  myopt(
      "--interprepairprogressmax", mconf.interp_repair_progress_max_var_repairs,
      fc_int,
      "Progress gate: once a var has been interp-repaired this many times and "
      "still needs more, drop interp for it (it is not generalising). 0=off.");

  // === Brute-force synthesis (--bruteforcesynth 1) knobs ===
  myopt("--bruteforcesynththresh", mconf.brute_force_synth_threshold, fc_int,
        "Cap: brute_force_synth runs only when |orig_sampl_cnf| ≤ this (after "
        "the minim pre-pass); above it it declines to Manthan. Each y "
        "allocates 2^N truth-table entries, so raising past ~20 OOMs.");
  myopt("--bruteforcesynthminim", mconf.brute_force_synth_minim, fc_int,
        "Dry-run backward minim on orig_sampl_cnf before enumeration: prunes "
        "sampling vars that the post-preproc CNF defines from the rest, "
        "shrinking the 2^N truth tables. 0=off, 1=on (default).");
  myopt("--bruteforcesynthminimmax", mconf.brute_force_synth_minim_max, fc_int,
        "Only attempt the minim pre-pass when |orig_sampl_cnf| ≤ this (default "
        "40). Above it the doubled-CNF minim is too expensive and unlikely to "
        "shrink below --bruteforcesynththresh, so it is skipped.");

  // Simplification options for minim
  myopt("--probe", conf.probe_based, fc_int,
        "Use simple probing to set (and define) some variables");
  myopt("--bvepresimp", conf.bve_pre_simplify, fc_int, "simplify");
  myopt("--simp", conf.simp, fc_int,
        "Do ~ sort of simplification during indep minimixation");
  myopt("--intree", conf.intree, fc_int, "intree");

  // Gate options
  myopt("--gates", do_gates, fc_int, "Turn on/off all gate-based definability");
  myopt(
      "--nogatebelow", conf.no_gates_below, fc_double,
      "Don't use gates below this incidence relative position (1.0-0.0) to "
      "minimize the independent set. Gates are not very accurate, but can save "
      "a LOT of time. We use them to get rid of most of the uppert part of the "
      "sampling set only. Default is 99% is free-for-all, the last 1% we test. "
      "At 1.0 we test everything, at 0.0 we try using gates for everything.");
  myopt("--orgate", conf.or_gate_based, fc_int,
        "Use 3-long gate detection in SAT solver to-define variables");
  myopt("--irreggate", conf.irreg_gate_based, fc_int,
        "Use irregular gate-based removal of vars from indep set");
  myopt("--itegate", conf.ite_gate_based, fc_int,
        "Use ITE gate detection in SAT solver to-define some variables");
  myopt("--xorgate", conf.xor_gates_based, fc_int,
        "Use XOR detection in SAT solver to-define some variables");

  // AppMC
  program.add_argument("--appmc")
      .flag()
      .action([&](const auto&) { simp_conf.appmc = true; })
      .help("Set CNF simplification options for appmc");

  // Detailed Configuration
  myopt("--sbvaclcut", etof_conf.sbva_cls_cutoff, fc_int,
        "SBVA heuristic cutoff. Higher -> only appied to more clauses");
  myopt("--sbvalitcut", etof_conf.sbva_lits_cutoff, fc_int,
        "SBVA heuristic cutoff. Higher -> only appied to larger clauses");
  myopt("--findbins", conf.oracle_find_bins, fc_int,
        "How aggressively find binaries via oracle");
  myopt("--sbvabreak", etof_conf.sbva_tiebreak, fc_int,
        "SBVA tie break: 1=sbva or 0=bva");
  myopt("--sbvamaxnewvars", etof_conf.sbva_max_new_vars, fc_int,
        "Max number of new variables SBVA may add. 0 = no limit");
  myopt("--gaussj", conf.gauss_jordan, fc_int,
        "Use XOR finding and Gauss-Jordan elimination");
  myopt("--bve", simp_conf.do_bve, fc_int,
        "Perform BVE during CNF simplification");
  myopt("--iter1", simp_conf.iter1, fc_int, "Puura iterations before oracle");
  myopt("--iter1grow", simp_conf.bve_grow_iter1, fc_int,
        "Puura BVE grow rate allowed before Oracle");
  myopt("--iter2", simp_conf.iter2, fc_int, "Puura iterations after oracle");
  myopt("--iter2grow", simp_conf.bve_grow_iter2, fc_int,
        "Puura BVE grow rate allowed after Oracle");
  myopt("--bvegrownonstop", simp_conf.bve_grow_nonstop, fc_int,
        "Do not stop BVE if nothing got eliminated, keep going until grow "
        "factor limit");
  myopt("--bveresolvmaxsz", simp_conf.bve_too_large_resolvent, fc_int,
        "Puura BVE max resolvent size in literals. -1 == no limit");
  myopt("--oraclemult", simp_conf.oracle_mult, fc_double,
        "Oracle multiplier for timeout (i.e. steps-out)");
  myopt("--oraclesparsify", simp_conf.oracle_sparsify, fc_int,
        "Use Oracle to sparsify");
  myopt("--oraclevivif", simp_conf.oracle_vivify, fc_int,
        "Use oracle to vivify");
  myopt("--oraclevivifgetl", simp_conf.oracle_vivify_get_learnts, fc_int,
        "Use oracle to vivify get learnts");
  myopt("--oracleextra", simp_conf.oracle_extra, fc_int,
        "Run an extra oracle-vivif-fast + oracle-sparsify-fast + occ-bve pass "
        "at the end of Puura's strategy");
  myopt("--distill", conf.distill, fc_int,
        "Distill clauses before minimization of indep");
  myopt("--weakenlim", simp_conf.weaken_limit, fc_int,
        "Limit to weaken BVE resolvents");
  myopt("--puurastrategy", simp_conf.puura_strategy, fc_int,
        "Puura iter1 simplification strategy: 0=default, 1=new-model");
  myopt("--red", redundant_cls, fc_int, "Also dump redundant clauses");

  // Debug
  etof_conf.do_renumber = false;
  myopt("--renumber", etof_conf.do_renumber, fc_int,
        "Renumber variables to start from 1...N in CNF.");
  myopt(
      "--specifiedorder", conf.specified_order_fname, fc_string,
      "Try to remove variables from the independent set in this order. File "
      "must contain a variable on each line. Variables start at ZERO. Variable "
      "from the BOTTOM will be removed FIRST. This is for DEBUG ONLY");
  myopt("--backward", do_backward, fc_int,
        "Run the backward pass to minimize the independent set");
  myopt("--debugminim", debug_minim, fc_string,
        "Create this file that is the CNF after indep set minimization");
  myopt("--cmsmult", conf.cms_glob_mult, fc_double,
        "Multiply timeouts in CMS by this. Default is -1, which means no "
        "change. Useful for debugging");

  program.add_argument("files").remaining().help("input file and output file");
}

void set_config(ArjunNS::Arjun* arj) {
  arj->set_verb(conf.verb);
  arj->set_distill(conf.distill);
  arj->set_specified_order_fname(conf.specified_order_fname);
  arj->set_intree(conf.intree);
  arj->set_bve_pre_simplify(conf.bve_pre_simplify);
  arj->set_cms_glob_mult(conf.cms_glob_mult);
  if (do_gates) {
    arj->set_or_gate_based(conf.or_gate_based);
    arj->set_ite_gate_based(conf.ite_gate_based);
    arj->set_xor_gates_based(conf.xor_gates_based);
    arj->set_irreg_gate_based(conf.irreg_gate_based);
  } else {
    cout << "c o NOTE: all gates are turned off due to `--gates 0`" << endl;
    arj->set_or_gate_based(0);
    arj->set_ite_gate_based(0);
    arj->set_xor_gates_based(0);
    arj->set_irreg_gate_based(0);
  }
  arj->set_no_gates_below(conf.no_gates_below);
  arj->set_probe_based(conf.probe_based);
  arj->set_backw_max_confl(conf.backw_max_confl);
  arj->set_seed(conf.seed);
  arj->set_gauss_jordan(conf.gauss_jordan);
  arj->set_simp(conf.simp);
  arj->set_extend_max_confl(conf.extend_max_confl);
  arj->set_unate_def_eq(conf.unate_def_eq);
  arj->set_unate_def_eq_max_per_var(conf.unate_def_eq_max_per_var);
  arj->set_unate_def_eq_max_confl(conf.unate_def_eq_max_confl);
  arj->set_unate_def_eq_dry_streak(conf.unate_def_eq_dry_streak);
  arj->set_unate_def_eq_noninput(conf.unate_def_eq_noninput);
  arj->set_oracle_find_bins(conf.oracle_find_bins);
}

// Run Arjun preprocessing on the formula in place.
static void runArjunPreproc(parser::Formula& formula,
                            const d4::OptionCounter& optionCounter,
                            const bipe::OptionPreproc& optionPreproc, int argc,
                            char** argv) {
  // Check if we have variables to project/minimize
  std::vector<uint32_t> projected;
  if (formula.quantifications[0].size()) {
    for (int v : formula.quantifications[0]) {
      projected.push_back(v - 1);
    }
  } else {
    for (unsigned i = 0; i < formula.nbVar; i++) {
      projected.push_back(i);
    }
  }

  bool all_indep = (projected.size() == formula.nbVar);

  // Variables whose positive and negative literal weights differ must not be
  // eliminated: removing them would change the weighted count.
  std::vector<int> varProtected;
  for (int i = 1; i <= formula.nbVar; i++) {
    std::string w1 = formula.weightMap.find(i) != formula.weightMap.end()
                         ? formula.weightMap[i]
                         : "";
    std::string w2 = formula.weightMap.find(-i) != formula.weightMap.end()
                         ? formula.weightMap[-i]
                         : "";
    if (w1 != w2) varProtected.push_back(i);
  }

  // Create Arjun SimplifiedCNF and populate it
  arjun = std::make_unique<ArjunNS::Arjun>();
  add_arjun_options();
  set_config(arjun.get());
  ArjunNS::FGenMpq fg;
  ArjunNS::SimplifiedCNF cnf(&fg);
  cnf.set_need_aig();
  cnf.set_sampl_vars(projected);
  cnf.new_vars(formula.nbVar);
  cnf.set_weighted(true);

  // set the clauses.
  for (const auto& cl : formula.clauses) {
    std::vector<CMSat::Lit> c;
    for (int lit : cl) {
      uint32_t var = std::abs(lit) - 1;
      bool sign = (lit < 0);
      c.push_back(CMSat::Lit(var, sign));
    }
    cnf.add_clause(c);
  }

  // Set weights for protected variables to prevent elimination
  ArjunNS::FMpq w_protect(mpq_class(1, 3));
  for (int v : varProtected) {
    cnf.set_lit_weight(CMSat::Lit(v - 1, false), w_protect);
  }

  // Preprocess with Arjun
  arjun->set_verb(optionCounter.verbosity.get());
  arjun->set_seed(0);
  arjun->standalone_minimize_indep(cnf, etof_conf.all_indep);
  arjun->standalone_elim_to_file(cnf, etof_conf, simp_conf);

  // Propagate back to formula with original variable mapping
  formula.clauses.clear();

  const auto new_to_orig_var = cnf.get_new_to_orig_var();

  int max = formula.nbVar;
  for (const auto& cl : cnf.get_clauses()) {
    std::vector<int> c;
    for (const auto& lit : cl) {
      if (new_to_orig_var.count(lit.var())) {
        CMSat::Lit orig_lit = new_to_orig_var.at(lit.var());
        int var = orig_lit.var() + 1;
        if (var > max) max = var;
        int human_lit = (lit.sign() ^ orig_lit.sign()) ? -var : var;
        c.push_back(human_lit);
      } else {
        int var = lit.var() + 1;
        if (var > max) max = var;
        int human_lit = lit.sign() ? -var : var;
        c.push_back(human_lit);
      }
    }
    formula.clauses.push_back(c);
  }

  std::cout << "----------> " << cnf.freeVars.size() << '\n';
  std::vector<bool> isFreeVar(formula.nbVar, false);
  for (auto v : cnf.freeVars) {
    isFreeVar[v] = true;
    std::cout << v << " is free\n";
  }

  for (int i = 0; i < formula.nbVar; i++) {
    std::cout << i + 1 << " is definied by " << cnf.get_def(i) << " "
              << cnf.get_lit_weight(CMSat::Lit(i, 1)) << '\n';

    if (cnf.defined(i) && !isFreeVar[i]) {
      const auto def = cnf.get_def(i);
      if (def.neg)
        formula.clauses.push_back({-i - 1});
      else
        formula.clauses.push_back({i + 1});
      if (i + 1 > max) max = i + 1;
    }
  }

  std::cout << "The maximum is " << max << '\n';
  formula.nbVar = max;
  std::cout << formula << '\n';
}

/**
   The main function.

   Usage: counter -i INPUT [-h]

   -i INPUT   Path to the input DIMACS file (required).

   Examples:
     counter -i formula.cnf
     counter -i formula.cnf --solver.solverName 1
*/
int main(int argc, char** argv) {
  auto start = std::chrono::system_clock::now();

  std::string inputPath;
  bool showHelp = false;

  // Simple manual scan for launcher options
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      showHelp = true;
    } else if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
      inputPath = argv[++i];
    }
  }

  // 1. Initialize configuration and registry
  d4::OptionDpllStyleMethod options;
  d4::OptionCounter optionCounter;
  bipe::OptionPreproc optionPreproc;

  d4::OptionRegistry registry;
  options.registerTo(registry);
  optionCounter.registerTo(registry);
  optionPreproc.registerTo(registry);

  registry.parseArgv(argc, argv);

  if (showHelp || inputPath.empty()) {
    if (!showHelp && inputPath.empty())
      std::cerr << "Missing required argument: -i INPUT\n";

    std::cout << "USAGE: " << argv[0] << " -i INPUT [Overrides...]\n"
              << "  -i, --input   Path to the input DIMACS file (required)\n"
              << "  -h, --help    Show this help screen\n";

    registry.displayHelp(std::cout);

    return showHelp ? 0 : 1;
  }

  // Check if input file exists
  if (!fs::exists(inputPath)) {
    std::cerr << "ERROR! Input file does not exist: " << inputPath << "\n";
    return 1;
  }

  parser::Formula formula;
  const std::string informat = optionCounter.informat.get();

  if (informat == "circuit") {
    parser::ParserCircuit parserCircuit;
    parserCircuit.parse_circuit(inputPath, formula);
  } else {
    parser::ParserDimacs parserDimacs;
    parserDimacs.parse_DIMACS(inputPath, formula);
    if (optionCounter.preprocEngine.get() == "arjun") {
      if (!formula.quantifications.empty() &&
          formula.quantifications[0].size() > 0) {
        std::cout << "c [PREPROC] Projected model counting detected. Falling "
                     "back to bipe engine for correctness.\n";
        runPreproc(formula, optionPreproc);
      } else {
        runArjunPreproc(formula, optionCounter, optionPreproc, argc, argv);
      }
    } else {
      runPreproc(formula, optionPreproc);
    }
  }

  counter(options, optionCounter, formula);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "c [COUNTER] Elapsed time: " << elapsed.count() << " seconds\n";

  return EXIT_SUCCESS;
}  // main