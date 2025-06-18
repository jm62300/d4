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
#include "ScoringMethod.hpp"

#include "cnf/ScoringMethodDlcs.hpp"
#include "cnf/ScoringMethodJwts.hpp"
#include "cnf/ScoringMethodMom.hpp"
#include "cnf/ScoringMethodVsads.hpp"
#include "cnf/ScoringMethodVsids.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/formulaManager/circuit/CircuitWithCnfManager.hpp"
#include "src/utils/ErrorCode.hpp"

namespace d4 {

/**
 * @brief Given a scoring method option and a CnfManager, this function returns
 * the appropiate scoring method.
 *
 * @param[in] ps is a CnfManager.
 * @param[in] am is an ActivityManager.
 * @param[in] sm is the scoring method we want.
 *
 * @return a scoring method.
 */
ScoringMethod *getScoringMethodCnf(CnfManager &ps, ActivityManager &am,
                                   const ScoringMethodType &sm) {
  switch (sm) {
    case SCORE_MOM:
      return new ScoringMethodMom(ps);
    case SCORE_DLCS:
      return new ScoringMethodDlcs(ps);
    case SCORE_VSIDS:
      return new ScoringMethodVsids(am);
    case SCORE_VSADS:
      return new ScoringMethodVsads(ps, am);
    case SCORE_JWTS:
      return new ScoringMethodJwts(ps);
  }

  return NULL;
}  // getScoringMethodCnf

/**
   Select from the arguments store in vm the good scoring method and return it.

   @param[in] vm, the arguments on the command line.
   @pararm[in] p, the problem manager.

   \return the scoring method
 */
ScoringMethod *ScoringMethod::makeScoringMethod(
    const OptionBranchingHeuristic &options, FormulaManager &p,
    ActivityManager &am, std::ostream &out) {
  switch (p.getProblemInputType()) {
    case PB_CIRC:
      try {
        CircuitWithCnfManager &ps = dynamic_cast<CircuitWithCnfManager &>(p);
        return getScoringMethodCnf(*(ps.getCnfManager()), am,
                                   options.scoringMethodType);
      } catch (std::bad_cast &bc) {
        std::cerr << "c bad_cast caught: " << bc.what() << '\n';
        std::cerr << "c A CNF formula was expeted\n";
        exit(ERROR_BAD_CAST);
      }
    case PB_QBF:
    case PB_TCNF:
    case PB_CNF:
      try {
        CnfManager &ps = dynamic_cast<CnfManager &>(p);
        return getScoringMethodCnf(ps, am, options.scoringMethodType);
      } catch (std::bad_cast &bc) {
        std::cerr << "c bad_cast caught: " << bc.what() << '\n';
        std::cerr << "c A CNF formula was expeted\n";
        exit(ERROR_BAD_CAST);
      }
    case PB_NONE:
      std::cerr << "c The problem type cannot be none!\n";
      exit(ERROR_BAD_TYPE_PROBLEM);
  }

  throw(FactoryException("Cannot create a ScoringMethod", __FILE__, __LINE__));
}  // makeScoringMethod

/**
 * @brief ScoringMethod::selectVariable implementation.
 */
Var ScoringMethod::selectVariable(std::vector<Var> &vars, FormulaManager &s,
                                  std::vector<bool> &isDecisionVariable) {
  Var ret = var_Undef;
  double bestScore = -1;
  assert(isDecisionVariable.size() >= (unsigned)s.getNbVariable());

  for (auto &v : vars) {
    if (s.varIsAssigned(v) || !isDecisionVariable[v]) continue;

    double current = computeScore(v);
    if (ret == var_Undef || current > bestScore) {
      ret = v;
      bestScore = current;
    }
  }

  return ret;
}  // selectVariable

}  // namespace d4
