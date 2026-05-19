/**
 * reducer
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "Combinaison.hpp"

#include "OccElimination.hpp"
#include "Vivification.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace reducer {

/**
 * @brief Combinaison::Combinaison implementation.
 */
Combinaison::Combinaison(std::ostream& out) : m_out(out) {}  // constructor

/**
 * @brief Combinaison::run implementation.
 */
void Combinaison::run(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
                      int nbIteration, bool verbose,
                      std::vector<std::vector<Lit>>& result,
                      const Timer& timer) {
  if (verbose) std::cout << "c [REDUCER] Combinaison\n";
  Propagator propagator(nbVar, clauses, m_out, verbose);
  run(propagator, nbIteration, verbose, timer);
  propagator.extractFormula(result);
}  // run

/**
 * @brief Combinaison::run implementation.
 */
void Combinaison::run(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
                      int nbIteration, bool verbose, const Timer& timer) {
  Propagator propagator(nbVar, clauses, m_out, verbose);
  run(propagator, nbIteration, verbose, timer);
  propagator.display(m_out);
}  // run

/**
 * @brief Combinaison::run implementation.
 */
void Combinaison::run(Propagator& propagator, int nbIteration, bool verbose,
                      const Timer& timer) {
  Vivification vivification(m_out);
  OccElimination occElimination(m_out);

  m_vivifier = &vivification;
  m_occEliminator = &occElimination;

  bool fixePoint = false;
  for (int iteration = 0;
       !propagator.getIsUnsat() && !timer.isTimeout() && !fixePoint &&
       (nbIteration == -1 || iteration < nbIteration);
       iteration++) {
    unsigned current = m_nbRemoveLit;
    fixePoint = true;

    if (!propagator.getIsUnsat() && !timer.isTimeout()) {
      occElimination.setNbRemoveLit(0);
      occElimination.setNbRemoveClause(0);
      occElimination.run(propagator, 1, false, timer);
    }

    if (!propagator.getIsUnsat() && !timer.isTimeout()) {
      vivification.setNbRemoveLit(0);
      vivification.setNbRemoveClause(0);
      vivification.run(propagator, 1, false, timer);
    }

    m_nbRemoveLit +=
        vivification.getNbRemoveLit() + occElimination.getNbRemoveLit();
    m_nbRemoveClause +=
        vivification.getNbRemoveClause() + occElimination.getNbRemoveClause();

    fixePoint = current == m_nbRemoveLit;
    if (verbose)
      m_out << "c [REDUCER Combinaison] #Iteration: " << iteration << "/"
            << nbIteration << " #lit-rm: " << m_nbRemoveLit
            << " #cl-rm: " << m_nbRemoveClause << "\n";
  }

  m_vivifier = m_occEliminator = nullptr;
}  // run

/**
 * @brief Combinaison::displayInfo implementation.
 */
void Combinaison::displayInfo() {}  // displayInfo

}  // namespace reducer
}  // namespace bipe