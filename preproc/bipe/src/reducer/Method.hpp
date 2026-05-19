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
#pragma once

#include <map>

#include "Propagator.hpp"
#include "src/utils/Timer.hpp"

namespace bipe {
namespace reducer {

/**
 * @brief Abstract base class for all CNF reduction and simplification methods.
 *
 * Defines the standard interface that all specific reduction techniques
 * (e.g., Vivification, Occurrence Elimination) must implement. Also acts
 * as a factory to instantiate these specific methods.
 */
class Method {
 protected:
  /// Tracks the total number of literals removed by the method.
  unsigned m_nbRemoveLit = 0;
  unsigned m_nbRemoveClause = 0;

 public:
  /**
   * @brief Virtual destructor to ensure proper cleanup of derived classes.
   */
  virtual ~Method();

  /**
   * @brief Factory method to create a specific reduction method instance.
   *
   * @param meth The string identifier of the method to create (e.g.,
   * "vivification").
   * @param out The output stream used for logging by the instantiated method.
   * @return A pointer to the newly allocated Method instance.
   */
  static Method* makeMethod(const std::string& meth, std::ostream& out);

  /**
   * @brief Executes the reduction method and stores the output in a separate
   * structure.
   *
   * @param nbVar The number of variables in the formula.
   * @param clauses The input set of clauses to be processed.
   * @param nbIteration The maximum number of iterations to perform. Set to -1
   *                    to run continuously until a fixed point is reached.
   * @param verbose If true, prints detailed progress information to the output
   * stream.
   * @param[out] result The resulting set of clauses after the reduction is
   * applied.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  virtual void run(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
                   int nbIteration, bool verbose,
                   std::vector<std::vector<Lit>>& result,
                   const Timer& timer) = 0;

  /**
   * @brief Executes the reduction method in-place on the given clauses.
   *
   * @param nbVar The number of variables in the formula.
   * @param[in,out] clauses The set of clauses to be processed and modified
   * in-place.
   * @param nbIteration The maximum number of iterations to perform. Set to -1
   *                    to run continuously until a fixed point is reached.
   * @param verbose If true, prints detailed progress information to the output
   * stream.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  virtual void run(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
                   int nbIteration, bool verbose, const Timer& timer) = 0;

  /**
   * @brief Executes the reduction method utilizing an existing Propagator.
   *
   * @param propagator The propagator engine used to manage and execute BCP.
   * @param nbIteration The maximum number of iterations to perform. Set to -1
   *                    to run continuously until a fixed point is reached.
   * @param verbose If true, prints detailed progress information to the output
   * stream.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  virtual void run(Propagator& propagator, int nbIteration, bool verbose,
                   const Timer& timer) = 0;

  inline unsigned getNbRemoveLit() { return m_nbRemoveLit; }
  inline unsigned getNbRemoveClause() { return m_nbRemoveClause; }
  inline void setNbRemoveLit(unsigned nb) { m_nbRemoveLit = nb; }
  inline void setNbRemoveClause(unsigned nb) { m_nbRemoveClause = nb; }
};

}  // namespace reducer
}  // namespace bipe