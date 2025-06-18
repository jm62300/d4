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
#pragma once

#include <src/problem/ProblemManager.hpp>
#include <src/problem/ProblemTypes.hpp>
#include <vector>

#include "src/options/formulaManager/OptionFormulaManager.hpp"

namespace d4 {
class FormulaManager {
 protected:
  unsigned m_nbVar;
  std::vector<lbool> m_currentValue;

 public:
  /**
   * @brief A constructor needed to know the number of variables.
   *
   * @param nbVar
   */
  FormulaManager(unsigned nbVar) : m_nbVar(nbVar) {
    m_currentValue.resize(m_nbVar + 1, l_Undef);
  }

  /**
   * @brief Generate an occurrence manager regarding the options given as
   * parameter.
   *
   * @param options gives the options.
   * @param p is the problem under consideration.
   * @param out is the stream where are printed out the logs.
   * @return a spec manager.
   */
  static FormulaManager *makeFormulaManager(const OptionSpecManager &options,
                                            ProblemManager &p,
                                            std::ostream &out);

  /**
   * @brief Get the number of variables.
   *
   * @return unsigned
   */
  inline unsigned getNbVariable() { return m_nbVar; }

  /**
   * @brief Virtual Destructor.
   */
  virtual ~FormulaManager() {}

  /**
   * @brief Given a list of literals, this function assigns this literals.
   *
   * @param[in] lits is the list of literal we search to assigned.
   *
   */
  inline void assignListLit(const std::vector<Lit> &lits) {
    for (auto &l : lits) {
      assert(m_currentValue[l.var()] == l_Undef);
      m_currentValue[l.var()] = l.sign();
    }
  }  // assignListLit

  /**
   * @brief Assign a variable to a value.
   *
   * @param v is the variable we want to assign.
   * @param val is the value given to v.
   */
  inline void assignLit(Var &v, lbool val) {
    m_currentValue[v] = val;
  }  // assignLit

  /**
   * @brief Get the value assigned to a variable.
   *
   * @param v is the variable we are looking for.
   *
   * @return the value assigned to v.
   */
  inline lbool getValue(Var v) { return m_currentValue[v]; }

  /**
   * @brief Get if the given variable is assigned.
   *
   * @param v is the variable.
   *
   * @return true if the variable is assigned, false otherwise.
   */
  inline bool varIsAssigned(Var v) { return m_currentValue[v] != l_Undef; }

  /**
   * @brief Get if the given literal is assigned or not.
   *
   * @param l is the literal we are looking for.
   * @return true if l is assigned, false otherwise.
   */
  inline bool litIsAssigned(Lit l) {
    return m_currentValue[l.var()] != l_Undef;
  }  // litIsAssigned

  /**
   * @brief Ask if the given literal is assigned to.
   *
   * @param l is the literal we are looking for.
   * @return true if l is assigned to true, false otherwise.
   */
  inline bool litIsAssignedToTrue(Lit l) {
    if (l.sign())
      return m_currentValue[l.var()] == l_False;
    else
      return m_currentValue[l.var()] == l_True;
  }  // litIsAssignedToTrue

  /**
   * @brief  Show the set of unit literals.
   *
   * @param[out] out is the stream used.
   */
  void showTrail(std::ostream &out);

  /**
   * @brief Compute a trivial partition of the formula by considering only one
   * component.
   *
   * @param[out] varConnected are the computed connected component.
   * @param setOfVar are the variables under consideration.
   * @param freeVar are the variables their are free (isFreeVar should return
   * true on them).
   *
   * @return the number of connected component.
   */
  virtual int computeTrivialConnectedComponent(
      std::vector<std::vector<Var>> &varConnected, std::vector<Var> &setOfVar,
      std::vector<Var> &freeVar);

  /**
   * @brief Search for the connected component of the formula.
   *
   * @param[out] varConnected are the computed connected component.
   * @param setOfVar are the variables under consideration.
   * @param freeVar are the variables their are free (isFreeVar should return
   * true on them).
   *
   * @return the number of connected component.
   */
  virtual int computeConnectedComponent(
      std::vector<std::vector<Var>> &varConnected, std::vector<Var> &setOfVar,
      std::vector<Var> &freeVar) = 0;

  /**
   * @brief Search for the connected component of the formula regarding a subset
   * of varaibles specified with a boolean vector.
   *
   * @param[out] varConnected are the computed connected component.
   * @param[in] setOfVar are the variables under consideration.
   * @param[in] isTargeted is a boolean vector that specifies the variable
   * under consideration.
   * @param[out] freeVar are the variables their are free (isFreeVar should
   * return true on them).
   *
   * @return the number of connected component.
   */
  virtual int computeConnectedComponentTargeted(
      std::vector<std::vector<Var>> &varConnected, std::vector<Var> &setOfVar,
      std::vector<bool> &isTargeted, std::vector<Var> &freeVar) = 0;

  /**
   * @brief This function is called to update the formula regarding a set of
   * literals we want to assigned.
   *
   * @param[in] lits is the set of literal under consideration.
   */
  virtual void preUpdate(const std::vector<Lit> &lits) = 0;

  /**
   * @brief This function is called to undo the modification done by the
   * function preUpdate.
   *
   * @param lits are the literals updated.
   */
  virtual void postUpdate(const std::vector<Lit> &lits) = 0;

  /**
   * @brief Display the initial formula.
   *
   * @param[out] out is the stream used.
   */
  virtual void showFormula(std::ostream &out) = 0;

  /**
   * @brief Display the current formula.
   *
   * @param[out] out is the stream used.
   */
  virtual void showCurrentFormula(std::ostream &out) = 0;

  /**
   * @brief Display the current formula.
   *
   * @param[out] out is the stream used.
   */
  virtual void showCurrentFormula(std::ostream &out,
                                  std::vector<bool> &isInComponent) = 0;

  /**
   * @brief Get the problem input type.
   *
   * @return a element of the enum ProblemInputType.
   */
  virtual ProblemInputType getProblemInputType() = 0;

  /**
   * @brief Display some information about the object.
   *
   * @param[out] out is the stream used.
   */
  virtual void printInformation(std::ostream &out) {}

  /**
   * @brief Ask if the given variable is free in the current formula.
   *
   * @param[in] v is the variable we are looking for.
   *
   * @return true if the variable is free, false otherwise.
   */
  virtual bool isFreeVariable(Var v) = 0;
};
}  // namespace d4
