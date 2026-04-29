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

#include <cassert>
#include <vector>

#include "../ProblemTypes.hpp"

namespace d4 {
/**
 * @brief Manages the sequence of quantification levels for a logic formula.
 *
 * This class stores variables in ordered groups (levels), which typically
 * represent alternating quantifier blocks in a Quantified Boolean Formula
 * (QBF). The outer vector represents the ordered sequence of levels, and the
 * inner vector holds the variables belonging to that specific quantification
 * level.
 */
class Quantification {
 private:
  std::vector<std::vector<Var>> m_quantification;

 public:
  /**
   * @brief Appends a new, empty quantification level to the end of the
   * sequence. This is used to start a new block of variables, typically when
   * switching from an existential to a universal quantifier (or vice versa).
   */
  inline void addLevel() { m_quantification.emplace_back(); }  // addLevel

  /**
   * @brief Appends a new quantification level initialized with a set of
   * variables.
   *
   * This function allows you to add an entire quantifier block in a single
   * operation, bypassing the need to create an empty level and push variables
   * one by one. The provided vector is copied to form the new level at the end
   * of the sequence.
   *
   * @param[in] vars A vector containing the variables to constitute the new
   * level.
   */
  inline void addLevel(const std::vector<Var>& vars) {
    m_quantification.push_back(vars);
  }  // addLevel

  /**
   * @brief Adds a variable to the most recently created quantification level.
   *
   * @pre At least one quantification level must exist before calling this
   * function.
   *
   * @param[in] v The variable to append to the current active level.
   */
  inline void addVariableToLevel(Var v) {
    assert(m_quantification.size());
    m_quantification.back().push_back(v);
  }  // addVariableToLevel

  /**
   * @brief Adds a variable to a specific, existing quantification level.
   *
   * @pre The target `level` must already exist (`level <
   * m_quantification.size()`).
   *
   * @param[in] v     The variable to insert.
   * @param[in] level The zero-indexed level ID where the variable should be
   * placed.
   */
  inline void addVariableToLevel(Var v, unsigned level) {
    assert(level < m_quantification.size());
    m_quantification[level].push_back(v);
  }  // addVariableToLevel

  /**
   * @brief Retrieves the entire quantification structure.
   *
   * @return const std::vector<std::vector<Var>>& A read-only reference to the
   * 2D vector of quantified variables.
   */
  inline const std::vector<std::vector<Var>>& getQuantification() {
    return m_quantification;
  }  // getQuantification

  /**
   * @brief Overwrites the current quantification structure with a new one.
   *
   * @param[in] quantification The new 2D vector of variables to copy into the
   * class.
   */
  inline void setQuantification(
      const std::vector<std::vector<Var>>& quantification) {
    m_quantification = quantification;
  }  // setQuantification

  /**
   * @brief Displays the quantification levels and their associated variables.
   *
   * Prints the hierarchical structure of the quantified variables to standard
   * output. Each level is printed on a new line, making it easy to visually
   * inspect the variable blocks (e.g., alternating forall/exists blocks) for
   * debugging.
   */
  void display(std::ostream& out) const {
    out << "display quantification:\n";
    for (unsigned i = 0; i < m_quantification.size(); ++i) {
      out << "c Level " << i << ": ";
      for (const auto& v : m_quantification[i]) out << v << " ";
      out << '\n';
    }
  }  // display
};
}  // namespace d4