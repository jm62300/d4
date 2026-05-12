/**
 * bipe
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

#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

namespace bipe {

/**
 * @brief Represents a Boolean variable.
 *
 * Variables are typically indexed starting from 1 (human-readable) or 0
 * (internal arrays).
 */
typedef int Var;

/**
 * @brief Represents a lifted boolean (lbool) using an 8-bit unsigned integer.
 *
 * Used to represent three-valued logic: True, False, and Undefined.
 */
typedef uint8_t lbool;

/// Special constant representing an undefined or unassigned variable.
const Var var_Undef = -1;

/// Lifted boolean state representing True.
const lbool l_True = 0;

/// Lifted boolean state representing False.
const lbool l_False = 1;

/// Lifted boolean state representing an unassigned/undefined value.
const lbool l_Undef = 2;

/**
 * @brief Represents a Boolean literal (a variable with an associated sign).
 *
 * This structure packs both the variable index and its sign (polarity) into a
 * single integer `m_x` for highly efficient storage and memory access.
 * The lowest bit (LSB) stores the sign, and the remaining upper bits store the
 * variable.
 */
struct Lit {
  /// The internal packed representation containing the variable and its sign.
  int m_x;

  /**
   * @brief Checks if the literal is negated.
   * @return True if the literal is negated (sign bit is 1), false otherwise.
   */
  inline bool sign() const { return m_x & 1; }

  /**
   * @brief Extracts the underlying variable from the literal.
   * @return The variable index.
   */
  inline Var var() const { return m_x >> 1; }

  /**
   * @brief Returns the negation of this literal.
   * @return A new literal representing the opposite polarity of this one.
   */
  inline Lit neg() const { return {m_x ^ 1}; }

  /**
   * @brief Retrieves the raw internal integer representation.
   * @return The packed integer `m_x`.
   */
  inline unsigned intern() const { return m_x; }

  /**
   * @brief Converts the literal to a human-readable DIMACS format integer.
   * @return A positive integer for true literals, and a negative integer for
   * negated literals.
   */
  inline int human() const { return (m_x & 1) ? -var() : var(); }

  bool operator==(Lit p) const { return m_x == p.m_x; }
  bool operator!=(Lit p) const { return m_x != p.m_x; }

  /**
   * @brief Compares two literals based on their internal representation.
   *
   * This specific ordering guarantees that a literal `p` and its negation `~p`
   * are adjacent to each other when sorted.
   */
  bool operator<(Lit p) const { return m_x < p.m_x; }

  friend Lit operator~(Lit p);
  friend std::ostream& operator<<(std::ostream& os, Lit l);

  /**
   * @brief Factory method to construct a literal from a variable and a sign.
   * @param v The boolean variable.
   * @param sign True if the literal should be negated, false otherwise.
   * @return The constructed literal.
   */
  static inline Lit makeLit(Var v, bool sign) { return {(v << 1) + sign}; }

  /**
   * @brief Factory method to construct a negative (false) literal.
   * @param v The boolean variable.
   * @return The constructed negative literal (~v).
   */
  static inline Lit makeLitFalse(Var v) { return {(v << 1) + 1}; }

  /**
   * @brief Factory method to construct a positive (true) literal.
   * @param v The boolean variable.
   * @return The constructed positive literal (v).
   */
  static inline Lit makeLitTrue(Var v) { return {v << 1}; }
};

/// Useful special constant representing an undefined literal.
const Lit lit_Undef = {-2};

/// Useful special constant representing an erroneous literal state.
const Lit lit_Error = {-1};

/**
 * @brief Utility function to print a list of literals to an output stream.
 *
 * @param out The output stream (e.g., std::cout).
 * @param v The vector of literals to display.
 */
inline void showListLit(std::ostream& out, std::vector<Lit>& v) {
  for (auto& l : v) out << l << " ";
}

/**
 * @brief Overloads the bitwise NOT operator to logically negate a literal.
 *
 * Flips the lowest bit to seamlessly toggle the polarity of the literal.
 *
 * @param p The literal to negate.
 * @return The negated literal.
 */
inline Lit operator~(Lit p) { return {p.m_x ^ 1}; }

/**
 * @brief Represents a Boolean clause (a disjunction of literals).
 *
 * Uses the "flexible array member" idiom (zero-length array `data[0]`).
 * This allows a `Clause` object and its literal data to be allocated
 * contiguously in a single block of memory, which drastically improves
 * CPU cache locality and performance during SAT solving.
 */
struct Clause {
  /// The total number of literals contained in this clause.
  unsigned size;

  /// The contiguous array of literals (flexible array member).
  Lit data[0];

  /**
   * @brief Accesses a literal at a specific index within the clause.
   * @param idx The index of the literal to retrieve.
   * @return A reference to the literal at the given index.
   */
  Lit& operator[](std::size_t idx) { return data[idx]; }

  /**
   * @brief Swaps the positions of two literals within the clause.
   *
   * Useful for managing watched literals without reallocating arrays.
   *
   * @param i The index of the first literal.
   * @param j The index of the second literal.
   */
  inline void swap(unsigned i, unsigned j) {
    assert(i < size && j < size);
    Lit tmp = data[i];
    data[i] = data[j];
    data[j] = tmp;
  }

  /**
   * @brief Prints the contents of the clause to an output stream.
   *
   * @param out The output stream used to display the clause.
   */
  inline void display(std::ostream& out) {
    for (unsigned i = 0; i < size; i++) out << data[i] << " ";
    std::cout << "\n";
  }
};

}  // namespace bipe