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

#include "src/treeDecomposition/TreeDecomposition.hpp"

namespace d4 {
class OptionTreeDecomposition {
 protected:
  TreeDecompositionMethod m_treeDecompositionMethod;

  /**
   * @brief Default constructor for OptionTreeDecomposition.
   *
   * This constructor is made protected to prevent instantiation of the base
   * class directly. It ensures that only derived classes can create instances
   * of OptionTreeDecomposition.
   */
  OptionTreeDecomposition() {}

  /**
   * @brief Display information about the option used to compute the tree
   * decomposition.
   *
   * @param out The output stream to which the object is printed.
   */
  virtual void display(std::ostream& out) const {}

 public:
  /**
   * @brief Virtual destructor for the OptionTreeDecomposition class.
   *
   * Ensures proper cleanup of derived class instances when deleted through a
   * base class pointer.
   */
  virtual ~OptionTreeDecomposition() {}

  /**
   * @brief Retrieves the tree decomposition method being used.
   *
   * This function returns the method applied for computing the tree
   * decomposition of a given formula. The method is represented as an
   * enumerated value of type TreeDecompositionMethod.
   *
   * @return The tree decomposition method currently set.
   */
  inline TreeDecompositionMethod getTreeDecompositionMethod() {
    return m_treeDecompositionMethod;
  }  // getTreeDecompositionMethod

  /**
   * @brief Overloads the stream output operator for OptionTreeDecomposition.
   *
   * This function allows an OptionTreeDecomposition object to be printed
   * directly to an output stream, such as std::cout, in a user-friendly format.
   * Declaring it as a friend function enables it to access the private and
   * protected members of OptionTreeDecomposition.
   *
   * @param out The output stream to which the object is printed.
   * @param dt The OptionTreeDecomposition object to be displayed.
   * @return A reference to the output stream after writing the object.
   */
  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionTreeDecomposition& dt);

  /**
   * @brief Creates a tree decomposition based on the selected options.
   *
   * This is a pure virtual function that must be implemented by derived
   * classes. It is responsible for constructing and returning a tree
   * decomposition object according to the specified configuration options.
   *
   * @param[in] inType gives information about the type of formula under
   * consideration.
   *
   * @return A pointer to a TreeDecomposition instance created based on the
   * options.
   */
  virtual TreeDecomposition* createTreeDecomposition(
      const ProblemInputType& inType) = 0;
};
}  // namespace d4