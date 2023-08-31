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

#include <string>

#include "src/exceptions/FactoryException.hpp"

namespace d4 {

enum OperationType { COUNTING, DDNNF_COMPILER };

class OperationTypeManager {
 public:
  static std::string getOperatorType(const OperationType& m) {
    if (m == COUNTING) return "counting";
    if (m == DDNNF_COMPILER) return "ddnnf-compiler";

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getOperatorType

  static OperationType getOperatorType(const std::string& m) {
    if (m == "counting") return COUNTING;
    if (m == "ddnnf-compiler") return DDNNF_COMPILER;

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getOperatorType
};

class OptionOperationManager {
 public:
  OperationType operatorType;
  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionOperationManager& dt) {
    out << " Operator option:"
        << " operator type("
        << OperationTypeManager::getOperatorType(dt.operatorType) << ")";
    return out;
  }  // <<
};

}  // namespace d4