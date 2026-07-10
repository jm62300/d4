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

#include <map>
#include <string>
#include <vector>

#include "src/problem/ProblemTypes.hpp"

namespace semiring {

class EmptySemiring {
 private:
  bool one(const std::vector<d4::Lit>& units) const { return true; }
  bool one(const std::vector<d4::Var>& free_vars) const { return true; }
  bool one(const std::vector<d4::Lit>& units,
           const std::vector<d4::Var>& free_vars) const {
    return true;
  }

 public:
  // Required by std::default_initializable
  EmptySemiring() = default;

  // Required by your SemiringPolicy constructor constraint
  EmptySemiring(unsigned nbVar,
                const std::map<d4::Lit, std::string>& literalWeights) {}

  // --- In-Place Multiplication ---
  bool& mul(bool& a, const bool& b) const { return a; }

  bool& add(bool& a, const bool& b, const std::vector<d4::Lit>& units,
            const std::vector<d4::Var>& free_vars) const {
    return a;
  }

  // Identities& Context - Aware Leaf Evaluation-- -

  static bool zero() { return false; }
  static bool one() { return false; }

  // --- Presets (Required by Policy) ---
  bool presetSum(int /* nb_gates */) const { return true; }
  bool presetMul(int /* nb_gates */) const { return true; }
};
}  // namespace semiring