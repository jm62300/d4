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

#include <boost/multiprecision/gmp.hpp>
#include <map>
#include <string>
#include <vector>

#include "src/problem/ProblemTypes.hpp"

namespace semiring {

// Your namespace alias
namespace mpz = boost::multiprecision;

enum TypeNode { TOP = 0, BOT = 1, AND = 2, OR = 3 };

struct NodeStruct {
  unsigned typeNode : 3;
  unsigned page : 10;
  unsigned posInPage : 32;
};

typedef uint64_t Node;

class DecDNNFSemiring {
 private:
  uint8_t** m_pages = NULL;
  unsigned m_nbPages = 0;

 public:
  // Required by std::default_initializable
  DecDNNFSemiring() = default;

  // Required by your SemiringPolicy constructor constraint
  DecDNNFSemiring(unsigned nbVar,
                  const std::map<d4::Lit, std::string>& literalWeights) {}

  // --- In-Place Multiplication ---
  Node& mul(Node& a, const Node& b) const {
    // TODO
    a *= b;
    return a;
  }

  // --- In-Place Standard Binary Add ---
  Node& add(Node& a, const Node& b) const {
    // TODO
    a += b;
    return a;
  }

  // --- In-Place Binary Add with Smoothing ---
  Node& add(Node& a, const Node& b, const std::vector<d4::Lit>& units) const {
    // TODO
    a += b;
    return a;
  }

  Node& add(Node& a, const Node& b,
            const std::vector<d4::Var>& free_vars) const {
    // TODO
    a += b * one(free_vars);
    return a;
  }

  Node& add(Node& a, const Node& b, const std::vector<d4::Lit>& units,
            const std::vector<d4::Var>& free_vars) const {
    // TODO
    a += b * one(free_vars);
    return a;
  }

  // --- In-Place Unary Adds (Smoothing a single branch) ---
  Node& add(Node& a, const std::vector<d4::Lit>& units) const {
    // TODO
    return a;
  }

  Node& add(Node& a, const std::vector<d4::Var>& free_vars) const {
    // TODO
    a *= one(free_vars);
    return a;
  }

  Node& add(Node& a, const std::vector<d4::Lit>& units,
            const std::vector<d4::Var>& free_vars) const {
    // TODO
    a *= one(free_vars);
    return a;
  }

  // Identities& Context - Aware Leaf Evaluation-- -

  Node zero() const { return 0; }

  Node one() const { return 1; }

  Node one(const std::vector<d4::Lit>& units) const {
    // TODO
    return 1;
  }

  Node one(const std::vector<d4::Var>& free_vars) const {
    // TODO
    return 1;
  }

  Node one(const std::vector<d4::Lit>& units,
           const std::vector<d4::Var>& free_vars) const {
    // TODO
    return 1;
  }

  // Presets (Required by Policy)
  Node presetSum(int nbGates) const {
    // TODO
    return 1;
  }

  Node presetMul(int nbGates) const {
    // TODO
    return 1;
  }
};
}  // namespace semiring