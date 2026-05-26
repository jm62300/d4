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

class DecDNNFSemiring {
 public:
  // Required by std::default_initializable
  DecDNNFSemiring() = default;

  // Required by your SemiringPolicy constructor constraint
  DecDNNFSemiring(unsigned nbVar,
                  const std::map<d4::Lit, std::string>& literalWeights) {}

  // --- In-Place Multiplication ---
  mpz::mpz_int& mul(mpz::mpz_int& a, const mpz::mpz_int& b) const {
    a *= b;
    return a;
  }

  // --- In-Place Standard Binary Add ---
  mpz::mpz_int& add(mpz::mpz_int& a, const mpz::mpz_int& b) const {
    a += b;
    return a;
  }

  // --- In-Place Binary Add with Smoothing ---
  mpz::mpz_int& add(mpz::mpz_int& a, const mpz::mpz_int& b,
                    const std::vector<d4::Lit>& units) const {
    a += b;
    return a;
  }

  mpz::mpz_int& add(mpz::mpz_int& a, const mpz::mpz_int& b,
                    const std::vector<d4::Var>& free_vars) const {
    a += b * one(free_vars);
    return a;
  }

  mpz::mpz_int& add(mpz::mpz_int& a, const mpz::mpz_int& b,
                    const std::vector<d4::Lit>& units,
                    const std::vector<d4::Var>& free_vars) const {
    a += b * one(free_vars);
    return a;
  }

  // --- In-Place Unary Adds (Smoothing a single branch) ---
  mpz::mpz_int& add(mpz::mpz_int& a, const std::vector<d4::Lit>& units) const {
    return a;
  }

  mpz::mpz_int& add(mpz::mpz_int& a,
                    const std::vector<d4::Var>& free_vars) const {
    a *= one(free_vars);
    return a;
  }

  mpz::mpz_int& add(mpz::mpz_int& a, const std::vector<d4::Lit>& units,
                    const std::vector<d4::Var>& free_vars) const {
    a *= one(free_vars);
    return a;
  }

  // Identities& Context - Aware Leaf Evaluation-- -

  mpz::mpz_int zero() const { return mpz::mpz_int(0); }

  mpz::mpz_int one() const { return mpz::mpz_int(1); }

  mpz::mpz_int one(const std::vector<d4::Lit>& units) const {
    return mpz::mpz_int(1);
  }

  mpz::mpz_int one(const std::vector<d4::Var>& free_vars) const {
    return mpz::mpz_int(1) << free_vars.size();
  }

  mpz::mpz_int one(const std::vector<d4::Lit>& units,
                   const std::vector<d4::Var>& free_vars) const {
    return mpz::mpz_int(1) << free_vars.size();
  }

  // --- 5. Presets (Required by Policy) ---
  mpz::mpz_int presetSum(int /* gate_id */) const { return mpz::mpz_int(0); }

  mpz::mpz_int presetMul(int /* gate_id */) const { return mpz::mpz_int(1); }
};
}  // namespace semiring