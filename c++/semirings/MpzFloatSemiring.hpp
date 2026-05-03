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

class MpzFloatSemiring {
 private:
  std::vector<mpz::mpf_float> m_weightLit;
  std::vector<mpz::mpf_float> m_weightVar;

 public:
  // Required by std::default_initializable
  MpzFloatSemiring() = default;

  // Required by your SemiringPolicy constructor constraint
  MpzFloatSemiring(unsigned nbVar,
                   const std::map<d4::Lit, std::string>& literalWeights) {
    m_weightLit.resize(2 * (nbVar + 1), mpz::mpf_float(1));
    for (const auto& [lit, weight] : literalWeights) {
      m_weightLit[lit.intern()] = mpz::mpf_float(weight);
    }

    m_weightVar.resize(nbVar + 1);
    for (unsigned i = 0; i <= nbVar; i++)
      m_weightVar[i] = m_weightLit[i << 1] + m_weightLit[1 + (i << 1)];
  }

  // --- In-Place Multiplication ---
  mpz::mpf_float& mul(mpz::mpf_float& a, const mpz::mpf_float& b) const {
    a *= b;
    return a;
  }

  // --- In-Place Standard Binary Add ---
  mpz::mpf_float& add(mpz::mpf_float& a, const mpz::mpf_float& b) const {
    a += b;
    return a;
  }

  // --- In-Place Binary Add with Smoothing ---
  mpz::mpf_float& add(mpz::mpf_float& a, const mpz::mpf_float& b,
                      const std::vector<d4::Lit>& units) const {
    a += b * one(units);
    return a;
  }

  mpz::mpf_float& add(mpz::mpf_float& a, const mpz::mpf_float& b,
                      const std::vector<d4::Var>& free_vars) const {
    a += b * one(free_vars);
    return a;
  }

  mpz::mpf_float& add(mpz::mpf_float& a, const mpz::mpf_float& b,
                      const std::vector<d4::Lit>& units,
                      const std::vector<d4::Var>& free_vars) const {
    a += b * one(units, free_vars);
    return a;
  }

  // --- In-Place Unary Adds (Smoothing a single branch) ---
  mpz::mpf_float& add(mpz::mpf_float& a,
                      const std::vector<d4::Lit>& units) const {
    a *= one(units);
    return a;
  }

  mpz::mpf_float& add(mpz::mpf_float& a,
                      const std::vector<d4::Var>& free_vars) const {
    a *= one(free_vars);
    return a;
  }

  mpz::mpf_float& add(mpz::mpf_float& a, const std::vector<d4::Lit>& units,
                      const std::vector<d4::Var>& free_vars) const {
    a *= one(units) * one(free_vars);
    return a;
  }

  // Identities& Context - Aware Leaf Evaluation-- -

  mpz::mpf_float zero() const { return mpz::mpf_float(0); }

  mpz::mpf_float one() const { return mpz::mpf_float(1); }

  mpz::mpf_float one(const std::vector<d4::Lit>& units) const {
    auto ret = mpz::mpf_float(1);
    for (auto l : units) ret *= m_weightLit[l.intern()];
    return ret;
  }

  mpz::mpf_float one(const std::vector<d4::Var>& free_vars) const {
    auto ret = mpz::mpf_float(1);
    for (auto v : free_vars) ret *= m_weightVar[v];
    return ret;
  }

  mpz::mpf_float one(const std::vector<d4::Lit>& units,
                     const std::vector<d4::Var>& free_vars) const {
    auto ret = mpz::mpf_float(1);
    for (auto l : units) ret *= m_weightLit[l.intern()];
    for (auto v : free_vars) ret *= m_weightVar[v];
    return ret;
  }

  // --- 5. Presets (Required by Policy) ---
  mpz::mpf_float presetSum(int /* gate_id */) const {
    return mpz::mpf_float(0);
  }

  mpz::mpf_float presetMul(int /* gate_id */) const {
    return mpz::mpf_float(1);
  }
};
}  // namespace semiring