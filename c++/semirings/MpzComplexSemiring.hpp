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
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "src/problem/ProblemTypes.hpp"
#include "src/utils/MpzTypes.hpp"

namespace semiring {

namespace mpz = d4MpzTypes;

class Complex {
 public:
  mpz::mpf_float real, im;

  Complex() : real(0), im(0) {}
  Complex(mpz::mpf_float r, mpz::mpf_float i) : real(r), im(i) {}
  Complex(const std::vector<std::string>& v) {
    assert(v.size() == 2);
    real = mpz::mpf_float(v[0]);
    im = mpz::mpf_float(v[1]);
  }

  Complex(const std::string& s) {
    std::vector<std::string> elements;
    std::istringstream iss(s);
    std::string current_word;

    while (iss >> current_word) {
      elements.push_back(current_word);
    }

    // A formula can mix plain real weights ("c p weight", one token) with
    // true complex pairs ("c p complex", two tokens) on literals of the same
    // (complex-typed) formula; a lone token means an implicit zero imaginary
    // part.
    assert(elements.size() == 1 || elements.size() == 2);
    real = mpz::mpf_float(elements[0]);
    im = (elements.size() == 2) ? mpz::mpf_float(elements[1])
                                : mpz::mpf_float(0);
  }

  Complex& operator+=(Complex const& obj) {
    real += obj.real;
    im += obj.im;
    return *this;  // Returns a reference to itself
  }

  Complex& operator*=(Complex const& obj) {
    mpz::mpf_float new_real = real * obj.real - im * obj.im;
    mpz::mpf_float new_im = real * obj.im + im * obj.real;

    real = new_real;
    im = new_im;
    return *this;
  }

  Complex operator*(Complex const& obj) const {
    Complex res(real * obj.real - im * obj.im, real * obj.im + im * obj.real);
    return res;
  }

  Complex operator+(Complex const& obj) const {
    Complex res(real + obj.real, im + obj.im);
    return res;
  }

  inline mpz::mpf_float norm() const { return real * real + im * im; }

  bool operator==(Complex const& obj) const {
    return real == obj.real && im == obj.im;
  }

  bool operator<(Complex const& obj) const { return norm() < obj.norm(); }
  bool operator>(Complex const& obj) const { return norm() > obj.norm(); }
  bool operator<=(Complex const& obj) const { return norm() <= obj.norm(); }
  bool operator>=(Complex const& obj) const { return norm() >= obj.norm(); }

  friend std::ostream& operator<<(std::ostream& os, const Complex& dt) {
    os << std::scientific << std::setprecision(8);
    os << dt.real.get_d() << " + " << dt.im.get_d() << 'i';
    os << std::defaultfloat;
    return os;
  }
};

class MpzComplexSemiring {
 private:
  std::vector<Complex> m_weightLit;
  std::vector<Complex> m_weightVar;

  Complex one(const std::vector<d4::Lit>& units) const {
    auto ret = Complex(1, 0);
    for (auto l : units) ret *= m_weightLit[l.intern()];
    return ret;
  }

  Complex one(const std::vector<d4::Var>& free_vars) const {
    auto ret = Complex(1, 0);
    for (auto v : free_vars) ret *= m_weightVar[v];
    return ret;
  }

  Complex one(const std::vector<d4::Lit>& units,
              const std::vector<d4::Var>& free_vars) const {
    auto ret = Complex(1, 0);
    for (auto l : units) ret *= m_weightLit[l.intern()];
    for (auto v : free_vars) ret *= m_weightVar[v];
    return ret;
  }

 public:
  // Required by std::default_initializable
  MpzComplexSemiring() = default;

  // Required by your SemiringPolicy constructor constraint
  MpzComplexSemiring(unsigned nbVar,
                     const std::map<d4::Lit, std::string>& literalWeights) {
    m_weightLit.resize(2 * (nbVar + 1), Complex(1, 0));
    for (const auto& [lit, weight] : literalWeights) {
      m_weightLit[lit.intern()] = Complex(weight);
    }

    m_weightVar.resize(nbVar + 1);
    for (unsigned i = 0; i <= nbVar; i++)
      m_weightVar[i] = m_weightLit[i << 1] + m_weightLit[1 + (i << 1)];
  }

  // --- In-Place Multiplication ---
  Complex& mul(Complex& a, const Complex& b) const {
    a *= b;
    return a;
  }

  Complex& add(Complex& a, const Complex& b, const std::vector<d4::Lit>& units,
               const std::vector<d4::Var>& free_vars) const {
    a += b * one(units, free_vars);
    return a;
  }

  // Identities& Context - Aware Leaf Evaluation-- -

  static Complex zero() { return Complex(0, 0); }

  static Complex one() { return Complex(1, 0); }

  // --- Presets (Required by Policy) ---
  Complex presetSum(int /* nb_gates */) const { return Complex(0, 0); }

  Complex presetMul(int /* nb_gates */) const { return Complex(1, 0); }
};
}  // namespace semiring