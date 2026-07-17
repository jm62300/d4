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

#include "src/problem/ProblemTypes.hpp"

namespace d4 {

/**
 * @brief Handles the translation of higher-level logic structures into SAT
 * formulas.
 */
class Translator {
 public:
  /**
   * @brief Applies the Tseitin transformation to convert a Boolean circuit into
   * CNF.
   *
   * The Tseitin encoding creates a set of clauses that are equisatisfiable to
   * the original gates. It does this by introducing a new variable (the output)
   * for each gate and constraining it to be logically equivalent to the gate's
   * operation.
   *
   * @param gates   A vector of Boolean circuit gates (AND, OR, IDENTITY) to be
   * encoded.
   * @param clauses The output 2D vector where the generated CNF clauses will be
   * appended.
   */
  static void tseitinEncoding(const std::vector<BcGate>& gates,
                              std::vector<std::vector<Lit>>& clauses) {
    std::vector<Lit> cl;
    for (auto& g : gates) {
      switch (g.gateType) {
        case BcGateType::AND:
          // ENCODING: Output = AND(Inputs)  <=>  (O <-> AND(Inputs))
          // Output implies all Inputs:  (~O V I_i) for all i
          // All Inputs imply Output: (O V ~I_1 V ~I_2 ... V ~I_n)

          cl.clear();
          cl.push_back(g.output);  // Start building clause 2 (Output V ...)

          for (auto l : g.input) {
            // Add clause 1: (~Output V Input_i)
            clauses.push_back({l, ~g.output});

            // Continue building clause 2: (... V ~Input_i)
            cl.push_back(~l);
          }
          // Push the completed giant clause 2
          clauses.push_back(cl);
          break;

        case BcGateType::OR:
          // ENCODING: Output = OR(Inputs)  <=>  (Output <-> OR(Inputs))
          // 1. Any Input implies Output: (~I_i V O) for all i
          // 2. Output implies an Input:  (~O V I_1 V I_2 ... V I_n)

          cl.clear();
          cl.push_back(~g.output);  // Start building clause 2 (~Output V ...)

          for (auto l : g.input) {
            // Add clause 1: (~Input_i V Output)
            clauses.push_back({~l, g.output});

            // Continue building clause 2: (... V Input_i)
            cl.push_back(l);
          }
          // Push the completed giant clause 2
          clauses.push_back(cl);
          break;
        case BcGateType::TERM:
          cl.resize(1);
          for (auto& l : g.input) {
            cl[0] = l;
            clauses.push_back(cl);
          }
          break;
        case BcGateType::CLAUSE:
          clauses.push_back(g.input);
          break;
        case BcGateType::IDENTITY:
          // ENCODING: Output = Input  <=>  (Output <-> Input)
          // 1. Output implies Input: (~Output V Input)
          // 2. Input implies Output: (~Input V Output)
          assert(g.input.size() == 1);
          clauses.push_back({g.input[0], ~g.output});
          clauses.push_back({~g.input[0], g.output});
          break;
      }
    }
  }  // tseitinEncoding
};

}  // namespace d4