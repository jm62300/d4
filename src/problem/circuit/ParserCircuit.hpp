/*
 * d4
 * Copyright (C) 2024  Univ. Artois & CNRS & KU Leuven
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

#include <fstream>

#include "../ProblemTypes.hpp"
#include "src/problem/circuit/ProblemManagerCircuit.hpp"
#include "src/utils/BufferRead.hpp"

namespace d4 {
class ParserCircuit {
 private:
  int parse_circuit_main(std::ifstream &in,
                         ProblemManagerCircuit *problemManager);

 public:
  int parse_circuit(const std::string &input_stream,
                    ProblemManagerCircuit *problemManager);
};
}  // namespace d4
