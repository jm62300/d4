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

#include "BufferRead.hpp"
#include "Formula.hpp"

namespace parser {

class ParserDimacs {
 private:
  int parse_DIMACS_main(BufferRead& in, Formula& formula);

 public:
  int parse_DIMACS(const std::string& input_stream, Formula& formula);
  int parse_DIMACS(const char* data, size_t len, Formula& formula);
  int parse_DIMACS_from_data(const std::string& data, Formula& formula);
};

}  // namespace parser
