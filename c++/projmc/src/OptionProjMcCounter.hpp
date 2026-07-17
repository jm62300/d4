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

#include <iostream>
#include <string>

#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"

namespace d4 {

class OptionProjMcCounter : public OptionGroup {
 public:
  OptionProjMcCounter(const std::string& name = "projmc",
                      const std::string& description =
                          "Projected model counter output options")
      : OptionGroup(name, description) {}

  Option<std::string> format{"format", "Output symbol", "s"};
  Option<std::string> outFormat{"outFormat", "Output style (classic|competition)",
                                "classic"};

  std::vector<OptionBase*> getAllOptions() override {
    return {&format, &outFormat};
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionProjMcCounter& dt) {
    out << " OptionProjMcCounter: format(" << dt.format.get() << ") outFormat("
        << dt.outFormat.get() << ")\n";
    return out;
  }
};

}  // namespace d4
