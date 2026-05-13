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
#include <map>

#include "src/exceptions/FactoryException.hpp"
#include "src/options/Option.hpp"
#include "src/options/OptionRegistry.hpp"
#include "src/options/EnumMetadata.hpp"

namespace d4 {
enum SpecUpdateType {
  SPEC_DYNAMIC,
  SPEC_DYNAMIC_BLOCKED_SIMP,
  SPEC_DYNAMIC_PURE_SIMP
};

class SpecUpdateManager {
 public:
  static std::string getSpecUpdate(const SpecUpdateType& m) {
    if (m == SPEC_DYNAMIC) return "dynamic";
    if (m == SPEC_DYNAMIC_BLOCKED_SIMP) return "dynamicBlockedSimp";
    if (m == SPEC_DYNAMIC_PURE_SIMP) return "dynamicPureSimp";

    throw(
        FactoryException("FormulaManager Update unknown", __FILE__, __LINE__));
  }  // getOperatorType

  static SpecUpdateType getSpecUpdate(const std::string& m) {
    if (m == "dynamic") return SPEC_DYNAMIC;
    if (m == "dynamicBlockedSimp") return SPEC_DYNAMIC_BLOCKED_SIMP;
    if (m == "dynamicPureSimp") return SPEC_DYNAMIC_PURE_SIMP;

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getSpectUpdate

  static std::map<int, std::string> getMapping() {
    return {
        {SPEC_DYNAMIC, "dynamic"},
        {SPEC_DYNAMIC_BLOCKED_SIMP, "dynamicBlockedSimp"},
        {SPEC_DYNAMIC_PURE_SIMP, "dynamicPureSimp"}};
  }
};

template <>
struct EnumMetadata<SpecUpdateType> {
  static std::string name() { return "SpecUpdateType"; }
  static std::map<int, std::string> mapping() { return SpecUpdateManager::getMapping(); }
};

class OptionSpecManager : public OptionGroup {
 public:
  OptionSpecManager(const std::string& name = "spec", const std::string& description = "Formula manager options")
      : OptionGroup(name, description) {}

  Option<SpecUpdateType> specUpdateType{"specUpdateType", "The occurrence manager used", SPEC_DYNAMIC};
  Option<bool> removeGates{"removeGates", "If some gates can be removed during the search", false};
  Option<bool> needFastNotSatisfied{"needFastNotSatisfied", "If we need fast not satisfied", false};

  std::vector<OptionBase*> getAllOptions() override {
    return {(OptionBase*)&specUpdateType, (OptionBase*)&removeGates, (OptionBase*)&needFastNotSatisfied};
  }

  friend std::ostream& operator<<(std::ostream& out, const OptionSpecManager& dt) {
    out << " Option Formula Manager:" << " update mode("
        << SpecUpdateManager::getSpecUpdate(dt.specUpdateType.get()) << ") rm-gates("
        << dt.removeGates.get() << ") need-not-satisfied(" << dt.needFastNotSatisfied.get()
        << ") ";
    return out;
  }
};
}  // namespace d4