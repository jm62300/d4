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
#include <concepts>
#include <map>
#include <string>
#include <vector>

namespace d4 {

template <typename T>
concept NumberType = std::regular<T> && requires(T a, const T b) {
  { a += b } -> std::same_as<T&>;
  { a *= b } -> std::same_as<T&>;
};

template <typename O, typename T>
concept SemiringPolicy =
    // Must be default constructible (e.g., O my_semiring;)
    std::default_initializable<O>

    // Must be constructible with your specific parsed variables
    && std::constructible_from<O, const std::vector<Var>&,
                               const std::map<Lit, std::string>&>

    // The method signatures
    && requires(O& ops, T a, T b, int preset_val, const std::vector<Lit>& units,
                const std::vector<Var>& free_vars) {
         { ops.add(a, b) } -> std::convertible_to<T>;
         { ops.add(a, b, units) } -> std::convertible_to<T>;
         { ops.add(a, b, free_vars) } -> std::convertible_to<T>;
         { ops.add(a, b, units, free_vars) } -> std::convertible_to<T>;
         { ops.add(b, units, free_vars) } -> std::convertible_to<T>;

         { ops.mul(a, b) } -> std::convertible_to<T>;

         { ops.zero() } -> std::convertible_to<T>;

         { ops.one() } -> std::convertible_to<T>;
         { ops.one(units) } -> std::convertible_to<T>;
         { ops.one(free_vars) } -> std::convertible_to<T>;
         { ops.one(units, free_vars) } -> std::convertible_to<T>;

         { ops.presetSum(preset_val) } -> std::convertible_to<T>;
         { ops.presetMul(preset_val) } -> std::convertible_to<T>;
       };

}  // namespace d4