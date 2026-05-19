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

#include "ParserDimacs.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "OptionCounter.hpp"

/**
 * @brief Run a counter using a configuration object.
 *
 * @param config        The DPLL method configuration.
 * @param optionCounter The counter-specific options.
 * @param formula       The parsed input formula.
 */
void counterDemo(const d4::OptionDpllStyleMethod& config,
                 const d4::OptionCounter& optionCounter,
                 const parser::Formula& formula);