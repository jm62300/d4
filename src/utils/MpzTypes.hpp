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

#include <gmpxx.h>

// Canonical home for the arbitrary-precision number types, backed directly
// by gmpxx (already a hard dependency of d4). Every place that used to write
// "namespace mpz = boost::multiprecision;" now instead aliases this single
// namespace, so multiple aliasing declarations never create ambiguous
// distinct namespaces when combined with "using namespace d4;".
namespace d4MpzTypes {
using mpz_int = mpz_class;
using mpf_float = mpf_class;
}  // namespace d4MpzTypes
