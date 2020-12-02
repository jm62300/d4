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
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "gtest/gtest.h"

#include "../src/hashing/HashString.hpp"

#define LEN (1<<6)

TEST(HashString_test, testSameKeyAndLENgth) {
    d4::HashString h;
    EXPECT_EQ(h.hash((char *) "a", LEN), h.hash((char *) "a", LEN));
}

TEST(HashString_test, testSameKeyButNotLENgth) {
    d4::HashString h;
    EXPECT_NE(h.hash((char *) "a", LEN), h.hash((char *) "a", LEN<<1));
}

TEST(HashString_test, testSameLENgthButNotKey) {
    d4::HashString h;
    EXPECT_NE(h.hash((char *) "a", LEN), h.hash((char *) "b", LEN));
}

TEST(HashString_test, testDiffKeyAndLENgth) {
    d4::HashString h;
    EXPECT_NE(h.hash((char *) "a", LEN), h.hash((char *) "b", LEN));
}
