#include "gtest/gtest.h"

#include "../src/hashing/HashString.hh"

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