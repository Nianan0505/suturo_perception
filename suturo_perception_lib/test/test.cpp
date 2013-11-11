#include "suturo_perception.h"
#include <iostream>
#include <gtest/gtest.h>

TEST(suturo_perception_test, magic_number_is_magic)
{
    SuturoPerception mc;
    EXPECT_EQ(mc.magicNumber(), 23);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}