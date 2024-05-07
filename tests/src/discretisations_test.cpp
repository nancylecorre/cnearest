#include "gtest/gtest.h"
#include "cnearest_lib/discretisations/single_point_discretisation.h"

TEST(TestSinglePointDiscretisation, TestSinglePointDiscretisation) {
    
    cnearest::SinglePointDescretisation discretisation(1, 2, 3);

    auto positions = discretisation.positions();

    ASSERT_EQ(positions.size(), 1);
    ASSERT_EQ(positions[0](0), 1);
    ASSERT_EQ(positions[0](1), 2);
    ASSERT_EQ(positions[0](2), 3);

}