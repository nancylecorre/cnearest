#include "gtest/gtest.h"
#include "force_node.h"

class ForceNodesTest : public ::testing::Test {
   protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(ForceNodesTest, test_creation) { 
cnearest::Vector3 position = {1, 2, 3};
    cnearest::ForceNode node(position);

}


TEST_F(ForceNodesTest, test_position) {
    cnearest::Vector3 position = {1, 2, 3};
    cnearest::ForceNode node(position);

    ASSERT_EQ(node.position()(0), position(0));
    ASSERT_EQ(node.position()(1), position(1));
    ASSERT_EQ(node.position()(2), position(2));
}


class ForceNodesParameterizedTests : public ::testing::TestWithParam<cnearest::Vector3> {
};

TEST_P(ForceNodesParameterizedTests, TestPositions) {
    cnearest::Vector3 position = GetParam();
    cnearest::ForceNode node(position);
    ASSERT_EQ(node.position()(0), position(0));
    ASSERT_EQ(node.position()(1), position(1));
    ASSERT_EQ(node.position()(2), position(2));
}



INSTANTIATE_TEST_SUITE_P(
    ForceNodesParameterizedTests_in_Z, ForceNodesParameterizedTests,
    ::testing::Values(cnearest::Vector3({0, 0, std::numeric_limits<double>::max()}), 
                      cnearest::Vector3({0, 0, 1e8}),
                      cnearest::Vector3({0,0,1000}),
                      cnearest::Vector3({0, 0, 1.0}),
                      cnearest::Vector3({0, 0, std::numeric_limits<double>::min()}),
                      cnearest::Vector3({0, 0, 0.0}),
                      cnearest::Vector3({0, 0, -std::numeric_limits<double>::min()}),
                      cnearest::Vector3({0,0,-1.0}), 
                      cnearest::Vector3({0,0,-1000}),
                      cnearest::Vector3({0, 0, -1e8}), 
                      cnearest::Vector3({0, 0, -std::numeric_limits<double>::max()})));


INSTANTIATE_TEST_SUITE_P(
    ForceNodesParameterizedTests_in_Y, ForceNodesParameterizedTests,
    ::testing::Values(cnearest::Vector3({0, std::numeric_limits<double>::max(),0}),
                      cnearest::Vector3({0, 1e8,0}), 
                      cnearest::Vector3({0, 1000,0}),
                      cnearest::Vector3({0, 1.0,0}),
                      cnearest::Vector3({0, std::numeric_limits<double>::min(),0}),
                      cnearest::Vector3({0, 0.0,0}),
                      cnearest::Vector3({0, -std::numeric_limits<double>::min(),0}),
                      cnearest::Vector3({0, -1.0, 0}), 
                      cnearest::Vector3({0, -1000,0}),
                      cnearest::Vector3({0, -1e8,0}),
                      cnearest::Vector3({0, -std::numeric_limits<double>::max(), 0})));


INSTANTIATE_TEST_SUITE_P(
    ForceNodesParameterizedTests_in_XY, ForceNodesParameterizedTests,
    ::testing::Values(cnearest::Vector3({std::numeric_limits<double>::max(),
                                         std::numeric_limits<double>::max(), 0}),
                      cnearest::Vector3({1e8, 1e8, 0}), cnearest::Vector3({0, 1000, 0}),
                      cnearest::Vector3({1.0, 1.0, 0}),
                      cnearest::Vector3({std::numeric_limits<double>::min(),
                                         std::numeric_limits<double>::min(), 0}),
                      cnearest::Vector3({0, 0.0, 0}),
                      cnearest::Vector3({-std::numeric_limits<double>::min(),
                                         -std::numeric_limits<double>::min(), 0}),
                      cnearest::Vector3({-1.0, -1.0,0}), cnearest::Vector3({0, -1000, 0}),
                      cnearest::Vector3({-1e8, -1e8, 0}),
                      cnearest::Vector3({-std::numeric_limits<double>::max(),
                                         -std::numeric_limits<double>::max(), 0})));