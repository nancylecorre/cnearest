#include "cnearest_lib/quadrature_node_set.h"
#include "gtest/gtest.h"

class QuadratureNodeSetTest : public ::testing::Test {
   protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(QuadratureNodeSetTest, test_creation) {
    cnearest::Vector3 position = {1, 2, 3};
    cnearest::Collection<cnearest::QuadratureNode> nodes = {position};
    cnearest::QuadratureNodeSet set(nodes);
}

TEST_F(QuadratureNodeSetTest, test_nodes) {
    cnearest::Vector3 position = {1, 2, 3};
    cnearest::Collection<cnearest::QuadratureNode> nodes = {position};
    cnearest::QuadratureNodeSet set(nodes);

    ASSERT_EQ(set.nodes().size(), 1);
    ASSERT_EQ(set.nodes()[0].position()(0), position(0));
    ASSERT_EQ(set.nodes()[0].position()(1), position(1));
    ASSERT_EQ(set.nodes()[0].position()(2), position(2));
}