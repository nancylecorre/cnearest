#include "force_node_collection.h"
#include "gtest/gtest.h"

class ForceNodeSetTest : public ::testing::Test {
   protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(ForceNodeSetTest, test_creation) {
    cnearest::Vector3 position = {1, 2, 3};
    cnearest::Collection<cnearest::ForceNode> nodes = {position};
    cnearest::ForceNodeCollection set(nodes);
}

TEST_F(ForceNodeSetTest, test_nodes) {
    cnearest::Vector3 position = {1, 2, 3};
    cnearest::Collection<cnearest::ForceNode> nodes = {position};
    cnearest::ForceNodeCollection set(nodes);

    ASSERT_EQ(set.size(), 1);
    ASSERT_EQ(set.nodes().size(), 1);
    ASSERT_EQ(set.nodes()[0].position()(0), position(0));
    ASSERT_EQ(set.nodes()[0].position()(1), position(1));
    ASSERT_EQ(set.nodes()[0].position()(2), position(2));
}