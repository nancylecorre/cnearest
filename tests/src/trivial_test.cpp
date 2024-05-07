#include "gtest/gtest.h"
#include "definitions.h"


class TrivialTest : public ::testing::Test {
 
 protected:
  virtual void SetUp(){};

  virtual void TearDown(){};

};

 TEST_F(TrivialTest, test_gtest) {
   ASSERT_TRUE(true);
 }

 
 TEST_F(TrivialTest, test_eigen_works) {
   Eigen::Vector3d a ={0,0,1};
   Eigen::Vector3d b ={0,0,2};
   ASSERT_EQ(a.dot(b), 2.0);
 }

