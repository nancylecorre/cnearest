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

 TEST_F(TrivialTest, test_eigen_using_blas) {

     // if this runs slower than expected may indicate that the blas connection has failed

     // machine dependent but:
     // without blas this takes ~11 sec
     // with openblas this takes < 0.2 sec

     Eigen::MatrixXd a = Eigen::MatrixXd::Random(1000, 1000);
     Eigen::MatrixXd b = Eigen::MatrixXd::Random(1000, 1000);
     Eigen::MatrixXd c = a * b;
 }

 TEST_F(TrivialTest, test_eigen_using_lapacke) {

     // if this runs slower than expected may indicate that the blas/lapacke connection has failed

     // machine dependent but:
     // without blas this takes ~14 sec
     // with openblas this takes < 0.5 sec

     Eigen::MatrixXd a = Eigen::MatrixXd::Random(1500, 1500);
     Eigen::VectorXd b = Eigen::VectorXd::Random(1500);
     auto lu = a.lu();

     auto c = lu.solve(b);
 }
