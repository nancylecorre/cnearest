#include "gtest/gtest.h"
#include "regularised_stokeslet.h"

struct DataRow {
    DataRow(cnearest::Vector3 offset, double epsilon, Eigen::Vector<double, 9> expected_result)
        : offset(offset), epsilon(epsilon), expected_result(expected_result.reshaped(3,3)) {}
    cnearest::Vector3 offset;
    double epsilon;
    cnearest::Matrix3 expected_result;
};

class RegularisedStokesletTensorTests : public ::testing::TestWithParam<DataRow> {};

TEST_P(RegularisedStokesletTensorTests, TestValues) {
    auto&[offset,epsilon, expected_result] = GetParam();
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q(offset);

    auto s = cnearest::RegularisedStokeslet::stokeslet_tensor(f, q, epsilon);
    EXPECT_NEAR(s(0, 0), expected_result(0, 0), 1.0e-9);
    EXPECT_NEAR(s(0, 1), expected_result(0, 1), 1.0e-9);
    EXPECT_NEAR(s(0, 2), expected_result(0, 2), 1.0e-9);

    EXPECT_NEAR(s(1, 0), expected_result(1, 0), 1.0e-9);
    EXPECT_NEAR(s(1, 1), expected_result(1, 1), 1.0e-9);
    EXPECT_NEAR(s(1, 2), expected_result(1, 2), 1.0e-9);

    EXPECT_NEAR(s(2, 0), expected_result(2, 0), 1.0e-9);
    EXPECT_NEAR(s(2, 1), expected_result(2, 1), 1.0e-9);
    EXPECT_NEAR(s(2, 2), expected_result(2, 2), 1.0e-9);
}

INSTANTIATE_TEST_SUITE_P(
    RegularisedStokesletTensorTests, RegularisedStokesletTensorTests,
    ::testing::Values(DataRow(
        {1, 0, 0}, 1.0e-5, {2, 0, 0, 0, 1, 0, 0, 0, 1}),
        DataRow({0, 1, 0}, 1.0e-5, {1, 0, 0, 0, 2, 0, 0, 0, 1}),
        DataRow({0, 0, 1}, 1.0e-5, {1, 0, 0, 0, 1, 0, 0, 0, 2}),
        DataRow({2, 0, 0}, 1.0e-5, {1, 0, 0, 0, 0.5, 0, 0, 0, 0.5}),
        DataRow({0, 2, 0}, 1.0e-5, {0.5, 0, 0, 0, 1, 0, 0, 0, 0.5}),
        DataRow({0, 0, 2}, 1.0e-5, {0.5, 0, 0, 0, 0.5, 0, 0, 0, 1})

    )

);