#include "regularised_stokeslet.h"
#include "gtest/gtest.h"

class RegularisedStokesletTest : public ::testing::Test {
   protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(RegularisedStokesletTest, test_stokeslet_tensor_size) {

    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 0, 1});

    auto s = cnearest::RegularisedStokeslet::stokeslet_tensor(f, q, 1.0e-5);
    ASSERT_EQ(s.rows(), 3);
    ASSERT_EQ(s.cols(), 3);
}

TEST_F(RegularisedStokesletTest, test_stokeslet_trivial) { ASSERT_TRUE(true); }

TEST_F(RegularisedStokesletTest, test_stokeslet_tensor_finite_values) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 0, 1});

    auto s = cnearest::RegularisedStokeslet::stokeslet_tensor(f, q, 1.0e-5);
    ASSERT_TRUE(std::isfinite(s(0, 0)));
    ASSERT_TRUE(std::isfinite(s(0, 1)));
    ASSERT_TRUE(std::isfinite(s(0, 2)));

    ASSERT_TRUE(std::isfinite(s(1, 0)));
    ASSERT_TRUE(std::isfinite(s(1, 1)));
    ASSERT_TRUE(std::isfinite(s(1, 2)));

    ASSERT_TRUE(std::isfinite(s(2, 0)));
    ASSERT_TRUE(std::isfinite(s(2, 1)));
    ASSERT_TRUE(std::isfinite(s(2, 2)));
}

TEST_F(RegularisedStokesletTest, test_stokeslet_tensor_unit_x_offset) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({1, 0, 0});

    auto s = cnearest::RegularisedStokeslet::stokeslet_tensor(f, q, 1.0e-5);
    EXPECT_NEAR(s(0, 0),2,1.0e-9);
    EXPECT_NEAR(s(0, 1),0,1.0e-9);
    EXPECT_NEAR(s(0, 2),0,1.0e-9);

    EXPECT_NEAR(s(1, 0),0,1.0e-9);
    EXPECT_NEAR(s(1, 1),1,1.0e-9);
    EXPECT_NEAR(s(1, 2),0,1.0e-9);

    EXPECT_NEAR(s(2, 0),0,1.0e-9);
    EXPECT_NEAR(s(2, 1),0,1.0e-9);
    EXPECT_NEAR(s(2, 2),1,1.0e-9);
}


TEST_F(RegularisedStokesletTest, test_stokeslet_tensor_unit_y_offset) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 1, 0});

    auto s = cnearest::RegularisedStokeslet::stokeslet_tensor(f, q, 1.0e-5);
    EXPECT_NEAR(s(0, 0), 1, 1.0e-9);
    EXPECT_NEAR(s(0, 1), 0, 1.0e-9);
    EXPECT_NEAR(s(0, 2), 0, 1.0e-9);

    EXPECT_NEAR(s(1, 0), 0, 1.0e-9);
    EXPECT_NEAR(s(1, 1), 2, 1.0e-9);
    EXPECT_NEAR(s(1, 2), 0, 1.0e-9);

    EXPECT_NEAR(s(2, 0), 0, 1.0e-9);
    EXPECT_NEAR(s(2, 1), 0, 1.0e-9);
    EXPECT_NEAR(s(2, 2), 1, 1.0e-9);
}


TEST_F(RegularisedStokesletTest, test_stokeslet_tensor_unit_z_offset) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 0, 1});

    auto s = cnearest::RegularisedStokeslet::stokeslet_tensor(f, q, 1.0e-5);
    EXPECT_NEAR(s(0, 0), 1, 1.0e-9);
    EXPECT_NEAR(s(0, 1), 0, 1.0e-9);
    EXPECT_NEAR(s(0, 2), 0, 1.0e-9);

    EXPECT_NEAR(s(1, 0), 0, 1.0e-9);
    EXPECT_NEAR(s(1, 1), 1, 1.0e-9);
    EXPECT_NEAR(s(1, 2), 0, 1.0e-9);

    EXPECT_NEAR(s(2, 0), 0, 1.0e-9);
    EXPECT_NEAR(s(2, 1), 0, 1.0e-9);
    EXPECT_NEAR(s(2, 2), 2, 1.0e-9);
}


TEST_F(RegularisedStokesletTest, test_pressure_size) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 0, 1});

    auto p = cnearest::RegularisedStokeslet::pressure(f, q, 1.0e-5);
    ASSERT_EQ(p.size(), 3);
}

TEST_F(RegularisedStokesletTest, test_pressure_finite_values) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 0, 1});

    auto p = cnearest::RegularisedStokeslet::pressure(f, q, 1.0e-5);
    ASSERT_TRUE(std::isfinite(p(0)));
    ASSERT_TRUE(std::isfinite(p(1)));
    ASSERT_TRUE(std::isfinite(p(2)));

}

TEST_F(RegularisedStokesletTest, test_pressure_unit_x_offset) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({1, 0, 0});

    auto p = cnearest::RegularisedStokeslet::pressure(f, q, 1.0e-5);
    EXPECT_NEAR(p(0), 2, 1.0e-9);
    EXPECT_NEAR(p(1), 0, 1.0e-9);
    EXPECT_NEAR(p(2), 0, 1.0e-9);

}

TEST_F(RegularisedStokesletTest, test_pressure_unit_y_offset) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 1, 0});

    auto p = cnearest::RegularisedStokeslet::pressure(f, q, 1.0e-5);
    EXPECT_NEAR(p(0), 0, 1.0e-9);
    EXPECT_NEAR(p(1), 2, 1.0e-9);
    EXPECT_NEAR(p(2), 0, 1.0e-9);

}

TEST_F(RegularisedStokesletTest, test_pressure_unit_z_offset) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 0, 1});

    auto p = cnearest::RegularisedStokeslet::pressure(f, q, 1.0e-5);
    EXPECT_NEAR(p(0), 0, 1.0e-9);
    EXPECT_NEAR(p(1), 0, 1.0e-9);
    EXPECT_NEAR(p(2), 2, 1.0e-9);

}


TEST_F(RegularisedStokesletTest, test_stress_tensor_size) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 0, 1});

    auto t = cnearest::RegularisedStokeslet::stress_tensor(f, q, 1.0e-5);
    ASSERT_EQ(t.size(), 3);
    for (auto& s : t) {
        ASSERT_EQ(s.rows(), 3);
        ASSERT_EQ(s.cols(), 3);
    }
}

TEST_F(RegularisedStokesletTest, test_stress_tensor_finite_values) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 0, 1});

    auto t = cnearest::RegularisedStokeslet::stress_tensor(f, q, 1.0e-5);
    for (auto& s : t) {
        ASSERT_TRUE(std::isfinite(s(0, 0)));
        ASSERT_TRUE(std::isfinite(s(0, 1)));
        ASSERT_TRUE(std::isfinite(s(0, 2)));

        ASSERT_TRUE(std::isfinite(s(1, 0)));
        ASSERT_TRUE(std::isfinite(s(1, 1)));
        ASSERT_TRUE(std::isfinite(s(1, 2)));

        ASSERT_TRUE(std::isfinite(s(2, 0)));
        ASSERT_TRUE(std::isfinite(s(2, 1)));
        ASSERT_TRUE(std::isfinite(s(2, 2)));
    }
}

TEST_F(RegularisedStokesletTest, test_stress_tensor_unit_x_offset) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({1, 0, 0});

    auto t = cnearest::RegularisedStokeslet::stress_tensor(f, q, 1.0e-5);
    for (int i = 0; i < 3;i++) {
        auto & s = t[i];
        EXPECT_NEAR(s(0, 0), i == 0 ? -6 : 0, 1.0e-9);
        EXPECT_NEAR(s(0, 1), 0, 1.0e-9);
        EXPECT_NEAR(s(0, 2), 0, 1.0e-9);

        EXPECT_NEAR(s(1, 0), 0, 1.0e-9);
        EXPECT_NEAR(s(1, 1), 0, 1.0e-9);
        EXPECT_NEAR(s(1, 2), 0, 1.0e-9);

        EXPECT_NEAR(s(2, 0), 0, 1.0e-9);
        EXPECT_NEAR(s(2, 1), 0, 1.0e-9);
        EXPECT_NEAR(s(2, 2), 0, 1.0e-9);
    }
}

TEST_F(RegularisedStokesletTest, test_stress_tensor_unit_y_offset) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 1, 0});

    auto t = cnearest::RegularisedStokeslet::stress_tensor(f, q, 1.0e-5);
    for (int i = 0; i < 3; i++) {
        auto& s = t[i];
        EXPECT_NEAR(s(0, 0), 0, 1.0e-9);
        EXPECT_NEAR(s(0, 1), 0, 1.0e-9);
        EXPECT_NEAR(s(0, 2), 0, 1.0e-9);

        EXPECT_NEAR(s(1, 0), 0, 1.0e-9);
        EXPECT_NEAR(s(1, 1), i == 1 ? -6 : 0, 1.0e-9);
        EXPECT_NEAR(s(1, 2), 0, 1.0e-9);

        EXPECT_NEAR(s(2, 0), 0, 1.0e-9);
        EXPECT_NEAR(s(2, 1), 0, 1.0e-9);
        EXPECT_NEAR(s(2, 2), 0, 1.0e-9);
    }
}

TEST_F(RegularisedStokesletTest, test_stress_tensor_unit_z_offset) {
    cnearest::ForceNode f({0, 0, 0});
    cnearest::QuadratureNode q({0, 0, 1});

    auto t = cnearest::RegularisedStokeslet::stress_tensor(f, q, 1.0e-5);
    for (int i = 0; i < 3; i++) {
        auto& s = t[i];
        EXPECT_NEAR(s(0, 0), 0, 1.0e-9);
        EXPECT_NEAR(s(0, 1), 0, 1.0e-9);
        EXPECT_NEAR(s(0, 2), 0, 1.0e-9);

        EXPECT_NEAR(s(1, 0), 0, 1.0e-9);
        EXPECT_NEAR(s(1, 1), 0, 1.0e-9);
        EXPECT_NEAR(s(1, 2), 0, 1.0e-9);

        EXPECT_NEAR(s(2, 0), 0, 1.0e-9);
        EXPECT_NEAR(s(2, 1), 0, 1.0e-9);
        EXPECT_NEAR(s(2, 2), i == 2 ? -6 : 0, 1.0e-9);
    }
}