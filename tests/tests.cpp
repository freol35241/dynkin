#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <math.h>
#include <doctest/doctest.h>
#include <Eigen/Dense>
#include <dynkin/dynkin.hpp>

#define DEG2RAD M_PI/180.0f

TEST_CASE("Single frame translation"){
    using namespace dynkin;
    Eigen::Vector3d expected;

    Frame f1 = Frame().set_position({1,1,1}).set_attitude({0,0,90*DEG2RAD});
    Transform t = transform(nullptr, &f1);

    expected << 1, 1, 1;
    CHECK(t.HTM.translation().isApprox(expected));

    t = transform(&f1, nullptr);

    expected << -1, 1, -1;
    CHECK(t.HTM.translation().isApprox(expected));
}

TEST_CASE("Single frame position"){
    using namespace dynkin;
    Eigen::Vector3d expected;

    Frame f1 = Frame().set_position({1,1,1}).set_attitude({90*DEG2RAD,0,90*DEG2RAD});
    Transform t = transform(nullptr, &f1);

    expected << 1, 1, 1;
    CHECK(t.apply_position({0,0,0}).isApprox(expected));

    expected << 1, 2, 1;
    CHECK(t.apply_position({1,0,0}).isApprox(expected));

    expected << 1, 1, 2;
    CHECK(t.apply_position({0,1,0}).isApprox(expected));

    expected << 2, 1, 1;
    CHECK(t.apply_position({0,0,1}).isApprox(expected));
}

TEST_CASE("Single frame vector"){
    using namespace dynkin;
    Eigen::Vector3d expected;

    Frame f1 = Frame().set_position({1,1,1}).set_attitude({0,90*DEG2RAD,90*DEG2RAD});
    Transform t = transform(nullptr, &f1);

    expected << 0, 0, 0;
    CHECK(t.apply_vector({0,0,0}).isApprox(expected));

    expected << 0, 0, -1;
    CHECK(t.apply_vector({1,0,0}).isApprox(expected));

    expected << -1, 0, 0;
    CHECK(t.apply_vector({0,1,0}).isApprox(expected));

    expected << 0, 1, 0;
    CHECK(t.apply_vector({0,0,1}).isApprox(expected));
}

TEST_CASE("Single frame wrench"){
    using namespace dynkin;
    Eigen::Vector6d expected, wrench;

    Frame f1 = Frame().set_position({1,1,1}).set_attitude({0,90*DEG2RAD,90*DEG2RAD});
    Transform t = transform(nullptr, &f1);

    expected << 0, 0, -1, -1, 1, 0;
    wrench << 1, 0, 0, 0, 0, 0;
    CHECK(t.apply_wrench(wrench).isApprox(expected));
}

TEST_CASE("Single frame HTM inverse"){
    using namespace dynkin;

    Frame f1 = Frame().set_position({124,-343,-13}).set_attitude({27*DEG2RAD,49*DEG2RAD,62*DEG2RAD});

    Transform t = transform(nullptr, &f1);
    Transform t_ = transform(&f1, nullptr);

    CHECK(t.HTM.isApprox(t_.inverse().HTM));
}

TEST_CASE("Single frame velocity"){
    using namespace dynkin;
    Eigen::Vector6d expected;

    Frame f1 = Frame().set_position({1,1,1}).set_attitude({0,90*DEG2RAD,90*DEG2RAD});
    f1.linear_velocity << 1, 0, 0;
    f1.angular_velocity << 0, 0, 1;

    expected << 1, 0, 0, 0, 0, 1;
    CHECK(f1.get_twist().isApprox(expected));
}

TEST_CASE("Chained frame velocity"){
    using namespace dynkin;
    Eigen::Vector6d expected;

    Frame f1 = Frame().set_attitude({0,0,90*DEG2RAD});
    f1.angular_velocity << 0, 0, 1;
    Frame f2 = Frame().set_parent(&f1).set_position({1,1,0});
    f2.linear_velocity << 1, 0, 0;
    f2.angular_velocity << 1, 0, 0;

    expected << 0, 1, 0, 1, 0, 1;
    CHECK(f2.get_twist().isApprox(expected));
}