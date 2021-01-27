#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <math.h>
#include <doctest/doctest.h>
#include <Eigen/Dense>
#include <dynkin/dynkin.hpp>

#define DEG2RAD M_PI/180.0f

TEST_CASE("Single frame translation"){
    using namespace dynkin;
    Eigen::Vector3d expected;

    Frame f1 = create_frame();
    f1->position() << 1, 1, 1;
    f1->set_attitude({0,0,90*DEG2RAD});

    Transform t = transform(nullptr, f1);

    expected << 1, 1, 1;
    CHECK(t.HTM.translation().isApprox(expected));

    t = transform(f1, nullptr);

    expected << -1, 1, -1;
    CHECK(t.HTM.translation().isApprox(expected));
}

TEST_CASE("Single frame position"){
    using namespace dynkin;
    Eigen::Vector3d expected;

    Frame f1 = create_frame();
    f1->position() << 1, 1, 1;
    f1->set_attitude({90*DEG2RAD,0,90*DEG2RAD});

    Transform t = transform(nullptr, f1);

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

    Frame f1 = create_frame();
    f1->position() << 1, 1, 1;
    f1->set_attitude({0,90*DEG2RAD,90*DEG2RAD});

    Transform t = transform(nullptr, f1);

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

    Frame f1 = create_frame();
    f1->position() << 1, 1, 1;
    f1->set_attitude({0,90*DEG2RAD,90*DEG2RAD});

    Transform t = transform(nullptr, f1);

    expected << 0, 0, -1, -1, 1, 0;
    wrench << 1, 0, 0, 0, 0, 0;
    CHECK(t.apply_wrench(wrench).isApprox(expected));
}

TEST_CASE("Single frame HTM inverse"){
    using namespace dynkin;

    Frame f1 = create_frame();
    f1->position() << 124,-343,-13;
    f1->set_attitude({27*DEG2RAD,49*DEG2RAD,62*DEG2RAD});

    Transform t = transform(nullptr, f1);
    Transform t_ = transform(f1, nullptr);

    CHECK(t.HTM.isApprox(t_.inverse().HTM));
}

TEST_CASE("Single frame velocity"){
    using namespace dynkin;
    Eigen::Vector6d expected;

    Frame f1 = create_frame();
    f1->position() << 1, 1, 1;
    f1->set_attitude({0,90*DEG2RAD,90*DEG2RAD});
    f1->linear_velocity() << 1, 0, 0;
    f1->angular_velocity() << 0, 0, 1;

    expected << 1, 0, 0, 0, 0, 1;
    CHECK(f1->get_twist().isApprox(expected));
}

TEST_CASE("Chained frame velocity"){
    using namespace dynkin;
    Eigen::Vector6d expected;

    Frame f1 = create_frame();
    f1->set_attitude({0,0,90*DEG2RAD});
    f1->angular_velocity() << 0, 0, 1;

    Frame f2 = f1->create_child();
    f2->position() << 1, 1, 0;
    f2->linear_velocity() << 1, 0, 0;
    f2->angular_velocity() << 1, 0, 0;

    expected << 0, 1, 0, 1, 0, 1;
    CHECK(f2->get_twist().isApprox(expected));
}

TEST_CASE("Test inertia constructor"){
    using namespace dynkin;
    using namespace dynkin::rigidbody;
    CHECK_THROWS_AS(generalized_inertia_matrix(-1, {1, 1, 1}), std::invalid_argument);
    CHECK_THROWS_AS(generalized_inertia_matrix(1, {-1, 1, 1}), std::invalid_argument);

    Eigen::Matrix6d expected = Eigen::Matrix6d::Identity();

    CHECK(generalized_inertia_matrix(1, {1,1,1}).isApprox(expected));
}

TEST_CASE("Test generalized coordinates"){
    using namespace dynkin;
    using namespace dynkin::rigidbody;

    RigidBody rb = create_rigidbody(generalized_inertia_matrix(1, {1,1,1}));
    rb->position() << 1,2,3;
    rb->set_attitude({0,1,1});
    Eigen::Vector6d pose = rb->generalized_coordinates();

    Eigen::Vector6d expected;
    expected << 1,2,3,0,1,1;

    CHECK(pose.isApprox(expected));
}

TEST_CASE("Test generalized velocities"){
    using namespace dynkin;
    using namespace dynkin::rigidbody;

    RigidBody rb = create_rigidbody(generalized_inertia_matrix(1, {1,1,1}));
    rb->position() << 1,1,1;
    rb->set_attitude({M_PI_2, 0, M_PI_2});
    rb->linear_velocity() << 1, 2, 3;
    rb->angular_velocity() << 0, 0, 1;
    Eigen::Vector6d vel = rb->generalized_velocities();

    Eigen::Vector6d expected;
    expected << 3, 1, 2, 0, -1, 0;

    CHECK(vel.isApprox(expected));
}

TEST_CASE("Test Coriolis-Centripetal acceleration with CoG offset"){
    using namespace dynkin;
    using namespace dynkin::rigidbody;

    RigidBody rb = create_rigidbody(generalized_inertia_matrix(1, {1,1,1}));
    rb->cog = {1, 0, 0};
    rb->angular_velocity() << 0, 0, 1;

    Eigen::Vector6d expected;
    expected << 1, 0, 0, 0, 0, 0;

    CHECK_EQ(rb->acceleration(Eigen::Vector6d::Zero()), expected);
}

TEST_CASE("Test Coriolis-Centripetal acceleration due to linear velocity"){
    using namespace dynkin;
    using namespace dynkin::rigidbody;

    RigidBody rb = create_rigidbody(generalized_inertia_matrix(1, {1,1,1}));
    rb->linear_velocity() << 1, 0, 0;
    rb->angular_velocity() << 0, 0, 1;

    Eigen::Vector6d expected;
    expected << 0, -1, 0, 0, 0, 0;

    CHECK_EQ(rb->acceleration(Eigen::Vector6d::Zero()), expected);
}

TEST_CASE("Test acceleration -> wrench"){
    using namespace dynkin;
    using namespace dynkin::rigidbody;

    RigidBody rb = create_rigidbody(generalized_inertia_matrix(1, {1,1,1}));
    Eigen::Vector6d wrench = Eigen::Vector6d::Ones();
    Eigen::Vector6d acc = rb->acceleration(wrench);

    CHECK_EQ(acc, Eigen::Vector6d::Ones());


    rb->cog = {1, 0, 0};
    acc = rb->acceleration(wrench);
    Eigen::Vector6d expected;
    expected << 1, 1, 3, 1, 2, 0;

    CHECK_EQ(acc, expected);


    Eigen::Vector6d f = rb->wrench(acc);
    CHECK_EQ(wrench, f);

}

TEST_CASE("Test acceleration -> wrench"){
    using namespace dynkin;
    using namespace dynkin::rigidbody;

    RigidBody rb = create_rigidbody(generalized_inertia_matrix(1, {1,1,1}));
    Eigen::Vector6d acc = Eigen::Vector6d::Ones();
    Eigen::Vector6d wrench = rb->wrench(acc);

    CHECK_EQ(wrench, Eigen::Vector6d::Ones());


    rb->cog = {1, 0, 0};
    wrench = rb->wrench(acc);
    Eigen::Vector6d expected;
    expected << 1, 2, 0, 1, 1, 3;

    CHECK_EQ(wrench, expected);


    Eigen::Vector6d a = rb->acceleration(wrench);
    CHECK_EQ(acc, a);

}