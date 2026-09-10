#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  Mirrors QuadPlane::verify_vtol_takeoff() yaw-align predicates:
    heading: fabsf(wrap_180_cd(target_cd - yaw_cd)) * 0.01f <= 5.0f
    rate:    yaw_rate_deg <= max(8, 0.4 * Q_TKOFF_YAW_RATE)   (RATE<=0 => 10)
*/
static float yaw_error_deg(int32_t target_cd, int32_t yaw_cd)
{
    return fabsf(wrap_180_cd(target_cd - yaw_cd)) * 0.01f;
}

static bool yaw_within_tol(int32_t target_cd, int32_t yaw_cd, float tol_deg = 5.0f)
{
    return yaw_error_deg(target_cd, yaw_cd) <= tol_deg;
}

static float yaw_rate_gate_degs(float tkoff_yaw_rate_degs)
{
    if (tkoff_yaw_rate_degs > 0.0f) {
        return MAX(8.0f, tkoff_yaw_rate_degs * 0.4f);
    }
    return 10.0f;
}

static bool settle_ready(int32_t target_cd, int32_t yaw_cd, float yaw_rate_deg,
                         float tkoff_yaw_rate_degs, float tol_deg = 5.0f)
{
    if (!yaw_within_tol(target_cd, yaw_cd, tol_deg)) {
        return false;
    }
    return yaw_rate_deg <= yaw_rate_gate_degs(tkoff_yaw_rate_degs);
}

TEST(TkoffYawAlign, ExactMatch)
{
    EXPECT_FLOAT_EQ(0.0f, yaw_error_deg(9000, 9000));
    EXPECT_TRUE(yaw_within_tol(9000, 9000));
}

TEST(TkoffYawAlign, WithinTolerance)
{
    // 4.9 deg error - should pass
    EXPECT_TRUE(yaw_within_tol(9000, 9000 - 490));
    EXPECT_TRUE(yaw_within_tol(9000, 9000 + 490));
    EXPECT_NEAR(4.9f, yaw_error_deg(9000, 9000 - 490), 0.01f);
}

TEST(TkoffYawAlign, AtToleranceBoundary)
{
    // exactly 5.0 deg
    EXPECT_TRUE(yaw_within_tol(0, 500));
    EXPECT_TRUE(yaw_within_tol(0, -500));
    EXPECT_FLOAT_EQ(5.0f, yaw_error_deg(0, 500));
}

TEST(TkoffYawAlign, OutsideTolerance)
{
    // 5.1 deg - should fail
    EXPECT_FALSE(yaw_within_tol(0, 510));
    EXPECT_FALSE(yaw_within_tol(0, -510));
    EXPECT_NEAR(5.1f, yaw_error_deg(0, 510), 0.01f);
}

TEST(TkoffYawAlign, WrapAroundZero)
{
    // target 1 deg, yaw 359 deg -> 2 deg error
    EXPECT_NEAR(2.0f, yaw_error_deg(100, 35900), 0.01f);
    EXPECT_TRUE(yaw_within_tol(100, 35900));

    // target 0, yaw 355 deg -> 5 deg (boundary)
    EXPECT_TRUE(yaw_within_tol(0, 35500));

    // target 0, yaw 354 deg -> 6 deg
    EXPECT_FALSE(yaw_within_tol(0, 35400));
}

TEST(TkoffYawAlign, OppositeHeading)
{
    // 180 deg error
    EXPECT_NEAR(180.0f, yaw_error_deg(0, 18000), 0.01f);
    EXPECT_FALSE(yaw_within_tol(0, 18000));

    // 90 deg error (typical East WP from North heading)
    EXPECT_NEAR(90.0f, yaw_error_deg(9000, 0), 0.01f);
    EXPECT_FALSE(yaw_within_tol(9000, 0));
}

TEST(TkoffYawAlign, NegativeTargetWest)
{
    // target 270 deg (= -90 via wrap), yaw 0 -> 90 deg
    EXPECT_NEAR(90.0f, yaw_error_deg(27000, 0), 0.01f);
    EXPECT_FALSE(yaw_within_tol(27000, 0));

    // target 270, yaw 269 -> 1 deg
    EXPECT_NEAR(1.0f, yaw_error_deg(27000, 26900), 0.01f);
    EXPECT_TRUE(yaw_within_tol(27000, 26900));
}

TEST(TkoffYawAlign, LargeCentidegreeInputs)
{
    // headings stored beyond 36000 should still wrap correctly
    EXPECT_NEAR(0.0f, yaw_error_deg(45000, 9000), 0.01f); // 450 deg == 90 deg
    EXPECT_TRUE(yaw_within_tol(45000, 9000));
}

TEST(TkoffYawAlign, RateGateFormula)
{
    // default Q_TKOFF_YAW_RATE=20 -> gate = max(8, 8) = 8
    EXPECT_FLOAT_EQ(8.0f, yaw_rate_gate_degs(20.0f));
    // RATE=50 -> gate = max(8, 20) = 20
    EXPECT_FLOAT_EQ(20.0f, yaw_rate_gate_degs(50.0f));
    // RATE=10 -> gate = max(8, 4) = 8
    EXPECT_FLOAT_EQ(8.0f, yaw_rate_gate_degs(10.0f));
    // RATE=0 (use ATC limits) -> fixed 10
    EXPECT_FLOAT_EQ(10.0f, yaw_rate_gate_degs(0.0f));
}

TEST(TkoffYawAlign, PrematureTransitionBlocked)
{
    // Flight-test failure mode (May 2026 Fighter-D 90 deg case):
    // heading briefly within 5 deg while RATE.Y still ~40-60 deg/s.
    const int32_t target_cd = 28100; // ~281 deg mission heading
    const int32_t yaw_cd = 28390;    // within 5 deg
    EXPECT_TRUE(yaw_within_tol(target_cd, yaw_cd));

    // Old behaviour (heading-only) would allow transition; rate gate must block.
    EXPECT_FALSE(settle_ready(target_cd, yaw_cd, 50.0f, 20.0f));
    EXPECT_FALSE(settle_ready(target_cd, yaw_cd, 62.0f, 20.0f));

    // After rate damps below gate (8 deg/s for RATE=20), allow transition.
    EXPECT_TRUE(settle_ready(target_cd, yaw_cd, 7.0f, 20.0f));
}

TEST(TkoffYawAlign, OvershootLeavesWindow)
{
    // After swinging through target, heading error exceeds 5 deg again
    // (settle timer must reset — mirrored by heading check failing).
    const int32_t target_cd = 28100;
    EXPECT_FALSE(yaw_within_tol(target_cd, 26810)); // ~13 deg past target
    EXPECT_FALSE(settle_ready(target_cd, 26810, 5.0f, 20.0f));
}

TEST(TkoffYawAlign, HighRateOppositeSwing)
{
    // 180 deg case: heading not yet aligned and rate very high (log peak 133 deg/s)
    EXPECT_FALSE(settle_ready(28100, 19890, 133.0f, 20.0f));
    EXPECT_FALSE(settle_ready(28100, 31990, 40.0f, 20.0f));
}

AP_GTEST_MAIN()
