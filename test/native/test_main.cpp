#include <PotIO.h>
#include "../support/MiniTest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

struct IntReader
{
    static constexpr int kFullScale = 1023;
    int *value{nullptr};
    IntReader() = default;
    explicit IntReader(int *v) : value(v) {}
    int operator()() const noexcept { return value ? *value : -1; }
};

struct SampleReader
{
    static constexpr int kFullScale = 1023;
    PotIO::RawSample *value{nullptr};
    SampleReader() = default;
    explicit SampleReader(PotIO::RawSample *v) : value(v) {}
    PotIO::RawSample operator()() const noexcept
    {
        return value ? *value : PotIO::RawSample::failure();
    }
};

struct NaNFilter
{
    bool valid() const noexcept { return true; }
    float operator()(float, float) const noexcept { return std::numeric_limits<float>::quiet_NaN(); }
};

struct NaNShape
{
    bool valid() const noexcept { return true; }
    float operator()(float) const noexcept { return std::numeric_limits<float>::quiet_NaN(); }
};

static PotIO::PotCalib calib1023()
{
    return PotIO::PotCalib{0u, 512u, 1023u};
}

using FrameLinear = PotIO::LinearPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
using FrameJoystick = PotIO::Joystick2D<IntReader, IntReader, PotIO::NoFilter, PotIO::NoFilter,
                                         PotIO::NoRateLimit, PotIO::NoRateLimit,
                                         PotIO::ShapeIdentity, PotIO::ShapeIdentity>;
using FrameContinuous = PotIO::ContinuousPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
using FrameStepped = PotIO::SteppedPot<5u, IntReader, PotIO::NoFilter>;

static_assert(std::is_trivially_copyable<FrameLinear::State>::value, "LinearPot State must remain trivially copyable.");
static_assert(std::is_trivially_copyable<FrameJoystick::State>::value, "Joystick2D State must remain trivially copyable.");
static_assert(std::is_trivially_copyable<FrameContinuous::State>::value, "ContinuousPot State must remain trivially copyable.");
static_assert(std::is_trivially_copyable<FrameStepped::State>::value, "SteppedPot State must remain trivially copyable.");

static void test_version_macros()
{
    MiniTest::begin("version macros expose PotIO package version");
    CHECK(std::strcmp(POTIO_VERSION, "1.1.0") == 0);
    CHECK(POTIO_VERSION_MAJOR == 1);
    CHECK(POTIO_VERSION_MINOR == 1);
    CHECK(POTIO_VERSION_PATCH == 0);
}

static void test_failed_read_retains_last_good()
{
    MiniTest::begin("failed read retains last-good LinearPot output");
    int raw = 1023;
    using Pot = PotIO::LinearPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
    Pot::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    Pot pot(cfg);

    pot.update(100u, 0.01f);
    CHECK(pot.state().status.valid);
    CHECK_NEAR(pot.centered(), 1.0, 1e-5);
    const uint32_t seq = pot.state().status.sequence;
    const uint32_t sample_ms = pot.state().status.sample_ms;

    raw = -1;
    pot.update(110u, 0.01f);
    CHECK(!pot.state().status.valid);
    CHECK(pot.state().status.error == PotIO::ReadError::ReaderFailure);
    CHECK(pot.state().status.has_value);
    CHECK(pot.state().status.sequence == seq);
    CHECK(pot.state().status.sample_ms == sample_ms);
    CHECK_NEAR(pot.centered(), 1.0, 1e-5);

    raw = 0;
    pot.update(120u, 0.01f);
    CHECK(pot.state().status.valid);
    CHECK(pot.state().status.sequence == seq + 1u);
    CHECK(pot.state().status.sample_ms == 120u);
    CHECK_NEAR(pot.centered(), -1.0, 1e-5);
}

static void test_explicit_reader_failure()
{
    MiniTest::begin("RawSample reader reports explicit failure");
    PotIO::RawSample raw = PotIO::RawSample::success(512);
    using Pot = PotIO::LinearPot<SampleReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
    Pot::Config cfg;
    cfg.reader = SampleReader(&raw);
    cfg.calib = calib1023();
    Pot pot(cfg);
    pot.update(1u, 0.01f);
    CHECK(pot.state().status.valid);

    raw = PotIO::RawSample::failure(PotIO::ReadError::ReaderFailure);
    pot.update(2u, 0.01f);
    CHECK(!pot.state().status.valid);
    CHECK(pot.state().status.error == PotIO::ReadError::ReaderFailure);
}

static void test_out_of_range_is_invalid()
{
    MiniTest::begin("out-of-range raw input is invalid");
    int raw = 2048;
    using Pot = PotIO::LinearPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
    Pot::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    Pot pot(cfg);
    pot.update(1u, 0.01f);
    CHECK(!pot.state().status.valid);
    CHECK(pot.state().status.error == PotIO::ReadError::OutOfRange);
    CHECK(!pot.state().status.has_value);
}

static void test_calibration_policies()
{
    MiniTest::begin("permissive and strict calibration policies");
    int raw = 512;
    using Pot = PotIO::LinearPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
    Pot::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = PotIO::PotCalib{100u, 50u, 900u};

    Pot permissive(cfg);
    permissive.update(1u, 0.01f);
    CHECK(permissive.state().status.valid);
    CHECK(!permissive.state().status.calibration_valid);
    CHECK((permissive.state().status.quality & PotIO::QualityCalibrationFallback) != 0u);

    cfg.calibration_policy = PotIO::CalibrationPolicy::RequireValid;
    Pot strict(cfg);
    strict.update(1u, 0.01f);
    CHECK(!strict.state().status.valid);
    CHECK(strict.state().status.error == PotIO::ReadError::CalibrationInvalid);
}

static void test_calibration_bounds_and_span()
{
    MiniTest::begin("calibration validates full-scale and minimum spans");
    CHECK(calib1023().valid_centered_for(1023, 1u));
    CHECK((!PotIO::PotCalib{0u, 512u, 1200u}.valid_centered_for(1023, 1u)));
    CHECK((!PotIO::PotCalib{100u, 101u, 900u}.valid_centered_for(1023, 2u)));
}

static void test_calibration_mapping_boundaries()
{
    MiniTest::begin("calibration endpoints and center map exactly");
    int raw = 100;
    using Pot = PotIO::LinearPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
    Pot::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = PotIO::PotCalib{100u, 500u, 900u};
    Pot pot(cfg);

    pot.update(1u, 0.01f);
    CHECK_NEAR(pot.calib01(), 0.0, 1e-6);
    raw = 500;
    pot.update(2u, 0.01f);
    CHECK_NEAR(pot.calib01(), 0.5, 1e-6);
    CHECK_NEAR(pot.centered(), 0.0, 1e-6);
    raw = 900;
    pot.update(3u, 0.01f);
    CHECK_NEAR(pot.calib01(), 1.0, 1e-6);
    raw = 0;
    pot.update(4u, 0.01f);
    CHECK_NEAR(pot.calib01(), 0.0, 1e-6);
    raw = 1023;
    pot.update(5u, 0.01f);
    CHECK_NEAR(pot.calib01(), 1.0, 1e-6);
}

static void test_linear_mapping_sweep()
{
    MiniTest::begin("linear mapping remains bounded and monotonic across ADC range");
    int raw = 0;
    using Pot = PotIO::LinearPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
    Pot::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    Pot pot(cfg);
    float previous = -2.0f;
    for (int value = 0; value <= 1023; ++value)
    {
        raw = value;
        pot.update(static_cast<uint32_t>(value + 1), 0.001f);
        CHECK(pot.valid());
        CHECK(pot.centered() >= -1.000001f && pot.centered() <= 1.000001f);
        CHECK(pot.centered() + 1e-6f >= previous);
        previous = pot.centered();
    }
}

static void test_filter_rate_history_is_independent()
{
    MiniTest::begin("filter and rate limiter keep independent history");
    int raw = 1023;
    using Pot = PotIO::LinearPot<IntReader, PotIO::EMAFilter, PotIO::SlewRate, PotIO::ShapeIdentity>;
    Pot::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    cfg.filter.alpha = 0.5f;
    cfg.rate.units_per_s = 0.2f;
    cfg.max_dt_s = 2.0f;
    Pot pot(cfg);
    pot.update(0u, 1.0f);
    raw = 0;
    for (int i = 0; i < 10; ++i)
        pot.update(static_cast<uint32_t>(i + 1), 1.0f);
    CHECK(pot.centered() < -0.98f);
}

static void test_nonfinite_processing_output_is_invalid()
{
    MiniTest::begin("non-finite processing output is rejected");
    int raw = 512;

    using BadShapePot = PotIO::LinearPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, NaNShape>;
    BadShapePot::Config shape_cfg;
    shape_cfg.reader = IntReader(&raw);
    shape_cfg.calib = calib1023();
    BadShapePot bad_shape(shape_cfg);
    bad_shape.update(1u, 0.01f);
    CHECK(!bad_shape.valid());
    CHECK(bad_shape.state().status.error == PotIO::ReadError::InvalidConfiguration);
    CHECK(!bad_shape.state().status.has_value);

    raw = 1023;
    using BadFilterPot = PotIO::LinearPot<IntReader, NaNFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
    BadFilterPot::Config filter_cfg;
    filter_cfg.reader = IntReader(&raw);
    filter_cfg.calib = calib1023();
    BadFilterPot bad_filter(filter_cfg);
    bad_filter.update(2u, 0.01f);
    CHECK(bad_filter.valid());
    CHECK_NEAR(bad_filter.centered(), 1.0, 1e-6);
    const uint32_t seq = bad_filter.state().status.sequence;

    raw = 0;
    bad_filter.update(3u, 0.01f);
    CHECK(!bad_filter.valid());
    CHECK(bad_filter.state().status.error == PotIO::ReadError::InvalidConfiguration);
    CHECK(bad_filter.state().status.sequence == seq);
    CHECK_NEAR(bad_filter.centered(), 1.0, 1e-6);
}

static void test_stepped_nonfinite_filter_does_not_quantize()
{
    MiniTest::begin("SteppedPot rejects non-finite filtered values before quantizing");
    int raw = 10;
    using SP = PotIO::SteppedPot<5u, IntReader, NaNFilter>;
    SP::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    SP pot(cfg);
    pot.update(1u, 0.01f);
    CHECK(pot.valid());
    const uint8_t step = pot.state().step;
    const uint32_t seq = pot.state().status.sequence;

    raw = 900;
    pot.update(2u, 0.01f);
    CHECK(!pot.valid());
    CHECK(pot.state().status.error == PotIO::ReadError::InvalidConfiguration);
    CHECK(pot.state().step == step);
    CHECK(pot.state().status.sequence == seq);
}

static void test_invalid_timing_holds_output()
{
    MiniTest::begin("invalid and excessive dt hold last-good output");
    int raw = 1023;
    using Pot = PotIO::LinearPot<IntReader, PotIO::NoFilter, PotIO::SlewRate, PotIO::ShapeIdentity>;
    Pot::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    cfg.max_dt_s = 0.25f;
    Pot pot(cfg);
    pot.update(1u, 0.01f);
    raw = 0;
    const float before = pot.centered();

    pot.update(2u, -0.1f);
    CHECK(pot.state().status.error == PotIO::ReadError::InvalidTiming);
    CHECK_NEAR(pot.centered(), before, 1e-6);
    pot.update(3u, std::numeric_limits<float>::infinity());
    CHECK(pot.state().status.error == PotIO::ReadError::InvalidTiming);
    pot.update(4u, std::numeric_limits<float>::quiet_NaN());
    CHECK(pot.state().status.error == PotIO::ReadError::InvalidTiming);
    pot.update(5u, 1.0f);
    CHECK(pot.state().status.error == PotIO::ReadError::TimingGap);
    CHECK_NEAR(pot.centered(), before, 1e-6);
}

static void test_invalid_builtin_policy_is_rejected()
{
    MiniTest::begin("invalid built-in configuration is rejected");
    int raw = 512;
    using Pot = PotIO::LinearPot<IntReader, PotIO::EMAFilter, PotIO::SlewRate, PotIO::ShapeIdentity>;
    Pot::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    cfg.rate.units_per_s = -1.0f;
    Pot pot(cfg);
    pot.update(1u, 0.01f);
    CHECK(!pot.state().status.valid);
    CHECK(pot.state().status.error == PotIO::ReadError::InvalidConfiguration);

    cfg.rate.units_per_s = 1.0f;
    cfg.filter.alpha = std::numeric_limits<float>::quiet_NaN();
    Pot nan_filter(cfg);
    nan_filter.update(2u, 0.01f);
    CHECK(!nan_filter.valid());
    CHECK(nan_filter.state().status.error == PotIO::ReadError::InvalidConfiguration);
}

static void test_invalid_enum_configuration_is_rejected()
{
    MiniTest::begin("invalid public enum configuration is rejected");
    int rx = 512;
    int ry = 512;
    using Joy = PotIO::Joystick2D<IntReader, IntReader, PotIO::NoFilter, PotIO::NoFilter,
                                  PotIO::NoRateLimit, PotIO::NoRateLimit,
                                  PotIO::ShapeIdentity, PotIO::ShapeIdentity>;
    Joy::Config cfg;
    cfg.readX = IntReader(&rx);
    cfg.readY = IntReader(&ry);
    cfg.calX = calib1023();
    cfg.calY = calib1023();
    cfg.geometry = static_cast<PotIO::JoystickGeometry>(255);
    Joy joy(cfg);
    joy.update(1u, 0.01f);
    CHECK(!joy.valid());
    CHECK(joy.state().status.error == PotIO::ReadError::InvalidConfiguration);
}

static void test_reset_processing_policy_is_explicit()
{
    MiniTest::begin("ResetProcessing deliberately reseeds pipeline after failure");
    int raw = 1023;
    using Pot = PotIO::LinearPot<IntReader, PotIO::EMAFilter, PotIO::SlewRate, PotIO::ShapeIdentity>;
    Pot::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    cfg.filter.alpha = 0.1f;
    cfg.rate.units_per_s = 0.1f;
    cfg.invalid_sample_policy = PotIO::InvalidSamplePolicy::ResetProcessing;
    Pot pot(cfg);
    pot.update(1u, 0.01f);
    CHECK_NEAR(pot.centered(), 1.0, 1e-6);
    raw = -1;
    pot.update(2u, 0.01f);
    CHECK(!pot.valid());
    raw = 0;
    pot.update(3u, 0.01f);
    CHECK(pot.valid());
    CHECK_NEAR(pot.centered(), -1.0, 1e-6);
}

static void test_axial_scaled_deadzone()
{
    MiniTest::begin("axial scaled deadzone is continuous");
    int rx = 512;
    int ry = 512;
    using Joy = PotIO::Joystick2D<IntReader, IntReader, PotIO::NoFilter, PotIO::NoFilter,
                                  PotIO::NoRateLimit, PotIO::NoRateLimit,
                                  PotIO::ShapeIdentity, PotIO::ShapeIdentity>;
    Joy::Config cfg;
    cfg.readX = IntReader(&rx);
    cfg.readY = IntReader(&ry);
    cfg.calX = calib1023();
    cfg.calY = calib1023();
    cfg.deadzone = PotIO::Deadzone::AxialScaled;
    cfg.deadzone_size = 0.2f;
    Joy joy(cfg);
    joy.update(1u, 0.01f);
    CHECK_NEAR(joy.x(), 0.0, 0.005);

    rx = 512 + 103; // approximately +0.20 centered; boundary remains near zero
    joy.update(2u, 0.01f);
    CHECK(std::fabs(joy.x()) < 0.02f);
    rx = 1023;
    joy.update(3u, 0.01f);
    CHECK_NEAR(joy.x(), 1.0, 0.01);
}

static void test_radial_square_has_no_corner_gain()
{
    MiniTest::begin("radial scaling does not amplify square-corner vectors");
    int rx = 922; // about +0.80
    int ry = 922;
    using Joy = PotIO::Joystick2D<IntReader, IntReader, PotIO::NoFilter, PotIO::NoFilter,
                                  PotIO::NoRateLimit, PotIO::NoRateLimit,
                                  PotIO::ShapeIdentity, PotIO::ShapeIdentity>;
    Joy::Config cfg;
    cfg.readX = IntReader(&rx);
    cfg.readY = IntReader(&ry);
    cfg.calX = calib1023();
    cfg.calY = calib1023();
    cfg.deadzone = PotIO::Deadzone::RadialScaled;
    cfg.deadzone_size = 0.1f;
    cfg.geometry = PotIO::JoystickGeometry::Square;
    Joy joy(cfg);
    joy.update(1u, 0.01f);
    CHECK(joy.x() <= 0.81f);
    CHECK(joy.y() <= 0.81f);
}

static void test_geometry_policies()
{
    MiniTest::begin("joystick geometry policies are explicit");
    int rx = 1023;
    int ry = 1023;
    using Joy = PotIO::Joystick2D<IntReader, IntReader, PotIO::NoFilter, PotIO::NoFilter,
                                  PotIO::NoRateLimit, PotIO::NoRateLimit,
                                  PotIO::ShapeIdentity, PotIO::ShapeIdentity>;
    Joy::Config cfg;
    cfg.readX = IntReader(&rx);
    cfg.readY = IntReader(&ry);
    cfg.calX = calib1023();
    cfg.calY = calib1023();
    cfg.deadzone = PotIO::Deadzone::None;
    cfg.geometry = PotIO::JoystickGeometry::MagnitudeClamp;
    Joy clamped(cfg);
    clamped.update(1u, 0.01f);
    CHECK_NEAR(clamped.x(), 0.707106, 0.005);
    CHECK_NEAR(clamped.y(), 0.707106, 0.005);

    cfg.geometry = PotIO::JoystickGeometry::SquareToCircle;
    Joy circle(cfg);
    circle.update(1u, 0.01f);
    CHECK_NEAR(circle.x(), 0.707106, 0.005);
    CHECK_NEAR(circle.y(), 0.707106, 0.005);
}

static void test_geometry_sweep_stays_bounded()
{
    MiniTest::begin("circular joystick geometries stay inside unit magnitude");
    int rx = 0;
    int ry = 0;
    using Joy = PotIO::Joystick2D<IntReader, IntReader, PotIO::NoFilter, PotIO::NoFilter,
                                  PotIO::NoRateLimit, PotIO::NoRateLimit,
                                  PotIO::ShapeIdentity, PotIO::ShapeIdentity>;

    for (int mode = 0; mode < 2; ++mode)
    {
        Joy::Config cfg;
        cfg.readX = IntReader(&rx);
        cfg.readY = IntReader(&ry);
        cfg.calX = calib1023();
        cfg.calY = calib1023();
        cfg.deadzone = PotIO::Deadzone::None;
        cfg.geometry = (mode == 0) ? PotIO::JoystickGeometry::MagnitudeClamp
                                   : PotIO::JoystickGeometry::SquareToCircle;
        Joy joy(cfg);
        uint32_t tick = 1u;
        for (int x = 0; x <= 1023; x += 93)
        {
            for (int y = 0; y <= 1023; y += 93)
            {
                rx = x;
                ry = y;
                joy.update(tick++, 0.001f);
                CHECK(joy.valid());
                CHECK(joy.magnitude() <= 1.0001f);
                CHECK(joy.x() >= -1.0001f && joy.x() <= 1.0001f);
                CHECK(joy.y() >= -1.0001f && joy.y() <= 1.0001f);
            }
        }
    }
}

static void test_angle_validity()
{
    MiniTest::begin("neutral joystick angle is explicitly invalid");
    int rx = 512;
    int ry = 512;
    using Joy = PotIO::Joystick2D<IntReader, IntReader, PotIO::NoFilter, PotIO::NoFilter,
                                  PotIO::NoRateLimit, PotIO::NoRateLimit,
                                  PotIO::ShapeIdentity, PotIO::ShapeIdentity>;
    Joy::Config cfg;
    cfg.readX = IntReader(&rx);
    cfg.readY = IntReader(&ry);
    cfg.calX = calib1023();
    cfg.calY = calib1023();
    cfg.deadzone = PotIO::Deadzone::None;
    Joy joy(cfg);
    joy.update(1u, 0.01f);
    CHECK(!joy.angleValid());
    rx = 1023;
    joy.update(2u, 0.01f);
    CHECK(joy.angleValid());
    CHECK_NEAR(joy.angleDeg(), 0.0, 0.1);
}

static void test_joystick_failure_is_atomic_across_axes()
{
    MiniTest::begin("joystick failure does not partially advance one axis");
    int rx = 1023;
    int ry = 512;
    using Joy = PotIO::Joystick2D<IntReader, IntReader, PotIO::NoFilter, PotIO::NoFilter,
                                  PotIO::NoRateLimit, PotIO::NoRateLimit,
                                  PotIO::ShapeIdentity, PotIO::ShapeIdentity>;
    Joy::Config cfg;
    cfg.readX = IntReader(&rx);
    cfg.readY = IntReader(&ry);
    cfg.calX = calib1023();
    cfg.calY = calib1023();
    Joy joy(cfg);
    joy.update(1u, 0.01f);
    const float x = joy.x();
    const float y = joy.y();
    const uint32_t seq = joy.state().status.sequence;
    rx = 0;
    ry = -1;
    joy.update(2u, 0.01f);
    CHECK(!joy.valid());
    CHECK_NEAR(joy.x(), x, 1e-6);
    CHECK_NEAR(joy.y(), y, 1e-6);
    CHECK(joy.state().status.sequence == seq);
}

static void test_continuous_wrap_and_discontinuity()
{
    MiniTest::begin("ContinuousPot detects wraps and rejects ambiguous jumps");
    int raw = 990;
    using CP = PotIO::ContinuousPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
    CP::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    cfg.max_turns_per_s = 8.0f;
    cfg.max_phase_delta = 0.45f;
    CP pot(cfg);
    pot.update(1000u, 0.02f);
    CHECK(pot.turnsValid());
    raw = 51;
    pot.update(1020u, 0.02f);
    CHECK(pot.valid());
    CHECK(pot.state().turns == 1);

    raw = 600;
    pot.update(1040u, 0.02f);
    CHECK(!pot.valid());
    CHECK(pot.state().status.error == PotIO::ReadError::Discontinuity);
    CHECK(!pot.turnsValid());
    CHECK(pot.state().turns == 1);

    pot.resynchronizeTurns(7);
    raw = 620;
    pot.update(1060u, 0.02f);
    CHECK(pot.valid());
    CHECK(pot.turnsValid());
    CHECK(pot.state().turns == 7);
}

static void test_continuous_dropout_uses_last_good_time()
{
    MiniTest::begin("ContinuousPot dropout window is measured from last good phase");
    int raw = 990;
    using CP = PotIO::ContinuousPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
    CP::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    cfg.max_turns_per_s = 8.0f;
    cfg.max_phase_delta = 0.45f;
    CP pot(cfg);

    pot.update(1000u, 0.01f);
    raw = -1;
    pot.update(1020u, 0.02f);
    CHECK(!pot.valid());
    CHECK(pot.turnsValid());

    raw = 51;
    pot.update(1040u, 0.02f);
    CHECK(pot.valid());
    CHECK(pot.turnsValid());
    CHECK(pot.state().turns == 1);

    raw = -1;
    pot.update(1060u, 0.02f);
    raw = 60;
    pot.update(1120u, 0.06f);
    CHECK(!pot.valid());
    CHECK(pot.state().status.error == PotIO::ReadError::Discontinuity);
    CHECK(!pot.turnsValid());
    CHECK(pot.state().turns == 1);
}

static void test_continuous_velocity_limit()
{
    MiniTest::begin("ContinuousPot enforces maximum plausible velocity");
    int raw = 100;
    using CP = PotIO::ContinuousPot<IntReader, PotIO::NoFilter, PotIO::NoRateLimit, PotIO::ShapeIdentity>;
    CP::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    cfg.max_turns_per_s = 1.0f;
    CP pot(cfg);
    pot.update(1u, 0.01f);
    raw = 200;
    pot.update(2u, 0.01f);
    CHECK(!pot.valid());
    CHECK(pot.state().status.error == PotIO::ReadError::Discontinuity);
}

static void test_stepped_change_sequence_survives_latest_state()
{
    MiniTest::begin("SteppedPot durable change sequence");
    int raw = 10;
    using SP = PotIO::SteppedPot<5u, IntReader, PotIO::NoFilter>;
    SP::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    SP pot(cfg);
    pot.update(1u, 0.01f);
    CHECK(!pot.state().changed);
    CHECK(pot.changeSequence() == 0u);
    raw = 900;
    pot.update(2u, 0.01f);
    CHECK(pot.state().changed);
    CHECK(pot.changeSequence() == 1u);
    raw = -1;
    pot.update(3u, 0.01f);
    CHECK(!pot.state().changed);
    CHECK(pot.changeSequence() == 1u);
    CHECK(!pot.valid());
}

static void test_stepped_hysteresis()
{
    MiniTest::begin("SteppedPot hysteresis holds boundary noise");
    int raw = 300;
    using SP = PotIO::SteppedPot<4u, IntReader, PotIO::NoFilter>;
    SP::Config cfg;
    cfg.reader = IntReader(&raw);
    cfg.calib = calib1023();
    cfg.hysteresis = 0.10f;
    SP pot(cfg);
    pot.update(1u, 0.01f);
    const uint8_t step = pot.state().step;
    raw = 510; // near the nominal 0.5 boundary
    pot.update(2u, 0.01f);
    CHECK(pot.state().step == step);
}

static void test_rolling_jitter_forgets_old_outlier()
{
    MiniTest::begin("rolling jitter forgets old outliers");
    PotIO::RollingJitterStats<4u> jitter;
    jitter.observe(0u);
    jitter.observe(100u);
    jitter.observe(101u);
    jitter.observe(102u);
    CHECK(jitter.peak_to_peak() == 102u);
    jitter.observe(103u);
    CHECK(jitter.peak_to_peak() == 3u);
    CHECK(jitter.full());
}

static void test_wrap_safe_dt()
{
    MiniTest::begin("internal dt is wrap-safe across uint32 millis rollover");
    PotIO::detail::DtState dt;
    CHECK_NEAR(dt.step(0xFFFFFFF0u), 0.0, 1e-9);
    CHECK_NEAR(dt.step(0x00000010u), 0.032, 1e-6);
}

int main()
{
    test_version_macros();
    test_failed_read_retains_last_good();
    test_explicit_reader_failure();
    test_out_of_range_is_invalid();
    test_calibration_policies();
    test_calibration_bounds_and_span();
    test_calibration_mapping_boundaries();
    test_linear_mapping_sweep();
    test_filter_rate_history_is_independent();
    test_nonfinite_processing_output_is_invalid();
    test_stepped_nonfinite_filter_does_not_quantize();
    test_invalid_timing_holds_output();
    test_invalid_builtin_policy_is_rejected();
    test_invalid_enum_configuration_is_rejected();
    test_reset_processing_policy_is_explicit();
    test_axial_scaled_deadzone();
    test_radial_square_has_no_corner_gain();
    test_geometry_policies();
    test_geometry_sweep_stays_bounded();
    test_angle_validity();
    test_joystick_failure_is_atomic_across_axes();
    test_continuous_wrap_and_discontinuity();
    test_continuous_dropout_uses_last_good_time();
    test_continuous_velocity_limit();
    test_stepped_change_sequence_survives_latest_state();
    test_stepped_hysteresis();
    test_rolling_jitter_forgets_old_outlier();
    test_wrap_safe_dt();

    std::printf("PASS: %d tests / %d assertions\n", MiniTest::tests, MiniTest::assertions);
    return 0;
}
