// test_fls.cpp -- self contained test suite for FLS.hpp / FLS.cpp.
//
// No test framework on purpose: the point of this package is that it needs
// nothing but a C++17 compiler, and a test suite that drags in gtest would
// contradict that. Build and run:
//
//     c++ -std=c++17 -Wall -Wextra -O2 FLS.cpp test_fls.cpp -o test_fls && ./test_fls
//
// SPDX-License-Identifier: MIT

#include "FLS.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace
{
int g_checks = 0;
int g_failures = 0;
const char *g_case = "";

void BeginCase(const char *name)
{
    g_case = name;
}

bool Check(bool ok, const char *expr, int line)
{
    ++g_checks;
    if (!ok)
    {
        ++g_failures;
        std::printf("  FAIL  [%s] line %d: %s\n", g_case, line, expr);
    }
    return ok;
}

bool CheckNear(double got, double want, double tol, const char *expr, int line)
{
    ++g_checks;
    if (!(std::fabs(got - want) <= tol))
    {
        ++g_failures;
        std::printf("  FAIL  [%s] line %d: %s  got %.6f want %.6f +/- %.6f\n", g_case, line, expr, got,
                    want, tol);
        return false;
    }
    return true;
}

#define CHECK(x) Check((x), #x, __LINE__)
#define CHECK_NEAR(a, b, t) CheckNear((a), (b), (t), #a " ~= " #b, __LINE__)

// ---------------------------------------------------------------------------
// A flat wall perpendicular to world +x, at x = wall_x.
//
// The truth here is computed from the plane equation and never from the model,
// so a bug in the model cannot make its own test pass.
// ---------------------------------------------------------------------------
class WallWorld : public fls::IWorld
{
  public:
    double wall_x = 25.0;
    double target_strength_db = fls::kUseDefaultTargetStrength;
    /** Wall normal. Default faces -x, i.e. straight back at the sensor. */
    fls::Vec3 normal{-1.0, 0.0, 0.0};
    mutable int calls = 0;

    fls::RayHit CastRay(const fls::Vec3 &origin, const fls::Vec3 &dir, double max_range) const override
    {
        ++calls;
        fls::RayHit out;
        if (!(dir.x > 1e-9))
        {
            return out; // pointing away from the wall
        }
        const double t = (wall_x - origin.x) / dir.x;
        if (!(t > 0.0) || t > max_range)
        {
            return out;
        }
        out.hit = true;
        out.range_m = t;
        out.incidence_cos = std::fabs(fls::Dot(dir, fls::Normalized(normal)));
        out.target_strength_db = target_strength_db;
        return out;
    }
};

/** Two walls: the near one must win, whatever order the rays run in. */
class TwoWallWorld : public fls::IWorld
{
  public:
    double near_x = 10.0;
    double far_x = 40.0;
    fls::RayHit CastRay(const fls::Vec3 &origin, const fls::Vec3 &dir, double max_range) const override
    {
        fls::RayHit out;
        if (!(dir.x > 1e-9))
        {
            return out;
        }
        // The near wall is a narrow strip, so only off-axis rays miss it.
        const double tn = (near_x - origin.x) / dir.x;
        const double hit_y = origin.y + dir.y * tn;
        const double hit_z = origin.z + dir.z * tn;
        if (tn > 0.0 && tn <= max_range && std::fabs(hit_y) < 0.5 && std::fabs(hit_z) < 0.5)
        {
            out.hit = true;
            out.range_m = tn;
            out.incidence_cos = std::fabs(dir.x);
            return out;
        }
        const double tf = (far_x - origin.x) / dir.x;
        if (tf > 0.0 && tf <= max_range)
        {
            out.hit = true;
            out.range_m = tf;
            out.incidence_cos = std::fabs(dir.x);
        }
        return out;
    }
};

class EmptyWorld : public fls::IWorld
{
  public:
    fls::RayHit CastRay(const fls::Vec3 &, const fls::Vec3 &, double) const override
    {
        return fls::RayHit();
    }
};

fls::Pose IdentityPose()
{
    fls::Pose p;
    p.position_m = fls::Vec3{0.0, 0.0, 0.0};
    p.forward = fls::Vec3{1.0, 0.0, 0.0};
    p.right = fls::Vec3{0.0, 1.0, 0.0};
    p.up = fls::Vec3{0.0, 0.0, 1.0};
    return p;
}

/** Runs the sensor until `want` pings have completed; returns the last one. */
fls::Output RunPings(fls::Fls &s, const fls::Pose &pose, const fls::IWorld &w, int want,
                     std::vector<fls::Output> *all = nullptr)
{
    fls::Output last;
    int seen = 0;
    for (int i = 0; i < 100000 && seen < want; ++i)
    {
        const fls::Output o = s.Update(0.02, pose, w);
        if (o.pinged)
        {
            ++seen;
            last = o;
            if (all != nullptr)
            {
                all->push_back(o);
            }
        }
    }
    return last;
}

// =========================================================================
// Tests
// =========================================================================

void TestVectorHelpers()
{
    BeginCase("VectorHelpers");
    CHECK_NEAR(fls::Length(fls::Vec3{3.0, 4.0, 0.0}), 5.0, 1e-12);
    const fls::Vec3 n = fls::Normalized(fls::Vec3{0.0, 0.0, 7.0});
    CHECK_NEAR(n.z, 1.0, 1e-12);
    // A degenerate vector must come back as zero, not as NaN: the model tests
    // Length() to decide whether the host handed it a usable basis.
    const fls::Vec3 z = fls::Normalized(fls::Vec3{0.0, 0.0, 0.0});
    CHECK(fls::Length(z) == 0.0);
    CHECK_NEAR(fls::Dot(fls::Vec3{1.0, 2.0, 3.0}, fls::Vec3{4.0, -5.0, 6.0}), 12.0, 1e-12);
}

void TestBeamWidthFollowsDataSheet()
{
    BeginCase("BeamWidthFollowsDataSheet");
    // MiniAlt_rI 07/27/23: 170 kHz-C is 18 deg, 200 kHz-A is 14 deg at -3 dB.
    CHECK_NEAR(fls::BeamWidthDegForFrequencyKhz(200.0), 14.0, 1e-12);
    CHECK_NEAR(fls::BeamWidthDegForFrequencyKhz(170.0), 18.0, 1e-12);
    // Anything the data sheet does not publish must return 0 so the caller is
    // forced to reject it. A silent default would be a fabricated spec.
    CHECK(fls::BeamWidthDegForFrequencyKhz(455.0) == 0.0);
    CHECK(fls::BeamWidthDegForFrequencyKhz(0.0) == 0.0);
    CHECK(fls::BeamWidthDegForFrequencyKhz(-200.0) == 0.0);
    CHECK(fls::BeamWidthDegForFrequencyKhz(169.0) == 0.0);

    fls::Config c;
    CHECK_NEAR(fls::Fls(c).BeamWidthDeg(), 14.0, 1e-12);
    c.frequency_khz = 170.0;
    CHECK_NEAR(fls::Fls(c).BeamWidthDeg(), 18.0, 1e-12);
    // An explicit beam width overrides the derivation.
    c.beam_width_deg = 9.0;
    CHECK_NEAR(fls::Fls(c).BeamWidthDeg(), 9.0, 1e-12);
}

void TestConfigValidation()
{
    BeginCase("ConfigValidation");
    CHECK(fls::Config().Validate().empty());

    fls::Config bad;
    bad.frequency_khz = 455.0;
    CHECK(!bad.Validate().empty());

    // ... but an explicit beam width makes an unlisted frequency acceptable.
    bad.beam_width_deg = 12.0;
    CHECK(bad.Validate().empty());

    fls::Config c;
    c.blanking_m = 20.0;
    CHECK(!c.Validate().empty());
    c = fls::Config();
    c.depth_filter_samples = 3;
    CHECK(!c.Validate().empty());
    c = fls::Config();
    c.smoothing_filter = 5;
    CHECK(!c.Validate().empty());
    c = fls::Config();
    c.max_range_m = 0.1;
    CHECK(!c.Validate().empty());
    c = fls::Config();
    c.beam_rings = 9;
    CHECK(!c.Validate().empty());
    c = fls::Config();
    c.dropout_probability = 1.5;
    CHECK(!c.Validate().empty());
    c = fls::Config();
    c.misses_to_lose_lock = 0;
    CHECK(!c.Validate().empty());

    // SetConfig must REJECT and keep the old configuration, not half apply it.
    fls::Fls s;
    const double before = s.BeamWidthDeg();
    fls::Config reject;
    reject.frequency_khz = 455.0;
    CHECK(!s.SetConfig(reject).empty());
    CHECK_NEAR(s.BeamWidthDeg(), before, 1e-12);
}

void TestUnconfiguredBeamWidthFailsLoudly()
{
    BeginCase("UnconfiguredBeamWidthFailsLoudly");
    // Constructed with a frequency the data sheet does not list: the sensor
    // must refuse to report a range rather than invent a beam width.
    fls::Config c;
    c.frequency_khz = 455.0;
    fls::Fls s(c);
    CHECK(s.BeamWidthDeg() == 0.0);
    WallWorld w;
    const fls::Output o = RunPings(s, IdentityPose(), w, 1);
    CHECK(o.pinged);
    CHECK(!o.has_range);
    CHECK(std::string(o.reject_reason).find("beam width") != std::string::npos);
}

void TestMountTransform()
{
    BeginCase("MountTransform");
    fls::Config c;
    c.mount_forward_m = 2.12; // nose of the hull
    c.mount_up_m = -0.004;
    fls::Fls s(c);

    fls::Pose p = IdentityPose();
    p.position_m = fls::Vec3{10.0, 3.0, -5.0};
    fls::Vec3 origin;
    fls::Vec3 axis;
    s.BeamFrame(p, origin, axis);
    CHECK_NEAR(origin.x, 12.12, 1e-9);
    CHECK_NEAR(origin.y, 3.0, 1e-9);
    CHECK_NEAR(origin.z, -5.004, 1e-9);
    CHECK_NEAR(axis.x, 1.0, 1e-9); // straight ahead

    // Pitch -90 must look along -up: that is the downward altimeter case.
    c.mount_pitch_deg = -90.0;
    fls::Fls down(c);
    down.BeamFrame(p, origin, axis);
    CHECK_NEAR(axis.x, 0.0, 1e-9);
    CHECK_NEAR(axis.z, -1.0, 1e-9);

    // Yaw +90 must look along +right.
    c.mount_pitch_deg = 0.0;
    c.mount_yaw_deg = 90.0;
    fls::Fls side(c);
    side.BeamFrame(p, origin, axis);
    CHECK_NEAR(axis.y, 1.0, 1e-9);
    CHECK_NEAR(axis.x, 0.0, 1e-9);

    // The mount offsets follow the host's basis, not world axes: rotating the
    // platform must carry the sensor with it.
    c = fls::Config();
    c.mount_forward_m = 2.0;
    fls::Fls f(c);
    fls::Pose turned = IdentityPose();
    turned.forward = fls::Vec3{0.0, 1.0, 0.0}; // platform faces world +y
    turned.right = fls::Vec3{-1.0, 0.0, 0.0};
    f.BeamFrame(turned, origin, axis);
    CHECK_NEAR(origin.y, 2.0, 1e-9);
    CHECK_NEAR(origin.x, 0.0, 1e-9);
    CHECK_NEAR(axis.y, 1.0, 1e-9);
}

void TestDegenerateBasisIsRejected()
{
    BeginCase("DegenerateBasisIsRejected");
    fls::Fls s;
    fls::Pose p = IdentityPose();
    p.forward = fls::Vec3{0.0, 0.0, 0.0};
    p.right = fls::Vec3{0.0, 0.0, 0.0};
    p.up = fls::Vec3{0.0, 0.0, 0.0};
    WallWorld w;
    const fls::Output o = RunPings(s, p, w, 1);
    CHECK(o.pinged);
    CHECK(!o.has_range);
    CHECK(std::string(o.reject_reason).find("degenerate") != std::string::npos);
}

void TestRayCountAndAxisFirst()
{
    BeginCase("RayCountAndAxisFirst");
    fls::Fls s;
    CHECK(s.RaysPerPing() == 13); // 1 centre + 2 rings * 6
    WallWorld w;
    RunPings(s, IdentityPose(), w, 1);
    const std::vector<fls::Ray> &rays = s.LastRays();
    CHECK(rays.size() == 13u);
    CHECK_NEAR(rays[0].off_axis_deg, 0.0, 1e-12);
    // The outermost ring sits on the -3 dB edge, i.e. half the beam width.
    double worst = 0.0;
    for (std::size_t i = 0; i < rays.size(); ++i)
    {
        if (rays[i].off_axis_deg > worst)
        {
            worst = rays[i].off_axis_deg;
        }
    }
    CHECK_NEAR(worst, 7.0, 1e-9); // 14 deg / 2

    fls::Config c;
    c.beam_rings = 0;
    fls::Fls single(c);
    CHECK(single.RaysPerPing() == 1);
    c.beam_rings = 3;
    c.beam_samples_per_ring = 8;
    CHECK(fls::Fls(c).RaysPerPing() == 25);
}

/** The headline number: the reported range against the data sheet bias. */
void TestReportedRangeCarriesDataSheetBias()
{
    BeginCase("ReportedRangeCarriesDataSheetBias");
    fls::Config c;
    c.dropout_probability = 0.0;
    c.range_noise_sigma_m = 0.0; // isolate the scale error
    fls::Fls s(c);
    WallWorld w;
    w.wall_x = 25.0;

    const fls::Output o = RunPings(s, IdentityPose(), w, 1);
    CHECK(o.has_range);
    CHECK_NEAR(o.true_range_m, 25.0, 1e-9);
    // +0.54 % of 25 m is 13.5 cm, then quantised to 1 cm.
    CHECK_NEAR(o.range_m, 25.0 * (1.0 + fls::spec::kScaleErrorFraction), 0.01);
    CHECK(o.range_m > o.true_range_m); // the data sheet's error is always high

    // Quantisation: every reported range is a whole number of centimetres.
    const double cm = o.range_m / fls::spec::kRangeResolutionMeters;
    CHECK_NEAR(cm, std::floor(cm + 0.5), 1e-6);

    // With noise on, the MEAN bias over many pings must still land on the
    // data sheet figure. This is measured, not asserted from a fixture.
    fls::Config n;
    n.dropout_probability = 0.0;
    fls::Fls noisy(n);
    std::vector<fls::Output> all;
    RunPings(noisy, IdentityPose(), w, 400, &all);
    double sum = 0.0;
    int used = 0;
    for (std::size_t i = 0; i < all.size(); ++i)
    {
        if (all[i].has_range)
        {
            sum += (all[i].range_m - all[i].true_range_m) / all[i].true_range_m;
            ++used;
        }
    }
    CHECK(used > 300);
    const double mean_bias = sum / static_cast<double>(used);
    std::printf("  measured mean bias over %d pings: %+.4f %% "
                "(data sheet %+.4f %%)\n",
                used, mean_bias * 100.0, fls::spec::kScaleErrorFraction * 100.0);
    CHECK_NEAR(mean_bias, fls::spec::kScaleErrorFraction, 0.0015);
}

void TestDeadZoneAndBlanking()
{
    BeginCase("DeadZoneAndBlanking");
    fls::Config c;
    c.dropout_probability = 0.0;
    fls::Fls s(c);
    WallWorld w;
    w.wall_x = 0.2; // inside the 0.4 m minimum range
    fls::Output o = RunPings(s, IdentityPose(), w, 1);
    CHECK(!o.has_range);
    CHECK(std::string(o.reject_reason).find("minimum range") != std::string::npos);

    // Operator blanking rejects a target that is otherwise perfectly visible.
    c.blanking_m = 5.0;
    fls::Fls b(c);
    w.wall_x = 3.0;
    o = RunPings(b, IdentityPose(), w, 1);
    CHECK(!o.has_range);
    fls::Fls b2(c);
    w.wall_x = 8.0;
    o = RunPings(b2, IdentityPose(), w, 1);
    CHECK(o.has_range);
}

void TestNoEchoWhenNothingInBeam()
{
    BeginCase("NoEchoWhenNothingInBeam");
    fls::Config c;
    c.dropout_probability = 0.0;
    fls::Fls s(c);
    EmptyWorld empty;
    const fls::Output o = RunPings(s, IdentityPose(), empty, 1);
    CHECK(o.pinged);
    CHECK(!o.has_range);
    CHECK(o.rays_hit == 0);
    CHECK(o.rays_cast == 13);
    CHECK(std::string(o.reject_reason) == "no target in beam");
}

void TestGrazingIncidenceIsRejected()
{
    BeginCase("GrazingIncidenceIsRejected");
    WallWorld w;
    w.wall_x = 20.0;
    // A wall almost edge-on to the beam axis: the normal is nearly
    // perpendicular to it, so the axial ray's incidence cosine is 0.05, below
    // the 0.10 threshold, and no echo returns.
    w.normal = fls::Vec3{-0.05, 1.0, 0.0};

    fls::Config c;
    c.dropout_probability = 0.0;
    c.beam_rings = 0; // axial ray only, so only incidence is under test
    fls::Fls axial(c);
    const fls::Output o = RunPings(axial, IdentityPose(), w, 1);
    CHECK(!o.has_range);
    CHECK(std::string(o.reject_reason).find("incidence") != std::string::npos);

    // But the FULL cone does see that same wall, and this is not a bug: the
    // beam is 14 deg wide, so its outermost rays strike the wall 7 deg away
    // from the axis, giving an incidence cosine near sin(7 deg) = 0.12 --
    // above the threshold. A real altimeter picks up an edge-on surface
    // through the rim of its beam in exactly this way, and a model that
    // rejected it would be hiding the cone's width.
    //
    // MEASURED against this wall: the axial ray gets 0.050, the inner ring
    // (3.5 deg) reaches 0.111 and the outer ring (7 deg) 0.155. The REPORTED
    // return is the inner ring's, not the outer one's, because first-return
    // selection takes the SHORTEST qualifying range and the inner ring is
    // nearer to the wall than the outer one: against this 20 m wall the slant
    // ranges are 20.037 m and 20.150 m.
    fls::Config full;
    full.dropout_probability = 0.0;
    fls::Fls cone(full);
    const fls::Output o2 = RunPings(cone, IdentityPose(), w, 1);
    CHECK(o2.has_range);
    CHECK_NEAR(o2.off_axis_deg, 3.5, 1e-6);      // off the axis: the inner ring
    CHECK_NEAR(o2.true_range_m, 20.037, 0.002);  // slant range, not 20.000 m

    // Raising the threshold above the rim's incidence rejects it again.
    full.min_incidence_cos = 0.25;
    fls::Fls strict(full);
    const fls::Output o3 = RunPings(strict, IdentityPose(), w, 1);
    CHECK(!o3.has_range);
    CHECK(std::string(o3.reject_reason).find("incidence") != std::string::npos);
}

void TestSonarEquationHidesSoftFarTargets()
{
    BeginCase("SonarEquationHidesSoftFarTargets");
    fls::Config c;
    c.dropout_probability = 0.0;
    WallWorld w;
    w.wall_x = 150.0;

    // MEASURED against the default tuning (SL 200, NL 40, alpha 55 dB/km,
    // DT 10): a -35 dB soft target at 150 m still returns 21.5 dB of SNR, so
    // it IS visible. The equation only hides it once the target gets weaker
    // than about -46 dB at this range. The test uses -55 dB (SNR 1.5 dB)
    // rather than a figure that merely sounds weak.
    w.target_strength_db = -35.0;
    fls::Fls soft(c);
    CHECK(RunPings(soft, IdentityPose(), w, 1).has_range);

    w.target_strength_db = -55.0;
    fls::Fls weak(c);
    const fls::Output a = RunPings(weak, IdentityPose(), w, 1);
    CHECK(!a.has_range);
    CHECK(std::string(a.reject_reason).find("threshold") != std::string::npos);

    // Geometry alone would have reported it: the sonar equation is doing the
    // work here, not the range gate.
    c.use_sonar_equation = false;
    fls::Fls geometric(c);
    const fls::Output b = RunPings(geometric, IdentityPose(), w, 1);
    CHECK(b.has_range);
    CHECK_NEAR(b.true_range_m, 150.0, 1e-9);

    // A hard target at the same range is visible with the equation on.
    c.use_sonar_equation = true;
    w.target_strength_db = fls::kUseDefaultTargetStrength;
    fls::Fls hard(c);
    CHECK(RunPings(hard, IdentityPose(), w, 1).has_range);
}

void TestFirstReturnWinsNotNearestGeometry()
{
    BeginCase("FirstReturnWinsNotNearestGeometry");
    fls::Config c;
    c.dropout_probability = 0.0;
    c.range_noise_sigma_m = 0.0;
    fls::Fls s(c);
    TwoWallWorld w;
    const fls::Output o = RunPings(s, IdentityPose(), w, 1);
    CHECK(o.has_range);
    // Only the centre ray reaches the narrow near wall; the device must still
    // report 10 m, because a single beam reports its FIRST return.
    CHECK_NEAR(o.true_range_m, 10.0, 1e-9);
    CHECK_NEAR(o.off_axis_deg, 0.0, 1e-9);
}

void TestAutoRangeSelection()
{
    BeginCase("AutoRangeSelection");
    fls::Config c;
    c.dropout_probability = 0.0;
    c.range_mode = fls::RangeMode::Auto;
    WallWorld w;

    // 25 m sits inside the 225 ft (68.58 m) Medium scale.
    fls::Fls medium(c);
    w.wall_x = 25.0;
    fls::Output o = RunPings(medium, IdentityPose(), w, 6);
    CHECK(o.has_range);
    CHECK(o.active_range == fls::RangeMode::Medium);
    CHECK_NEAR(o.range_scale_m, 225.0 * fls::spec::kMetersPerFoot, 1e-9);

    // Hysteresis: a step DOWN needs the target below 45 % of the lower mode's
    // maximum, i.e. below 17.0688 * 0.45 = 7.681 m. So 10 m deliberately
    // STAYS on Medium -- a target sitting near a boundary must not make the
    // device oscillate between two range scales.
    fls::Fls hold(c);
    w.wall_x = 10.0;
    o = RunPings(hold, IdentityPose(), w, 8);
    CHECK(o.has_range);
    CHECK(o.active_range == fls::RangeMode::Medium);

    // Below that threshold it does come down to the 56 ft (17.07 m) Short
    // scale, which also raises the ping rate.
    fls::Fls shortm(c);
    w.wall_x = 5.0;
    o = RunPings(shortm, IdentityPose(), w, 8);
    CHECK(o.has_range);
    CHECK(o.active_range == fls::RangeMode::Short);
    CHECK(shortm.EffectivePingsPerSecond() == 4);

    // 150 m needs the 675 ft (205.74 m) Very Long scale.
    fls::Fls longm(c);
    w.wall_x = 150.0;
    o = RunPings(longm, IdentityPose(), w, 8);
    CHECK(o.has_range);
    CHECK(o.active_range == fls::RangeMode::VeryLong);
}

void TestPinnedRangeModeTruncates()
{
    BeginCase("PinnedRangeModeTruncates");
    fls::Config c;
    c.dropout_probability = 0.0;
    c.range_mode = fls::RangeMode::Short; // 17.07 m
    fls::Fls s(c);
    WallWorld w;
    w.wall_x = 40.0;
    const fls::Output o = RunPings(s, IdentityPose(), w, 1);
    CHECK(!o.has_range);
    CHECK(std::string(o.reject_reason).find("range scale") != std::string::npos);
    CHECK(o.active_range == fls::RangeMode::Short);
}

void TestPingRateFollowsRangeMode()
{
    BeginCase("PingRateFollowsRangeMode");
    fls::Config c;
    c.range_mode = fls::RangeMode::Short;
    fls::Fls s(c);
    CHECK(s.EffectivePingsPerSecond() == 4); // auto = half of 8
    c.pings_per_second = 2;
    CHECK(fls::Fls(c).EffectivePingsPerSecond() == 2);
    // A request above what the mode allows is clamped, not honoured.
    c.pings_per_second = 50;
    CHECK(fls::Fls(c).EffectivePingsPerSecond() == 8);
    c.range_mode = fls::RangeMode::VeryLong;
    CHECK(fls::Fls(c).EffectivePingsPerSecond() == 3);

    // One ping per Update at most, even across an absurdly long frame.
    fls::Config d;
    d.dropout_probability = 0.0;
    fls::Fls burst(d);
    WallWorld w;
    const fls::Output o = burst.Update(60.0, IdentityPose(), w);
    CHECK(o.pinged);
    CHECK(burst.PingCount() == 1);

    // dt <= 0 must do nothing at all.
    const long long before = burst.PingCount();
    burst.Update(0.0, IdentityPose(), w);
    burst.Update(-1.0, IdentityPose(), w);
    CHECK(burst.PingCount() == before);
}

void TestLockIsLostAfterConsecutiveMisses()
{
    BeginCase("LockIsLostAfterConsecutiveMisses");
    fls::Config c;
    c.dropout_probability = 0.0;
    c.misses_to_lose_lock = 3;
    fls::Fls s(c);
    WallWorld wall;
    EmptyWorld empty;

    RunPings(s, IdentityPose(), wall, 3);
    CHECK(s.HasLock());

    RunPings(s, IdentityPose(), empty, 2);
    CHECK(s.HasLock()); // two misses is not yet enough
    RunPings(s, IdentityPose(), empty, 1);
    CHECK(!s.HasLock()); // the third drops it

    RunPings(s, IdentityPose(), wall, 1);
    CHECK(s.HasLock()); // and it re-acquires
    CHECK(s.MissCount() == 3);
}

void TestDisabledSensorIsSilent()
{
    BeginCase("DisabledSensorIsSilent");
    fls::Config c;
    c.dropout_probability = 0.0;
    fls::Fls s(c);
    WallWorld w;
    RunPings(s, IdentityPose(), w, 2);
    CHECK(s.HasLock());

    s.SetEnabled(false);
    CHECK(!s.HasLock());
    const long long before = s.PingCount();
    for (int i = 0; i < 200; ++i)
    {
        const fls::Output o = s.Update(0.02, IdentityPose(), w);
        CHECK(!o.pinged);
        CHECK(!o.has_range);
    }
    CHECK(s.PingCount() == before);
    CHECK(w.calls == before * 13);

    s.SetEnabled(true);
    CHECK(RunPings(s, IdentityPose(), w, 1).has_range);
}

void TestDeterminism()
{
    BeginCase("Determinism");
    fls::Config c;
    WallWorld w;
    std::vector<fls::Output> a;
    std::vector<fls::Output> b;
    fls::Fls s1(c);
    fls::Fls s2(c);
    RunPings(s1, IdentityPose(), w, 60, &a);
    RunPings(s2, IdentityPose(), w, 60, &b);
    CHECK(a.size() == b.size());
    bool identical = a.size() == b.size();
    for (std::size_t i = 0; identical && i < a.size(); ++i)
    {
        identical = a[i].has_range == b[i].has_range && a[i].range_m == b[i].range_m;
    }
    CHECK(identical);

    // A different seed must actually change the stream, otherwise the noise
    // and dropout models are not doing anything.
    c.seed = 12345ull;
    std::vector<fls::Output> d;
    fls::Fls s3(c);
    RunPings(s3, IdentityPose(), w, 60, &d);
    bool differs = false;
    for (std::size_t i = 0; i < d.size() && i < a.size(); ++i)
    {
        if (d[i].range_m != a[i].range_m)
        {
            differs = true;
        }
    }
    CHECK(differs);

    // Reset must rewind exactly, not approximately.
    fls::Fls s4{fls::Config()};
    std::vector<fls::Output> e;
    RunPings(s4, IdentityPose(), w, 20, &e);
    s4.Reset();
    std::vector<fls::Output> f;
    RunPings(s4, IdentityPose(), w, 20, &f);
    bool rewound = e.size() == f.size();
    for (std::size_t i = 0; rewound && i < e.size(); ++i)
    {
        rewound = e[i].range_m == f[i].range_m;
    }
    CHECK(rewound);
}

void TestDropoutHappensAndIsBounded()
{
    BeginCase("DropoutHappensAndIsBounded");
    fls::Config c;
    c.dropout_probability = 0.2;
    c.misses_to_lose_lock = 100; // keep lock so only dropout shows
    fls::Fls s(c);
    WallWorld w;
    std::vector<fls::Output> all;
    RunPings(s, IdentityPose(), w, 500, &all);
    int dropped = 0;
    for (std::size_t i = 0; i < all.size(); ++i)
    {
        if (!all[i].has_range)
        {
            ++dropped;
        }
    }
    const double rate = static_cast<double>(dropped) / static_cast<double>(all.size());
    std::printf("  measured dropout rate: %.3f (configured 0.200)\n", rate);
    CHECK_NEAR(rate, 0.2, 0.06);

    // Zero means zero: no stray rejections when everything is in view.
    fls::Config none;
    none.dropout_probability = 0.0;
    fls::Fls clean(none);
    std::vector<fls::Output> good;
    RunPings(clean, IdentityPose(), w, 200, &good);
    int misses = 0;
    for (std::size_t i = 0; i < good.size(); ++i)
    {
        if (!good[i].has_range)
        {
            ++misses;
        }
    }
    CHECK(misses == 0);
}

void TestFiltersSmoothTheOutput()
{
    BeginCase("FiltersSmoothTheOutput");
    // The averaging filter must reduce the spread of the reported range.
    fls::Config raw;
    raw.dropout_probability = 0.0;
    raw.range_noise_sigma_m = 0.10;
    fls::Config filtered = raw;
    filtered.depth_filter_samples = 4;

    WallWorld w;
    w.wall_x = 30.0;
    double spread[2] = {0.0, 0.0};
    const fls::Config *cfgs[2] = {&raw, &filtered};
    for (int k = 0; k < 2; ++k)
    {
        fls::Fls s(*cfgs[k]);
        std::vector<fls::Output> all;
        RunPings(s, IdentityPose(), w, 300, &all);
        double sum = 0.0;
        double sumsq = 0.0;
        int n = 0;
        for (std::size_t i = 0; i < all.size(); ++i)
        {
            if (all[i].has_range)
            {
                sum += all[i].range_m;
                sumsq += all[i].range_m * all[i].range_m;
                ++n;
            }
        }
        const double mean = sum / n;
        spread[k] = std::sqrt(sumsq / n - mean * mean);
    }
    std::printf("  measured sigma: unfiltered %.4f m, DFILTER=4 %.4f m\n", spread[0], spread[1]);
    CHECK(spread[1] < spread[0] * 0.75);
    // Four-sample averaging should approach sigma/2.
    CHECK_NEAR(spread[1], spread[0] * 0.5, spread[0] * 0.25);
}

void TestOutputHoldsBetweenPings()
{
    BeginCase("OutputHoldsBetweenPings");
    fls::Config c;
    c.dropout_probability = 0.0;
    c.pings_per_second = 2; // a ping every 0.5 s
    fls::Fls s(c);
    WallWorld w;
    const fls::Output first = RunPings(s, IdentityPose(), w, 1);
    CHECK(first.pinged);
    // The next Update is between pings: pinged must be false while the range
    // stays readable, so a host polling every frame does not see a gap.
    const fls::Output between = s.Update(0.02, IdentityPose(), w);
    CHECK(!between.pinged);
    CHECK(between.has_range);
    CHECK(between.range_m == first.range_m);
    CHECK(between.time_s > first.time_s);
    CHECK(s.LastOutput().range_m == first.range_m);
}

void TestBeamOriginIsReportedForDrawing()
{
    BeginCase("BeamOriginIsReportedForDrawing");
    fls::Config c;
    c.dropout_probability = 0.0;
    c.mount_forward_m = 2.0;
    fls::Fls s(c);
    fls::Pose p = IdentityPose();
    p.position_m = fls::Vec3{5.0, 0.0, 0.0};
    WallWorld w;
    w.wall_x = 25.0;
    const fls::Output o = RunPings(s, p, w, 1);
    CHECK(o.has_range);
    // The beam leaves the mount point, not the platform origin: 7 m, so the
    // wall is 18 m away, not 20 m.
    CHECK_NEAR(o.beam_origin_m.x, 7.0, 1e-9);
    CHECK_NEAR(o.true_range_m, 18.0, 1e-9);
    CHECK_NEAR(o.beam_axis.x, 1.0, 1e-9);
    CHECK(s.LastRays()[0].hit);
    CHECK_NEAR(s.LastRays()[0].range_m, 18.0, 1e-9);
}

void TestRangeModeTableMatchesDataSheet()
{
    BeginCase("RangeModeTableMatchesDataSheet");
    // Data sheet range scales in feet; these are the exact conversions.
    CHECK_NEAR(fls::GetRangeModeInfo(fls::RangeMode::Short).max_meters, 17.0688, 1e-6);
    CHECK_NEAR(fls::GetRangeModeInfo(fls::RangeMode::Medium).max_meters, 68.58, 1e-6);
    CHECK_NEAR(fls::GetRangeModeInfo(fls::RangeMode::Long).max_meters, 137.16, 1e-6);
    CHECK_NEAR(fls::GetRangeModeInfo(fls::RangeMode::VeryLong).max_meters, 205.74, 1e-6);
    CHECK(std::string(fls::ToString(fls::RangeMode::Short)) == "Short");
    CHECK(fls::spec::kMaxRangeMeters == 200.0);
    CHECK(fls::spec::kMinRangeMeters == 0.4);
    CHECK(fls::spec::kRangeResolutionMeters == 0.01);
    // Transducer body, data sheet DIMENSIONS: 1.0 in and 2.30 in.
    CHECK_NEAR(fls::spec::kTransducerDiameterMeters, 0.0254, 1e-9);
    CHECK_NEAR(fls::spec::kTransducerLengthMeters, 0.0584, 1e-9);
}

} // namespace

int main()
{
    std::printf("FLS self test\n");
    TestVectorHelpers();
    TestBeamWidthFollowsDataSheet();
    TestConfigValidation();
    TestUnconfiguredBeamWidthFailsLoudly();
    TestMountTransform();
    TestDegenerateBasisIsRejected();
    TestRayCountAndAxisFirst();
    TestReportedRangeCarriesDataSheetBias();
    TestDeadZoneAndBlanking();
    TestNoEchoWhenNothingInBeam();
    TestGrazingIncidenceIsRejected();
    TestSonarEquationHidesSoftFarTargets();
    TestFirstReturnWinsNotNearestGeometry();
    TestAutoRangeSelection();
    TestPinnedRangeModeTruncates();
    TestPingRateFollowsRangeMode();
    TestLockIsLostAfterConsecutiveMisses();
    TestDisabledSensorIsSilent();
    TestDeterminism();
    TestDropoutHappensAndIsBounded();
    TestFiltersSmoothTheOutput();
    TestOutputHoldsBetweenPings();
    TestBeamOriginIsReportedForDrawing();
    TestRangeModeTableMatchesDataSheet();

    std::printf("%s: %d checks, %d failures\n", g_failures == 0 ? "PASS" : "FAIL", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
