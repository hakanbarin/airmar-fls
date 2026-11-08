// FLS.cpp -- implementation of the single beam forward looking sonar model.
// See FLS.hpp for the integration contract.
//
// SPDX-License-Identifier: MIT

#include "FLS.hpp"

#include <cmath>

namespace airmar
{

// ===========================================================================
// Geometry
// ===========================================================================

Vec3 operator+(const Vec3 &a, const Vec3 &b)
{
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec3 operator-(const Vec3 &a, const Vec3 &b)
{
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}
Vec3 operator*(const Vec3 &v, double s)
{
    return Vec3{v.x * s, v.y * s, v.z * s};
}
double Dot(const Vec3 &a, const Vec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
double Length(const Vec3 &v)
{
    return std::sqrt(Dot(v, v));
}
Vec3 Normalized(const Vec3 &v)
{
    const double L = Length(v);
    if (!(L > 1e-12) || !std::isfinite(L))
    {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{v.x / L, v.y / L, v.z / L};
}

// ===========================================================================
// Device tables
// ===========================================================================

RangeModeInfo GetRangeModeInfo(RangeMode mode)
{
    // Data sheet range scales, in feet, with the ping rate each one allows.
    switch (mode)
    {
    case RangeMode::Short:
        return RangeModeInfo{56.0 * spec::kMetersPerFoot, 8, "Short"};
    case RangeMode::Medium:
        return RangeModeInfo{225.0 * spec::kMetersPerFoot, 8, "Medium"};
    case RangeMode::Long:
        return RangeModeInfo{450.0 * spec::kMetersPerFoot, 4, "Long"};
    case RangeMode::VeryLong:
        return RangeModeInfo{675.0 * spec::kMetersPerFoot, 3, "VeryLong"};
    case RangeMode::Auto:
    default:
        return RangeModeInfo{675.0 * spec::kMetersPerFoot, 3, "Auto"};
    }
}

const char *ToString(RangeMode mode)
{
    return GetRangeModeInfo(mode).name;
}

double BeamWidthDegForFrequencyKhz(double frequency_khz)
{
    // Only the two housings the data sheet publishes a figure for. Anything
    // else returns 0 and the caller must reject it -- a silent fallback would
    // report a beam width the hardware does not have.
    if (frequency_khz > 169.5 && frequency_khz < 170.5)
    {
        return spec::kBeamWidthDeg170kHz;
    }
    if (frequency_khz > 199.5 && frequency_khz < 200.5)
    {
        return spec::kBeamWidthDeg200kHz;
    }
    return 0.0;
}

// ===========================================================================
// Config
// ===========================================================================

std::string Config::Validate() const
{
    if (beam_width_deg == 0.0)
    {
        if (!(BeamWidthDegForFrequencyKhz(frequency_khz) > 0.0))
        {
            return "frequency_khz must be 170 or 200 (the two housings the data "
                   "sheet gives a beam angle for), or set beam_width_deg explicitly";
        }
    }
    else if (!(beam_width_deg > 0.0) || !(beam_width_deg < 180.0))
    {
        return "beam_width_deg must be 0 (derive from frequency) or in (0,180)";
    }
    if (!(min_range_m > 0.0))
    {
        return "min_range_m must be positive";
    }
    if (!(max_range_m > min_range_m))
    {
        return "max_range_m must be greater than min_range_m";
    }
    if (blanking_m < 0.0 || blanking_m > 15.0)
    {
        return "blanking_m must be within 0..15 m";
    }
    if (beam_rings < 0 || beam_rings > 6)
    {
        return "beam_rings must be within 0..6";
    }
    if (beam_samples_per_ring < 1 || beam_samples_per_ring > 32)
    {
        return "beam_samples_per_ring must be within 1..32";
    }
    if (dropout_probability < 0.0 || dropout_probability > 1.0)
    {
        return "dropout_probability must be within 0..1";
    }
    if (misses_to_lose_lock < 1)
    {
        return "misses_to_lose_lock must be at least 1";
    }
    if (!(speed_of_sound_mps > 0.0))
    {
        return "speed_of_sound_mps must be positive";
    }
    if (depth_filter_samples != 0 && depth_filter_samples != 2 && depth_filter_samples != 4)
    {
        return "depth_filter_samples must be 0, 2 or 4";
    }
    if (smoothing_filter != 0 && smoothing_filter != 2 && smoothing_filter != 4 && smoothing_filter != 8)
    {
        return "smoothing_filter must be 0, 2, 4 or 8";
    }
    if (pings_per_second < 0)
    {
        return "pings_per_second must be 0 (auto) or positive";
    }
    if (!std::isfinite(mount_forward_m) || !std::isfinite(mount_right_m) || !std::isfinite(mount_up_m) ||
        !std::isfinite(mount_yaw_deg) || !std::isfinite(mount_pitch_deg))
    {
        return "mount transform contains a non-finite value";
    }
    return std::string();
}

// ===========================================================================
// Construction
// ===========================================================================

Fls::Fls()
{
    ApplyConfig();
    Reset();
}

Fls::Fls(const Config &config)
{
    // A rejected configuration is NOT silently replaced by defaults: the
    // fields are kept so the caller can see what it asked for, and a beam
    // width that could not be derived stays at 0, which makes every ping
    // report "beam width not configured" instead of inventing a number.
    config_ = config;
    ApplyConfig();
    Reset();
}

std::string Fls::SetConfig(const Config &config)
{
    const std::string Error = config.Validate();
    if (!Error.empty())
    {
        return Error;
    }
    config_ = config;
    ApplyConfig();
    Reset();
    return std::string();
}

const Config &Fls::GetConfig() const
{
    return config_;
}

void Fls::ApplyConfig()
{
    beam_width_deg_ = config_.beam_width_deg > 0.0 ? config_.beam_width_deg
                                                   : BeamWidthDegForFrequencyKhz(config_.frequency_khz);
    BuildCone();
}

void Fls::Reset()
{
    clock_s_ = 0.0;
    ping_accumulator_s_ = 0.0;
    pings_ = 0;
    locks_ = 0;
    misses_ = 0;
    consecutive_misses_ = 0;
    locked_ = false;
    filter_count_ = 0;
    filter_cursor_ = 0;
    smoothed_ = 0.0;
    has_smoothed_ = false;
    for (int I = 0; I < kMaxFilter; ++I)
    {
        filter_history_[I] = 0.0;
    }
    rng_state_ = config_.seed != 0ull ? config_.seed : 0x9E3779B97F4A7C15ull;
    rng_has_spare_ = false;
    rng_spare_ = 0.0;
    active_range_ = config_.range_mode == RangeMode::Auto ? RangeMode::Medium : config_.range_mode;
    last_ = Output();
    last_.active_range = active_range_;
    last_.range_scale_m = RangeScaleMeters();
}

void Fls::SetEnabled(bool enabled)
{
    enabled_ = enabled;
    if (!enabled)
    {
        // Same as the real unit's PING OFF: the transmitter stops and lock,
        // filters and smoothing are dropped rather than frozen.
        locked_ = false;
        consecutive_misses_ = 0;
        filter_count_ = 0;
        has_smoothed_ = false;
        last_.has_range = false;
        last_.locked = false;
    }
}

bool Fls::IsEnabled() const
{
    return enabled_;
}

// ===========================================================================
// Beam cone
//
// Stored in the sensor's own frame as coefficients on (axis, perp1, perp2):
// x is along the beam axis, y and z span the plane perpendicular to it. The
// centre ray is always index 0. Rings are phase shifted against each other so
// the samples do not line up in spokes.
// ===========================================================================

void Fls::BuildCone()
{
    cone_local_.clear();
    cone_off_axis_deg_.clear();

    cone_local_.push_back(Vec3{1.0, 0.0, 0.0});
    cone_off_axis_deg_.push_back(0.0);

    const int Rings = config_.beam_rings < 0 ? 0 : (config_.beam_rings > 6 ? 6 : config_.beam_rings);
    const int PerRing = config_.beam_samples_per_ring < 1
                            ? 1
                            : (config_.beam_samples_per_ring > 32 ? 32 : config_.beam_samples_per_ring);
    if (Rings > 0 && beam_width_deg_ > 0.0)
    {
        const double kPi = 3.14159265358979323846;
        const double HalfAngleRad = beam_width_deg_ * 0.5 * kPi / 180.0;
        for (int Ring = 1; Ring <= Rings; ++Ring)
        {
            const double RingAngle = HalfAngleRad * (static_cast<double>(Ring) / static_cast<double>(Rings));
            const double SinRing = std::sin(RingAngle);
            const double CosRing = std::cos(RingAngle);
            for (int I = 0; I < PerRing; ++I)
            {
                const double Phase = (2.0 * kPi) * (static_cast<double>(I) / static_cast<double>(PerRing)) +
                                     (kPi / static_cast<double>(PerRing)) * static_cast<double>(Ring - 1);
                cone_local_.push_back(
                    Normalized(Vec3{CosRing, SinRing * std::cos(Phase), SinRing * std::sin(Phase)}));
                cone_off_axis_deg_.push_back(RingAngle * 180.0 / kPi);
            }
        }
    }
    // Sized once here so a ping never allocates.
    rays_.assign(cone_local_.size(), Ray());
}

int Fls::RaysPerPing() const
{
    return static_cast<int>(cone_local_.size());
}

// ===========================================================================
// Mount transform
//
// Extrinsic composition in the PLATFORM's frame: pitch about `right` first,
// then yaw about `up`. FLS never learns the handedness or the axis names --
// it only ever combines the basis vectors the host handed it.
// ===========================================================================

void Fls::BeamFrame(const Pose &platform, Vec3 &out_origin_m, Vec3 &out_axis) const
{
    const Vec3 F = Normalized(platform.forward);
    const Vec3 R = Normalized(platform.right);
    const Vec3 U = Normalized(platform.up);

    out_origin_m = platform.position_m + F * config_.mount_forward_m + R * config_.mount_right_m +
                   U * config_.mount_up_m;

    const double kPi = 3.14159265358979323846;
    const double P = config_.mount_pitch_deg * kPi / 180.0;
    const double Y = config_.mount_yaw_deg * kPi / 180.0;
    const double Cp = std::cos(P);
    const double Sp = std::sin(P);
    const double Cy = std::cos(Y);
    const double Sy = std::sin(Y);
    out_axis = Normalized(F * (Cp * Cy) + R * (Cp * Sy) + U * Sp);
}

// ===========================================================================
// Random -- xorshift64* with Box-Muller, identical on every platform
// ===========================================================================

double Fls::NextUnit()
{
    rng_state_ ^= rng_state_ >> 12;
    rng_state_ ^= rng_state_ << 25;
    rng_state_ ^= rng_state_ >> 27;
    const std::uint64_t Bits = rng_state_ * 0x2545F4914F6CDD1Dull;
    return static_cast<double>(Bits >> 11) * (1.0 / 9007199254740992.0);
}

double Fls::NextGaussian()
{
    if (rng_has_spare_)
    {
        rng_has_spare_ = false;
        return rng_spare_;
    }
    double U = 0.0;
    double V = 0.0;
    double S = 0.0;
    do
    {
        U = NextUnit() * 2.0 - 1.0;
        V = NextUnit() * 2.0 - 1.0;
        S = U * U + V * V;
    } while (S >= 1.0 || S <= 1e-12);
    const double Factor = std::sqrt(-2.0 * std::log(S) / S);
    rng_spare_ = V * Factor;
    rng_has_spare_ = true;
    return U * Factor;
}

// ===========================================================================
// Acoustics
// ===========================================================================

double Fls::GateMin() const
{
    return config_.min_range_m > config_.blanking_m ? config_.min_range_m : config_.blanking_m;
}

double Fls::SnrDb(double range_m, double incidence_cos, double target_strength_db) const
{
    const double Clamped = incidence_cos > 1e-3 ? incidence_cos : 1e-3;
    const double Spreading = 20.0 * std::log10(range_m > 0.1 ? range_m : 0.1);
    const double Absorption = config_.absorption_db_per_km * range_m / 1000.0;
    const double TwoWayLoss = 2.0 * (Spreading + Absorption);
    const double Strength = target_strength_db + 20.0 * std::log10(Clamped);
    return config_.source_level_db - TwoWayLoss + Strength - config_.noise_level_db;
}

int Fls::EffectivePingsPerSecond() const
{
    const RangeModeInfo Info = GetRangeModeInfo(active_range_);
    if (config_.pings_per_second > 0)
    {
        return config_.pings_per_second < Info.max_pings_per_second ? config_.pings_per_second
                                                                    : Info.max_pings_per_second;
    }
    // AUTO: half the maximum the mode allows, at least once per second.
    const int Half = Info.max_pings_per_second / 2;
    return Half > 1 ? Half : 1;
}

RangeMode Fls::ActiveRangeMode() const
{
    return active_range_;
}

double Fls::RangeScaleMeters() const
{
    const double ModeMax = GetRangeModeInfo(active_range_).max_meters;
    return ModeMax < config_.max_range_m ? ModeMax : config_.max_range_m;
}

void Fls::UpdateAutoRange(double candidate_range_m)
{
    if (config_.range_mode != RangeMode::Auto)
    {
        active_range_ = config_.range_mode;
        return;
    }
    // Smallest mode that still contains the target. The two thresholds differ
    // (0.95 up, 0.45 down) so a target sitting on a boundary does not make the
    // device oscillate between two range scales.
    const int Highest = static_cast<int>(RangeMode::VeryLong);
    int Mode = static_cast<int>(active_range_);
    if (Mode > Highest)
    {
        Mode = Highest;
    }
    if (candidate_range_m > 0.0)
    {
        while (Mode < Highest &&
               candidate_range_m > GetRangeModeInfo(static_cast<RangeMode>(Mode)).max_meters * 0.95)
        {
            ++Mode;
        }
        while (Mode > 0 &&
               candidate_range_m < GetRangeModeInfo(static_cast<RangeMode>(Mode - 1)).max_meters * 0.45)
        {
            --Mode;
        }
    }
    active_range_ = static_cast<RangeMode>(Mode);
}

double Fls::ApplyFilters(double range_m)
{
    double Value = range_m;
    if (config_.depth_filter_samples >= 2)
    {
        const int Window =
            config_.depth_filter_samples > kMaxFilter ? kMaxFilter : config_.depth_filter_samples;
        filter_history_[filter_cursor_ % kMaxFilter] = range_m;
        ++filter_cursor_;
        if (filter_count_ < Window)
        {
            ++filter_count_;
        }
        const int Used = filter_count_ < Window ? filter_count_ : Window;
        double Sum = 0.0;
        for (int I = 0; I < Used; ++I)
        {
            const int Index = (filter_cursor_ - 1 - I + 2 * kMaxFilter) % kMaxFilter;
            Sum += filter_history_[Index];
        }
        Value = Sum / static_cast<double>(Used);
    }
    if (config_.smoothing_filter >= 2)
    {
        const double Alpha = 1.0 / static_cast<double>(config_.smoothing_filter);
        if (!has_smoothed_)
        {
            smoothed_ = Value;
            has_smoothed_ = true;
        }
        else
        {
            smoothed_ += (Value - smoothed_) * Alpha;
        }
        Value = smoothed_;
    }
    return Value;
}

// ===========================================================================
// One ping
// ===========================================================================

bool Fls::Fire(const Pose &platform, const IWorld &world, Output &out)
{
    ++pings_;
    out = Output();
    out.pinged = true;
    out.time_s = clock_s_;
    out.ping_index = pings_;

    Vec3 Origin;
    Vec3 Axis;
    BeamFrame(platform, Origin, Axis);
    out.beam_origin_m = Origin;
    out.beam_axis = Axis;

    // A beam width that could not be derived is a configuration error, not a
    // reason to invent a default cone.
    if (!(beam_width_deg_ > 0.0))
    {
        out.reject_reason = "beam width not configured (frequency_khz must be 170 or 200)";
        return true;
    }
    if (Length(Axis) < 0.5)
    {
        out.reject_reason = "platform basis is degenerate (forward/right/up not usable)";
        return true;
    }

    // Perpendicular pair of the sensor frame, carried through the same
    // extrinsic pitch-then-yaw composition as the axis.
    const Vec3 F = Normalized(platform.forward);
    const Vec3 R = Normalized(platform.right);
    const Vec3 U = Normalized(platform.up);
    const double kPi = 3.14159265358979323846;
    const double P = config_.mount_pitch_deg * kPi / 180.0;
    const double Y = config_.mount_yaw_deg * kPi / 180.0;
    const double Cp = std::cos(P);
    const double Sp = std::sin(P);
    const double Cy = std::cos(Y);
    const double Sy = std::sin(Y);
    const Vec3 Perp1 = Normalized(F * (-Sy) + R * Cy);
    const Vec3 Perp2 = Normalized(F * (-Sp * Cy) + R * (-Sp * Sy) + U * Cp);

    // Search out to the hardware limit, not to the current range scale: in
    // Auto mode the device has to see the far target before it can select the
    // range that contains it.
    const double Low = GateMin();
    const double SearchHigh = config_.max_range_m;

    bool Found = false;
    double BestRange = 0.0;
    double BestSnr = 0.0;
    double BestOffAxis = 0.0;
    const char *Reason = "no target in beam";
    int Hits = 0;

    for (std::size_t I = 0; I < cone_local_.size(); ++I)
    {
        const Vec3 &C = cone_local_[I];
        const Vec3 Direction = Normalized(Axis * C.x + Perp1 * C.y + Perp2 * C.z);
        Ray &R0 = rays_[I];
        R0.direction = Direction;
        R0.off_axis_deg = cone_off_axis_deg_[I];
        R0.hit = false;
        R0.range_m = 0.0;

        const RayHit Hit = world.CastRay(Origin, Direction, SearchHigh);
        if (!Hit.hit || !std::isfinite(Hit.range_m))
        {
            continue;
        }
        ++Hits;
        R0.hit = true;
        R0.range_m = Hit.range_m;

        if (Hit.range_m < Low)
        {
            Reason = "inside blanking / minimum range";
            continue;
        }
        if (Hit.range_m > SearchHigh)
        {
            Reason = "beyond maximum range";
            continue;
        }
        const double IncidenceCos = Hit.incidence_cos;
        if (IncidenceCos < config_.min_incidence_cos)
        {
            Reason = "incidence too shallow";
            continue;
        }
        double Snr = 0.0;
        if (config_.use_sonar_equation)
        {
            const double Strength = Hit.target_strength_db <= -999.0 ? config_.target_strength_db
                                                                     : Hit.target_strength_db;
            Snr = SnrDb(Hit.range_m, IncidenceCos, Strength);
            if (Snr < config_.detection_threshold_db)
            {
                Reason = "echo below detection threshold";
                continue;
            }
        }
        // FIRST return: the shortest range that produces a detectable echo.
        if (!Found || Hit.range_m < BestRange)
        {
            Found = true;
            BestRange = Hit.range_m;
            BestSnr = Snr;
            BestOffAxis = cone_off_axis_deg_[I];
        }
    }
    out.rays_cast = static_cast<int>(cone_local_.size());
    out.rays_hit = Hits;
    if (!Found && Hits > 0 && Reason == std::string("no target in beam"))
    {
        Reason = "no echo above threshold";
    }

    if (Found && config_.dropout_probability > 0.0 && NextUnit() < config_.dropout_probability)
    {
        Found = false;
        Reason = "random dropout";
    }

    UpdateAutoRange(Found ? BestRange : 0.0);
    out.active_range = active_range_;
    out.range_scale_m = RangeScaleMeters();

    // A fixed range mode truncates the sounding exactly as the hardware does;
    // Auto has just widened itself to fit, so this only bites when the
    // operator pinned a mode that is too short.
    const double High = out.range_scale_m < config_.max_range_m ? out.range_scale_m : config_.max_range_m;
    if (Found && BestRange > High)
    {
        Found = false;
        Reason = "beyond range scale in use";
    }

    if (!Found)
    {
        ++misses_;
        ++consecutive_misses_;
        if (consecutive_misses_ >= config_.misses_to_lose_lock)
        {
            locked_ = false;
            filter_count_ = 0;
            has_smoothed_ = false;
        }
        out.has_range = false;
        out.locked = locked_;
        out.reject_reason = Reason;
        return true;
    }

    consecutive_misses_ = 0;
    locked_ = true;
    ++locks_;

    const double Error =
        BestRange * config_.scale_error_fraction + NextGaussian() * config_.range_noise_sigma_m;
    double Reported = ApplyFilters(BestRange + Error);
    if (Reported < Low)
    {
        Reported = Low;
    }
    if (Reported > High)
    {
        Reported = High;
    }
    // 1 cm resolution on the wire.
    Reported =
        std::floor(Reported / spec::kRangeResolutionMeters + 0.5) * spec::kRangeResolutionMeters;

    out.has_range = true;
    out.locked = true;
    out.range_m = Reported;
    out.true_range_m = BestRange;
    out.snr_db = BestSnr;
    out.off_axis_deg = BestOffAxis;
    out.reject_reason = "";
    return true;
}

// ===========================================================================
// Update
// ===========================================================================

Output Fls::Update(double dt_s, const Pose &platform, const IWorld &world)
{
    last_.pinged = false;
    if (!(dt_s > 0.0) || !std::isfinite(dt_s))
    {
        return last_;
    }
    clock_s_ += dt_s;
    last_.time_s = clock_s_;

    if (!enabled_)
    {
        ping_accumulator_s_ = 0.0;
        last_.has_range = false;
        last_.locked = false;
        last_.reject_reason = "disabled";
        return last_;
    }

    const double Period = 1.0 / static_cast<double>(EffectivePingsPerSecond());
    ping_accumulator_s_ += dt_s;
    if (ping_accumulator_s_ + 1e-9 < Period)
    {
        return last_;
    }
    // One ping per call at most: a long frame must not fire a burst of pings
    // the transducer could never have transmitted.
    ping_accumulator_s_ = 0.0;

    Output Fresh;
    Fire(platform, world, Fresh);
    last_ = Fresh;
    return last_;
}

// ===========================================================================
// Accessors
// ===========================================================================

const Output &Fls::LastOutput() const
{
    return last_;
}
bool Fls::HasLock() const
{
    return locked_;
}
double Fls::BeamWidthDeg() const
{
    return beam_width_deg_;
}
double Fls::Time() const
{
    return clock_s_;
}
long long Fls::PingCount() const
{
    return pings_;
}
long long Fls::LockCount() const
{
    return locks_;
}
long long Fls::MissCount() const
{
    return misses_;
}
const std::vector<Ray> &Fls::LastRays() const
{
    return rays_;
}

} // namespace airmar
