// FLS.hpp -- Forward Looking Sonar / altimeter, single beam.
//
// Simulation model of the AIRMAR 200 m Mini Altimeter Kit, written against the
// data sheet `MiniAlt_rI 07/27/23`. Drop FLS.hpp + FLS.cpp into any project
// and you have the sensor; nothing else is required.
//
// ===========================================================================
// WHAT THIS IS
// ===========================================================================
// This is NOT "a ray cast converted to a number". A real altimeter frequently
// returns no number at all, and the parts that decide whether a number exists
// are exactly the parts that make an autonomy stack fail in the water:
//
//   * a finite beam cone (14 deg at 200 kHz), sampled and reduced to the
//     FIRST return, not the nearest geometry
//   * an active sonar equation, so a weak or steeply angled target produces
//     no echo even though the geometry is well inside maximum range
//   * the 0.4 m minimum range and the operator blanking distance
//   * the four sounding range modes, their ping rates, and the automatic
//     mode selection with hysteresis
//   * bottom lock acquisition and loss after consecutive misses
//   * 1 cm quantisation, the data sheet's +0.54 % scale error, noise, dropout
//
// ===========================================================================
// HOW TO INTEGRATE (the whole job is one function)
// ===========================================================================
// 1. Implement `fls::IWorld::CastRay` with your engine's ray cast:
//
//        class MyWorld : public fls::IWorld {
//        public:
//            fls::RayHit CastRay(const fls::Vec3& origin_m,
//                                const fls::Vec3& dir,
//                                double max_range_m) const override
//            {
//                fls::RayHit out;
//                // ... your engine's trace here ...
//                out.hit           = did_hit;
//                out.range_m       = distance_in_METRES;
//                out.incidence_cos = fabs(dot(dir, surface_normal));
//                return out;
//            }
//        };
//
// 2. Fill in `fls::Config` -- every static transform and device setting is a
//    plain field you type in by hand. See the `Config` comments.
//
// 3. Call `Update` once per frame with your platform's pose:
//
//        fls::Output o = sonar.Update(dt_seconds, pose, world);
//        if (o.has_range) { /* o.range_m is what the device reports */ }
//
// That is the entire contract. FLS never calls your engine except through
// `CastRay`, never allocates after the first ping, and holds no global state.
//
// ===========================================================================
// UNITS AND AXES
// ===========================================================================
// All lengths are METRES, all angles DEGREES, all times SECONDS.
//
// FLS imposes NO axis convention and does not know whether your engine is
// left or right handed. You supply your platform's own basis vectors in
// `Pose` (forward / right / up) and FLS composes the mount transform from
// them. In Unreal that is GetActorForwardVector() / RightVector() / UpVector()
// with the location scaled by 0.01; in a ROS/FLU frame it is x / y / z.
//
// ===========================================================================
// DETERMINISM
// ===========================================================================
// The random generator is a hand written xorshift64* with a Box-Muller
// transform rather than <random>, because the standard library distributions
// are not specified to produce identical sequences across implementations, and
// a simulated sensor that behaves differently on Linux and Windows is not a
// sensor anyone can test against. Same seed and same inputs give the same
// bytes on every platform and compiler.
//
// ===========================================================================
// DEPENDENCIES
// ===========================================================================
// C++17 standard library only: <cstdint>, <string>, <vector>, <cmath>.
// No engine headers, no ROS, no OS calls, no threads, no I/O, no exceptions.
//
// SPDX-License-Identifier: MIT

#ifndef FLS_HPP
#define FLS_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace fls
{

// ===========================================================================
// Geometry
// ===========================================================================

/** A point or direction. Lengths in metres. No axis convention implied. */
struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Vec3 operator+(const Vec3 &a, const Vec3 &b);
Vec3 operator-(const Vec3 &a, const Vec3 &b);
Vec3 operator*(const Vec3 &v, double s);
double Dot(const Vec3 &a, const Vec3 &b);
double Length(const Vec3 &v);
/** Unit vector. Returns {0,0,0} if the input is degenerate -- check it. */
Vec3 Normalized(const Vec3 &v);

/**
 * The carrying platform's pose, in your own frame and handedness.
 *
 * Supply the three basis vectors your engine already gives you. They should
 * be unit length and mutually perpendicular; FLS normalises them but does not
 * re-orthogonalise, so a skewed basis gives a skewed cone.
 */
struct Pose
{
    Vec3 position_m;                  // platform origin, world, metres
    Vec3 forward{1.0, 0.0, 0.0};
    Vec3 right{0.0, 1.0, 0.0};
    Vec3 up{0.0, 0.0, 1.0};
};

// ===========================================================================
// The one thing the host must implement
// ===========================================================================

/** Sentinel for RayHit::target_strength_db: "use Config::target_strength_db". */
constexpr double kUseDefaultTargetStrength = -1000.0;

/** Result of one ray cast, filled in by the host. */
struct RayHit
{
    /** False means the ray reached max_range_m without hitting anything. */
    bool hit = false;
    /** Distance from the ray origin to the surface, METRES. */
    double range_m = 0.0;
    /**
     * |dot(ray direction, surface normal)|, 0..1.
     *
     * 1.0 is a head-on hit, 0.0 is a grazing hit. This is what makes a steep
     * slope invisible to a real altimeter, so it is worth filling in properly
     * rather than leaving at 1.0. If your trace gives you a normal, use it.
     */
    double incidence_cos = 1.0;
    /**
     * Per-material target strength in dB. Leave at the sentinel to use the
     * configured default. Mud, weed and fish are far weaker reflectors than
     * rock or steel: something like -35 dB versus the -15 dB default.
     */
    double target_strength_db = kUseDefaultTargetStrength;
};

/**
 * The host's ray cast. The ONLY call FLS makes into your engine.
 *
 * `direction` is a unit vector in your world frame. Return the FIRST
 * intersection within `max_range_m`. Must not be blocked by the carrying
 * platform itself or by water volumes -- exclude those in your trace, or the
 * sensor will report the inside of its own hull.
 *
 * Called `rays_per_ping()` times per ping (13 by default), and it must be
 * safe to call from wherever you call `Update`.
 */
class IWorld
{
  public:
    virtual ~IWorld() = default;
    virtual RayHit CastRay(const Vec3 &origin_m, const Vec3 &direction, double max_range_m) const = 0;
};

// ===========================================================================
// Device constants, straight from the data sheet
// ===========================================================================
namespace spec
{
/** Data sheet: 0.4 m minimum, "limited in manual mode". */
constexpr double kMinRangeMeters = 0.4;
/** Data sheet: 200 m Mini Altimeter Kit. */
constexpr double kMaxRangeMeters = 200.0;
/** Data sheet: 1 cm resolution on the wire. */
constexpr double kRangeResolutionMeters = 0.01;
/**
 * Scale error, from the data sheet's own accuracy table rather than rounded
 * by hand. "ACCURACY (Based on tank testing)": 3.05 m reads 3.07 m,
 * 4.57 m reads 4.59 m, 5.79 m reads 5.82 m -- always high, by +0.656 %,
 * +0.438 % and +0.518 %, mean +0.537 %.
 */
constexpr double kScaleErrorFraction = 0.0054;
/** -3 dB beam angle of the 200 kHz-A housing. */
constexpr double kBeamWidthDeg200kHz = 14.0;
/** -3 dB beam angle of the 170 kHz-C housing. */
constexpr double kBeamWidthDeg170kHz = 18.0;
constexpr double kDefaultFrequencyKhz = 200.0;
constexpr double kDefaultSpeedOfSoundMps = 1500.0;
constexpr double kMetersPerFoot = 0.3048;
/** Transducer body, data sheet DIMENSIONS: 1.0 in diameter, 2.30 in long. */
constexpr double kTransducerDiameterMeters = 0.0254;
constexpr double kTransducerLengthMeters = 0.0584;
} // namespace spec

/**
 * Sounding range modes. The data sheet lists them in feet; the metre values
 * here are the exact conversions.
 */
enum class RangeMode
{
    Short = 0,    //  56 ft =  17.07 m
    Medium = 1,   // 225 ft =  68.58 m
    Long = 2,     // 450 ft = 137.16 m
    VeryLong = 3, // 675 ft = 205.74 m
    Auto = 4      // pick the smallest mode that contains the target
};

struct RangeModeInfo
{
    double max_meters = 0.0;
    int max_pings_per_second = 1;
    const char *name = "?";
};

RangeModeInfo GetRangeModeInfo(RangeMode mode);
const char *ToString(RangeMode mode);

/**
 * -3 dB beam width for a transducer housing frequency.
 *
 * Returns 0.0 for anything the data sheet does not list. That is deliberate:
 * a caller that silently fell back to a default would be reporting a beam
 * width the hardware does not have. Check the result and reject it.
 */
double BeamWidthDegForFrequencyKhz(double frequency_khz);

// ===========================================================================
// Configuration -- every field is meant to be typed in by hand
// ===========================================================================
struct Config
{
    // -----------------------------------------------------------------------
    // STATIC MOUNT TRANSFORM
    //
    // Where the transducer sits on your vehicle and where it points, given in
    // the PLATFORM's own frame using the basis vectors you pass in `Pose`.
    // Measure these off your model once and type them in.
    // -----------------------------------------------------------------------
    /** Along Pose::forward, metres. Example: 2.12 for a nose-mounted unit. */
    double mount_forward_m = 0.0;
    /** Along Pose::right, metres. */
    double mount_right_m = 0.0;
    /** Along Pose::up, metres. */
    double mount_up_m = 0.0;
    /** Rotate the beam axis about Pose::up. + turns toward Pose::right. */
    double mount_yaw_deg = 0.0;
    /** Rotate the beam axis about Pose::right. + tilts toward Pose::up.
     *  0 looks straight ahead; -90 is a downward looking altimeter. */
    double mount_pitch_deg = 0.0;

    // -----------------------------------------------------------------------
    // TRANSDUCER
    // -----------------------------------------------------------------------
    /** 170.0 or 200.0. Any other value is rejected by Validate(). */
    double frequency_khz = spec::kDefaultFrequencyKhz;
    /** -3 dB beam width. 0 means "derive it from frequency_khz" (recommended). */
    double beam_width_deg = 0.0;

    // -----------------------------------------------------------------------
    // BEAM SAMPLING -- how finely the cone is searched
    //
    // The device is SINGLE BEAM: one ping yields one distance and carries no
    // bearing information. These rays are an implementation detail of finding
    // the first return inside the cone, not something the hardware reports.
    // Defaults give 1 + 2*6 = 13 rays. More rays cost more ray casts.
    // -----------------------------------------------------------------------
    int beam_rings = 2;            // 0..6; 0 means centre ray only
    int beam_samples_per_ring = 6; // 1..32

    // -----------------------------------------------------------------------
    // RANGE GATE
    // -----------------------------------------------------------------------
    double min_range_m = spec::kMinRangeMeters;
    double max_range_m = spec::kMaxRangeMeters;
    /** Operator blanking (the real unit's DBLANK): reject anything closer.
     *  0..15 m. Use it to ignore a thruster wake or a bow wave. */
    double blanking_m = 0.0;
    RangeMode range_mode = RangeMode::Auto;

    // -----------------------------------------------------------------------
    // ACOUSTICS
    //
    // With the sonar equation on, detection requires
    //   SNR = SL - 2*(20 log10 r + alpha*r/1000) + TS + 20 log10(cos) - NL >= DT
    // which is why a soft or steeply angled target drops out well before the
    // geometric range limit does. Turn it off for a pure geometric range.
    // -----------------------------------------------------------------------
    bool use_sonar_equation = true;
    double source_level_db = 200.0;       // tuning parameter, 100 W transmitter
    double noise_level_db = 40.0;         // in-band ambient noise
    double detection_threshold_db = 10.0;
    /** Sea water at 200 kHz is roughly 55 dB/km; fresh water far less. */
    double absorption_db_per_km = 55.0;
    double target_strength_db = -15.0;
    /** Below this incidence cosine the echo never comes back at all. */
    double min_incidence_cos = 0.10;

    // -----------------------------------------------------------------------
    // ERRORS
    // -----------------------------------------------------------------------
    double scale_error_fraction = spec::kScaleErrorFraction;
    double range_noise_sigma_m = 0.02;
    /** Fraction of otherwise good pings that return nothing. 0..1. */
    double dropout_probability = 0.02;
    std::uint64_t seed = 0x9E3779B97F4A7C15ull;

    // -----------------------------------------------------------------------
    // OUTPUT FILTERS (the real unit's DFILTER and SFILTER options)
    // -----------------------------------------------------------------------
    /** Moving average over the last N pings. 0 (off), 2 or 4. */
    int depth_filter_samples = 0;
    /** Exponential smoothing with alpha = 1/N. 0 (off), 2, 4 or 8. */
    int smoothing_filter = 0;

    // -----------------------------------------------------------------------
    // TIMING AND LOCK
    // -----------------------------------------------------------------------
    /** Ping rate. 0 selects AUTO, derived from the active range mode. */
    int pings_per_second = 0;
    double speed_of_sound_mps = spec::kDefaultSpeedOfSoundMps;
    /** Consecutive misses before bottom lock is dropped. */
    int misses_to_lose_lock = 3;

    /**
     * Returns an empty string when the configuration is usable, otherwise a
     * human readable reason. Call it at startup: a sensor configured with a
     * frequency the hardware does not have should fail loudly, not quietly
     * report a beam width nobody specified.
     */
    std::string Validate() const;
};

// ===========================================================================
// Output
// ===========================================================================

/** One sampled ray, exposed so the host can draw the cone if it wants to. */
struct Ray
{
    Vec3 direction;            // unit vector, world frame
    double off_axis_deg = 0.0; // angle from the beam axis
    bool hit = false;
    double range_m = 0.0;
};

struct Output
{
    /** True when a ping completed in this Update. Range fields are only
     *  meaningful on a ping; between pings the previous output is unchanged. */
    bool pinged = false;
    /** True when the device has a usable return, i.e. bottom/target lock. */
    bool has_range = false;

    /** What the device reports: filtered, biased, noisy, quantised to 1 cm. */
    double range_m = 0.0;
    /** The geometric distance before the error model. FOR PLOTS AND TESTS
     *  ONLY -- an autonomy stack must never read this. */
    double true_range_m = 0.0;

    double snr_db = 0.0;
    /** Maximum range scale currently in use (what DPT field 3 reports). */
    double range_scale_m = 0.0;
    /** Off-axis angle of the ray that produced the return. */
    double off_axis_deg = 0.0;
    RangeMode active_range = RangeMode::Medium;
    bool locked = false;

    long long ping_index = 0;
    double time_s = 0.0;
    /** Why there is no range. Empty when has_range is true. */
    const char *reject_reason = "";

    /** Beam origin and axis in world space, for drawing. */
    Vec3 beam_origin_m;
    Vec3 beam_axis{1.0, 0.0, 0.0};
    int rays_cast = 0;
    int rays_hit = 0;
};

// ===========================================================================
// The sensor
// ===========================================================================
class Fls
{
  public:
    Fls();
    explicit Fls(const Config &config);

    /** Replaces the configuration and resets derived state. Returns the same
     *  string as Config::Validate(); on a non-empty return the previous
     *  configuration is kept and nothing changes. */
    std::string SetConfig(const Config &config);
    const Config &GetConfig() const;

    /** Clears clock, lock, filters and reseeds the generator. */
    void Reset();

    /** A disabled sensor stops transmitting and loses lock, like PING OFF. */
    void SetEnabled(bool enabled);
    bool IsEnabled() const;

    /**
     * Advances the model by dt_s and fires at most one ping.
     *
     * At most one ping per call even if dt_s spans several ping periods: a
     * long frame must not fire a burst the transducer could never have
     * transmitted. Returns the current output; check `pinged` to tell a fresh
     * measurement from a repeat of the last one.
     */
    Output Update(double dt_s, const Pose &platform, const IWorld &world);

    const Output &LastOutput() const;
    bool HasLock() const;
    /** Effective -3 dB beam width, after the frequency derivation. */
    double BeamWidthDeg() const;
    int RaysPerPing() const;
    int EffectivePingsPerSecond() const;
    RangeMode ActiveRangeMode() const;
    double RangeScaleMeters() const;
    double Time() const;
    long long PingCount() const;
    long long LockCount() const;
    long long MissCount() const;

    /** The rays of the most recent ping, in world space. Empty before the
     *  first ping. Index 0 is always the beam axis. */
    const std::vector<Ray> &LastRays() const;

    /** Beam origin and axis for the given pose, without firing a ping.
     *  Useful for drawing the sensor before it has pinged. */
    void BeamFrame(const Pose &platform, Vec3 &out_origin_m, Vec3 &out_axis) const;

  private:
    // --- implementation detail below this line -----------------------------
    void ApplyConfig();
    void BuildCone();
    bool Fire(const Pose &platform, const IWorld &world, Output &out);
    double GateMin() const;
    double SnrDb(double range_m, double incidence_cos, double target_strength_db) const;
    void UpdateAutoRange(double candidate_range_m);
    double ApplyFilters(double range_m);
    double NextUnit();
    double NextGaussian();

    static constexpr int kMaxFilter = 8;

    Config config_;
    double beam_width_deg_ = spec::kBeamWidthDeg200kHz;

    // cone, in the SENSOR's own frame: +x is the beam axis
    std::vector<Vec3> cone_local_;
    std::vector<double> cone_off_axis_deg_;
    std::vector<Ray> rays_;

    bool enabled_ = true;
    double clock_s_ = 0.0;
    double ping_accumulator_s_ = 0.0;
    long long pings_ = 0;
    long long locks_ = 0;
    long long misses_ = 0;
    int consecutive_misses_ = 0;
    bool locked_ = false;
    RangeMode active_range_ = RangeMode::Medium;

    double filter_history_[kMaxFilter] = {0.0};
    int filter_count_ = 0;
    int filter_cursor_ = 0;
    double smoothed_ = 0.0;
    bool has_smoothed_ = false;

    std::uint64_t rng_state_ = 0x9E3779B97F4A7C15ull;
    bool rng_has_spare_ = false;
    double rng_spare_ = 0.0;

    Output last_;
};

} // namespace fls

#endif // FLS_HPP
