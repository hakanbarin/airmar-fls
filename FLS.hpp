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
// INTEGRATION -- READ THIS SECTION AND NOTHING ELSE IS NEEDED
// ===========================================================================
//
// These instructions are written to be executed literally. Every file below is
// complete and compiles as given; nothing is elided and there are no
// placeholders except the ones marked <LIKE THIS>. If you are an AI agent
// integrating this sensor, do the steps in order and do not improvise.
//
// ---------------------------------------------------------------------------
// STEP 1 -- copy the two files
// ---------------------------------------------------------------------------
// Copy FLS.hpp and FLS.cpp into the module that will own the sensor, for
// example <YourProject>/Source/<YourModule>/. No build file change is needed:
// these two files include only <cstdint> <string> <vector> <cmath>.
//
// ---------------------------------------------------------------------------
// STEP 2 -- implement airmar::IWorld (the only call this model makes out)
// ---------------------------------------------------------------------------
// Create a new file FlsUnrealWorld.h next to them, with EXACTLY this content.
// It needs no .cpp.
//
// === BEGIN FILE FlsUnrealWorld.h ===
// #pragma once
//
// #include "CoreMinimal.h"
// #include "Engine/World.h"
// #include "CollisionQueryParams.h"
// #include "EngineUtils.h"
// #include "GameFramework/Actor.h"
//
// #include "FLS.hpp"
//
// /**
//  * Adapts Unreal's line trace to airmar::IWorld.
//  *
//  * UNREAL IS CENTIMETRES, THIS MODEL IS METRES. Every conversion is in this
//  * file and nowhere else; do not scale anything outside it.
//  *
//  * Construct it fresh each tick (it is a cheap stack object) or keep one and
//  * call RefreshIgnoreList() when the level changes.
//  */
// class FFlsUnrealWorld : public airmar::IWorld
// {
// public:
//     FFlsUnrealWorld(UWorld *InWorld, const AActor *InPlatform,
//                     ECollisionChannel InChannel = ECC_WorldStatic,
//                     bool bIgnoreWaterBodies = true)
//         : World(InWorld), Channel(InChannel)
//     {
//         Params.bTraceComplex = false;
//
//         // PITFALL 1 -- the sensor must not see its own vehicle. Ignoring
//         // only the owner is NOT enough when the sensor is its own actor
//         // attached to the vehicle: walk the whole attachment chain.
//         if (InPlatform)
//         {
//             Params.AddIgnoredActor(InPlatform);
//             const AActor *Root = InPlatform;
//             while (Root->GetAttachParentActor())
//             {
//                 Root = Root->GetAttachParentActor();
//                 Params.AddIgnoredActor(Root);
//             }
//             TArray<AActor *> Attached;
//             Root->GetAttachedActors(Attached, true, true);
//             for (const AActor *A : Attached)
//             {
//                 if (A) { Params.AddIgnoredActor(A); }
//             }
//         }
//
//         // PITFALL 2 -- WATER BLOCKS TRACES. This was MEASURED: with a water
//         // body in the level, every ray hit at 0.000 m and the sensor
//         // reported the inside of the water volume. The sensor is submerged,
//         // so water must be excluded from the query.
//         //
//         // Matched by class name on purpose, so this file compiles whether or
//         // not the Water plugin is enabled in the target project.
//         if (bIgnoreWaterBodies && World)
//         {
//             for (TActorIterator<AActor> It(World); It; ++It)
//             {
//                 if (It->GetClass()->GetName().Contains(TEXT("WaterBody")))
//                 {
//                     Params.AddIgnoredActor(*It);
//                 }
//             }
//         }
//     }
//
//     /** Add anything else the beam must pass through (nets, markers, FX). */
//     void IgnoreActor(const AActor *A) { if (A) { Params.AddIgnoredActor(A); } }
//
//     airmar::RayHit CastRay(const airmar::Vec3 &OriginM, const airmar::Vec3 &Dir,
//                            double MaxRangeM) const override
//     {
//         airmar::RayHit Out;
//         if (!World) { return Out; }
//
//         const FVector Start(OriginM.x * 100.0, OriginM.y * 100.0, OriginM.z * 100.0);
//         const FVector D(Dir.x, Dir.y, Dir.z);
//         const FVector End = Start + D * (MaxRangeM * 100.0);
//
//         FHitResult Hit;
//         if (!World->LineTraceSingleByChannel(Hit, Start, End, Channel, Params))
//         {
//             return Out;   // nothing in range: a real miss, report it as one
//         }
//
//         Out.hit     = true;
//         Out.range_m = Hit.Distance * 0.01;   // cm -> m
//
//         // PITFALL 3 -- fill this in. Leaving it at 1.0 makes every surface a
//         // perfect reflector and removes one of the two effects that make the
//         // model behave like hardware.
//         Out.incidence_cos = FMath::Abs(FVector::DotProduct(D, Hit.ImpactNormal));
//
//         // OPTIONAL -- mud, weed and fish reflect far more weakly than rock.
//         // Tag such actors "SoftTarget" in the level to get that behaviour.
//         if (const AActor *A = Hit.GetActor())
//         {
//             if (A->ActorHasTag(TEXT("SoftTarget"))) { Out.target_strength_db = -35.0; }
//         }
//         return Out;
//     }
//
// private:
//     UWorld *World = nullptr;
//     ECollisionChannel Channel = ECC_WorldStatic;
//     FCollisionQueryParams Params;
// };
// === END FILE FlsUnrealWorld.h ===
//
// ---------------------------------------------------------------------------
// STEP 3 -- drive it from a component
// ---------------------------------------------------------------------------
// Component or actor is YOUR choice; this model has no opinion and no
// dependency either way. The component below is complete and header-only:
// drop it in, attach it to your vehicle, set the mount fields in the details
// panel. If your project prefers an actor, the same three calls move into
// AActor::Tick unchanged.
//
// === BEGIN FILE FlsSonarComponent.h ===
// #pragma once
//
// #include "CoreMinimal.h"
// #include "Components/ActorComponent.h"
// #include "DrawDebugHelpers.h"
//
// #include "FLS.hpp"
// #include "FlsUnrealWorld.h"
//
// #include "FlsSonarComponent.generated.h"
//
// UCLASS(ClassGroup = (Sensors), meta = (BlueprintSpawnableComponent))
// class UFlsSonarComponent : public UActorComponent
// {
//     GENERATED_BODY()
//
// public:
//     UFlsSonarComponent()
//     {
//         PrimaryComponentTick.bCanEverTick = true;
//     }
//
//     // ---- STATIC MOUNT, typed in by hand, METRES, in the OWNER's frame ----
//     // Measure these off your vehicle model once. "Forward/Right/Up" mean the
//     // owning actor's own vectors, so a turning vehicle carries the sensor
//     // with it automatically.
//     UPROPERTY(EditAnywhere, Category = "FLS|Mount")
//     float MountForwardM = 0.f;    // e.g. 2.12 for a nose-mounted unit
//     UPROPERTY(EditAnywhere, Category = "FLS|Mount")
//     float MountRightM = 0.f;
//     UPROPERTY(EditAnywhere, Category = "FLS|Mount")
//     float MountUpM = 0.f;
//     UPROPERTY(EditAnywhere, Category = "FLS|Mount")
//     float MountYawDeg = 0.f;      // + turns the beam toward Right
//     UPROPERTY(EditAnywhere, Category = "FLS|Mount")
//     float MountPitchDeg = 0.f;    // 0 = straight ahead, -90 = looking down
//
//     // ---- DEVICE ----
//     UPROPERTY(EditAnywhere, Category = "FLS|Device")
//     float FrequencyKhz = 200.f;   // 200 -> 14 deg beam, 170 -> 18 deg beam
//     UPROPERTY(EditAnywhere, Category = "FLS|Device")
//     float BlankingM = 0.f;        // operator dead zone, 0..15 m
//     UPROPERTY(EditAnywhere, Category = "FLS|Device")
//     int32 PingsPerSecond = 0;     // 0 = auto from the range mode
//
//     // ---- TRACE ----
//     UPROPERTY(EditAnywhere, Category = "FLS|Trace")
//     TEnumAsByte<ECollisionChannel> TraceChannel = ECC_WorldStatic;
//     UPROPERTY(EditAnywhere, Category = "FLS|Trace")
//     bool bIgnoreWaterBodies = true;
//
//     UPROPERTY(EditAnywhere, Category = "FLS|Debug")
//     bool bDrawBeam = true;
//
//     // ---- OUTPUT, readable from Blueprint ----
//     UPROPERTY(BlueprintReadOnly, Category = "FLS")
//     bool bHasRange = false;
//     UPROPERTY(BlueprintReadOnly, Category = "FLS")
//     float RangeMeters = 0.f;
//
//     virtual void BeginPlay() override
//     {
//         Super::BeginPlay();
//
//         airmar::Config C;
//         C.mount_forward_m = MountForwardM;
//         C.mount_right_m   = MountRightM;
//         C.mount_up_m      = MountUpM;
//         C.mount_yaw_deg   = MountYawDeg;
//         C.mount_pitch_deg = MountPitchDeg;
//         C.frequency_khz   = FrequencyKhz;
//         C.blanking_m      = BlankingM;
//         C.pings_per_second = PingsPerSecond;
//
//         // FAIL LOUDLY. Do not swallow this: a sensor that silently fell back
//         // to defaults would report a beam width nobody specified.
//         const std::string Err = Sonar.SetConfig(C);
//         if (!Err.empty())
//         {
//             UE_LOG(LogTemp, Error, TEXT("[FLS] refusing to run: %s"),
//                    *FString(Err.c_str()));
//             SetComponentTickEnabled(false);
//             return;
//         }
//         UE_LOG(LogTemp, Log, TEXT("[FLS] %.0f kHz, beam %.1f deg, %d rays, %d ping/s"),
//                FrequencyKhz, Sonar.BeamWidthDeg(), Sonar.RaysPerPing(),
//                Sonar.EffectivePingsPerSecond());
//     }
//
//     virtual void TickComponent(float DeltaTime, ELevelTick TickType,
//                                FActorComponentTickFunction *ThisTickFunction) override
//     {
//         Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
//
//         const AActor *Platform = GetOwner();
//         if (!Platform || !GetWorld()) { return; }
//
//         // The three basis vectors Unreal already gives you, plus the
//         // location in METRES. This is the whole coordinate handover.
//         const FVector L = Platform->GetActorLocation();
//         const FVector F = Platform->GetActorForwardVector();
//         const FVector R = Platform->GetActorRightVector();
//         const FVector U = Platform->GetActorUpVector();
//
//         airmar::Pose Pose;
//         Pose.position_m = airmar::Vec3{L.X * 0.01, L.Y * 0.01, L.Z * 0.01};
//         Pose.forward    = airmar::Vec3{F.X, F.Y, F.Z};
//         Pose.right      = airmar::Vec3{R.X, R.Y, R.Z};
//         Pose.up         = airmar::Vec3{U.X, U.Y, U.Z};
//
//         const FFlsUnrealWorld World(GetWorld(), Platform, TraceChannel.GetValue(),
//                                     bIgnoreWaterBodies);
//         const airmar::Output O = Sonar.Update(DeltaTime, Pose, World);
//
//         bHasRange   = O.has_range;
//         RangeMeters = static_cast<float>(O.range_m);
//
//         if (bDrawBeam && O.has_range)
//         {
//             const FVector Start(O.beam_origin_m.x * 100.0, O.beam_origin_m.y * 100.0,
//                                 O.beam_origin_m.z * 100.0);
//             const FVector Dir(O.beam_axis.x, O.beam_axis.y, O.beam_axis.z);
//             DrawDebugLine(GetWorld(), Start, Start + Dir * (O.range_m * 100.0),
//                           FColor::Green, false, 0.25f, 0, 3.f);
//         }
//     }
//
//     /** The sensor itself. Public so you can read LastRays(), ActiveRangeMode()
//      *  and the counters, or call SetEnabled(false) to stop transmitting. */
//     airmar::Fls Sonar;
// };
// === END FILE FlsSonarComponent.h ===
//
// ---------------------------------------------------------------------------
// STEP 4 -- verify the integration, do not assume it
// ---------------------------------------------------------------------------
// Put a static mesh wall a known distance in front of the vehicle, with
// collision enabled on the trace channel, and check all five:
//
//   1. The log line from BeginPlay appears and says "beam 14.0 deg, 13 rays".
//      If it says "refusing to run", read the reason; do not work around it.
//   2. RangeMeters is non-zero and within a few percent of the real distance.
//      It should read very slightly HIGH -- the data sheet's bias is +0.54 %.
//   3. RangeMeters is NOT 0.000. A hard 0.000 means a trace is hitting
//      something at the origin: water, or your own hull. See PITFALLS 1 and 2.
//   4. Remove the wall. bHasRange must become false within
//      misses_to_lose_lock pings, and must NOT keep repeating the last value.
//   5. Move the wall closer than 0.4 m. bHasRange must go false: that is the
//      hardware dead zone, not a bug.
//
// If 2 is off by a factor of 100, a centimetre/metre conversion is missing.
// If 2 is off by the mount offset, the beam origin is being taken from the
// actor instead of from Output::beam_origin_m.
//
// ---------------------------------------------------------------------------
// WHAT NOT TO DO
// ---------------------------------------------------------------------------
//   * Do not read Output::true_range_m in anything but a test or a plot. It is
//     the geometry before the error model -- using it is cheating, and the
//     stack will fail the first time it meets real hardware.
//   * Do not treat a missing range as a zero range. has_range == false means
//     "the device has no echo", which is not the same as "the obstacle is at
//     0 m". Hold the last value with its age, or declare the reading stale.
//   * Do not call Update more than once per frame with the same dt; it fires
//     at most one ping per call, but you will double the simulated clock.
//   * Do not add engine types to FLS.hpp or FLS.cpp. The CI gate in this
//     repository fails the build if either file grows an include outside the
//     four standard headers. Put engine code in the adapter, where it belongs.
//   * Do not raise source_level_db or lower detection_threshold_db to "make it
//     see further" without saying so. Those are tuning values, not data sheet
//     values, and a number produced by moving them is a simulation result.
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

// ===========================================================================
// WHY THE NAMESPACE IS `airmar` AND NOT `fls`
// ===========================================================================
// It was `fls` first. That does not compile on macOS or any BSD, because
// their libc declares `int fls(int)` -- "find last bit set" -- in
// <strings.h>, and a namespace name collides with an ordinary function name
// at global scope:
//
//     FLS.hpp:91:11: error: redefinition of 'fls' as different kind of symbol
//
// It built cleanly on Linux and Windows, so only the macOS CI job caught it.
// macOS is no longer in the matrix -- instead `tools/libc_collision_check.cpp`
// declares `fls()` the way BSD libc does and must compile, so the collision is
// now caught on every platform rather than on one. Please do not "simplify"
// the namespace back: that check, and the <strings.h> include at the top of
// test_fls.cpp, both exist to stop it.
// ===========================================================================

#ifndef FLS_HPP
#define FLS_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace airmar
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

} // namespace airmar

#endif // FLS_HPP
