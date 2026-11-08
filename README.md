# FLS — Forward Looking Sonar / altimeter, single beam

A simulation model of the **AIRMAR 200 m Mini Altimeter Kit**, written against
the data sheet `MiniAlt_rI 07/27/23`.

Two files. No dependencies. Drop them in and you have the sensor.

```
FLS.hpp
FLS.cpp
```

Everything else in this repository is tests, CI and this document.

---

## What it actually models

This is **not** "a ray cast converted to a number". A real altimeter very often
returns no number at all, and the parts that decide whether a number exists are
exactly the parts that make an autonomy stack fail in the water:

| Behaviour | Why it matters |
|---|---|
| A finite **14°** beam cone, reduced to the **first return** | the nearest geometry is not what the device reports |
| An active **sonar equation** | a weak or steeply angled target gives no echo well inside maximum range |
| **0.4 m** minimum range and operator blanking | the dead zone a bow-mounted unit really has |
| Four **range scales** with ping rates, and auto selection with hysteresis | range mode changes the update rate |
| **Bottom lock** acquisition and loss | the gap your controller must survive |
| **1 cm** quantisation, **+0.54 %** scale error, noise, dropout | the number is never the truth |

A sensor that always answers is the most dangerous kind of fake.

---

## Integration — the whole job is one function

### 1. Implement the ray cast

`airmar::IWorld::CastRay` is the **only** call FLS makes into your engine.

```cpp
class MyWorld : public airmar::IWorld {
public:
    airmar::RayHit CastRay(const airmar::Vec3& origin_m,
                        const airmar::Vec3& dir,
                        double max_range_m) const override
    {
        airmar::RayHit out;
        // ... your engine's trace ...
        out.hit           = did_hit;
        out.range_m       = distance_in_METRES;
        out.incidence_cos = fabs(dot(dir, surface_normal));   // 0..1
        return out;
    }
};
```

### 2. Fill in the config

Every static transform and device setting is a plain field you type in by hand.

```cpp
airmar::Config cfg;
cfg.mount_forward_m = 2.12;   // transducer at the nose, along the hull axis
cfg.mount_up_m      = -0.004; // on the hull centreline
cfg.mount_pitch_deg = 0.0;    // looking straight ahead (-90 = downward altimeter)
cfg.frequency_khz   = 200.0;  // 200 -> 14 deg beam, 170 -> 18 deg beam

const std::string err = cfg.Validate();
if (!err.empty()) { /* fail loudly: do not ship a sensor you did not configure */ }
```

### 3. Update once per frame

```cpp
airmar::Fls  sonar(cfg);
MyWorld   world;

airmar::Pose pose;
pose.position_m = /* platform origin, METRES */;
pose.forward    = /* your engine's forward vector */;
pose.right      = /* ... right ... */;
pose.up         = /* ... up ... */;

const airmar::Output o = sonar.Update(dt_seconds, pose, world);
if (o.has_range) {
    use(o.range_m);          // what the device reports
}
```

That is the entire contract. FLS never allocates after the first ping, holds no
global state, starts no threads and performs no I/O.

---

## Unreal Engine

**The authoritative integration recipe lives in `FLS.hpp` itself**, in the
`INTEGRATION` block at the top. It is written to be executed literally and
contains two complete files, delimited so they can be pulled straight out:

```
=== BEGIN FILE FlsUnrealWorld.h ===      the airmar::IWorld adapter (line trace)
=== BEGIN FILE FlsSonarComponent.h ===   a ready UActorComponent, header-only
```

It is in the header rather than here because the deliverable is two files, and
the person receiving them should not need this document. To get those files
back out as real files:

```bash
bash tools/extract_unreal_files.sh /path/to/YourProject/Source/YourModule
```

Component or actor is your call — this model has no opinion and no dependency
either way. Driving it is three calls:

```cpp
airmar::Pose Pose;                         // the owner's own basis vectors
Pose.position_m = { L.X*0.01, L.Y*0.01, L.Z*0.01 };   // Unreal cm -> metres
Pose.forward = { F.X, F.Y, F.Z };
Pose.right   = { R.X, R.Y, R.Z };
Pose.up      = { U.X, U.Y, U.Z };

const FFlsUnrealWorld World(GetWorld(), GetOwner());
const airmar::Output O = Sonar.Update(DeltaTime, Pose, World);
```

### This is verified, not asserted

`tools/unreal_build_check.sh` extracts the recipe out of the header's comments
and builds it as a module in a throwaway Unreal project that has nothing to do
with this repository:

```bash
UE_ROOT=/path/to/UE_5.3 bash tools/unreal_build_check.sh
```

Last run: **UE 5.3.0, Linux, UnrealHeaderTool with `-WarningsAsErrors`, exit 0
in 20 s.** The check's `Build.cs` deliberately lists nothing beyond
`Core`, `CoreUObject` and `Engine`, so if the recipe ever came to need another
engine module the build would fail instead of the README quietly being wrong.

GitHub Actions cannot run this — the runners have no Unreal Engine. What CI
does enforce is that the recipe is still extractable and still contains the
adapter and the component, so the comment block cannot rot unnoticed.

## Pitfalls (measured, not guessed)

These cost real debugging time in the simulator this model came out of. They
are not Unreal-specific in principle, but Unreal is where they bite.

1. **Water bodies block traces.** With an `AWaterBody` in the level, every ray
   returned a hit at **0.000 m** — the sensor was reporting the inside of the
   water volume. The sensor is *inside* the water, so the water must be
   excluded from the query. Add every water actor to the ignore list.
2. **Your own hull blocks the trace.** If the sensor is a separate actor
   attached to the vehicle, ignoring only `GetOwner()` is not enough: ignore
   the whole attachment chain, as the adapter above does.
3. **Units.** Unreal is centimetres, FLS is metres. `Hit.Distance` is
   centimetres from `Start`.
4. **`incidence_cos` is worth filling in.** Leave it at `1.0` and every
   surface becomes a perfect reflector, which removes one of the two effects
   that make the sensor realistic.
5. **A wide beam sees edge-on surfaces.** With a 14° cone, the rim rays strike
   a surface that is edge-on to the axis at up to 7° off it, so the device gets
   an echo the axial ray would have missed. That is correct, and the test suite
   pins it.

---

## Configuration reference

The fields you will actually touch. All lengths metres, angles degrees, times
seconds. See `FLS.hpp` for the rest, each with its own comment.

| Field | Default | Meaning |
|---|---|---|
| `mount_forward_m` / `mount_right_m` / `mount_up_m` | 0 | transducer offset in the platform's own frame |
| `mount_yaw_deg` / `mount_pitch_deg` | 0 | beam direction; `-90` pitch = downward altimeter |
| `frequency_khz` | 200 | `170` or `200`; anything else is rejected |
| `beam_width_deg` | 0 | `0` derives it from the frequency (recommended) |
| `beam_rings` / `beam_samples_per_ring` | 2 / 6 | cone sampling, 13 rays total |
| `min_range_m` / `max_range_m` | 0.4 / 200 | hardware range gate |
| `blanking_m` | 0 | operator dead zone, 0–15 m |
| `range_mode` | `Auto` | `Short` / `Medium` / `Long` / `VeryLong` / `Auto` |
| `use_sonar_equation` | `true` | `false` gives a pure geometric range |
| `min_incidence_cos` | 0.10 | below this, no echo at all |
| `scale_error_fraction` | 0.0054 | the data sheet's accuracy bias |
| `range_noise_sigma_m` | 0.02 | per-ping noise |
| `dropout_probability` | 0.02 | fraction of good pings that return nothing |
| `depth_filter_samples` | 0 | moving average: 0, 2 or 4 |
| `smoothing_filter` | 0 | exponential, alpha = 1/N: 0, 2, 4 or 8 |
| `pings_per_second` | 0 | `0` = auto from the range mode |
| `misses_to_lose_lock` | 3 | consecutive misses before lock drops |
| `seed` | fixed | determinism; same seed, same bytes, every platform |

`Output::true_range_m` is the geometric distance before the error model. It is
there **for plots and tests only** — an autonomy stack that reads it is
cheating and will not survive contact with hardware.

---

## Where the numbers come from

Everything traceable to **AIRMAR 200 m Mini Altimeter Kit, `MiniAlt_rI 07/27/23`**:

| Quantity | Value | Data sheet section |
|---|---|---|
| Beam angle, 200 kHz-A | 14° at −3 dB | BEAM ANGLE |
| Beam angle, 170 kHz-C | 18° at −3 dB | BEAM ANGLE |
| Range | 0.4 – 200 m | DEPTH RANGE |
| Resolution | 1 cm | RESOLUTION |
| Accuracy bias | +0.537 % mean | ACCURACY (tank testing): 3.05→3.07, 4.57→4.59, 5.79→5.82 m |
| Range scales | 56 / 225 / 450 / 675 ft | RANGE SCALES |
| Transducer body | ⌀25.4 mm × 58.4 mm | DIMENSIONS |

The accuracy figure is the **mean of the data sheet's own tank table**
(+0.656 %, +0.438 %, +0.518 %), not a rounded guess. The test suite measures
the model's realised bias over 400 pings and asserts it lands on that figure:
the last run measured **+0.5445 %** against the data sheet's **+0.5400 %**.

The tuning parameters — `source_level_db`, `noise_level_db`,
`detection_threshold_db`, `absorption_db_per_km`, `target_strength_db`,
`range_noise_sigma_m`, `dropout_probability` — are **not** from the data sheet.
They are plausible simulation values, documented as such in `FLS.hpp`, and you
should expect to tune them against your own water. Treat anything derived from
them as a simulation result, never as a hardware specification.

---

## Tests

541 checks, no test framework required.

```bash
c++ -std=c++17 -Wall -Wextra -Werror -O2 FLS.cpp test_fls.cpp -o test_fls && ./test_fls
```

or

```bash
cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure
```

The suite covers the mount transform (including the rotated-platform case),
the data sheet tables, the dead zone and blanking, grazing incidence, the
sonar equation, first-return selection against two walls, auto range
selection and its hysteresis, pinned-mode truncation, ping scheduling, lock
loss and re-acquisition, determinism and reset, dropout rate, filter
behaviour, and the between-pings hold.

Three of those tests exist because the first version of the suite asserted
what the model *ought* to do and the model disagreed; the model was right each
time. The comments record the measured numbers.

CI runs the matrix on Linux, Windows and macOS against C++17 and C++20, plus
an AddressSanitizer / UndefinedBehaviorSanitizer build, plus a gate that fails
the build if `FLS.hpp` or `FLS.cpp` ever grows an include outside
`<cstdint> <string> <vector> <cmath>` or mentions an engine type. The
portability claim is enforced, not asserted.

---

## License

MIT. See `LICENSE`.
