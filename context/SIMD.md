# SIMD, `-march`, Eigen Alignment and ABI — A Working Tutorial

**Status:** reference tutorial. Written 2026-07-24 while diagnosing the `basalt_slam_node` `SIGSEGV` in `on_activate`.
**Audience:** anyone building `slam` (and its vendored `ext/basalt`) inside this workspace, or debugging a crash that "makes no sense".
**Companion documents:** [`/ws/ros_ws/context/simd-eigen-abi-mismatch.md`](../../../context/simd-eigen-abi-mismatch.md) (the condensed investigation record), [`build-and-abi.md`](build-and-abi.md) (the package-level build and ABI lookup context) and [`/ws/plans/basalt-slam-defect-remediation.md`](../../../../plans/basalt-slam-defect-remediation.md) (the remediation plan).

---

## Motivation

`basalt_slam_node` crashed with `SIGSEGV` inside `/lib64/ld-linux-x86-64.so.2` — the *dynamic linker* — while `tf2_ros::TransformBroadcaster` was creating a `/tf` publisher at `src/basalt/node.cpp:98`. Nothing in that backtrace is wrong. The actual defect is that `libbasalt.so` and `basalt_slam_node` were compiled with different `-march` settings, which made them disagree about the size and alignment of `basalt::Controller` — 1056 bytes/32-byte-aligned on one side, 1008 bytes/16-byte-aligned on the other. Constructing that object overflowed the heap by 48 bytes; the loader's next allocation walked the damaged free list and died.

To understand *why* a compiler optimisation flag silently changed a class's memory layout, you need four ideas stacked on top of each other:

1. **SIMD** — CPUs have wide registers that process several numbers per instruction. This is what `xmm`/`ymm`/`zmm` are.
2. **`-march`** — the flag that tells the compiler which of those registers/instructions it is allowed to use. It also defines preprocessor macros describing what it enabled.
3. **Eigen's alignment policy** — Eigen reads those macros and changes the *alignment* of its matrix types so that wide loads/stores are legal and fast. Alignment changes `sizeof` and member offsets.
4. **The ODR / ABI contract** — C++ assumes every translation unit agrees on what a class looks like. When step 3 differs between two translation units, that assumption is violated and the linker cannot detect it.

This document walks all four, then covers how to debug this class of bug and how to choose build flags so it cannot happen again.

---

## Part I — Preliminaries: SIMD and the x86 vector registers

### 1.1 Scalar versus vector execution

A conventional ("scalar") instruction operates on one value. To add two arrays of 8 doubles, a scalar CPU issues 8 `addsd` (add scalar double) instructions.

SIMD — **S**ingle **I**nstruction, **M**ultiple **D**ata — gives the CPU registers wide enough to hold several values, and instructions that apply one operation to all lanes simultaneously. A single 512-bit `vaddpd` (add **p**acked **d**ouble) adds 8 doubles in one instruction. The number of values per register is the **lane count** or **packet size**; Eigen calls it the *packet*.

This is data parallelism inside one core, and it is orthogonal to threading. Basalt uses both: TBB for thread parallelism across the bundle-adjustment blocks, SIMD for the arithmetic inside each block.

### 1.2 The x86 vector register files: `xmm`, `ymm`, `zmm`

These names are register *classes*, each introduced by a different instruction-set extension, and each one nests inside the next:

| Name | Width | Introduced by | Year | Lanes (double) | Lanes (float) | Register count (64-bit mode) |
|------|-------|---------------|------|----------------|---------------|------------------------------|
| `xmm0…15` | 128-bit | SSE / SSE2 | 1999 / 2000 | 2 | 4 | 16 |
| `ymm0…15` | 256-bit | AVX (AVX2 adds integer ops) | 2011 (Sandy Bridge) / 2013 (Haswell) | 4 | 8 | 16 |
| `zmm0…31` | 512-bit | AVX-512 | 2016 (Skylake-SP, Knights Landing) | 8 | 16 | 32 |

They are physically the same register file: `xmm5` is the low 128 bits of `ymm5`, which is the low 256 bits of `zmm5`. AVX-512 also widens the file from 16 to 32 registers (`xmm16`–`xmm31` only exist if AVX-512 is enabled), which by itself reduces register spilling in heavy numerical loops.

So when `objdump` shows an instruction naming `%zmm6`, that instruction is AVX-512 and can only execute on a CPU that supports AVX-512. `%ymm` means AVX/AVX2. `%xmm` means SSE-family (and is essentially universal on x86-64 — SSE2 is part of the base x86-64 ABI, which is why plain scalar `double` math already uses `xmm` registers).

### 1.3 What "40093 zmm instructions" actually measured

The exact command was:

```bash
objdump -d build/slam/ext/basalt/libbasalt.so | grep -c "%zmm"
```

which counts **disassembled lines mentioning a `zmm` operand**. Results for our build of `libbasalt.so`:

| Register class | Count | Meaning |
|---|---|---|
| `%xmm` | 326328 | SSE-width work — baseline, present in any x86-64 binary |
| `%ymm` | 28494 | 256-bit AVX/AVX2 instructions |
| `%zmm` | 40093 | 512-bit AVX-512 instructions |
| `%k0`–`%k7` | 328 | AVX-512 opmask registers (see §1.6) — these exist *only* under AVX-512 |

Two caveats on methodology, so you read such numbers honestly:

- This is a **static** count of instructions present in the file, not a dynamic count of instructions executed. A single `zmm` instruction inside a hot loop matters far more than a thousand in cold initialisation code.
- One line can mention two registers of the same class, so the count is "lines containing", not "operands". For a yes/no question — *was this object built with AVX-512 enabled?* — it is perfectly adequate. 40093 is not a borderline number; zero versus tens of thousands is the signal.

The comparison that mattered was against the node's own objects:

```bash
objdump -d build/slam/CMakeFiles/basalt_slam_node.dir/src/basalt/slam.cpp.o | grep -c ymm   # 0
objdump -d build/slam/CMakeFiles/basalt_slam_node.dir/src/basalt/node.cpp.o  | grep -c zmm   # 0
```

Zero `ymm`, zero `zmm`. The two halves of the same executable were compiled for different instruction sets. That is the whole bug in one measurement.

### 1.4 Reading a real basalt loop

Here is an actual inner loop from `libbasalt.so`, inside `Eigen::internal::general_matrix_vector_product<double,…>::run` (AT&T syntax, as `objdump` prints it):

```asm
150c60:  vmovupd (%rdx,%rax,8),%zmm0                 ; load 8 doubles from x[]
150c67:  vfmadd231pd (%r11,%rax,8),%zmm0,%zmm6       ; acc6 += row0[] * x[]
150c6e:  vfmadd231pd (%r14,%rax,8),%zmm0,%zmm5       ; acc5 += row1[] * x[]
150c75:  vfmadd231pd 0x0(%r13,%rax,8),%zmm0,%zmm4    ; acc4 += row2[] * x[]
150c7d:  vfmadd231pd (%r12,%rax,8),%zmm0,%zmm3       ; acc3 += row3[] * x[]
150c84:  vfmadd231pd (%rbx,%rax,8),%zmm0,%zmm2       ; …
150c8b:  vfmadd231pd (%r15,%rax,8),%zmm0,%zmm1
150c92:  vfmadd231pd (%rdi,%rax,8),%zmm0,%zmm8
150c99:  vfmadd231pd (%rsi,%rax,8),%zmm0,%zmm9
150ca0:  add    $0x8,%rax                            ; advance by 8 doubles
150ca4:  cmp    %r8,%rax
150ca7:  jne    150c60
```

This is a matrix–vector product, unrolled 8 rows deep. Each iteration loads 8 doubles of the vector once into `zmm0` and multiply-accumulates it against 8 different matrix rows into 8 separate accumulators. Per iteration that is **64 multiply-adds = 128 floating-point operations in 9 instructions**. The scalar equivalent would be 128 instructions. That factor is why basalt asks for `-march=native`, and why simply deleting the flag is not a free fix.

Note also `vmovupd` versus `vmovapd` in the listing: **u**naligned versus **a**ligned move. Which one the compiler emits is decided by Eigen's alignment reasoning — that is the bridge to Part III.

### 1.5 FMA — fused multiply-add

`vfmadd231pd` is an **FMA** instruction: **F**used **M**ultiply-**A**dd. It computes `a * b + c` as a single operation.

Decoding the mnemonic:

- `v` — VEX/EVEX-encoded (AVX-family) instruction.
- `fmadd` — fused multiply-add (there are also `fmsub`, `fnmadd`, `fnmsub` for sign variants).
- `231` — which operands are multiplied and which accumulates. Numbering the operands 1, 2, 3 in Intel order (`op1` is the destination): `231` means `op1 = op2 * op3 + op1`. Variants `132` (`op1 = op1*op3 + op2`) and `213` (`op1 = op2*op1 + op3`) exist so the compiler can pick whichever avoids a register copy.
- `p` — packed (all lanes), as opposed to `s` for scalar.
- `d` — double precision (`s` = single).

So `vfmadd231pd (%r11,%rax,8),%zmm0,%zmm6` in AT&T order is Intel's `vfmadd231pd zmm6, zmm0, [r11+rax*8]` → `zmm6 += zmm0 * mem`, eight lanes at once. It is precisely a dot-product accumulation step, which is why it dominates linear-algebra code.

FMA matters for two independent reasons:

1. **Throughput.** One instruction instead of two (`vmulpd` then `vaddpd`), and modern cores can issue two FMAs per cycle. Combined with 512-bit width this is where the headline speedups come from.
2. **Accuracy.** The "fused" part means the intermediate product `a*b` is *not* rounded before the addition — there is a single rounding at the end instead of two. This makes FMA slightly more accurate than separate multiply and add. It also makes results **bit-for-bit different** from a non-FMA build, which is worth knowing when a SLAM trajectory differs in the last digits between two builds: that is expected, not a bug. It also means floating-point results are not reproducible across `-march` levels.

FMA (specifically FMA3) arrived with Intel Haswell and AMD Piledriver, which is why `-march=haswell` and `x86-64-v3` are the usual "modern baseline" choices — they are the level at which AVX2 + FMA become available.

### 1.6 Opmask registers `k0`–`k7`

AVX-512 added eight **opmask** registers. They hold one bit per lane and let a single instruction operate on a subset of lanes, or merge/zero the rest — for example handling the ragged tail of a loop whose length is not a multiple of 8 without a scalar cleanup loop. Their presence in a binary (`grep -cE '%k[0-7]'` → 328 in our `libbasalt.so`) is another unambiguous AVX-512 fingerprint, because no earlier extension has them.

### 1.7 The ARM side — NEON and SVE (relevant to the Jetson target)

This workspace targets a Jetson Orin Nano (`docs/electrical_roadmap.md:3`), i.e. AArch64, so the equivalent vocabulary matters:

- **NEON (ASIMD)** is ARM's baseline SIMD, mandatory on AArch64. Registers `v0`–`v31` are **128-bit** — the same width as `xmm`, i.e. 2 doubles per register. There is no 256-bit or 512-bit NEON.
- **SVE / SVE2** are ARM's vector-length-agnostic extensions (128–2048 bit, decided by the hardware). The Orin's Cortex-A78AE cores are ARMv8.2-A and do **not** implement SVE.
- ARM has its own FMA instructions (`fmla`/`fmls`), so §1.5 carries over.

The consequence for this bug is important and slightly counter-intuitive: because NEON is 128-bit, Eigen's maximum alignment on AArch64 is **16 bytes regardless of `-march`**. The layout divergence described in Part IV is therefore **x86-only** — it cannot reproduce on the Orin. A bug that only appears on the developer workstation and never on the target board is exactly the kind that survives for months, which is an argument for fixing it structurally rather than by luck.

---

## Part II — `-march`, `-mtune`, `-mcpu`: what they actually do

### 2.1 The three flags

- **`-march=X`** — "you may emit instructions from X's instruction set". This is a **correctness** flag: run the binary on a CPU that lacks those instructions and you get `SIGILL` (illegal instruction), typically at the first hot loop.
- **`-mtune=X`** — "schedule/optimise assuming CPU X, but do not emit instructions outside the current `-march`". Purely a **performance** hint; the binary still runs anywhere `-march` allows.
- **`-mcpu=X`** — on ARM, historically the combination of both. On x86 GCC it is a deprecated alias for `-mtune`.

Setting `-march` implies a matching `-mtune` unless you override it.

### 2.2 `native`, and how it is resolved

`-march=native` means "detect the CPU **of the machine running the compiler** and use its full feature set". On x86 GCC does this by executing `CPUID` at compile time.

Three consequences follow directly, and all three bite this repository:

1. **The result is not a fixed value.** The same source, the same commit, the same container yields a different binary on a Zen 5 workstation than on a Xeon or in CI. Our build host is an **AMD Ryzen 9 9950X** (Zen 5), which implements AVX-512 with a full-width datapath — hence `EIGEN_MAX_ALIGN_BYTES` of 64. On a Zen 2 machine the same command would have produced 32, and the bug in Part IV would have had a different (still fatal) shape.
2. **The result depends on the *build* machine, never the *target*.** This is what makes it unusable under cross-compilation (§2.5, §6.6).
3. **The value is invisible in the build log** unless you go looking. Nothing prints "you just enabled AVX-512".

You can always ask the compiler what it resolved:

```bash
gcc -march=native -Q --help=target | grep -E "^\s+-march=|-mavx|-mfma|-mavx512f"
gcc -march=native -dM -E -x c++ /dev/null | grep -E "AVX|FMA|SSE"
```

### 2.3 Microarchitecture levels: `x86-64-v1` … `v4`

Because "pick a specific CPU name" ages badly, Intel, AMD, Red Hat and SUSE agreed on four portable **microarchitecture levels**, supported by GCC 11+ and Clang 12+ as `-march=x86-64-v{2,3,4}`:

| Level | Adds | Roughly equivalent to | Eigen `EIGEN_MAX_ALIGN_BYTES` |
|-------|------|----------------------|-------------------------------|
| `x86-64` (v1) | SSE, SSE2 — the base x86-64 ABI | Any x86-64 CPU (2003+) | 16 |
| `x86-64-v2` | SSE3, SSSE3, SSE4.1, SSE4.2, POPCNT, CMPXCHG16B | Nehalem (2008) | 16 |
| `x86-64-v3` | **AVX, AVX2, FMA**, BMI1/2, F16C, LZCNT, MOVBE, XSAVE | Haswell (2013) | 32 |
| `x86-64-v4` | AVX512F, AVX512BW, AVX512CD, AVX512DQ, AVX512VL | Skylake-SP (2017) | 64 |

These are the right vocabulary for a deployable build: they state a *contract* ("runs on anything from Haswell onward") rather than a wish. Note that upstream Basalt already does exactly this — every packaged/`.deb` job in `ext/basalt/.gitlab-ci.yml` (lines 158, 170, 183) pins `CXX_MARCH: 'haswell'`, and only the developer build at line 6 uses `native`.

### 2.4 The hidden second effect: feature macros

This is the part people miss, and the direct cause of our crash. `-march` does not only change code generation — it defines **preprocessor macros** that any header can read and change its behaviour accordingly. Measured on our build host:

```bash
diff <(g++ -dM -E -x c++ /dev/null) <(g++ -march=native -dM -E -x c++ /dev/null)
```

```
> #define __AVX__ 1
> #define __AVX2__ 1
> #define __FMA__ 1
> #define __AVX512F__ 1
> #define __AVX512DQ__ 1
> #define __AVX512BW__ 1
> #define __AVX512VL__ 1
> #define __AVX512CD__ 1
> #define __SSE4_1__ 1
> #define __SSE4_2__ 1
> #define __BMI__ 1
…
```

A header that branches on `__AVX512F__` and changes a type's `alignas(...)` has just made `-march` an **ABI switch**. Eigen is exactly such a header (Part III). So is any library layered on it — Sophus, basalt-headers, PCL, and so on.

The mental model to internalise:

> In a codebase built on Eigen, `-march` is not an optimisation flag. It is part of the ABI, like `-std`, `-D_GLIBCXX_USE_CXX11_ABI`, or the target triple. Every translation unit that shares a type must agree on it.

### 2.5 Other consequences of `-march=native`

- **`SIGILL` on deployment.** A binary built with AVX-512 on this workstation and run on a machine without it dies with an illegal instruction. In this workspace that is a live risk, because `ros_ws/install/` is a *mounted, persistent* tree (see [`/ws/context/sitl-workspace-migration.md`](../../../../context/sitl-workspace-migration.md)) — the artefacts survive container replacement, and `colcon` will not rebuild them just because you moved to another host.
- **Non-reproducible builds.** Two developers cannot compare binaries or bisect a numerical difference.
- **Frequency behaviour.** On several Intel server generations, sustained 512-bit work triggers licence-based downclocking, so AVX-512 can be a *net loss* for mixed workloads. (Zen 4/5 handle this differently; the point is that "widest is fastest" is not automatic — measure.)
- **Unusable when cross-compiling.** See §6.6.

---

## Part III — Alignment, and why Eigen cares so much

### 3.1 What alignment means at the hardware level

An object is *N-byte aligned* if its address is a multiple of N. Every C++ type has an `alignof`, and the compiler guarantees objects are placed accordingly — by padding structs, by adjusting stack frames, and by requesting suitably aligned memory from `operator new`.

For SIMD this stops being bookkeeping and becomes performance and, historically, correctness:

- A 512-bit **aligned** load (`vmovapd`) requires its address to be a multiple of 64. On older ISAs, violating this **faults** (`SIGSEGV`). The AVX-encoded aligned moves still fault on misalignment.
- The **unaligned** forms (`vmovupd`) accept any address. On modern cores they are nearly as fast *when the data happens to be aligned*, and slower when a load straddles a cache line.
- Crossing a cache-line (64 B) or page boundary costs extra cycles; alignment guarantees a 64-byte vector never straddles a line.

So Eigen tries to over-align its data, so it can use the aligned instructions and avoid split accesses.

### 3.2 "Fixed-size vectorizable" Eigen types

Eigen classifies a fixed-size type as **fixed-size vectorizable** when its total byte size is a multiple of 16 (more precisely, of the packet size). Examples: `Vector4d` (32 B), `Matrix4d` (128 B), `Matrix2d` (32 B), `Vector4f` (16 B), `Quaterniond` (32 B), `Matrix<double,12,1>` (96 B).

Those types get an `alignas` applied. Types whose size is not a multiple of 16 — `Vector3d` (24 B), `Matrix3d` (72 B), `Matrix<double,9,1>` (72 B) — are *not* over-aligned, so they are immune to this whole problem. That is why `Calibration<double>`'s `CalibAccelBias` member (a `Matrix<double,9,1>`, 72 B) was harmless while its `CalibGyroBias` member (a `Matrix<double,12,1>`, 96 B) was not.

### 3.3 `EIGEN_MAX_ALIGN_BYTES` — the governing knob

From the Eigen documentation: *"Must be a power of two, or 0. Defines an upper bound on the memory boundary in bytes on which dynamically and statically allocated data may be aligned by Eigen."* It is an **upper bound**, not the alignment itself. The effective alignment of a given type is:

```
alignof(T) = min( EIGEN_MAX_ALIGN_BYTES , largest power of two dividing sizeof(T) )
```

If you do not define it, Eigen computes a default from the ISA macros of §2.4. Measured on our toolchain against basalt's vendored Eigen 3.4:

| Compiler flags | `EIGEN_MAX_ALIGN_BYTES` | `alignof(Matrix<double,12,1>)` |
|---|---|---|
| *(none)* | 16 | 16 |
| `-msse2` | 16 | 16 |
| `-mavx` | 32 | 32 |
| `-mavx2 -mfma` | 32 | 32 |
| `-mavx512f` *(alone)* | 32 | 32 |
| `-march=native` *(Zen 5)* | **64** | **32** |

Two subtleties visible in that table, both worth understanding:

- `-mavx512f` **alone** still yields 32, because Eigen only enables its AVX-512 path when the full set `AVX512F + DQ + BW + VL + CD` is present. Feature macros are consulted as a group; do not assume one flag implies the Eigen path.
- With the bound at 64, `Matrix<double,12,1>` is aligned to **32**, not 64 — because 96 bytes is `32 × 3`, and 64 does not divide 96. This is the `min(bound, largest-power-of-two-divisor)` rule in action, and it is why you must reason about *both* the bound and the type's size.

Related directives, for completeness:

| Macro | Effect |
|---|---|
| `EIGEN_MAX_ALIGN_BYTES` | Upper bound on alignment for **both** static and dynamic allocation. The ABI-relevant one. |
| `EIGEN_MAX_STATIC_ALIGN_BYTES` | Same, but for statically allocated data only. Defaults to `EIGEN_MAX_ALIGN_BYTES` if only that is set. |
| `EIGEN_DONT_VECTORIZE` | Disables explicit vectorization entirely. A blunt instrument — costs the performance basalt exists for. |
| `EIGEN_UNALIGNED_VECTORIZE` | Default `1`. If `0`, expressions whose destination cannot be aligned are not vectorized at all. |
| `EIGEN_DONT_ALIGN` / `EIGEN_DONT_ALIGN_STATICALLY` | Deprecated Eigen 3.2-era spellings; prefer setting the byte counts to `0`/`16`. |

The documentation states the intended use case for `EIGEN_MAX_ALIGN_BYTES` explicitly: enforcing **binary compatibility between code compiled with different SIMD options** — for instance compiling AVX code that must stay ABI-compatible with existing SSE code by defining it to 16. That is exactly our situation.

### 3.4 How alignment propagates into struct layout

C++ layout rules: a struct's alignment is the maximum of its members' alignments, each member is placed at the next offset satisfying its own alignment, and the total size is rounded up to a multiple of the struct's alignment. So raising one member's alignment from 16 to 32 can (a) insert padding before it, (b) shift every subsequent member, and (c) grow the total size.

Demonstrated with a minimal model of the real basalt types (full source in §5.4):

```
                          no -march        -march=native
EIGEN_MAX_ALIGN_BYTES        16                 64
alignof(Matrix<double,12,1>) 16                 32
sizeof/alignof(CalibGyroBias) 96 / 16           96 / 32
sizeof/alignof(Calib)        192 / 16          224 / 32
offsetof(Calib, g)            80                 96
sizeof/alignof(Controller)   224 / 16          288 / 32
offsetof(Controller, more)   208                256
```

One `-march` flag, and every offset in the struct moved.

### 3.5 The classic Eigen alignment failure modes

Eigen's own documentation enumerates four well-known ways to get this wrong. They are worth knowing because they produce *similar* symptoms and you need to tell them apart:

1. **Fixed-size vectorizable members in a class allocated with `new`** — fixed by `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` (largely obsolete since C++17 aligned `new`).
2. **STL containers of fixed-size vectorizable types** — `std::vector<Eigen::Vector4d>` needs `Eigen::aligned_allocator`. Basalt's `Eigen::aligned_vector` alias exists for this (`Calibration<Scalar>::T_i_c`).
3. **Passing such types by value** — the ABI may not preserve alignment across the call.
4. **Compiler making a wrong assumption about stack alignment** — an old 32-bit x86 issue.

There is a fifth, not in that list because it is a *build system* failure rather than a coding one, and it is the one that bit us:

5. **Two translation units compiled with different `-march`, sharing a type.** No source-level mistake exists anywhere. The code is correct; the build is incoherent.

---

## Part IV — The mechanism, step by step

### 4.1 One class, two layouts

`slam/CMakeLists.txt:159` pulls basalt in as a subproject:

```cmake
add_subdirectory(ext/basalt)          # line 159
add_executable(basalt_slam_node       # line 161
  src/node.cpp src/slam.cpp src/basalt/slam.cpp src/basalt/node.cpp src/basalt/driver.cpp)
target_link_libraries(basalt_slam_node PUBLIC basalt …)
```

Inside `ext/basalt/CMakeLists.txt`:

```cmake
if(NOT CXX_MARCH)
  set(CXX_MARCH native)                                    # line 75-76
endif()
set(BASALT_MARCH_FLAGS "-march=${CXX_MARCH}")              # line 80
…
set(CMAKE_CXX_FLAGS "${BASALT_CXX_FLAGS} ${BASALT_MARCH_FLAGS} ${BASALT_PASSED_CXX_FLAGS}")   # line 255
```

CMake `set()` without `PARENT_SCOPE` is **directory-scoped**. `add_subdirectory` creates a new scope, so line 255 modifies `CMAKE_CXX_FLAGS` for `ext/basalt/**` only. `basalt_slam_node`, declared in the parent directory, never sees it. The generated build files confirm this exactly:

```
# build/slam/ext/basalt/CMakeFiles/basalt.dir/flags.make
CXX_FLAGS = -Wall -Wextra -Werror … -DEIGEN_DONT_PARALLELIZE -march=native -Wall -O3 -std=c++17 -O3 -g …

# build/slam/CMakeFiles/basalt_slam_node.dir/flags.make
CXX_FLAGS =  -Wall -O3 -std=c++17 -O2 -g -DNDEBUG                    ← no -march
```

Nobody wrote a bug. The scoping rule did it silently.

### 4.2 The affected type

`basalt::Controller` (`ext/basalt/include/basalt/controller.h:71`) holds a `Calibration<double>` **by value**:

```cpp
basalt::VioConfig vio_config_;
basalt::Calibration<double> calib_;        // ← by value, not a pointer
```

`Calibration<Scalar>` in turn holds, by value (`basalt-headers/include/basalt/calibration/calibration.hpp:128-131`):

```cpp
CalibAccelBias<Scalar> calib_accel_bias;   // Eigen::Matrix<Scalar,9,1>  → 72 B, align 8 either way
CalibGyroBias<Scalar>  calib_gyro_bias;    // Eigen::Matrix<Scalar,12,1> → 96 B, align 16 vs 32  ← the culprit
```

### 4.3 The measured divergence

Both objects were compiled with `-g`, so the disagreement is recorded in their DWARF and can be read directly out of the build tree — no guessing:

```bash
objdump --dwarf=info build/slam/ext/basalt/CMakeFiles/basalt.dir/src/controller.cpp.o | grep -A6 'DW_AT_name.*: Controller$'
objdump --dwarf=info build/slam/CMakeFiles/basalt_slam_node.dir/src/basalt/slam.cpp.o  | grep -A6 'DW_AT_name.*: Controller$'
```

| Type | in `libbasalt.so` (`-march=native`) | in `basalt_slam_node` (no `-march`) |
|---|---|---|
| `CalibGyroBias<double>` | 96 B | 96 B |
| `Calibration<double>` | **416 B** | **384 B** |
| `basalt::Controller` | **1056 B**, `DW_AT_alignment` **32** | **1008 B**, `DW_AT_alignment` **16** |

### 4.4 How that becomes a heap overflow

At `src/basalt/slam.cpp:26`:

```cpp
mpController = std::make_unique<basalt::Controller>(
    mpConfigurationFilePath, mpCalibrationFilePath, mpSlamMode);
```

- `make_unique` is a template, **inlined into `slam.cpp.o`** — the non-AVX translation unit. It calls `operator new(1008)`.
- `Controller::Controller(...)` is defined in `ext/basalt/src/controller.cpp`, i.e. **inside `libbasalt.so`** — the AVX-512 translation unit. It initialises a **1056-byte** object.
- The constructor therefore writes **48 bytes past the end of the allocation**, straight into glibc's adjacent chunk header.
- Independently, the object is only 16-byte aligned but the library's code assumes 32 — so any aligned 256-bit access to `calib_gyro_bias` is also undefined behaviour in its own right.

Neither the compiler nor the linker can see this: each TU is internally consistent, and the mangled symbol names are identical because the *type* is nominally identical. This is a textbook ODR violation, and the standard's remedy for ODR violations is "no diagnostic required".

### 4.5 Why the crash surfaced inside `ld.so`

Heap corruption is a *delayed-fault* bug: the damage is done at time T, the crash happens at time T+n when some later allocation touches the damaged metadata.

Here `on_activate` constructs the `Controller` (line 93), corrupting the arena, and four lines later does:

```cpp
mpTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(
    tf2_ros::TransformBroadcaster(this));                      // node.cpp:97-98
```

Creating that `/tf` publisher is the process's **first** use of `tf2_msgs`, so `rosidl_typesupport_cpp` `dlopen`s `libtf2_msgs__rosidl_typesupport_fastrtps_cpp.so`. `dlopen` is one of the most allocation-heavy operations in a process — link maps, scope arrays, dependency lists, path strings — so it is overwhelmingly likely to be the first thing to walk the corrupted free list. Hence a backtrace whose top eleven frames are all inside `/lib64/ld-linux-x86-64.so.2` and whose only application frame is an innocent `TransformBroadcaster` constructor.

**The lesson to carry forward: when a crash lands inside `ld.so`, `malloc`, `free`, or `operator new`, the reported location is almost never the bug. Those functions are the *detectors*, not the cause.**

### 4.6 The second boundary: `Sophus::SE3d`

The same divergence applies to anything else crossing the library boundary. `Sophus::SE3d` contains a `Quaterniond` (32 B, fixed-size vectorizable) so it is 16-byte aligned in the node and 32-byte aligned in basalt. `src/basalt/slam.cpp:50` constructs `Sophus::SE3d()` in the node's TU and passes it by const reference into `Controller::initialize()`, which will read it with the library's alignment assumptions. `Sophus::SE3f` happens to be safe (its `Quaternionf` is 16 B, aligned 16 either way) — which is exactly the kind of accidental survival that makes these bugs intermittent.

---

## Part V — How to debug this class of problem

### 5.1 Read the backtrace for *responsibility*, not location

Split the stack into three zones:

1. **The reporter** — `ld.so`, `malloc`, `free`, `_int_malloc`, `operator new`. If the fault is here, treat the location as meaningless and look at *what ran before*.
2. **The bridge** — library code doing something ordinary (`rcl_publisher_init`, `SharedLibrary::SharedLibrary`). Rarely the culprit.
3. **The last application frame** — `BasaltSLAMNode::on_activate at node.cpp:98`. Your entry point into the investigation, but the question to ask is *"what did we do just before this?"*, not *"what is wrong with this line?"*

In our log the answer to that question was on the two lines directly above: we constructed a large object from a vendored library.

### 5.2 Tell-tales that say "heap corruption"

- The fault is inside the allocator or the loader, with no application frame nearby.
- The crash address is a plausible pointer, not `0x0` or `0x8` (a null-deref usually has a small offset).
- Moving unrelated code changes whether it crashes, or moves the crash somewhere else.
- `free(): invalid next size`, `malloc(): corrupted top size`, `double free or corruption` — glibc's own diagnostics, which are the same failure caught a little earlier.
- It reproduces deterministically at the same *logical* point (here: always the first `dlopen` after `Controller` construction) even though the faulting instruction looks random.

### 5.3 Dynamic tools, in the order I would reach for them

**1. glibc's built-in checks — zero setup, try first.**

```bash
MALLOC_CHECK_=3 MALLOC_PERTURB_=42 ros2 run slam basalt_slam_node
```
`MALLOC_CHECK_=3` enables consistency checks and aborts on the spot; `MALLOC_PERTURB_` fills freed/allocated memory with a byte pattern so use-after-free and reads of uninitialised heap fail loudly. This often converts "crash somewhere in `ld.so`" into "abort inside the function that actually did the damage".

**2. AddressSanitizer — the right tool for this bug.** ASan instruments memory accesses and places poisoned redzones around every heap allocation, so a 48-byte overrun is reported **at the instruction that writes it**, with the allocation site attached — precisely the information the raw backtrace was missing.

```bash
colcon build --packages-select slam \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
               -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -g" \
               -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address"
ASAN_OPTIONS=detect_leaks=0:abort_on_error=1 ros2 launch slam basalt_slam.launch.py
```
Two caveats for a ROS 2 node: ASan must be first in the link order (build the executable with it, or use `LD_PRELOAD=$(gcc -print-file-name=libasan.so)`), and you will want `detect_leaks=0` initially because ROS middleware reports a lot of benign one-time allocations. Expect roughly 2× slowdown and ~3× memory.

*Important caveat for this particular bug:* ASan finds the overflow but will point at the `Controller` constructor and the `make_unique` allocation site. It tells you *that* the sizes disagree, not *why*. The "why" comes from §5.4.

**3. Valgrind/Memcheck** when you cannot rebuild. No recompilation needed, catches invalid writes, but ~20–50× slower — usually too slow for a live SLAM node consuming a camera stream, though fine for a replay of a recorded bag.

**4. `gdb` with a watchpoint** once you suspect a specific address: break after the allocation, `watch -l *(char*)(ptr+1008)`, continue. Precise, but requires you to already know where to look.

### 5.4 Static / build-level verification — how this one was actually solved

Dynamic tools tell you memory was corrupted. Build-level inspection tells you *why the two halves disagreed*, and for an ABI mismatch it is faster and more conclusive. This is the sequence that cracked it, and it generalises:

**a. Compare the actual compile flags of the two sides.** CMake writes them verbatim:

```bash
grep -E "^CXX_FLAGS|^CXX_DEFINES" build/<pkg>/CMakeFiles/<target>.dir/flags.make
grep -E "^CXX_FLAGS|^CXX_DEFINES" build/<pkg>/<subdir>/CMakeFiles/<lib>.dir/flags.make
```
Or, with `-DCMAKE_EXPORT_COMPILE_COMMANDS=1` (already set in `scripts/build_ros_packages.sh:22`), diff the entries in `compile_commands.json`. Look for `-march`, `-mavx*`, `-std`, `-D_GLIBCXX_USE_CXX11_ABI`, `-DNDEBUG`, and any `-D` that a header might branch on. **Any difference in a macro that a shared header reads is a suspect.**

**b. Confirm the flags reached the machine code.**

```bash
for r in xmm ymm zmm; do echo -n "$r: "; objdump -d <lib-or-object> | grep -c "%$r"; done
```
Wildly different `ymm`/`zmm` counts between two objects of the same program is the fingerprint.

**c. Ask the debug info for the layout — the decisive test.** If both sides were built with `-g` (RelWithDebInfo, as here), the compiler has already recorded its opinion of every type:

```bash
objdump --dwarf=info <object.o> | grep -A6 'DW_AT_name.*: <TypeName>$' | grep -E "byte_size|alignment"
```
Run it against one object from each side and compare `DW_AT_byte_size` and `DW_AT_alignment`. **If they differ, you have proven an ODR violation, and there is nothing left to argue about.** This took under a minute and is the single highest-value technique in this document. (`pahole -C <TypeName> <binary>` gives the same answer with a nicer per-member layout dump if it is installed.)

**d. Reproduce the layout divergence in isolation.** A ~20-line program that includes the same headers and prints `sizeof`/`alignof`/`offsetof`, compiled twice with the two flag sets, converts a hypothesis into a demonstration:

```cpp
#include <Eigen/Core>
#include <cstdio>
#include <cstddef>
struct CalibGyroBias { Eigen::Matrix<double,12,1> gyro_bias_full_; };
struct CalibAccelBias { Eigen::Matrix<double, 9,1> accel_bias_full_; };
struct Calib { long cam_time_offset_ns; CalibAccelBias a; CalibGyroBias g; double rate; };
struct Controller { void* vio; Calib calib_; void* more; };
int main(){
  printf("EIGEN_MAX_ALIGN_BYTES = %d\n", (int)EIGEN_MAX_ALIGN_BYTES);
  printf("sizeof/alignof(Calib)      = %zu / %zu\n", sizeof(Calib), alignof(Calib));
  printf("sizeof/alignof(Controller) = %zu / %zu\n", sizeof(Controller), alignof(Controller));
  printf("offsetof(Controller, more) = %zu\n", offsetof(Controller,more));
}
```
```bash
g++ -std=c++17 -O3            -I<eigen> abi.cpp -o abi_plain  && ./abi_plain
g++ -std=c++17 -O3 -march=native -I<eigen> abi.cpp -o abi_native && ./abi_native
```

**e. Interrogate the toolchain directly.**

```bash
gcc -march=native -Q --help=target | grep enabled     # what did `native` resolve to?
diff <(gcc -dM -E -x c++ /dev/null) <(gcc -march=native -dM -E -x c++ /dev/null)   # which macros changed?
```

### 5.5 A 10-minute triage checklist

When a C++/Eigen/ROS process crashes somewhere that makes no sense:

1. Is the faulting frame in `ld.so` / `malloc` / `free` / `operator new`? → suspect heap corruption; ignore the reported location.
2. What ran immediately before, in the last application frame? Did it construct or destroy an object from a *different* library or subproject?
3. `grep CXX_FLAGS` for both sides. Do `-march`, `-std`, `-D` sets match?
4. `objdump | grep -c ymm` on one object from each side. Do they agree?
5. `objdump --dwarf=info | grep byte_size/alignment` for the shared type on each side. Do they agree?
6. If yes to all: re-run under `MALLOC_CHECK_=3`, then ASan, and treat it as an ordinary memory bug.

### 5.6 Preventing recurrence

- **Set the ABI-relevant macros explicitly** at the top-level build rather than letting each subproject default them (Part VI).
- **Assert the contract in code.** A `static_assert(sizeof(basalt::Controller) == N)` is brittle, but a header-level `static_assert(EIGEN_MAX_ALIGN_BYTES == 16, "ABI mismatch: see context/SIMD.md")` included by both the library and its consumers is cheap and catches the mismatch **at compile time, in every TU**, which is exactly where it should be caught.
- **Never let `-march=native` leak into a distributed artefact.** Pin a microarchitecture level.
- **Keep `-g` on release builds** (`RelWithDebInfo` already does). The DWARF is what made this diagnosable in minutes.

---

## Part VI — Choosing optimal build flags

### 6.1 The framework: who consumes this binary?

| Scenario | `-march` choice | Reasoning |
|---|---|---|
| Throwaway benchmark on one machine | `native` | Maximum speed, artefact never moves. Basalt's default assumes this. |
| Artefact in a mounted/persistent tree, or a Docker image, or CI | A **microarchitecture level** (`x86-64-v3`) | Reproducible, portable across the fleet, still gets AVX2 + FMA. |
| Multi-arch (`buildx --platform linux/amd64,linux/arm64`) | **Per-architecture** value; never `native` | `native` is meaningless when the compiler is not running on the target (§6.6). |
| Shipping to unknown hardware | `x86-64-v2` or baseline | Correctness over speed. |

For this repository the answer is the second and third rows: `install/` is mounted and persistent, and `ARCHITECTURE.md:6` commits to an arm64 + amd64 matrix with a Jetson Orin Nano target.

### 6.2 Recommended settings

**The load-bearing change — decouple ABI from `-march`:** define `EIGEN_MAX_ALIGN_BYTES=16` for *both* the basalt subdirectory and the parent `slam` targets.

Why 16 and not 32: every other ROS 2 package in the system — `rclcpp`, `tf2_eigen`, `cv_bridge`, `pcl_conversions`, and the Debian `libeigen3-dev` consumers — is built by the distribution with **no** `-march`, hence `EIGEN_MAX_ALIGN_BYTES = 16`. Choosing 16 makes basalt agree with the entire rest of the process, not merely with `basalt_slam_node`. Choosing 32 would fix the internal boundary while leaving the same latent hazard against every ROS package that shares an Eigen type.

What it costs: Eigen keeps vectorizing under AVX/AVX-512, but uses **unaligned** loads/stores (`vmovupd` instead of `vmovapd`) for over-16-byte packets. On any core from Haswell onward the penalty for unaligned access to data that is naturally well-placed is small — typically low single-digit percent on this kind of workload. That is the price of a coherent ABI, and it is worth measuring rather than assuming (§6.5).

**The second change — make `CXX_MARCH` explicit and per-architecture**, instead of inheriting `native` from `ext/basalt/CMakeLists.txt:75-76`:

| Target | Suggested `CXX_MARCH` | Notes |
|---|---|---|
| amd64 (dev, CI, Docker) | `x86-64-v3` | AVX2 + FMA; runs on everything from Haswell (2013) / Zen 1 onward. Matches upstream basalt's `haswell` choice for packaged builds. |
| arm64 (Jetson Orin Nano) | `armv8.2-a+fp16+dotprod` | Cortex-A78AE feature set. **`x86-64-v3` or `haswell` would be an immediate hard error here** — the value must be selected per architecture. |

Add `-mtune=native` on developer machines if you want host-specific scheduling without host-specific *instructions*; it is ABI-neutral and safe.

### 6.3 Why not simply propagate `-march=native` to the node

It is the smaller diff, and it is the wrong fix:

- It makes the executable non-portable in a workspace whose `install/` tree is explicitly designed to be mounted and reused.
- It **inverts** the problem at the next boundary: the node also links `tf2_eigen`, `cv_bridge` and other Eigen-using ROS packages built without AVX. Fixing the basalt boundary this way opens the ROS boundary.
- It leaves the ABI dependent on whichever machine ran the compiler — the property that caused the bug.

### 6.4 Why pinning `EIGEN_MAX_ALIGN_BYTES` is the change that matters

Choosing a good `CXX_MARCH` fixes *this* occurrence. Pinning `EIGEN_MAX_ALIGN_BYTES` makes the *class* of bug impossible: the layout no longer depends on the arch flag at all, so it survives someone adding a subproject with different flags, a QEMU-emulated build resolving `native` differently, or a future CMake refactor that changes scope boundaries. Do both, but understand that the second is the structural fix and the first is hygiene.

### 6.5 Measuring, not guessing

Before accepting any performance claim in §6.2, measure on a fixed input:

- Build three variants — `native`, `x86-64-v3`, and `x86-64-v3 -DEIGEN_MAX_ALIGN_BYTES=16` — and run basalt's own dataset benchmarks (`ext/basalt/scripts/eval_full/`, plus the EuRoC/TUM-VI sequences already downloaded per `ext/basalt/context/README.md`).
- Compare wall-clock per frame and the VIO/BA stage timings, not just total runtime.
- Expect trajectory output to differ in the last digits between `-march` levels (FMA contraction, §1.5). Compare against ground truth with an ATE/RPE metric, not by diffing files.

### 6.6 Cross-compilation and the multi-arch matrix

`-march=native` is resolved from the machine **running the compiler**, which makes it unusable the moment build host ≠ target:

- **With a real cross toolchain** (`aarch64-linux-gnu-g++` on x86): the compiler cannot query a foreign target, and rejects `native` outright. A hard build failure — at least it is loud.
- **Under `docker buildx --platform linux/arm64` on an amd64 host**: the aarch64 toolchain typically runs under QEMU user-mode emulation. AArch64 GCC resolves `native` by reading CPU identification from `/proc/cpuinfo`; under QEMU that does not map to a recognised part, so GCC warns and silently falls back to the generic baseline. You get an untuned `armv8-a` build and no error. This is the worse outcome, because nothing tells you.
- **`ext/basalt/CMakeLists.txt:79`** guards the flag with `IF(NOT APPLE OR NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")`, which only suppresses it on Apple Silicon. On Linux/aarch64 the flag is still emitted — so a single global `CXX_MARCH` value cannot serve both legs of the buildx matrix. It must be chosen from `CMAKE_SYSTEM_PROCESSOR`.
- Because AArch64 caps Eigen at 16 bytes regardless (§1.7), pinning `EIGEN_MAX_ALIGN_BYTES=16` also has the pleasant property of making the **x86 and ARM builds share one layout**, which removes an entire category of "works on the Jetson, crashes on the workstation" surprises.

---

## Part VII — Quick reference

**Register widths**

| | `xmm` | `ymm` | `zmm` | NEON `v` |
|---|---|---|---|---|
| Width | 128 b | 256 b | 512 b | 128 b |
| doubles / floats | 2 / 4 | 4 / 8 | 8 / 16 | 2 / 4 |
| ISA | SSE2 | AVX/AVX2 | AVX-512 | AArch64 baseline |

**`EIGEN_MAX_ALIGN_BYTES` by ISA** (measured, Eigen 3.4): none/SSE2 → 16; AVX or AVX2+FMA → 32; full AVX-512 (F+DQ+BW+VL+CD) → 64; AArch64/NEON → 16.
**Effective alignment** = `min(EIGEN_MAX_ALIGN_BYTES, largest power of two dividing sizeof(T))`.

**Diagnostic one-liners**

```bash
grep -E "^CXX_FLAGS" build/<pkg>/CMakeFiles/<target>.dir/flags.make      # what flags were used
objdump -d <obj> | grep -c '%zmm'                                        # was AVX-512 enabled
objdump --dwarf=info <obj> | grep -A6 'DW_AT_name.*: <Type>$'            # size + alignment of a type
gcc -march=native -Q --help=target | grep enabled                        # what did `native` resolve to
diff <(gcc -dM -E -x c++ /dev/null) <(gcc -march=native -dM -E -x c++ /dev/null)   # which macros changed
MALLOC_CHECK_=3 MALLOC_PERTURB_=42 ./prog                                # cheap heap-corruption check
```

---

## References

**Eigen**
- [Eigen 3.4 — Preprocessor directives](https://libeigen.gitlab.io/eigen/docs-3.4/TopicPreprocessorDirectives.html) — normative definitions of `EIGEN_MAX_ALIGN_BYTES`, `EIGEN_MAX_STATIC_ALIGN_BYTES`, `EIGEN_DONT_VECTORIZE`, `EIGEN_UNALIGNED_VECTORIZE`.
- [Eigen issue #1331 — "Alignment on 16 bytes, whereas AVX requires 32?"](https://gitlab.com/libeigen/eigen/-/issues/1331) and [issue #2982 — "AVX Alignment Issue"](https://gitlab.com/libeigen/eigen/-/issues/2982) — upstream discussion of exactly this mismatch.
- [eigen mailing list — "eigen mis-compiling with native arch"](https://www.mail-archive.com/eigen@lists.tuxfamily.org/msg00627.html) — the `-march=native`-across-TUs failure mode.
- [FEniCS/dolfinx PR #143 — "Control Eigen alignment for compatibility with AVX user code"](https://github.com/FEniCS/dolfinx/pull/143) — another project adopting the pin-`EIGEN_MAX_ALIGN_BYTES` remedy.
- [PCL issue #1791](https://github.com/PointCloudLibrary/pcl/issues/1791) and [issue #4587](https://github.com/PointCloudLibrary/pcl/issues/4587) — the same class of bug in the PCL/ROS ecosystem.
- [MoveIt PR #1382 — "Fix alignment of Eigen transforms"](https://github.com/moveit/moveit/pull/1382) and [ROS Answers — "moveit segfaults due to eigen"](https://answers.ros.org/question/317202/moveit-segfaults-due-to-eigen) — prior art from ROS itself.

**Compiler and ISA**
- [GCC — x86 Options](https://gcc.gnu.org/onlinedocs/gcc/x86-Options.html) — `-march`, `-mtune`, `native`, and the `x86-64-v2/v3/v4` levels.
- [MaskRay — `-march=`, `-mcpu=`, and `-mtune=`](https://maskray.me/blog/2022-08-28-march-mcpu-mtune) — the clearest explanation of the three flags across architectures.
- [Phoronix — GCC 11's x86-64 microarchitecture feature levels](https://www.phoronix.com/news/GCC-11-x86-64-Feature-Levels) — background on the psABI level agreement.
- [Lindevs — Compiling for a specific x86-64 microarchitecture level](https://lindevs.com/compile-for-specific-x86-64-microarchitecture-level-using-gcc-or-g-compiler) — practical usage.
- [HenrikBengtsson/x86-64-level](https://github.com/HenrikBengtsson/x86-64-level) — script to report a machine's supported level, useful for fleet auditing.

**Debugging**
- [AddressSanitizer wiki (google/sanitizers)](https://github.com/google/sanitizers/wiki/AddressSanitizer) — usage and what it detects.
- [Red Hat Developer — Memory error checking in C and C++: comparing sanitizers and Valgrind](https://developers.redhat.com/blog/2021/05/05/memory-error-checking-in-c-and-c-comparing-sanitizers-and-valgrind) — choosing between the tools.
- [Undo — A quick introduction to Valgrind and AddressSanitizer](https://undo.io/resources/gdb-watchpoint/a-quick-introduction-to-using-valgrind-and-addresssanitizer/).
- [Wikibooks — Linux Applications Debugging Techniques / Heap corruption](https://en.wikibooks.org/wiki/Linux_Applications_Debugging_Techniques/Heap_corruption) — `MALLOC_CHECK_`, `MALLOC_PERTURB_` and friends.
- [Heap corruption crashes: how to diagnose and fix them](https://dev.to/legacycpp/s2-heap-corruption-crashes-how-to-diagnose-and-fix-them-5gll) — the delayed-fault mental model.

**In-repo**
- `ext/basalt/CMakeLists.txt:75-80, 255` — `CXX_MARCH` default and flag composition.
- `ext/basalt/.gitlab-ci.yml:6, 158, 170, 183` — upstream's `native` (dev) vs `haswell` (packaged) split.
- `CMakeLists.txt:159-177` — `add_subdirectory(ext/basalt)` and the scope boundary.
- `/ws/ros_ws/context/simd-eigen-abi-mismatch.md` — condensed investigation record.
- `/ws/plans/basalt-slam-defect-remediation.md` — the remediation plan.
