# frame3ddn
 * Nuget is not maintained (I've lost my login).
![Build Status](https://github.com/ssippe/frame3ddn/workflows/.NET/badge.svg)

Incomplete c# implementation of frame3dd. 

# Original Source
* http://frame3dd.sourceforge.net/
* https://github.com/pslack/frame3dd

# Documentation
* [Original](http://svn.code.sourceforge.net/p/frame3dd/code/trunk/doc/Frame3DD-manual.html)
* [Backup](/docs/originalWebDocumentation/Frame3DD%20User%20Manual.html)

# Units
This project will have comments that reflect the default units: Newtons, millimeters, and tonnes. Other units are supported. See the documentation for details.


# Feature Comparison to Frame3dd C++ Version

Upstream is actually written in C (`pslack/frame3dd`) — "C++ version" here means the original
native build maintained at https://github.com/pslack/frame3dd. The table below compares this C#
port (`Frame3ddn`) to upstream and, where relevant, to Microstran (the commercial program whose
file formats this port also supports).

### Input file formats

| Format | Upstream `frame3dd` | Frame3ddn (C#) |
|---|---|---|
| `.csv` (frame3dd CSV) | ✅ | ✅ `CsvInputParser` |
| `.3dd` (legacy whitespace-delimited) | ✅ | ✅ `ThreeDdInputParser` |
| `.arc` (Microstran) | ✅ parser exists in `microstran/` but is *not* wired to the solver | ✅ `ArcInputParser` — fully wired through `Solver` |

### Output file formats

| Format | Upstream `frame3dd` | Frame3ddn (C#) |
|---|---|---|
| `.out` (text result) — write | ✅ | ✅ `OutWriter` (static + modal sections) |
| `.out` — read | ❌ | ✅ `OutOutputParser` (load cases + modal — used by tests for cross-validation) |
| `_out.CSV` (CSV result) — write | ✅ | ❌ |
| `_out.CSV` — read | ❌ | ✅ `CsvOutputParser` |
| `.m` (Matlab script) | ✅ | ❌ |
| Microstran `.p1` — read | ❌ | ✅ `P1OutputParser` (used by tests for round-trip checks) |

### Static load cases

| Feature | Upstream | Frame3ddn |
|---|---|---|
| Concentrated nodal loads | ✅ | ✅ |
| Uniformly-distributed element loads | ✅ | ✅ |
| Trapezoidally-distributed element loads | ✅ | ✅ |
| Concentrated interior point loads | ✅ | ✅ |
| Temperature loads | ✅ | ✅ |
| Prescribed displacements at restrained DoFs | ✅ | ✅ |
| Self-weight gravity loading | ✅ | ✅ |

### Computed output

| Field | Upstream | Frame3ddn |
|---|---|---|
| Node displacements (global) | ✅ | ✅ |
| Frame element end forces (local) | ✅ | ✅ |
| Support reactions (global) | ✅ | ✅ |
| Peak frame element internal forces (local) | ✅ | ✅ |
| Natural frequencies (Hz) | ✅ | ✅ `Output.ModalResults[i].FrequencyHz` |
| Mass-normalised mode shapes | ✅ | ✅ `Output.ModalResults[i].ModeShape` (length = 6·nN) |

### Analysis options

| Feature | Upstream | Frame3ddn | Notes |
|---|---|---|---|
| Shear deformation | ✅ | ✅ | Toggled via `IncludeShearDeformation` |
| Geometric stiffness (P-Δ second-order) | ✅ | ✅ | Quasi Newton-Raphson loop, mirrors upstream `geometric_K`. Modal analysis uses the static-loop's converged K so axial-force softening from gravity loading propagates into the modal frequencies, matching upstream `main.c`. |
| Large-displacement / co-rotational kinematics | ❌ | ❌ | Neither program updates nodal coordinates between iterations. Flexural shortening (the `lateral-column.p1` Z-disp residual) is a **Microstran** option, not a frame3dd one. |
| Dynamic modal analysis — Subspace–Jacobi iteration | ✅ | ✅ `Eigensolver.Subspace` (default — every shipped upstream example uses this) |
| Dynamic modal analysis — Stodola inverse iteration | ✅ | ✅ `Eigensolver.Stodola` (selected via `DynamicAnalysisInput.Method == 2`) |
| Extra node inertia / element mass | ✅ | ✅ `DynamicAnalysisInput.ExtraNodeInertia` / `ExtraElementMass` |
| Mode-shape animation data | ✅ | ⚠️ parsed but not rendered (`DynamicAnalysisInput.AnimatedModes`, `PanRate`) |
| Matrix condensation (static / dynamic / Guyan) | ✅ | ⚠️ parsed but not solved (`DynamicAnalysisInput.CondensationMethod` etc.) |
| Plotting (gnuplot mesh / saveplot scripts) | ✅ | ❌ |

# How to use
* Install `Frame3ddn` from NuGet
* Read an input file with the parser that matches its format:
  ```csharp
  using StreamReader sr = new StreamReader("model.csv");
  Input input = CsvInputParser.Parse(sr);              // frame3dd .csv
  // or:  Input input = ThreeDdInputParser.Parse(sr);  // frame3dd .3dd
  // or:  Input input = ArcInputParser.Parse(sr);      // Microstran .arc
  ```
  Or build an `Input` from code:
  ```csharp
  Input input = new Input(title, nodes, frameElements, reactionInputs, loadCases,
      includeShearDeformation: false, includeGeometricStiffness: false,
      exaggerateMeshDeformations: 10, zoomScale: 1, xAxisIncrementForInternalForces: 100);
  ```
  (Sample inputs live under `src/Frame3ddn.Test/TestData/frame3dd-examples/` — the upstream
  `exA`–`exJ` examples — or on the frame3dd website.)
* Solve:
  ```csharp
  Output output = new Solver().Solve(input);
  ```
* Read structured results from:
  * `output.LoadCaseOutputs` — per-load-case displacements, end forces, reactions, peak internal forces
  * `output.ModalResults` — natural frequencies and mass-normalised mode shapes (empty when
    the input does not request modal analysis or sets `ModesCount = 0`)
  * `output.TextOutput` — rendered upstream-style `.out` text (load cases + modal section)

# Test coverage
The test suite (`Frame3ddn.Test`) runs all 10 upstream `exA`–`exJ` examples through the solver
in three configurations and compares the result against the corresponding reference output:

| Test | Input format | Reference format | What's checked |
|---|---|---|---|
| `Frame3ddExamplesCsv`        | `.csv`                            | `_out.CSV` | static (modal not in CSV) |
| `Frame3ddExamples3dd`        | `.3dd`                            | `.out`     | static + modal frequencies (1% rel.tol) |
| `TestCases2018`              | NT-derivative `.csv` (historical) | `.out`     | static (no modal in fixtures) |

Plus dedicated tests for the modal pipeline:

| Test | What it pins |
|---|---|
| `OutParserModalTest.Frame3ddExamples_LowestModeMatchesUpstream` | Lowest mode of each non-rigid example matches upstream within 1% |
| `OutParserModalTest.ParseModalResults_RoundTripsThroughOutWriter` | Writer/parser round-trip preserves frequency and mode-shape data |
| `EigensolverTest`                                                | LDLᵀ, Jacobi, Subspace, Stodola on hand-verifiable systems |
| `MassMatrixTest` / `MassMatrixDiagnosticTest`                    | Mass-matrix assembly matches upstream (full-matrix dump cross-validation) |

Plus parser-level tests in `CsvParserTest`, `ArcParserTest`, and `DynamicAnalysisInputTest`,
and a Microstran-reference comparison `ArcVsP1Test` for the lateral-column variants.
 


