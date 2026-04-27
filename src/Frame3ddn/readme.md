# frame3ddn
[![NuGet version (Frame3ddn)](https://img.shields.io/nuget/v/Frame3ddn.svg?style=flat-square)](https://www.nuget.org/packages/Frame3ddn/)
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
| `.csv` (frame3dd CSV) | ✅ | ✅ `CsvParser` |
| `.3dd` (legacy whitespace-delimited) | ✅ | ✅ `ThreeDdParser` |
| `.arc` (Microstran) | ✅ parser exists in `microstran/` but is *not* wired to the solver | ✅ `ArcParser` — fully wired through `Solver` |

### Output file formats

| Format | Upstream `frame3dd` | Frame3ddn (C#) |
|---|---|---|
| `.out` (text result) — write | ✅ | ✅ `OutWriter` |
| `.out` — read | ❌ | ✅ `OutParser` (used by tests for round-trip checks) |
| `_out.CSV` (CSV result) — write | ✅ | ❌ |
| `_out.CSV` — read | ❌ | ✅ `OutCsvParser` |
| `.m` (Matlab script) | ✅ | ❌ |
| Microstran `.p1` — read | ❌ | ✅ `P1Parser` (used by tests for round-trip checks) |

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

### Analysis options

| Feature | Upstream | Frame3ddn | Notes |
|---|---|---|---|
| Shear deformation | ✅ | ✅ | Toggled via `IncludeShearDeformation` |
| Geometric stiffness (P-Δ second-order) | ✅ | ✅ | Quasi Newton-Raphson loop, mirrors upstream `geometric_K` |
| Large-displacement / co-rotational kinematics | ❌ | ❌ | Neither program updates nodal coordinates between iterations. Flexural shortening (the `lateral-column.p1` Z-disp residual) is a **Microstran** option, not a frame3dd one. |
| Dynamic modal analysis | ✅ subspace iteration / Stodola | ⚠️ partial — input parses (consume-and-skip) but the modal solver is not implemented |
| Extra node inertia / element mass | ✅ | ❌ |
| Mode-shape animation data | ✅ | ❌ |
| Matrix condensation (static / dynamic / Guyan) | ✅ | ❌ |
| Plotting (gnuplot mesh / saveplot scripts) | ✅ | ❌ |

# How to use
* Install `Frame3ddn` from NuGet
* Read an input file with the parser that matches its format:
  ```csharp
  using StreamReader sr = new StreamReader("model.csv");
  Input input = CsvParser.Parse(sr);              // frame3dd .csv
  // or:  Input input = ThreeDdParser.Parse(sr);  // frame3dd .3dd
  // or:  Input input = ArcParser.Parse(sr);      // Microstran .arc
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
* Read structured results from `output.LoadCaseOutputs` or rendered text from `output.TextOutput`.

# Test coverage
The test suite (`Frame3ddn.Test`) runs all 10 upstream `exA`–`exJ` examples through the solver
in three configurations and compares the result against the corresponding reference output:

| Test | Input format | Reference format |
|---|---|---|
| `Frame3ddExamplesCsv` | `.csv` | `_out.CSV` |
| `Frame3ddExamples3dd` | `.3dd` | `.out` |
| `TestCases2018`       | NT-derivative `.csv` (historical) | `.out` |

Plus parser-level tests in `CsvParserTest` and `ArcParserTest`, and a Microstran-reference
comparison `ArcVsP1Test` for the lateral-column variants.
 
 # Steps to publish
 * Update package version and release note in project property.
 * commit and tag in git with the version name
 * Change configuration from debug to release, then right click on the project and pack.
 * Run the following command line from the folder that contains Frame3ddn.[version].nupkg
   >`nuget push Frame3ddn.[version].nupkg [API key] -Source https://api.nuget.org/v3/index.json`
 * It will take around 30 mins. Login https://www.nuget.org/ to check status in Manage Packages.

