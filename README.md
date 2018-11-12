# frame3ddn
[![NuGet version (Frame3ddn)](https://img.shields.io/nuget/v/Frame3ddn.svg?style=flat-square)](https://www.nuget.org/packages/Frame3ddn/)

Incomplete c# implementation of frame3dd. 


* http://frame3dd.sourceforge.net/
* https://github.com/pslack/frame3dd

# Features
- Input file format:
  - [X] CSV
  - [ ] 3DD
- Static Load case:
  - [ ] Node loads
  - [X] Uniformly-distributed element loads
  - [X] Trapezoidally-distributed element loads
  - [ ] Concentrated interior point loads
  - [ ] Temperature loads
  - [ ] Prescribed displacements.
- Output
  - [X] Node displacements (global)
  - [X] Frame element end forces (local)
  - [X] Reactions (global)
  - [X] Peak frame element internal forces (local)
- Others
  - [ ] Dynamic modal analysis
  - [ ] Extra node inertia
  - [ ] Extra frame element mass
  - [ ] Mode shape animation
  - [ ] Matrix condensation
  - [ ] Plotting results.
 
 # How to use
 * Install frame3ddn from NuGet
 * Create your own input or get it from a file
    - Input input = new Input(title, nodes, frameElements, reactionInputs, loadCases, false, false, 10, 1, 100);
    - StreamReader sr = new StreamReader(//Your File or file address); Input input = Input.Parse(sr);
 * Solver solver = new Solver();
 * Output output = solver.Solve(input);
 * Directly use the result from output.LoadCaseOutputs in your code or get its text output from output.TextOutput.
 
### Status
[![Build Status](https://travis-ci.org/ssippe/frame3ddn.png)](https://travis-ci.org/ssippe/frame3ddn)
