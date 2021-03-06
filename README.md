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


# Features
- Input file format:
  - [X] CSV
  - [ ] 3DD
- Static Load case:
  - [X] Node loads
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
  - [ ] Geometric stiffness
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
      
      (You can find examples in frame3ddn/src/Frame3ddn.Test/TestData or frame3dd website)
 * Solver solver = new Solver();
 * Output output = solver.Solve(input);
 * Directly use the result from output.LoadCaseOutputs in your code or get its text output from output.TextOutput.
 
 # Steps to publish
 * Update package version and release note in project property.
 * commit and tag in git with the version name
 * Change configuration from debug to release, then right click on the project and pack.
 * Run the following command line from the folder that contains Frame3ddn.[version].nupkg
   >`nuget push Frame3ddn.[version].nupkg [API key] -Source https://api.nuget.org/v3/index.json`
 * It will take around 30 mins. Login https://www.nuget.org/ to check status in Manage Packages.

