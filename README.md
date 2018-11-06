# !!DO NOT USE!!

# frame3ddn
very incomplete and experimental c# implementation of frame3dd. 


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

### Status
[![Build Status](https://travis-ci.org/ssippe/frame3ddn.png)](https://travis-ci.org/ssippe/frame3ddn)
