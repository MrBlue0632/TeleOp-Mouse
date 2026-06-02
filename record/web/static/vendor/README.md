# Static WebGL Vendor Files

This dashboard vendors a pinned Three.js runtime so the teleoperation UI can run on a robot LAN without loading browser modules from the public internet.

- `three.module.min.js`: Three.js 0.178.0 from jsDelivr / npm package `three`
- `three.core.min.js`: companion core module imported by `three.module.min.js`
- `controls/OrbitControls.js`: Three.js 0.178.0 examples control module

Three.js is distributed under the MIT license.
