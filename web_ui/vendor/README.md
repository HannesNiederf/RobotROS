# Third-party files (offline, spec 13.1)

| File | Source | Version | SHA-256 | License |
|---|---|---|---|---|
| `roslib.min.js` | https://github.com/RobotWebTools/roslibjs, file `build/roslib.min.js` | tag `1.4.1` (commit `d81d899e3e5caf1b7ab1977e8bf33d4a2d2848e6`) | `3c510df2bc04be9d5b07efafb01eb5f473d8a5092d93292351a715ca60daf9d6` | BSD-3-Clause (roslibjs), see below |

The browserify bundle already contains EventEmitter2 (MIT), so no separate `eventemitter2` file is needed
(spec 13.1 lists it separately; ADR 0005). Do not edit the file; to update, replace it and this table.

The roslibjs license text (BSD-3-Clause) is kept next to the file as `roslib.LICENSE`, as the license requires.
