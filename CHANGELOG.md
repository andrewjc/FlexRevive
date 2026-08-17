# Changelog

## Unreleased

* Fixed ForceEnableWeaponDebris appearing to do nothing, so the mod worked only for people who had already set bNVFlexEnable=1 themselves. Every debris mesh the game owns is in Fallout4 - Nvflex.ba2, which is in none of the archive lists in Fallout4.ini; the engine mounts it from code behind a test of bNVFlexEnable that has already run by the time F4SE loads a plugin. Turning the setting on was therefore too late for it, and the game ran with Flex enabled, the solver installed, and no mesh for any chunk to use
* Added f4kit::archive::Mount, which mounts that archive itself when the setting had to be turned on. The routine is reached from the call site that names the archive rather than from an address, and is accepted only if the rest of the image calls it too, so no build-specific offset is involved
* Fixed a blast's falloff always being read as quadratic, because linearFalloff was taken from offset 24 of the force field entry, which is the mode field and is always zero. The flag is a byte at offset 40, and the engine writes it as 1
* Changed the force field entry to be read by named offset, with the layout the engine actually writes written down beside it, since only three of its eleven fields were ever identified
* Changed the force field log to fire when a blast begins rather than on every call, so it described the first explosion of a session four times over and no later one at all
* Added a blast line reporting how many settled pieces a force field disturbed, so a blast that arrives and reaches nothing is distinguishable from one that never arrived
* Changed the wake trace to count blasts separately from movers, which shared a counter and were reported as one

## 1.1.1

* Fixed debris falling through terrain, because the world sweep found the surface and then nothing acted on the result
* Fixed settled debris never quite stopping, because a contact returned a share of the speed gravity added each substep and the no-bounce threshold was never applied
* Fixed debris creeping across surfaces, because friction was always capped against the normal impulse and the static friction threshold was never applied
* Fixed ImpactTorque doing nothing, because the tunable was read from the ini and never used
* Fixed Rolling doing nothing, because the tunable was read from the ini and never used
* Changed the piece-versus-piece pass to honour the engine's sleep threshold, which only the world contact was using
* Changed pairs::Settings to initialise its members, since a caller that missed one got whatever was on the stack
* Added the mesh contact response back, so a piece lands on a triangle mesh the same way it lands on a hull
* Added tests/test_terrain.cpp, dropping and throwing pieces at landscape-sized geometry at the world coordinates the game actually uses
* Fixed the ini file never being read, because the reader was split into f4kit without being given the path
* Added a warning when a setting is read with no ini file set, since silent fallback to defaults reads exactly like a file of defaults
* Fixed debris hanging in mid-air, because a falling chunk counted a neighbouring falling chunk as holding it up and the whole cluster went to sleep
* Changed support to come only from a piece that is itself at rest, so it climbs from the ground upward
* Fixed a settled piece re-seeded somewhere else staying asleep at the position it was handed, because the resting flag was only ever set and never cleared
* Added slots::OnReseed, deciding what happens to a recycled slot in one place
* Added tests/test_scene.cpp, stepping many pieces over time so sleeping, support and the floor are tested together rather than one rule at a time
* Fixed a piece's particles drifting away from it, because they were advanced under their own gravity and nothing stops a particle against the world
* Changed particles to be carried by the piece that owns them, so the buffers the engine reads describe the same debris as the transforms
* Changed the particle carry to run last in the substep, so it sees the displacement neighbours added and the velocity a piece settling on the pile ended with
* Fixed a missing pair of braces leaving the particle slot map built from an unchecked container pointer
* Changed the unused particle owner map to not be built at all, since nothing read it
* Changed particles::BuildOwners and OwnerIsResting to be removed, since walking the pieces answers the same question without inverting the map
* Changed the piece runs to be checked against the engine's own arrays rather than against a second map built from the same data
* Changed three solver status accessors and a derived geometry offset to be removed, none of them called or read
* Added particles::CarryParticle, leaving the fourth component alone since it is inverse mass rather than a coordinate

## 1.1.0

* Fixed settled debris periodically leaping about, because a recycled slot was read as a launch velocity when the value there was accumulated free fall
* Changed container particles to hold still with the piece that owns them, so their velocities stop drifting to terminal velocity
* Added a plausibility bound on spawn velocity, since a genuine launch runs 80 to 270 units/s
* Fixed a whole heap unsettling when you walked into it, because a wake spread through static contact with nothing pushing
* Changed a contact to wake a sleeping neighbour only when one of the pair is actually moving
* Fixed pieces flying off when walked into, because the collider's velocity was scaled on the way out of its frame but not on the way in
* Changed how far a chunk is carried when walked into to fall off with its mass
* Fixed pair damping injecting momentum, because it pulled both chunks toward their plain average instead of their centre of mass
* Fixed a pair being resolved twice in a step when two of a piece's neighbouring cells shared a hash bucket
* Fixed a crash if a caller asked the triangle test for a hit without a normal
* Changed SettleRate to 1.5 because a heap kept wiggling after it looked still
* Changed SpawnVelocityScale to 0.3 because chunks flew too far off the surface they came from
* Added SpawnVelocityScale, separating how hard the gun throws new debris from how hard it disturbs old debris
* Changed the contact solver to rotate the direction into the chunk's frame rather than 26 points into the world, about four times faster
* Changed the leaf vector maths to be inlined, since crossing a translation unit cost more than the work
* Added link-time code generation to release builds
* Added unit tests: 11 suites over the maths, collision, contacts, mass response, pair contacts, broadphase, slot tracking and particle ownership
* Added src/f4kit, a reusable plugin layer: import patching, PE scanning, engine settings, crash reporting, logging, INI reading, threads
* Changed the solver's internal names so nothing borrows the original library's vocabulary

## 1.0.9

* Fixed the game crashing when firing a minigun, because flexExtGetParticleData has six outputs and the sixth was left unwritten
* Changed debris chunks to carry real particles so the game's own budget bounds them
* Added DebrisQuality, reporting the tier the game allocated rather than pretending to set it
* Changed the solver to step from the clock, because the game's fixed 10 ms ran slow below 100 fps
* Added collision against convex hulls, which most of the world turns out to be
* Fixed debris ignoring actors, because a capsule's unused trailing floats hold denormals rather than zeros
* Added per-chunk mass from particle counts, so a slab no longer answers a shove like a splinter
* Added Heft, scaling air resistance and bounce together
* Added SettleRate, bleeding off the relative motion of touching chunks
* Added ImpactShock, so shooting a pile scatters it
* Fixed a settled pile leaping apart when the game compacted its debris pool
* Changed piece-versus-piece to walk a hash grid instead of the square of the pool
* Changed collision meshes to be indexed once on upload

## 1.0.8

* Added per-chunk silhouettes and inertia tensors, so a piece tips onto a flat face instead of freezing at its landing angle
* Changed contacts to resolve against the support point rather than a sphere, which produces the torque that does it
* Changed friction to a Coulomb impulse capped by the normal impulse, converting a skid into a tumble
* Fixed substeps being ignored, so contacts land where the engine expects them
* Added more of the engine's material parameters: particle friction, sleep threshold, max speed
* Added tests for the hull and the inertia tensor

## 1.0.7

* Fixed actors never colliding with debris, because a shape's type is three bits with the dynamic bit at 8, not 256
* Fixed overlapping explosions reading nonsense, because force field entries are 44 bytes rather than 28
* Changed the collision vertex stride to try three first, since that is what the data actually is

## 1.0.6

* Changed chunk assets to be built natively instead of calling into the original library
* Added voxelisation by ray parity, exact for closed meshes and needing no distance field
* Changed collision radius to the mean half extent plus a half cell, because a sphere through a chunk's corner holds it off the ground
* Added tests for the asset builder, the one part that runs without the game

## 1.0.5

* Changed flexExtCreateRigidFromMesh to be forwarded rather than stubbed, so chunks describe themselves
* Added per-piece size and inertia read back from the asset

## 1.0.4

* Added force field support, so explosions scatter debris
* Changed sleeping so a piece parks only on a surface that could hold it up

## 1.0.3

* Added collision against spheres, capsules and boxes, so debris reacts to actors
* Added rolling, so round pieces roll down slopes instead of only sliding

## 1.0.2

* Fixed debris being culled everywhere but the world origin, because flexGetBounds was a no-op
* Added a thread pool for the per-piece world sweep
* Changed the solver to skip settled pieces entirely

## 1.0.1

* Fixed the plugin not loading under old-gen F4SE, which recognises a plugin by F4SEPlugin_Query

## 1.0.0

* Added weapon debris physics on the CPU, redirecting the game's imports into this plugin
