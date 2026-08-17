# FlexRevive

**Restores Fallout 4's weapon debris on modern hardware.** Chunks fly off surfaces you
shoot, tumble, bounce off the world and settle, all simulated by this plugin instead of
NVIDIA's long-dead library.

## Description

Fallout 4 can throw physical chunks off surfaces you shoot, concrete off a wall, splinters
off a crate. It has been able to since 2016. Almost nobody has seen it, because turning it
on crashes the game on modern hardware.

Weapon debris is driven by NVIDIA Flex, a GPU particle solver Bethesda shipped as
`flexRelease_x64.dll`. It is built against CUDA 7.5, released in 2015, which predates
Pascal, Turing, Ampere, Ada and Blackwell. Every consumer GPU of the last several years.

On modern hardware Flex typically fails to initialise a device, does not report it, and
dereferences a null pointer the first time the game asks for a collision mesh, inside
`flexCreateTriangleMesh`. The game vanishes without a message, usually within seconds of
walking around.

Because there was never a fix, the standard advice for a decade has been "turn weapon
debris off". That works, and it means a finished feature has effectively never shipped.

## What this mod does

FlexRevive hooks Fallout 4's import table for both Flex DLLs into its own implementation
and takes over the debris physics. This is not a crash guard, a null check, or a switch
that quietly disables the feature. Weapon debris genuinely runs.

## Installation

**With a mod manager (recommended):** install the archive with Vortex or Mod Organizer 2
and enable it. The layout is the standard F4SE plugin layout and is detected automatically.

**Manually:** copy the `F4SE` folder into your `Fallout 4\Data` folder, so you end up with:

```
Fallout 4\Data\F4SE\Plugins\FlexRevive.dll
Fallout 4\Data\F4SE\Plugins\FlexRevive.ini
```

Then launch through `f4se_loader.exe`.

### You do not need to enable weapon debris yourself

Almost every existing setup has weapon debris switched off, because that was the only way
to avoid the crash. FlexRevive turns it back on for you.

It does cost one restart, and there is no way around that. The game reads the weapon debris
setting once while it starts, well before it loads any plugin, and builds its debris system
only if it was on. By the time FlexRevive can run, that decision has been made for the whole
session. So the plugin enables the setting for next time and says so in the log; start the
game again and debris works. Once only, not every launch.

If you would rather manage it yourself, set `ForceEnableWeaponDebris=0` in the INI and
enable it the normal way (`bNVFlexEnable=1` under `[NVFlex]` in `Fallout4Prefs.ini`, or the
Weapon Debris option in the launcher).

### Checking that it worked

Shoot a concrete wall or a metal surface at close range. Chunks should fly off, tumble,
bounce and settle.

If nothing happens, open `Documents\My Games\Fallout4\F4SE\FlexRevive.log`. It records
exactly what the plugin did, in order:

- `34 Flex entry points redirected` means the plugin is installed and in control.
- `weapon debris is enabled` means the game built its debris system and the plugin has
  something to drive.
- `RESTART THE GAME` means debris was switched off, so nothing can spawn this session. The
  plugin has enabled it for the next one. Start the game again.
- `advanced N rigid pieces` means debris is actually being simulated.

If the log file does not exist at all, F4SE did not load the plugin. You probably launched
without `f4se_loader.exe`.

### Uninstalling

Remove the two files, or disable the mod in your manager. FlexRevive adds no scripts, no
forms and no save data, so nothing is left behind and existing saves are unaffected. You
may want to turn weapon debris back off afterwards, since without the plugin it will crash
again.

## Main features

Weapon debris when you shoot stuff. Pieces fly off the surface they came from, respond to
gravity, tumble, collide with one another, and get scattered by the engine's force fields:
explosions, grenades and the rest. Walk into a pile and you kick it out of the way. Shoot into
one and it scatters.

Each chunk carries its own mass, taken from the particle count the game gives it, so a slab
shoulders a splinter aside instead of trading with it evenly and larger pieces cut through the
air better than small ones.

Everything lives in `Data\F4SE\Plugins\FlexRevive.ini`, which the plugin writes with full
commentary the first time it runs, so the config file documents itself. Delete it to
restore defaults.

```
[General]     Enabled, ForceEnableWeaponDebris, UseEngineSpawnData, RealTimestep,
              DebrisQuality, EngineParticles, VerboseLog
[Physics]     GravityScale, DragScale, RestitutionScale, FrictionScale
[Rotation]    SpawnSpin, SpawnBurst, SpawnVelocityScale, ImpactTorque, Rolling
[Collision]   DebrisVsDebris, PieceRadiusScale, SettleRate, ImpactShock, Heft
[Limits]      MaxPieces
[Performance] SolverThreads, ComputeBackend
```

### How much debris you get

This is set by `iQuality:NVFlex` in `Fallout4Prefs.ini`, not by this plugin. The game reads it
during startup, before any plugin loads, so nothing here can change it:

```
iQuality=0   6000 particles   culled 2000 units away   32 neighbours
iQuality=1  16000 particles   culled 3000 units away   48 neighbours
iQuality=2  32768 particles   culled 4000 units away   64 neighbours
```

The budget is a real ceiling: the game refuses to spawn a chunk once the particles in flight
plus that chunk's own would exceed it, and refusing one abandons the rest of that impact's
debris. `DebrisQuality` in FlexRevive.ini does not set the tier, it checks what the game
actually allocated and says so in the log.

There are only three tiers, but the game reads the numbers out of the tier you pick rather than
hardcoding them, so tier 2 can be given larger ones. Keep `iQuality=2` and raise its entries
under `[NVFlex]`:

```ini
iMaxParticles2=65536
fKillRadius2=6000.0000
iMaxNeighbors2=96
```

Raise `MaxPieces` in FlexRevive.ini to match, or the solver caps what the game hands it.

Heavier, more grounded debris:

```ini
Heft=2.2
GravityScale=1.3
RestitutionScale=0.4
FrictionScale=1.5
```

Cinematic, slower and showier tumbling:

```ini
Heft=0.8
GravityScale=0.7
SpawnSpin=22.0
SpawnVelocityScale=0.8
ImpactTorque=1.8
RestitutionScale=1.6
```

Calmer debris, if chunks fly too far off the surface they came from or a heap scatters too
easily when you shoot it:

```ini
SpawnVelocityScale=0.2
ImpactShock=0.5
SettleRate=1.5
```

Lowest cost, if you are CPU-limited:

```ini
DebrisVsDebris=0
MaxPieces=512
```

Each chunk is built from the mesh the engine hands over when it creates the fragment, so
pieces collide at their own size and shape, carry their own inertia, and leave the surface at
the speed the game says they do. Contacts resolve against the chunk's real silhouette, which
is what lets a piece tip onto a flat side instead of stopping at whatever angle it was at.
If debris still rests visibly above the ground, or sinks into it, `PieceRadiusScale` scales
every piece at once. `UseEngineSpawnData=0` turns the measurement off and goes back to
inferring both, which is the fallback if it ever misbehaves.

## Performance

The simulation runs on the CPU, spread across a few threads, at the substep rate the engine
asks for. Collision meshes are indexed, so a piece only tests the geometry it sweeps past, and
settled debris costs effectively nothing.
Sustained fire from something like a minigun is the heaviest case, since it keeps a lot of
pieces in the air at once. If you are CPU-limited, `DebrisVsDebris=0` and a lower `MaxPieces`
are the two knobs to play with.

`SolverThreads` defaults to your core count less two, which leaves the game its own headroom.
Raising it usually makes frames worse rather than better, since it takes cores away from the
engine. Set it to 1 to keep everything on the game thread.

Worth saying plainly: for most hardware the original never ran on your GPU either, because
it could not initialise one. No GPU path is being given up here.

## Requirements

- **Fallout 4**, developed and tested on 1.11.221. Nothing in the plugin is
  version-specific. Old-gen (the 1.10.x line) is supported since 1.0.1, earlier versions
  never loaded under old-gen F4SE. Reports welcome.
- **[Fallout 4 Script Extender (F4SE)](https://f4se.silverlock.org/)** matching your game
  version.

## Tools used

- **[Claude Code](https://claude.com/claude-code)**
- **[Ghidra](https://ghidra-sre.org/)**
- **cdb.exe** (Windows Debugging Tools)
- **Capstone**
- **Python**
- **Buffout 4**
- **MSVC** and **CMake**
- In-game test kit (`tools/fo4_testkit.cpp`)

## Shout outs

**ianpatt, behippo and purplelunchbox** for the Fallout 4 Script Extender.

**Bethesda Game Studios**, who shipped the feature and then left it broken for a decade.

**NVIDIA** for Flex, and for publishing an API clear enough to re-implement from the
outside. Their interface made it possible to fill in the missing pieces.

**Everyone who tests this and reports what breaks.** It is new physics in an old engine and
it will meet surfaces I have not thought of. A log file and a description of what the
debris did wrong is genuinely useful, and I would rather hear about it than not.

MIT licence, see `LICENSE.txt`. No NVIDIA Flex code is used or redistributed.
