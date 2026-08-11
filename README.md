# Boids 3D — Real-Time Flocking Simulator

Interactive 3D **boids** (flocking) simulator written in **pure C** with [raylib](https://www.raylib.com/), capable of simulating **thousands of agents in real time** thanks to a uniform spatial grid for neighbor lookups, **OpenMP** parallelization and **GPU instanced rendering** with custom GLSL shaders.

<img width="1917" height="1077" alt="Emergent flocking patterns" src="https://github.com/user-attachments/assets/77356c55-380d-4e06-bea6-08e57bb85d3c" />

## Features

### Simulation
- **Classic Reynolds rules** — separation, alignment and cohesion, with weights tunable live from the UI.
- **Realistic perception model**: configurable vision radius and rear blind-spot angle per boid.
- **Wander noise** from a precomputed noise table, for organic, non-mechanical motion.
- **Three boundary modes** switchable on the fly: bounce, toroidal wrap-around, and wall steering (walls avoided like obstacles).
- **Seed-deterministic simulation**: enter a seed and reproduce the exact same flock.

### SDF-Based Obstacles
- Place obstacles **in real time** by aiming with the camera: spheres, boxes, toruses and cylinders, with free yaw/pitch rotation.
- Avoidance is solved with analytic **signed distance fields (SDFs)** per shape and surface normals via numeric gradient — the same approach used in ray marching.
- Trajectory **lookahead** for collision anticipation, plus direct physical contact resolution.
- Obstacle maps can be saved to and loaded from disk (`.obs`).

### Performance
- **Uniform spatial grid (spatial hashing)** with per-cell linked lists: neighbor search drops from O(n²) to ~O(n), with dynamic resizing based on world size and vision radius.
- **OpenMP** in the hot paths (force computation and integration), with atomic sections where needed.
- **Instanced rendering**: all boids are drawn in a single `DrawMeshInstanced` call with a custom GLSL shader that also smuggles the per-instance color inside the transform matrix to save an extra buffer.
- Only visible boids are uploaded to the GPU each frame.

### UI & Tooling
- **Full control panel built with raygui**: boid count, speeds, per-rule weights, agility, world size… all adjustable live.
- **Presets** (`.cfg`): save and load named configurations from the UI itself.
- **Snapshots** (`.snap`): serialize the **entire simulation state** (position and velocity of every boid) to a binary format, with validation against corrupted files.
- **Three camera modes**: free flight, first person and third person following a specific boid.
- **Debug overlays**: occupied grid cells, vision radius and blind angle of the followed boid, world bounds, speed heatmap, FPS counter.

## Gallery

Some emergent scenarios captured during simulation:

<img width="1919" height="1064" alt="Emergent flocking scenario" src="https://github.com/user-attachments/assets/0fb0bcdb-7024-4bb0-b179-6ea585770c49" />
<img width="1915" height="1077" alt="Emergent flocking scenario" src="https://github.com/user-attachments/assets/06c61e47-ba3b-4dde-a59e-db9655e7267b" />

## Controls

| Key / action | Effect |
|---|---|
| `W A S D` + mouse | Move free camera (`Space`/`Shift` up/down, `Ctrl` speed boost) |
| `Tab` | Show / hide the UI |
| `P` | Pause the simulation |
| `R` | Reset the simulation |
| `B` | Cycle boundary mode (bounce → wrap → steer) |
| `C` | Cycle camera mode (free → 1st person → 3rd person) |
| `O` | Toggle obstacle placement mode |
| Left / right click | Place / remove obstacle |
| Arrow keys + wheel | Rotate and scale the obstacle before placing it |
| `F11` | Borderless fullscreen |

## Building (Windows)

Requirements: **GCC (MinGW-w64)** in your `PATH`. raylib is already bundled in `include/` and `lib/`.

```bat
build.bat
```

Manual equivalent:

```bat
gcc boids.c -o boids.exe -O3 -Wall -fopenmp -I.\include -L.\lib -lraylibdll
```

> `raylib.dll` must sit next to the executable.

## Project Structure

```
boids.c        # The whole simulation: rules, spatial grid, SDFs, rendering, UI
vec3.h         # Custom header-only 3D vector math library
include/       # raylib, raymath, rlgl, raygui
presets/       # Example configurations (.cfg)
obstacles/     # Obstacle maps (.obs)
snapshots/     # Saved simulation states (.snap)
build.bat      # Windows build script
```

## Technical Highlights

- **C99 with no external dependencies** beyond raylib: the vector math, spatial grid, SDFs and serialization are all custom implementations.
- **Defensive I/O validation**: presets and snapshots loaded from disk are sanitized (clamping, NaN checks, memory limits) so a hand-edited or corrupted file can never break the simulation.
- **Cost-aware memory management**: the grid caps its cells per axis to bound the maximum allocation and reuses allocations across frames.
