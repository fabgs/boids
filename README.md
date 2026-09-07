# Boids 3D — Real-Time Flocking Simulator

Interactive 3D **boids** (flocking) simulator written in **pure C** with [raylib](https://www.raylib.com/), capable of simulating **thousands of agents in real time** thanks to a uniform spatial grid for neighbor lookups, multithreaded force computation (**OpenMP** on desktop, a **pthreads** pool on the web) and **GPU instanced rendering** with custom GLSL shaders. The same code compiles to **WebAssembly** and runs in the browser, desktop or mobile.

**▶ Try it in the browser: [fabgs.dev/boids](https://fabgs.dev/boids/)** · Part of [fabgs.dev](https://fabgs.dev/)

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
- **Multithreaded hot paths** (force computation and render matrices) through a small `parallel_for(n, fn, ctx)` abstraction: **OpenMP** on desktop, a persistent **pthreads pool** on the web, sequential fallback otherwise. Shared counters use atomics.
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
| Panel `X` button | Hide the UI (same as `Tab`) |

On touch devices (web version on a phone or tablet) the controls are gestures instead:

| Gesture | Effect |
|---|---|
| One-finger drag | Rotate the camera |
| Pinch | Move forward / backward (distance of the ghost in placement mode) |
| Two-finger drag | Move sideways / up and down |
| Tap | Select a boid to follow, or place an obstacle in placement mode |
| Long press | Remove the obstacle under the finger (placement mode) |
| Panel `X` / `Show UI` button | Hide / show the UI |

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

## Web version (WebAssembly)

The same `boids.c` compiles to WebAssembly with [Emscripten](https://emscripten.org/) and runs in the browser on WebGL 2, no rewrite and no `ASYNCIFY`. Live build: **https://fabgs.dev/boids/** (published from `master` by the `Deploy web` GitHub Actions workflow; Pages must be set to *Source: GitHub Actions* in the repo settings).

What changes on the web, all behind `#if defined(PLATFORM_WEB)`:

| Desktop | Web |
|---|---|
| `while (!WindowShouldClose())` in `main` | The browser drives `update_draw_frame()` through `emscripten_set_main_loop`, so the application state lives at file scope |
| OpenMP `parallel for` | A persistent **pthreads pool** (one worker per logical core) behind the same `parallel_for(n, fn, ctx)` abstraction. Needs `SharedArrayBuffer`, i.e. COOP/COEP headers |
| GLSL 330 instancing shader | GLSL ES 3.00 (`#version 300 es`), same code otherwise |
| `SetTargetFPS(60)` | `requestAnimationFrame` at the monitor rate; a fixed-step accumulator keeps the simulation at 60 steps/s on 120/144 Hz screens |
| `rlEnableWireMode` (`glPolygonMode`) for obstacle wireframes | Edges emitted as `RL_LINES` (no `glPolygonMode` in WebGL) |
| Presets / snapshots / maps on disk next to the .exe | Virtual filesystem: bundled examples are preloaded, user saves go to `/persist`, mounted on **IndexedDB** so they survive reloads |
| Fixed 1920x1080 window | Canvas fills the viewport and follows browser resizes; the UI panel compresses its rows if the viewport is shorter than the panel |
| Keyboard + mouse | On touch devices (coarse pointer) a touch mode kicks in: gesture camera, tap / long press instead of clicks, a floating button instead of `Tab`, and a lower default boid count. Add `?touch` to the URL to force it on desktop |
| No page navigation | The HTML shell adds a small link back to the portfolio (`/`, i.e. [fabgs.dev](https://fabgs.dev/)) |

### Building the web version

Requirements: **emsdk 6.0.9** (`lib/libraylib.web.a` was compiled with it; other versions may fail to link). raylib for web is bundled, built with `GRAPHICS_API_OPENGL_ES3` and `-pthread` (the exact commands are documented in `build_web.sh`).

```bat
build_web.bat              :: Windows (activates %USERPROFILE%\emsdk if emcc is not in PATH)
./build_web.sh             # Linux / macOS / Git Bash
./build_web.sh --no-threads   # single-threaded build, no COOP/COEP headers required
```

Output goes to `dist/`. Test it locally with `python web/serve.py` (a plain `http.server` will not do: the page needs the `Cross-Origin-Opener-Policy: same-origin` and `Cross-Origin-Embedder-Policy: require-corp` headers for threads). GitHub Pages cannot send custom headers, so `web/coi-serviceworker.js` injects them client-side; on Netlify, Cloudflare Pages or your own server, set the headers and the service worker becomes a no-op.

### Performance on the web

Rendering is unchanged (GPU instancing on WebGL 2). The simulation is compiled with `-msimd128` and scales with cores through the pthread pool, but WebAssembly is somewhat slower than native code per thread, so the boid count you can hold at 60 fps is lower than the native build on the same machine. On phones the default boid count drops to 10,000, and browsers that do not report the core count (Safari on iOS) get a 4-thread pool.

## Project Structure

```
boids.c        # The whole simulation: rules, spatial grid, SDFs, rendering, UI
vec3.h         # Custom header-only 3D vector math library
include/       # raylib, raymath, rlgl, raygui
lib/           # raylib for Windows (.a/.dll) and for web (libraylib.web.a)
presets/       # Example configurations (.cfg)
obstacles/     # Obstacle maps (.obs)
snapshots/     # Saved simulation states (.snap)
build.bat      # Windows build script
build_web.bat  # Web (WebAssembly) build script, Windows
build_web.sh   # Web (WebAssembly) build script, bash
web/           # HTML shell, coi-serviceworker.js and a local dev server with COOP/COEP headers
.github/       # GitHub Actions workflow that builds and deploys the web version to Pages
LICENSE        # MIT
```

## Technical Highlights

- **C99 with no external dependencies** beyond raylib: the vector math, spatial grid, SDFs and serialization are all custom implementations.
- **Defensive I/O validation**: presets and snapshots loaded from disk are sanitized (clamping, NaN checks, memory limits) so a hand-edited or corrupted file can never break the simulation.
- **Cost-aware memory management**: the grid caps its cells per axis to bound the maximum allocation and reuses allocations across frames.
- **One codebase, two platforms**: every desktop/web difference is isolated behind `PLATFORM_WEB`, and the native build is unchanged by the port.

## License

MIT, see [LICENSE](LICENSE). Made by [Fabián Godoy](https://fabgs.dev/) ([@fabgs](https://github.com/fabgs)).
