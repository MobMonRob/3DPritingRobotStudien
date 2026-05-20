# PlanarSlicer — Classical Horizontal Slicing Algorithm

## 1. Purpose and Scope

The **PlanarSlicer** module implements the classical planar slicing algorithm for additive manufacturing toolpath generation. It decomposes a three-dimensional triangle mesh into a sequence of horizontal cross-sections (layers) by intersecting the mesh with a series of parallel planes perpendicular to the build direction (up-axis). This algorithm serves dual purposes:

1. **Production Use**: generates conventional 3-axis (XYZ) planar toolpaths for FDM/FFF 3D printing
2. **Reference Implementation**: provides a validated baseline for testing the slicing pipeline and comparing against non-planar algorithms (Crystal)

The core responsibilities are strictly bounded:
- **Input**: watertight triangle mesh (`geometry::TriangleMesh`) and layer thickness parameter
- **Output**: ordered sequence of contour layers (`std::vector<Layer>`), each containing closed 2D polygonal contours represented as point-pair segments
- **Not responsible for**: infill generation, path planning, collision avoidance, G-code emission (handled by downstream pipeline stages)

---

## 2. Algorithm Overview

The slicing process follows a deterministic four-stage pipeline:

```
┌───────────────────┐
│  Triangle Mesh M  │
│  Bounds [yMin,yMax]│
└─────────┬─────────┘
          │
          ▼
┌────────────────────────────────────┐
│ Stage 1: Layer Height Calculation │
│ ─────────────────────────────────  │
│ • Compute layer count N            │
│ • Generate slice heights           │
│   y_i = yMin + (i + 0.5) × h       │
└─────────┬──────────────────────────┘
          │
          ▼ (for each layer i = 0 ... N-1)
┌────────────────────────────────────┐
│ Stage 2: Plane-Mesh Intersection  │
│ ─────────────────────────────────  │
│ • Iterate over all triangles       │
│ • Detect edge-plane crossings      │
│ • Collect raw segment endpoints    │
└─────────┬──────────────────────────┘
          │
          ▼
┌────────────────────────────────────┐
│ Stage 3: Contour Chaining          │
│ ─────────────────────────────────  │
│ • Greedy nearest-neighbour search  │
│ • Assemble segments into rings     │
│ • Close loops at origin point      │
└─────────┬──────────────────────────┘
          │
          ▼
┌────────────────────────────────────┐
│ Stage 4: Ring Ordering             │
│ ─────────────────────────────────  │
│ • Reorder rings to minimise        │
│   inter-ring travel distance       │
│ • Flip rings if back-entry shorter │
└─────────┬──────────────────────────┘
          │
          ▼
┌────────────────────────────────────┐
│ Stage 5: Segment Pair Emission     │
│ ─────────────────────────────────  │
│ • Convert rings to (p_i, p_{i+1})  │
│ • Store in Layer.contour_points    │
└─────────┬──────────────────────────┘
          │
          ▼
    [Layer Vector]
```

Each stage is stateless and operates on the output of the previous stage, enabling independent verification and testing.

---

## 3. Implementation Details

### 3.1 Layer Height Calculation

The slicer computes the number of layers **N** from the mesh bounding box height **H** and the user-specified layer thickness **h**:

```cpp
N = ⌈H / h⌉
```

Slice planes are positioned at:

```
y_i = yMin + (i + 0.5) × h    for i ∈ [0, N-1]
```

**Rationale for +0.5 offset**:  
Placing the plane at the layer center (rather than the bottom or top) ensures that thin features at the mesh boundaries are not clipped. For example, a 1 mm tall mesh sliced with h = 0.2 mm would produce 5 slices; starting at yMin + 0 would miss the top 0.1 mm, while yMin + 0.5h captures geometry centered at each layer height.

**Edge case handling**:
- If `y_i > yMax`, the plane is clamped to `yMax - ε` (ε = 10⁻⁶) to keep intersection points numerically stable.
- Meshes with zero height along the up-axis produce an empty layer vector and log a warning.

---

### 3.2 Triangle-Plane Intersection (Stage 2)

For each triangle **(v₀, v₁, v₂)** in the mesh, compute the signed distance from each vertex to the plane:

```
d_i = v_i[upAxis] - planePos
```

An edge **(vA, vB)** crosses the plane if `sgn(dA) ≠ sgn(dB)`. The intersection point **P** is computed via linear interpolation:

```
t = dA / (dA - dB)
P = vA + t × (vB - vA)
```

**Corner-vertex disambiguation (P3 fix)**:  
When a vertex lies exactly on the plane (`|d| < ε`), two adjacent edges would both report that vertex as an intersection point, causing double-counting. The algorithm applies the rule:

> If the **endpoint B** of an edge lies exactly on the plane (`|dB| < ε`), skip this crossing. The adjacent edge (where that vertex is the **start endpoint A**) will handle it.

This ensures each vertex is counted exactly once, preventing duplicate points that would create zero-length segments and corrupt the contour chaining stage.

**Degenerate cases**:
- **Edge-on-plane**: If both endpoints lie on the plane (`|dA|, |dB| < ε`), the edge is silently ignored. The triangle is tangent to the slice plane and contributes no contour.
- **Face-on-plane**: If all three vertices lie on the plane, all three edges are ignored (no crossings detected). This is geometrically correct: a coplanar triangle does not define a 1D contour.

Each triangle produces at most one segment (two intersection points). Segments are stored as raw endpoint pairs `{a, b}` in an unsorted vector for chaining in Stage 3.

---

### 3.3 Contour Chaining (Stage 3)

The raw segments from Stage 2 are unordered and disconnected. Stage 3 assembles them into closed contour rings (polygons) via a greedy nearest-neighbour algorithm.

**Algorithm**:

1. **Initialise**: Mark all segments as unused.
2. **Find seed**: Pick the first unused segment. Mark it as used and initialise a ring: `ring = [a, b]`. Set `cursor = b` and `origin = a`.
3. **Grow ring**:
   - Find the unused segment **(p, q)** whose nearest endpoint to `cursor` is minimal.
   - If `dist²(cursor, p) < dist²(cursor, q)`, append `p` (forward), else append `q` (reversed).
   - Update `cursor` to the newly appended point.
   - If `dist²(cursor, origin) ≤ snapTol²`, the ring is closed → terminate.
   - If no unused segment is within `snapTol` of `cursor`, the ring is open → terminate (malformed mesh).
4. **Repeat**: Return to step 2 until all segments are used or no valid seed remains.

**Snap tolerance**:  
A threshold `snapTol = 10⁻⁴ mm` (0.1 µm) defines when two points are considered coincident. Intersection points on shared mesh edges are computed deterministically (same formula, same floating-point operations), so exact matches are common. The small epsilon handles numerical noise from degenerate triangles or non-watertight geometry.

**Multiple contours per layer**:  
A single slice plane may intersect multiple disconnected regions (e.g., a part with holes, or multiple parts arranged on the build plate). Each disconnected component becomes an independent ring. The algorithm iterates until all segments are consumed, producing as many rings as necessary.

**Failure modes**:
- **Open contours**: If a ring terminates without closing, the mesh is non-manifold or has gaps. The partial ring is discarded (only rings with ≥2 points are kept).
- **Self-intersections**: The algorithm does not detect or prevent self-intersecting polygons. Such cases arise from overlapping faces in the input mesh and must be handled by a pre-processing step (mesh repair).

---

### 3.4 Ring Ordering (Stage 4)

When a layer contains **K > 1** contour rings (e.g., outer perimeter + hole islands), the order in which they are printed affects:
- **Travel distance**: the robot must lift/traverse between disconnected contours.
- **Print quality**: large jumps through the interior risk stringing or collision with previously printed material.

Stage 4 reorders the rings to minimise total inter-ring travel distance using a greedy nearest-neighbour heuristic:

**Algorithm**:

1. **Seed selection**: Choose the ring whose centroid has the smallest Z-coordinate (assuming Y-up axis, Z represents horizontal position). This heuristic starts near the origin, reducing the initial travel move from the park position.
2. **Greedy chaining**:
   - Set `cursor` to the last point of the most recently ordered ring.
   - For each unused ring **R**, compute:
     - `d_front = dist²(cursor, R.front())`
     - `d_back = dist²(cursor, R.back())`
   - Select the ring with minimum `min(d_front, d_back)`.
   - If `d_back < d_front`, reverse the ring (flip traversal direction).
   - Append the ring to the output sequence and update `cursor`.
3. **Repeat** until all rings are ordered.

**Optimality**:  
This is a greedy O(K²) approximation to the traveling salesman problem (TSP). It does not guarantee a globally optimal path but produces acceptable results for typical layer counts (K ≈ 2–10 rings/layer). For K > 50, a more sophisticated TSP solver (e.g., Christofides algorithm) would be warranted.

**Ring reversal**:  
Reversing a ring does not change the printed geometry (a closed loop can be traversed in either direction), but it affects which endpoint the nozzle approaches first. Flipping the ring when the back endpoint is closer than the front reduces the travel distance without changing the contour shape.

---

### 3.5 Segment Pair Emission (Stage 5)

The ordered rings are converted into a flat sequence of **segment pairs** `(p_i, p_{i+1})`, which is the canonical representation for the `Layer` data structure:

```cpp
for each ring R in ordered_rings:
    for i = 0 ... R.size() - 2:
        points.push_back(R[i])
        points.push_back(R[i+1])
        normals.push_back(plane_normal)  // (0, 1, 0) for Y-up
        normals.push_back(plane_normal)
```

**Format invariants**:
- `Layer.contour_points.size()` is always even (pairs of start/end points).
- `Layer.contour_normals.size() == Layer.contour_points.size()` (one normal per endpoint).
- Segment pairs within a ring are consecutive (`p_i`'s end coincides with `p_{i+1}`'s start).
- Gaps between rings are detected by the downstream `PathPlanner`, which inserts travel moves.

**Normal vector assignment**:  
All segment endpoints receive the plane normal `(0, 1, 0)` (Y-up convention). This is a placeholder for 3-axis printing, where tool orientation is fixed. Non-planar slicers (Crystal) compute per-vertex normals from the mesh surface to define 5-axis tool orientations.

---

## 4. Data Structures

### 4.1 Input: `geometry::TriangleMesh`

A triangle mesh representation with:
- **vertices**: `std::vector<glm::vec3>` — vertex positions in world space.
- **faces**: `std::vector<glm::uvec3>` — index triples referencing `vertices`.
- **normals**: `std::vector<glm::vec3>` — per-vertex normals (not used by PlanarSlicer).
- **bounds**: axis-aligned bounding box (min, max).

**Preconditions**:
- The mesh must be **watertight** (manifold, closed surface). Open meshes or meshes with holes produce undefined contours.
- Faces must use **counter-clockwise winding** for outward-facing normals (not enforced, affects normal sign only).

### 4.2 Output: `std::vector<Layer>`

Each `Layer` represents one horizontal cross-section:

```cpp
struct Layer
{
    uint32_t index;                      // Layer number (0 = bottom)
    float thickness;                     // Layer height (mm)
    std::vector<glm::vec3> contour_points;   // Segment endpoints (even count)
    std::vector<glm::vec3> contour_normals;  // Normal per endpoint
    size_t contour_count;                // Number of points in contour (excl. infill)
    int up_axis;                         // Build direction (0=X, 1=Y, 2=Z)
    uint32_t branch_id;                  // Multi-branch tracking (0 for planar)
};
```

**Segment encoding**:  
Contours are stored as consecutive point pairs. For a ring with N vertices, `contour_points` contains N−1 pairs:
```
[v0, v1, v1, v2, v2, v3, ..., v_{N-2}, v_{N-1}]
```

The path planner interprets this as N−1 line segments. Gaps between rings (non-consecutive endpoints) trigger travel move insertion.

**Coverage metric**:  
The slicer tracks `coverage = emitted_layers / expected_layers`. For a well-formed mesh, this is 1.0. Sparse meshes or meshes with large voids report < 1.0, indicating that some slice planes intersected no geometry.

---

## 5. Complexity Analysis

### 5.1 Time Complexity

For a mesh with **T** triangles, **N** layers, and **E** contour segments per layer:

| Stage | Complexity | Notes |
|-------|------------|-------|
| Stage 1: Layer heights | O(1) | Simple arithmetic |
| Stage 2: Plane intersection | O(T) per layer → **O(NT)** total | Dominant term for large meshes |
| Stage 3: Contour chaining | O(E²) per layer | E ≈ T/N for well-distributed geometry |
| Stage 4: Ring ordering | O(K²) per layer, K ≪ E | Negligible (K ≈ 2–10 rings/layer) |
| Stage 5: Emission | O(E) per layer | Linear in segment count |
| **Overall** | **O(NT + NE²)** | |

**Approximation for uniform meshes**:  
If the mesh density is uniform, E ≈ T/N (each triangle contributes to ~1 layer on average). Then:
- Stage 2: O(NT)
- Stage 3: O(N × (T/N)²) = O(T²/N)

For N ≫ √T, Stage 2 dominates. For N ≪ √T, Stage 3 dominates. Typical FDM prints have N ≈ 100–1000, T ≈ 10⁴–10⁶, so **Stage 2 is the bottleneck**.

### 5.2 Space Complexity

- **Transient storage**: O(E) for raw segment buffer, ring buffers.
- **Output storage**: O(NE) for all layers combined.
- **Peak memory**: O(T + E) (input mesh + one layer's worth of segments).

For a typical mesh (T = 10⁵, N = 100, E ≈ 1000/layer), peak memory ≈ 100 MB (assuming 12 bytes/vertex, 12 bytes/face, 24 bytes/segment).

---

## 6. Edge Cases and Robustness

### 6.1 Thin Features

**Problem**: Features thinner than the layer height (e.g., 0.1 mm fins sliced at 0.2 mm) may be missed entirely if no slice plane intersects them.

**Mitigation**: The +0.5h offset in layer height calculation (see §3.1) ensures at least one slice passes through the center of any feature taller than 0.5h. Features thinner than 0.5h are inherently unprintable at the given resolution and are omitted by design.

### 6.2 Non-Manifold Geometry

**Problem**: Meshes with T-junctions, overlapping faces, or duplicate vertices produce ambiguous intersection topology. Segments may fail to chain into closed loops.

**Behavior**: Partial rings (open contours) are discarded. The slicer logs a warning if `contour_points.empty()` for a layer that should contain geometry.

**Recommendation**: Preprocess the mesh with a repair tool (e.g., MeshLab, OpenSCAD CGAL remesh) before slicing.

### 6.3 Zero-Height Meshes

**Problem**: Meshes collapsed along the up-axis (e.g., a 2D plane embedded in 3D) have `height < ε`.

**Behavior**: Return empty layer vector + warning. Coverage = 0.0.

### 6.4 Numerical Precision

**Floating-point considerations**:
- **Snap tolerance** (10⁻⁴ mm) is 100× larger than float32 epsilon at typical mesh scales (1–100 mm), ensuring robust comparisons.
- **Edge-on-plane**: Linear interpolation `t = dA / (dA - dB)` is stable for `|dA - dB| > ε`. When `dA ≈ dB ≈ 0` (edge lies on plane), both terms are discarded by the `|dB| < ε` check.
- **Catastrophic cancellation**: The ring-closing test uses `dist²(cursor, origin) ≤ snapTol²` to avoid sqrt() cancellation errors when cursor ≈ origin.

---

## 7. Configuration Parameters

### 7.1 `SlicerParams`

```cpp
struct SlicerParams
{
    float layer_thickness{0.2f};  // Layer height (mm)
    int up_axis{1};               // 0=X, 1=Y, 2=Z (Y-up default)
};
```

**Constraints**:
- `layer_thickness > 0` (enforced by pipeline; ≤0 triggers default).
- `up_axis ∈ {0,1,2}` (clamped internally; invalid values default to Y-up).

**Typical ranges**:
- **FDM printing**: 0.1 mm (high detail) to 0.4 mm (fast draft).
- **Resin printing**: 0.025 mm to 0.1 mm.

### 7.2 Hardcoded Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| `kSnapTol` | 10⁻⁴ mm | Vertex coincidence threshold |
| `kPlaneEpsilon` | 10⁻⁶ mm | Plane distance epsilon (edge-on-plane detection) |
| `kMinHeight` | 10⁻⁶ mm | Minimum mesh height (reject degenerate meshes) |

---

## 8. Testing and Validation

### 8.1 Unit Tests (`slicing_test.cpp`)

**Coverage**:
1. **Empty mesh**: Verify empty layer vector + warning.
2. **Unit cube**: Verify layer count = ⌈1.0 / thickness⌉.
3. **Layer thickness dependency**: Thinner layers → more slices.
4. **Contour closure**: All `contour_points.size() % 2 == 0`.
5. **Normal consistency**: `contour_normals.size() == contour_points.size()`.

**Regression guards**:
- **P3 corner-vertex fix**: Cube vertices shared by 3 faces must not produce duplicate points.
- **Ring ordering**: Multi-island test case verifies travel distance is locally minimal.

### 8.2 Integration Tests

**Test meshes**:
- **Stanford Bunny** (T = 69k): Verify manifoldness handling.
- **Perforated plate** (multiple holes): Verify ring ordering across K > 10 contours/layer.
- **Thin-wall bracket**: Verify 0.4 mm walls survive 0.2 mm slicing.

**Metrics**:
- **Coverage**: Must be 1.0 ± 0.01 for watertight meshes.
- **Contour closure**: All rings satisfy `dist(first, last) < snapTol`.
- **Print time**: Slicing 100 layers of a 10⁵-triangle mesh must complete in < 500 ms (macOS ARM64, release build).

---

## 9. Limitations and Future Work

### 9.1 Current Limitations

1. **No adaptive layer height**: All layers use uniform thickness. Flat regions could use thicker layers for speed; curved regions need finer layers for accuracy.
2. **Single outer perimeter**: No multi-wall support. Downstream infill generator handles this, but it's not slice-aware.
3. **No overhang detection**: Layers are sliced independently. Overhangs requiring support are not flagged.
4. **Greedy TSP**: Ring ordering is O(K²) greedy, suboptimal for K > 20.

### 9.2 Potential Improvements

**Adaptive layer height**:  
Compute local curvature from mesh normals and vary `thickness[i]` per layer. Requires changes to the `Layer` data structure (per-layer thickness instead of global parameter).

**Multi-wall generation**:  
Offset contours inward by `wall_thickness` using polygon offsetting (e.g., Clipper library). Requires handling of self-intersections during inward offset.

**Overhang analysis**:  
Compare `Layer[i].contour` with `Layer[i−1].contour`. Regions in layer i not supported by layer i−1 are flagged for support structure generation.

**Exact TSP solver**:  
For K > 50, replace greedy ring ordering with Christofides algorithm (1.5× optimal guarantee) or Concorde TSP solver (exact, exponential worst-case).

---

## 10. References and Related Work

### 10.1 Classical Slicing Algorithms

1. **Rock & Wozny (1991)**: _"A flexible file format for solid freeform fabrication"_. First description of STL-based triangle-plane intersection for RP.
2. **Kulkarni et al. (2000)**: _"A review of process planning techniques in layered manufacturing"_. Comprehensive survey of slicing algorithms up to 2000.
3. **Ding et al. (2016)**: _"Adaptive slicing algorithms based on STL models"_. Modern adaptive layer height methods.

### 10.2 Contour Chaining

4. **Held (2001)**: _"On the Computational Geometry of Pocket Machining"_ (Springer LNCS 500). Discusses greedy chaining vs. optimal chaining for 2.5D milling.

### 10.3 Internal Project References

- **Crystal**: Non-planar iso-surface slicing using harmonic scalar fields (6-DOF toolpaths).
- **PathPlanner** (`common/path_planner.cpp`): Converts segment pairs → travel moves + extrusion moves.
- **InfillGenerator** (`common/infill_generator.cpp`): Generates rectilinear infill inside closed contours.

---

## 11. API Usage Example

```cpp
#include "planar/planar_slicer.h"
#include "triangle_mesh.h"

// Load mesh from file (e.g., STL, OBJ)
geometry::TriangleMesh mesh = loadMeshFromFile("model.stl");

// Configure slicer
slicing::SlicerParams params;
params.layer_thickness = 0.2f;  // 200 µm layers
params.up_axis = 1;             // Y-up coordinate system

// Slice
slicing::PlanarSlicer slicer(params);
std::vector<slicing::Layer> layers = slicer.slice(mesh);

// Verify success
if (slicer.coverage() < 0.95f)
{
    std::cerr << "Warning: sparse coverage (" 
              << slicer.coverage() * 100.0f << "%)\n";
}

// Process layers
for (const auto& layer : layers)
{
    std::cout << "Layer " << layer.index 
              << ": " << layer.contour_points.size() / 2 
              << " segments\n";
}
```

---

## 12. Glossary

| Term | Definition |
|------|------------|
| **Contour** | A closed 2D polygon resulting from a plane-mesh intersection. |
| **Layer** | One horizontal slice at a fixed height, containing one or more contours. |
| **Segment pair** | Two consecutive points `(p_i, p_{i+1})` representing a line segment. |
| **Ring** | A closed contour (outer perimeter or hole) within a layer. |
| **Snap tolerance** | Distance threshold (10⁻⁴ mm) below which two points are considered coincident. |
| **Coverage** | Fraction of expected layers that contain non-empty geometry (1.0 = full mesh sliced). |
| **Up-axis** | Build direction perpendicular to slice planes (0=X, 1=Y, 2=Z). |
| **Manifold mesh** | A closed, watertight surface with no holes, T-junctions, or overlapping faces. |
| **Greedy TSP** | Nearest-neighbour heuristic for ordering contours (O(K²) approximation). |
| **Y-up convention** | Coordinate system where Y points upward (OpenGL standard; differs from Z-up CAD systems). |
