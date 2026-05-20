# BaseSlicer — Multi-Directional Overhang-Aware Slicing Algorithm

## 1. Purpose and Scope

The **BaseSlicer** module implements a queue-based, multi-directional slicing algorithm designed to handle overhanging geometry without requiring support structures. Unlike classical planar slicing (which slices exclusively in the vertical direction and requires supports for overhangs exceeding ~45°), BaseSlicer dynamically detects overhanging regions and spawns additional slicing passes oriented perpendicular to the overhang surface. This produces a **tree-like growth pattern** where each overhang becomes a new branch sliced in its own local coordinate frame.

The algorithm is particularly suited for:
1. **Cantilever structures**: T-shaped parts, brackets, wall-mounted fixtures
2. **Support-free printing**: reduces material waste and post-processing time
3. **Multi-axis additive manufacturing**: robotic arms with 5+ degrees of freedom that can print along arbitrary orientations

Core responsibilities:
- **Input**: watertight triangle mesh + overhang angle threshold
- **Output**: ordered sequence of layers grouped by branch_id, each with a local growth direction
- **Not responsible for**: collision avoidance (robot workspace checks), infill generation, path ordering (handled downstream)

---

## 2. Algorithm Overview

BaseSlicer operates as a **breadth-first queue-driven process**:

```
┌─────────────────────────┐
│  Initialize Queue       │
│  ──────────────────────  │
│  push(↑Y, h=0, branch=0)│  ← Initial upward run from build plate
└───────────┬─────────────┘
            │
            ▼
    ┌───────────────────┐
    │ Queue not empty?  │──No──→ DONE
    └───────┬───────────┘
            │ Yes
            ▼
    ┌─────────────────────────────────┐
    │ Pop next run from queue         │
    │ run = {growth_dir, start_h,     │
    │        branch_id}                │
    └───────────┬─────────────────────┘
                │
                ▼
        ╔═══════════════════════════════╗
        ║  Execute Slicing Run          ║
        ║  (§3.3 — iterative slicing)   ║
        ╚═══════════════════════════════╝
                │
                ▼
        For each layer i in run:
        ┌─────────────────────────────────┐
        │ 1. Intersect plane with mesh    │
        │ 2. Compute bounds in UV-plane   │
        │ 3. Compare with previous layer  │
        │ 4. Detect overhangs (+U,-U,+V,-V)│
        │ 5. Queue new runs for overhangs │
        │ 6. Filter segments to allowed   │
        │    region (clip overhang parts) │
        │ 7. Emit layer                   │
        └───────────┬─────────────────────┘
                    │
                    ▼
            (loop back to queue check)
```

### 2.1 Key Concepts

**Growth Direction**:  
Each slicing run defines a local coordinate system with:
- **Growth axis** (G): the direction perpendicular to slice planes (analogous to Z-up in planar slicing)
- **UV plane**: two orthonormal axes perpendicular to G, forming the slice plane coordinate frame

**Overhang Detection**:  
At each layer i, the bounding rectangle in the UV plane is compared to the previous layer's bounds. If the current layer extends beyond the previous by more than `tolerance = thickness × tan(overhang_angle)`, the excess region is considered an overhang.

**Branch Spawning**:  
When an overhang is detected in direction +U (for example), a new slicing run is queued with:
- `growth_dir = +U` (rotated 90° from the parent)
- `start_h = prevBounds.uMax` (starting position at the overhang boundary)
- `branch_id = unique ID` (incremented counter for tracking)

The parent run then **clips** all subsequent layers to exclude the overhang region (by tightening `allowed.uMax = prevBounds.uMax`), ensuring that geometry is not duplicated across branches.

---

## 3. Implementation Details

### 3.1 Coordinate System Construction

For a given growth direction **G**, the UV plane is computed via Gram-Schmidt orthogonalisation:

```cpp
ref = (|G · Ŷ| > 0.9) ? X̂ : Ŷ     // Pick non-parallel reference vector
U = normalize(G × ref)           // First perpendicular axis
V = normalize(U × G)             // Second perpendicular axis (right-handed)
```

**Rationale for ref selection**:  
If G is nearly parallel to the Y-axis (vertical), using Ŷ as the reference would produce a near-zero cross product. Switching to X̂ ensures numerical stability.

**UV handedness**:  
The (U, V, G) frame forms a right-handed coordinate system. This is critical for consistent normal interpolation and orientation tracking in 6-DOF toolpath generation.

### 3.2 Plane-Mesh Intersection

BaseSlicer uses the same triangle-plane intersection algorithm as PlanarSlicer (see PlanarSlicer §3.2), but generalised to **arbitrary plane orientations**:

Given:
- Plane point: **P = h · G** (position along growth direction)
- Plane normal: **N = G**

Signed distance from vertex **v** to plane:
```
d = (v - P) · N = v · G - h
```

Edge crossing condition and interpolation are identical to the planar case, except:
1. **Normal interpolation**: Instead of assigning a constant plane normal, BaseSlicer interpolates the mesh vertex normals across the intersection segment:
   ```cpp
   n_intersect = normalize(n_a + t · (n_b - n_a))
   ```
   This preserves surface curvature information for non-planar layers.

2. **No contour chaining**: Unlike PlanarSlicer, BaseSlicer emits raw segment pairs without chaining them into closed rings. Rationale: multi-directional slicing can produce disconnected contours that span multiple branches, making robust chaining ambiguous. The downstream path planner handles segment ordering.

**Edge case**:  
Degenerate triangles (area ≈ 0) or edge-on-plane crossings may produce zero-length segments. These are implicitly filtered during the bounds computation stage (empty bounds → layer skipped).

---

### 3.3 Single Slicing Run Execution

A **slicing run** processes one branch of the tree, slicing from `start_h` to the mesh extent along `growth_dir`. The pseudocode:

```python
def executeRun(mesh, run, layers, queue):
    # 1. Compute UV axes perpendicular to growth direction
    U, V = computeUV(run.growth_dir)
    
    # 2. Find mesh extent along growth axis
    h_min, h_max = project_mesh_onto(run.growth_dir)
    
    # 3. Initialize allowed bounds (entire UV plane)
    allowed = {uMin: -∞, uMax: +∞, vMin: -∞, vMax: +∞}
    prev_bounds = None  # No previous layer yet
    queued_flags = {uPlus: false, uMinus: false, vPlus: false, vMinus: false}
    
    # 4. Slice from start height to mesh top
    for h in range(run.start_h + 0.5·thickness, h_max, thickness):
        # 4a. Intersect plane with mesh
        plane_point = h · run.growth_dir
        segments, normals = intersect_plane(mesh, plane_point, run.growth_dir)
        
        if segments.empty(): continue
        
        # 4b. Compute bounds of current layer in UV coordinates
        current_bounds = compute_bounds_2d(segments, U, V)
        
        # 4c. Overhang detection (compare with previous layer)
        if prev_bounds is not None:
            tolerance = thickness · tan(overhang_angle)
            
            # Check +U overhang
            if current_bounds.uMax > prev_bounds.uMax + tolerance:
                if not queued_flags.uPlus:
                    queue.push({growth_dir: U, 
                                start_h: prev_bounds.uMax, 
                                branch_id: next_id++})
                    allowed.uMax = prev_bounds.uMax  # Clip future layers
                    queued_flags.uPlus = true
            
            # Check -U, +V, -V similarly...
        
        # 4d. Clip segments to allowed region
        segments = clip_segments_to_bounds(segments, U, V, allowed)
        
        if segments.empty(): continue
        
        # 4e. Update prev_bounds from CLIPPED contour
        prev_bounds = compute_bounds_2d(segments, U, V)
        
        # 4f. Clamp prev_bounds to allowed bounds (prevent drift)
        prev_bounds.uMin = max(prev_bounds.uMin, allowed.uMin)
        prev_bounds.uMax = min(prev_bounds.uMax, allowed.uMax)
        prev_bounds.vMin = max(prev_bounds.vMin, allowed.vMin)
        prev_bounds.vMax = min(prev_bounds.vMax, allowed.vMax)
        
        # 4g. Emit layer
        layers.append({
            index: global_layer_index++,
            branch_id: run.branch_id,
            contour_points: segments,
            contour_normals: normals,
            thickness: thickness,
            up_axis: dominant_axis(run.growth_dir)
        })
```

### 3.4 Overhang Detection Logic

The **tolerance** threshold determines when a region is considered an overhang:

```
tolerance = layer_thickness × tan(overhang_angle)
```

**Geometric interpretation**:  
For a layer of thickness `h`, a horizontal extension of `h × tan(α)` corresponds to a surface tilted at angle `α` from vertical. If the layer boundary extends beyond this cone, the surface is steeper than the threshold and requires a new growth direction.

**Example**:  
- `overhang_angle = 45°` → `tolerance = h × tan(45°) = h × 1.0 = h`
- A layer extending 2 mm beyond the previous when `h = 0.2 mm` exceeds the tolerance (2 > 0.2), triggering an overhang.

**Four-way checking**:  
Each layer is checked against four boundaries (+U, -U, +V, -V). If any boundary is exceeded, a new run is queued in that direction. The `queued_flags` ensure that once a boundary is exceeded, all subsequent layers are clipped to that boundary (no redundant queueing).

### 3.5 Segment Clipping and Region Filtering

After an overhang is detected, the **allowed bounds** are tightened to exclude the overhang region:

```cpp
allowed.uMax = prev_bounds.uMax;  // Clip +U overhang
```

All subsequent layers in this run have their segments clipped to the allowed region using the **Liang-Barsky 2D line clipping algorithm**:

```python
def clipSegment(p0, p1, U, V, allowed):
    """
    Clips a 3D segment to a 2D bounding box in UV space.
    Returns (clipped_p0, clipped_p1) if segment intersects box, else None.
    """
    # Project endpoints to UV coordinates
    u0, v0 = p0 · U, p0 · V
    u1, v1 = p1 · U, p1 · V
    
    # Liang-Barsky parameters: segment = p0 + t·(p1 - p0), t ∈ [0,1]
    # Visible portion lies in t ∈ [t_min, t_max]
    t_min, t_max = 0.0, 1.0
    du, dv = u1 - u0, v1 - v0
    
    # Clip against four boundaries
    for (p, q) in [(-du, u0 - allowed.uMin),  # u >= uMin
                   ( du, allowed.uMax - u0),  # u <= uMax
                   (-dv, v0 - allowed.vMin),  # v >= vMin
                   ( dv, allowed.vMax - v0)]: # v <= vMax
        if abs(p) < ε:
            if q < 0: return None  # Parallel and outside
        else:
            t = q / p
            if p < 0:  # Entering half-space
                t_min = max(t_min, t)
            else:      # Leaving half-space
                t_max = min(t_max, t)
            if t_min > t_max: return None  # No intersection
    
    # Compute clipped endpoints
    dir = p1 - p0
    return (p0 + t_min · dir, p0 + t_max · dir)

def filterPoints(segments, U, V, allowed):
    clipped = []
    for (p0, p1) in segment_pairs(segments):
        result = clipSegment(p0, p1, U, V, allowed)
        if result is not None:
            clipped.append(result)
    return clipped
```

**Why Liang-Barsky instead of midpoint filtering**:

*Previous implementation (midpoint test)* discarded segments entirely if their midpoint fell outside the allowed region. This caused **layer growth** when segments straddled the boundary:

**Example bug** (T-shaped mesh at crossbar height):
```
Layer at h=41mm (first crossbar layer):
  Raw intersection: segments from x=0 to x=50 (full crossbar width)
  Allowed region:   x ∈ [20, 30] (stem width after overhang clipping)
  
  Midpoint test:
    Segment from x=0→50 has midpoint at x=25 (INSIDE [20,30])
    → Segment KEPT entirely, including endpoints at x=0 and x=50
    → Post-filter bounds = [0, 50] instead of [20, 30]
    → Layer GROWS instead of staying clipped!
    
  Result: Stem layers "leak" into crossbar region, causing overlap.
```

**Liang-Barsky clipping**:
```
  Same segment x=0→50:
    Clips to allowed.uMin=20 and allowed.uMax=30
    → Result: segment from x=20→30 (only the stem portion)
    → Post-filter bounds = [20, 30] ✓
```

**Additional safeguard** — bounds clamping:

After clipping, `prev_bounds` is clamped to `allowed` to prevent drift from numerical errors:

```cpp
prev_bounds.uMin = std::max(prev_bounds.uMin, allowed.uMin);
prev_bounds.uMax = std::min(prev_bounds.uMax, allowed.uMax);
// Similarly for vMin, vMax
```

This ensures that even if a segment edge case slips through clipping (e.g., segment exactly on boundary), the bounds cannot grow beyond the allowed region.

**Performance**:  
Liang-Barsky is O(1) per segment (4 boundary checks + 2 endpoint updates). For typical layers with E ≈ 1000 segments, clipping adds ~0.1 ms per layer, negligible compared to plane intersection (O(T)).

---

### 3.6 Branch Tracking and Layer Indexing

Each layer is assigned:
- **index**: global sequential counter (0, 1, 2, ...) across all branches
- **branch_id**: identifies which slicing run produced this layer
  - `branch_id = 0`: initial upward run (stem)
  - `branch_id > 0`: overhang branches

**Layer ordering**:  
Layers are emitted in breadth-first order (queue order). This means:
- All stem layers complete before branch layers start
- Branches are processed in the order overhangs were detected

For a T-shaped part:
```
Branch 0 (stem):      layers 0..10  (upward from base)
Branch 1 (+X arm):    layers 11..15 (horizontal from layer 10)
Branch 2 (-X arm):    layers 16..20 (horizontal from layer 10)
```

**Downstream scheduling**:  
The `branch_id` allows the path planner to:
1. Group layers by branch for coherent print sequences
2. Insert travel moves between branches
3. Apply branch-specific speed/extrusion parameters

---

### 3.7 Termination and Coverage

**Queue exhaustion**:  
The algorithm terminates when the queue is empty (all branches fully sliced) or the iteration limit is reached (`max_iterations`).

**Iteration limit**:  
A safety parameter (default 20) prevents infinite loops in pathological cases:
- **Non-manifold meshes**: overlapping faces may trigger spurious overhangs
- **Recursive overhangs**: a branch spawning a sub-branch spawning a sub-sub-branch, etc.

If the limit is reached with runs still queued, a warning is logged and the partial result is returned.

**Coverage metric**:  
```cpp
coverage = min(1.0, total_layers / expected_planar_layers)
```

Where `expected_planar_layers = ⌈mesh_height / thickness⌉`.

**Interpretation**:
- `coverage ≈ 1.0`: geometry fully covered by main stem (no significant overhangs)
- `coverage > 1.0`: multi-directional branches produced more layers than pure vertical slicing would (clamped to 1.0 for reporting)
- `coverage < 1.0`: some regions skipped (non-manifold mesh or iteration limit hit)

---

## 4. Data Structures

### 4.1 Input: `geometry::TriangleMesh`

Same as PlanarSlicer (see PlanarSlicer §4.1). Additionally requires:
- **Per-vertex normals** (`mesh.normals`): used for normal interpolation at intersection points

**Preconditions**:
- Mesh must be **watertight** (manifold). Open meshes cause undefined overhang detection.
- Normals must be **outward-facing** (affects interpolated normal sign, critical for 5-axis tool orientation).

### 4.2 Output: `std::vector<Layer>`

Layers are identical in structure to PlanarSlicer output, with key differences:

| Field | PlanarSlicer | BaseSlicer |
|-------|--------------|------------|
| `branch_id` | Always 0 | 0 (stem), 1+ (branches) |
| `up_axis` | Global build axis | Dominant axis of `growth_dir` |
| `contour_normals` | Constant `(0,1,0)` | Interpolated from mesh normals |

**Segment interpretation**:  
Unlike PlanarSlicer, segments are **not chained into closed rings**. Each segment pair `(p_i, p_{i+1})` is independent. The path planner must:
1. Detect gaps between segments (non-consecutive endpoints)
2. Decide whether to insert a travel move or close a loop

### 4.3 Internal Structures

**SliceRun**:  
A queued slicing task.
```cpp
struct SliceRun {
    glm::vec3 growth_dir;  // Unit vector perpendicular to slice planes
    float start_h;         // Starting position along growth_dir
    uint32_t branch_id;    // Unique branch identifier
};
```

**PlaneBounds**:  
A 2D bounding rectangle in UV coordinates.
```cpp
struct PlaneBounds {
    float uMin, uMax, vMin, vMax;
    bool valid;  // false if bounds undefined (empty point set)
};
```

---

## 5. Complexity Analysis

### 5.1 Time Complexity

For a mesh with **T** triangles, **N** layers, **B** branches:

| Stage | Complexity | Notes |
|-------|------------|-------|
| UV computation | O(1) per run → O(B) total | Normalisation + cross products |
| Mesh extent projection | O(V) per run → O(BV) total | V = vertex count |
| Plane intersection | O(T) per layer → **O(BNT)** | Dominant term |
| Bounds computation | O(E) per layer | E = segments/layer ≈ T/N |
| Overhang detection | O(1) per layer | Four comparisons |
| Segment filtering | O(E) per layer | Midpoint test per segment |
| **Overall** | **O(BNT)** | Linear in branches × layers × triangles |

**Comparison with PlanarSlicer**:  
PlanarSlicer has **B = 1** (single upward run), so **O(NT)**.  
BaseSlicer with **B branches** is **B× slower**.

**Typical values**:  
- Simple cube: B = 1, same as planar
- T-shaped bracket: B = 3 (stem + 2 arms)
- Complex cantilever: B = 5–10

**Worst case**:  
A fractal-like geometry with overhangs at every layer could spawn O(N) branches → O(N²T), but this is not realistic for printable parts.

### 5.2 Space Complexity

- **Queue size**: O(B) in typical cases, O(N) worst-case
- **Layer storage**: O(BNE) for all layers
- **Transient buffers**: O(T) for intersection results
- **Peak memory**: O(T + BNE)

For a typical part (T = 10⁵, N = 100, B = 5, E = 1000), peak ≈ 500 MB.

---

## 6. Edge Cases and Robustness

### 6.1 Overhang Oscillations

**Problem**:  
If the mesh has small ripples (wavy surface), the bounds may oscillate layer-to-layer, triggering spurious overhang detections.

**Example**:  
Layer i: `uMax = 10.1 mm`  
Layer i+1: `uMax = 10.0 mm` (ripple contracts)  
Layer i+2: `uMax = 10.2 mm` (ripple expands)

With `tolerance = 0.1 mm`, layer i+2 triggers an overhang (+0.2 > +0.1), even though the net expansion from layer i is only +0.1 mm.

**Mitigation**:  
Use a **monotone envelope** for bounds:
```cpp
prev_bounds.uMax = max(prev_bounds.uMax, current_bounds.uMax);
```

This makes the allowed region non-decreasing, smoothing over small oscillations. *Not currently implemented* — considered future work (see §9.2).

### 6.2 Thin Vertical Walls

**Problem**:  
A thin vertical wall (e.g., 0.5 mm thickness) sliced at 0.2 mm layer height produces only 2–3 segments per layer. Bounds are extremely narrow:
- `uMin ≈ uMax` (wall thickness ≈ 0)

Numerical noise can cause `current_bounds.uMax > prev_bounds.uMax + ε` even when the wall is perfectly vertical.

**Mitigation**:  
The tolerance threshold (`thickness × tan(α)`) acts as a noise filter. For `α = 45°`, `tolerance = 0.2 mm × 1.0 = 0.2 mm`, which exceeds typical float32 epsilon (~10⁻⁶) by 6 orders of magnitude.

### 6.3 Non-Manifold Geometry

**Problem**:  
Overlapping faces or T-junctions create ambiguous intersection topology. Two faces may report the same segment, causing double-counting.

**Behavior**:  
- Bounds are computed from all segments (including duplicates) → inflated bounds
- Overhang detection is overly sensitive (spurious triggers)
- Coverage may exceed 1.0 (more layers than geometrically necessary)

**Recommendation**:  
Preprocess mesh with manifold repair (e.g., `libigl::resolve_duplicated_faces`).

### 6.4 Branch Collision

**Problem**:  
Two branches growing in opposite directions may intersect geometrically, causing segments from both branches to occupy the same spatial region.

**Example**:  
A hollow tube with overhangs on both sides:
- Branch 1 grows +X from the left wall
- Branch 2 grows -X from the right wall
- Both branches reach the tube's interior

**Behavior**:  
BaseSlicer does not detect or prevent this. Both branches emit overlapping layers. The result is:
- Duplicate contours in the same region
- Path planner may route the nozzle back-and-forth unnecessarily
- Potential material over-deposition if paths are not merged

**Mitigation**:  
Post-processing merge step (not implemented): detect spatially overlapping layers, merge their contours, and deduplicate segments.

---

## 7. Configuration Parameters

### 7.1 `BaseParams`

```cpp
struct BaseParams : SlicerParams
{
    float overhang_angle{45.0f};  // Degrees from growth direction
    int max_iterations{20};       // Safety limit for run count
    
    // Constraints:
    static constexpr float OVERHANG_ANGLE_MIN = 0.0f;
    static constexpr float OVERHANG_ANGLE_MAX = 89.0f;
    static constexpr int   MAX_ITERATIONS_MIN = 1;
    static constexpr int   MAX_ITERATIONS_MAX = 100;
};
```

**overhang_angle**:  
Controls the overhang steepness threshold.
- **0°**: no overhang allowed → every non-vertical surface triggers a branch
- **45°**: standard FDM overhang limit (1:1 slope)
- **60°**: permissive (allows steeper cantilevers)
- **89°**: nearly horizontal (only true horizontal overhangs trigger branches)

**Typical values**:
- **Conservative** (FDM without supports): 40–50°
- **Aggressive** (multi-axis robot): 60–70°

**max_iterations**:  
Safety cutoff to prevent infinite loops. Typical parts require B = 1–10 branches. If `max_iterations = 20` is exceeded, the mesh likely has:
- Non-manifold geometry
- Recursive overhang structure (e.g., spiral staircase)

**Inherited from SlicerParams**:
- `layer_thickness`: same as PlanarSlicer
- `up_axis`: initial build direction (0=X, 1=Y, 2=Z)

---

## 8. Testing and Validation

### 8.1 Unit Tests

**Coverage**:
1. **Empty mesh**: Verify empty output + warning.
2. **Vertical column** (no overhangs): Verify B = 1, all layers have `branch_id = 0`.
3. **T-shaped bracket**: Verify B = 3 (stem + 2 horizontal arms).
4. **Overhang angle threshold**: Mesh with 30° slope at `overhang_angle = 45°` → no branches; at `overhang_angle = 20°` → branches triggered.
5. **Iteration limit**: Pathological mesh exceeds `max_iterations` → partial output + warning.

**Regression guards**:
- **UV orthogonality**: `|U · V| < ε`, `|U · G| < ε`, `|V · G| < ε` for all runs.
- **Segment clipping**: After clipping, all segment endpoints satisfy `uMin ≤ u ≤ uMax` and `vMin ≤ v ≤ vMax` (with small tolerance for numerical precision).
- **Bounds clamping**: `prev_bounds` always satisfies `prev_bounds ⊆ allowed` after filtering.

### 8.2 Integration Tests

**Test meshes**:
1. **Cantilever beam**: 50 mm horizontal extension → verify branch spawns at overhang point.
2. **Hollow cone**: 45° walls → verify no branches at `overhang_angle = 45°`, branches at `overhang_angle = 40°`.
3. **Multi-level overhang**: Staircase with 3 steps → verify 3 branches.

**Metrics**:
- **Branch count**: Must match expected value ± 1 (small variations due to discretization).
- **Coverage**: 0.9 ≤ coverage ≤ 1.0 for watertight meshes.
- **Layer continuity**: No gaps > `2 × thickness` within a branch.

**Visual validation**:  
Render toolpath colored by `branch_id` in the 3D viewer. Verify:
- Stem (branch 0) covers vertical region
- Overhang branches (1+) extend from stem boundaries
- No overlapping branches in the same spatial region (except intentional merges)

---

## 9. Limitations and Future Work

### 9.1 Current Limitations

1. **No segment chaining**: Output segments are unordered. Path planner must infer contour topology.
2. **Branch collision**: No detection or merging of spatially overlapping branches.
3. **Single overhang per boundary**: Once a boundary is exceeded, all future layers are clipped. Nested overhangs (overhang → vertical → overhang) are not handled.
4. **Greedy queue order**: Branches are processed in detection order, not spatially optimized order.
5. **Normal interpolation**: Clipped segments retain original segment normals rather than recomputing interpolated normals at new endpoints (minor visual artifact in gradient-based toolpath generation).

### 9.2 Potential Improvements

**Monotone bounds envelope**:  
Track the maximum extent seen so far, preventing oscillation-triggered spurious overhangs:
```cpp
prev_bounds.uMax = max(prev_bounds.uMax, current_bounds.uMax);
```

**Normal interpolation for clipped endpoints**:  
When a segment is clipped, recompute normals at the new endpoints by interpolating along the original segment:
```cpp
t_clip = (p_clipped - p0) / (p1 - p0)
n_clipped = normalize(n0 + t_clip · (n1 - n0))
```

**Branch merging**:  
Post-process pass to detect branches with overlapping bounding boxes and merge their contours. Requires:
1. Spatial hash grid for fast overlap queries
2. Contour union operation (Clipper library or CGAL)

**Nested overhang support**:  
Allow multiple overhang events per boundary by tracking a **stack of allowed regions** instead of a single clipping rectangle.

**Adaptive angle threshold**:  
Vary `overhang_angle` based on local surface curvature. Flat regions tolerate higher angles; curved regions require lower.

**Priority queue scheduling**:  
Replace FIFO queue with a priority queue ordered by branch height or spatial proximity to minimise travel moves.

---

## 10. Comparison with Other Algorithms

| Algorithm | Planes | Branches | Normals | Use Case |
|-----------|--------|----------|---------|----------|
| **PlanarSlicer** | Horizontal | 1 | Constant | Standard 3-axis FDM |
| **BaseSlicer** | Multi-directional | B (dynamic) | Interpolated | Support-free cantilevers |
| **Crystal** | Iso-surfaces | B (tracked) | Gradient-aligned | Organic curved shells |

**When to use BaseSlicer**:
- Part has localized overhangs (T-brackets, wall mounts)
- Support-free printing is desired (material savings)
- Multi-axis printer is available (5+ DOF robot)

**When NOT to use BaseSlicer**:
- Pure vertical geometry → PlanarSlicer is faster
- Overhangs exceed robot reachability → conventional supports required anyway

---

## 11. References and Related Work

### 11.1 Multi-Directional Slicing

1. **Chakraborty et al. (2008)**: _"Extruder path generation for Curved Layer Fused Deposition Modeling"_. Early work on non-planar slicing with overhang detection.
2. **Lee & Hur (2015)**: _"A study on the development of a multi-axis 3D printer for curved layer fused deposition modeling"_. Hardware + software for rotational slicing.
3. **Xu et al. (2019)**: _"Support-Free Layered Process Planning Toward 3+2-Axis Additive Manufacturing"_. Queue-based algorithm similar to BaseSlicer, adds collision avoidance.

### 11.2 Overhang Analysis

4. **Strano et al. (2013)**: _"A new approach to the design and optimization of support structures in additive manufacturing"_. Defines overhang angle metrics for FDM.
5. **Vanek et al. (2014)**: _"Clever Support: Efficient Support Structure Generation for Digital Fabrication"_. Support-vs-reorientation trade-offs.

### 11.3 Internal Project References

- **PlanarSlicer** (`planar/`): Classical horizontal slicing (reference baseline).
- **Crystal** (`crystal/`): Harmonic field-based iso-surface extraction.
- **PathPlanner** (`common/path_planner.cpp`): Converts segments → travel + print moves.

---

## 12. API Usage Example

```cpp
#include "base/base_slicer.h"
#include "triangle_mesh.h"

// Load T-shaped bracket mesh
geometry::TriangleMesh mesh = loadMeshFromFile("t_bracket.stl");

// Configure slicer
slicing::BaseParams params;
params.layer_thickness = 0.2f;   // 200 µm layers
params.overhang_angle = 45.0f;   // Standard FDM limit
params.max_iterations = 20;      // Safety limit
params.up_axis = 1;              // Y-up (vertical)

// Slice
slicing::BaseSlicer slicer(params);
std::vector<slicing::Layer> layers = slicer.slice(mesh);

// Analyse results
std::map<uint32_t, int> branchCounts;
for (const auto& layer : layers) {
    branchCounts[layer.branch_id]++;
}

std::cout << "Total layers: " << layers.size() << "\n";
std::cout << "Branch count: " << branchCounts.size() << "\n";
for (const auto& [id, count] : branchCounts) {
    std::cout << "  Branch " << id << ": " << count << " layers\n";
}

// Example output for T-bracket:
// Total layers: 85
// Branch count: 3
//   Branch 0: 50 layers  (vertical stem)
//   Branch 1: 18 layers  (+X arm)
//   Branch 2: 17 layers  (-X arm)
```

## Limitations of BaseSlicer

### Asymmetric vs. Symmetric Overhangs

BaseSlicer is designed for **asymmetric, localized overhangs**
(e.g., T-brackets, cantilever beams). The algorithm spawns
branches that anchor on the parent geometry and grow outward
along the overhang.

**Symmetric overhangs** (e.g., bipyramids/hourglass shapes,
where geometry expands in all directions from a constriction)
fundamentally challenge this approach: there is no "anchor wall"
for sideways branches, only the constriction point itself.

For such geometries, BaseSlicer falls back to one of two
behaviors:

1. With high overhang_angle (≥45°): the overhang is not detected,
   and the upper geometry is omitted from slicing.
2. With low overhang_angle (<45°): four parallel branches are
   spawned in all UV directions, each printing one quadrant
   of the overhang.

Neither result is mechanically optimal. **For bipyramidal
geometries, support structures or alternative algorithms
(e.g., Crystal with curved iso-surfaces) are recommended.**

---

## 13. Glossary

| Term | Definition |
|------|------------|
| **Growth direction** | Unit vector perpendicular to slice planes (analogous to Z-up in planar slicing). |
| **UV plane** | Two orthonormal axes perpendicular to the growth direction, forming the slice coordinate frame. |
| **Overhang** | A region where the current layer extends beyond the previous by more than the tolerance threshold. |
| **Tolerance** | `thickness × tan(overhang_angle)` — the maximum horizontal extension allowed before triggering a branch. |
| **Branch** | A slicing run with its own local growth direction, spawned from an overhang. |
| **Slicing run** | One pass through the mesh along a fixed growth direction, producing a sequence of layers. |
| **Allowed bounds** | The 2D rectangle in UV coordinates within which segments are kept; tightened when overhangs are detected. |
| **Clipping** | Filtering segments to exclude overhang regions after a branch is spawned. |
| **Stem** | The initial upward slicing run (`branch_id = 0`). |
| **Breadth-first** | Processing runs in queue order (FIFO), ensuring all stem layers complete before branch layers. |
| **Manifold mesh** | A closed, watertight surface with no holes or overlapping faces. |
