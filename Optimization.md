# Histogram-based NID alignment

Fine calibration searches for the **camera–LiDAR pose** that makes **what the camera sees** and **what the LiDAR measures** line up statistically. For each 3D point you ask: if I project this point into the image with the current pose, does the **gray value at that pixel** tend to go with this point's **LiDAR intensity**?

When the pose is wrong, points land on unrelated pixels, so gray and intensity look like unrelated random draws. When the pose is right, the same surfaces produce similar brightness in the image and similar (rank-normalized) intensity in the cloud, so the two signals look **coupled**.

The optimizer **minimizes NID** (Normalized Information Distance), a number built from **histograms** of those paired values.

## Step by step: building the histograms

1. **Prepare signals** (mostly in preprocessing):
   - Image: grayscale, then values in \([0, 1]\) (divide by 255).
   - LiDAR: intensities are **histogram-equalized** per cloud so they span \([0, 1]\) in rank order (comparable spread even when raw reflectivity units differ).

2. **For a trial pose** `T_camera_lidar`:
   - Transform each point into the camera frame and project it into the image.
   - Skip points outside the image or field of view (and optionally hidden behind closer points via depth-buffer culling).

3. **Bin each pair** into a **2D joint histogram** of size `bins × bins`:
   - One axis: image gray bin.
   - Other axis: LiDAR intensity bin.
   - Bin index is essentially `floor(value × bins)`, clamped to `0 … bins-1`.

4. **Marginals**: count how often each image bin and each LiDAR bin appear (row/column sums of the joint histogram).

5. **Normalize** counts to probabilities (divide by total mass).

The BFGS and Nelder–Mead paths use the same histogram idea; they differ mainly in **how** gray is read at the projected location (see [Optimizers](#how-the-optimizers-use-nid)).

## What the bins do

Think of bins as **brightness buckets** on a 0–1 scale.

| Setting | Effect |
|--------|--------|
| **More bins** (e.g. 32 vs 16) | Finer distinction between dark vs mid vs bright; can capture sharper structure when alignment is good. |
| **Fewer bins** | Smoother, more stable estimate from many points; less detail. |
| **Too many bins** | Many cells stay nearly empty → noisy mutual information → flaky NID. |
| **Too few bins** | You merge real structure (e.g. shadows vs highlights) → weaker discrimination between good and bad poses. |

Default in `calibrate` is **16** (`--nid_bins`). Image and LiDAR use the **same** bin count and the same \([0,1]\) mapping, so you are comparing **coarse joint statistics**, not pixel-perfect photoconsistency.

## What NID means (and MI underneath)

From the histograms the code computes **entropies** (how spread out a distribution is) and **mutual information (MI)**:

- \(H_\text{image}\), \(H_\text{lidar}\): uncertainty in gray and in intensity alone.
- \(H_\text{joint}\): uncertainty in the **pair** (gray, intensity).
- **MI** \(= H_\text{image} + H_\text{lidar} - H_\text{joint}\): how much knowing one reduces uncertainty about the other. High MI ⇒ the two variables are **statistically linked**.

**NID** in this codebase is:

\[
\text{NID} = \frac{H_\text{joint} - \text{MI}}{H_\text{joint}}
\]

In plain terms:

- **Low NID** ⇒ a large share of the joint uncertainty is **shared** (high MI relative to the joint histogram). Gray and intensity "move together" across points, which is what you want when projection is correct.
- **High NID** ⇒ they look more **independent** (pairings could be random). Typical of a bad pose.

So calibration **minimizes NID**. It is related to normalized mutual-information style metrics: when alignment is good, dependence is strong and NID goes down.

NID is **global**: one score per frame (or sum over several bags), not a per-point residual. That makes it robust to lighting differences as long as **statistical** agreement holds, but it needs enough points and varied scene content.

## How the optimizers use NID

Both methods adjust the **6-DOF pose** (rotation + translation). There is an **outer loop**: re-run inner optimization, update view culling, stop when the pose barely changes.

### `nid_bfgs` (default)

- Uses **Ceres** on the SE(3) pose with **automatic differentiation** through the full NID computation (`NIDCost`).
- **Sub-pixel image gray**: cubic B-spline interpolation at the projected location so NID changes smoothly when the pose nudges. That gives usable **gradients**.
- **BFGS** (with line search) follows descent directions on the summed NID over all dataset frames.
- Stays near the initial guess: updates that move more than ~**20 cm** or ~**2°** from the current iterate are rejected.
- Generally **faster and smoother** when the initial pose is already reasonable.

### `nid_nelder_mead`

- Treats NID as a **black box**: no analytical gradients.
- Pose is `init_pose * exp(ξ)` with a 6-vector `ξ` (small rotation/translation in tangent space).
- **Nelder–Mead simplex**: maintains a simplex of trial poses, reflects/expands/contracts it based on which trials lower NID.
- Gray is read at **integer pixels** (`CostCalculatorNID`), not splines.
- Tuned with `--nelder_mead_init_step` and `--nelder_mead_convergence_criteria`.
- Useful when you want a **derivative-free** search or BFGS is brittle; often **slower** and more sensitive to step size.

## Intuition checklist

- **Histogram** = how often do we see dark camera + bright LiDAR, mid + mid, and so on, for the current projection.
- **Bins** = coarseness of that table; default 16 is a compromise.
- **NID** = how unrelated do gray and intensity look? Push it down by sliding the pose until projected points sit on pixels whose brightness **co-varies** with their LiDAR intensity.
- **BFGS** = gradient-based, spline-smoothed; **Nelder–Mead** = simplex, pixel-snapped, no derivatives.
