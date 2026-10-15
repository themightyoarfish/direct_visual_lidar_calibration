# Histogram-based NID alignment

Fine calibration searches for the **camera–LiDAR pose** that makes **what the camera sees** and **what the LiDAR measures** line up statistically. For each 3D point you ask: if I project this point into the image with the current pose, does the **gray value at that pixel** tend to go with this point's **LiDAR intensity**?

When the pose is wrong, points land on unrelated pixels, so gray and intensity look like unrelated random draws. When the pose is right, the same surfaces produce similar brightness in the image and similar (rank-normalized) intensity in the cloud, so the two signals look **coupled**.

The optimizer **minimizes NID** (Normalized Information Distance), a number built from **histograms** of those paired values.

## Glossary (math terms in plain language)

**Histogram**  
A table of counts: how many times each brightness bucket appears. Here we also build a 2D table of (image gray, LiDAR intensity) pairs.

**Joint histogram**  
The 2D table: cell \((i, j)\) counts how many projected points had image bin \(i\) and LiDAR bin \(j\) at once.

**Marginal histogram**  
Counts for one variable only, ignoring the other. Image marginals are row sums of the joint table; LiDAR marginals are column sums. “Marginal” just means “one dimension of the pair.”

**Entropy**  
A single number that answers: *how unpredictable is this histogram?*

- If almost all points fall in one bin, entropy is **low** (you can guess the value easily).
- If points are spread evenly across many bins, entropy is **high** (lots of uncertainty).

In code, entropy is computed from bin probabilities \(p_k\) with a sum of \(-p_k \log p_k\) (with a tiny floor so \(\log 0\) never happens). You do not need the formula to use the idea: **entropy measures spread / surprise in one distribution**.

**Mutual information (MI)**  
A number that answers: *if I know the image gray for a point, how much does that tell me about its LiDAR intensity (and vice versa)?*

- **High MI**: the two tend to occur in predictable combinations (e.g. dark image bins often pair with low LiDAR bins). They are **dependent**.
- **Low MI**: knowing one barely changes what you expect for the other. They look **independent**, like two unrelated dice rolls.

MI is built from three entropies: image alone, LiDAR alone, and the **joint** pair. When alignment is wrong, pairings are scrambled and MI drops.

**NID (Normalized Information Distance)**  
A normalized score derived from joint entropy and MI (see below). **Lower is better** for calibration: it means image and LiDAR intensities look more tightly linked given the current pose.

**Simplex (Nelder–Mead)**  
Not “a triangle in math class” only—in optimization, a **simplex** is a small set of **trial solutions** that span the search space. For 6 pose parameters, Nelder–Mead keeps **7** poses (the current best plus six perturbed copies, one per parameter direction).

The algorithm moves that set of trials: reflect the worst point through the centroid, try expanding or shrinking the shape, and replace bad trials with better ones—**without computing derivatives**. Picture an amoeba crawling downhill on the NID landscape.

**Gradient**  
The direction in pose space where NID increases fastest. Moving opposite to the gradient (with a suitable step size) lowers NID. BFGS estimates this from automatic differentiation through the spline-sampled image.

**BFGS**  
A gradient-based iterative method: build an approximate curvature of the cost, pick a descent direction, line-search along it, repeat. Fast when the cost is smooth (hence splines in `nid_bfgs`).

**6-DOF pose / SE(3)**  
Six numbers: three for rotation, three for translation, describing how the LiDAR frame sits relative to the camera.

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

From the normalized histograms the code computes:

- \(H_\text{image}\): entropy of image grays (marginal).
- \(H_\text{lidar}\): entropy of LiDAR intensities (marginal).
- \(H_\text{joint}\): entropy of the **pair** (joint histogram).

Then:

\[
\text{MI} = H_\text{image} + H_\text{lidar} - H_\text{joint}
\]

**MI** is large when the joint histogram is more “peaked” than you would expect from the marginals alone—that is, when gray and intensity tend to show up together in specific bins.

**NID** in this codebase is:

\[
\text{NID} = \frac{H_\text{joint} - \text{MI}}{H_\text{joint}}
\]

You can read this as: *what fraction of the joint “spread” is **not** explained by shared information between image and LiDAR?*

- **Low NID**: MI is a large part of \(H_\text{joint}\). Gray and intensity **move together** across points—good alignment.
- **High NID**: MI is small relative to \(H_\text{joint}\). Pairings look **random**—bad alignment.

Calibration **minimizes NID**.

NID is **global**: one score per frame (or sum over several bags), not a per-point residual. That makes it robust to lighting differences as long as **statistical** agreement holds, but it needs enough points and varied scene content.

## How the optimizers use NID

Both methods adjust the **6-DOF pose** (rotation + translation). There is an **outer loop**: re-run inner optimization, update view culling, stop when the pose barely changes.

### `nid_bfgs` (default)

- Uses **Ceres** on the SE(3) pose with **automatic differentiation** through the full NID computation (`NIDCost`).
- **Sub-pixel image gray**: cubic B-spline interpolation at the projected location so NID changes smoothly when the pose nudges. That gives usable **gradients** (see glossary).
- **BFGS** (with line search) follows descent directions on the summed NID over all dataset frames.
- Stays near the initial guess: updates that move more than ~**20 cm** or ~**2°** from the current iterate are rejected.
- Generally **faster and smoother** when the initial pose is already reasonable.

### `nid_nelder_mead`

- Treats NID as a **black box**: no gradients; each trial pose is evaluated by rebuilding histograms and NID.
- Pose is `init_pose * exp(ξ)` with a 6-vector `ξ` (small rotation/translation in tangent space).
- **Nelder–Mead** maintains a **simplex** of trial poses (see glossary): reflect, expand, or contract the set to find lower NID.
- Gray is read at **integer pixels** (`CostCalculatorNID`), not splines.
- Tuned with `--nelder_mead_init_step` and `--nelder_mead_convergence_criteria`.
- Useful when you want a **derivative-free** search or BFGS is brittle; often **slower** and more sensitive to step size.

## Intuition checklist

- **Histogram** = how often do we see dark camera + bright LiDAR, mid + mid, and so on, for the current projection.
- **Bins** = coarseness of that table; default 16 is a compromise.
- **Entropy** = how spread out one histogram is; **MI** = how much two histograms “agree” when paired.
- **NID** = how much of the joint spread is *not* shared information; push it down by improving the pose.
- **BFGS** = gradient-based, spline-smoothed; **Nelder–Mead** = simplex of trial poses, pixel-snapped, no derivatives.
