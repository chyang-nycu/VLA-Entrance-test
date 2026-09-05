---
theme: academic
layout: cover
class: text-white
coverDate: "2026-09-03"
fonts:
  sans: Montserrat
  serif: Roboto Slab
  mono: Roboto Mono
hideInToc: true
themeConfig:
  paginationX: r
  paginationY: t
  paginationPagesDisabled: [1]
title: From Classical Control to VLA-Oriented G1 Demonstrations
info: |
  # From Classical Control to VLA-Oriented G1 Demonstrations
  A research-oriented manipulation project on the Unitree G1: failure-driven
  controller redesign, an action-representation study, and an
  object-conditioned task-specification evaluation, audited against the
  original entrance-test scope.
mdc: true
transition: slide-left
---

# From Classical Control to VLA-Oriented G1 Demonstrations

<div class="text-lg opacity-80 mt-4 max-w-2xl">
Research question: when a classical expert controller fails, what does
diagnosing it teach us about the interfaces a future learned policy
would need?
</div>

---

# Entrance-Test Ask vs. What Was Delivered

<div class="grid grid-cols-2 gap-6 text-sm">
<div>

**Asked for**
- Up to 3 Unitree G1 tasks, increasing difficulty, in Isaac Lab
- Text instruction + setup variants per task
- Verified model-free environment control
- A data-collection pipeline, planned for VLA training
- *Optional*: integrate an existing policy, run inference

</div>
<div>

**Delivered**
- **2 of 3** tasks, genuinely increasing in difficulty
- **MuJoCo**, not Isaac Lab — a host-availability deviation
- Model-free control: **complete**, real contact/actuation
- VLA-oriented data pipeline: **complete**
- Existing-policy integration: **not attempted**

</div>
</div>

<div class="mt-6 p-3 rounded bg-white/5 border text-sm text-center">
Full requirement-by-requirement evidence is on the coverage slide later
in this talk.
</div>

---

# What I Owned

<div class="grid grid-cols-2 gap-8 text-sm">
<div>

**Research / engineering decisions**
- Problem decomposition and task scope
- Acceptance criteria and evaluation metrics
- Controller architecture decisions
- Failure hypotheses and instrumentation design
- Experiment design and result interpretation
- VLA observation/action interface design
- Judging whether a result was actually valid

</div>
<div>

**Implementation workflow**

I used an AI coding agent to accelerate implementation, testing, and
documentation.

I remained responsible for problem decomposition, architecture
decisions, experiment design, reviewing generated implementations,
interpreting failures, and validating every claimed result.

</div>
</div>

---

# Building the Missing Manipulation Interface

<div class="grid grid-cols-2 gap-8 items-center text-sm">
<div>

```mermaid {scale: 0.6}
graph TD
  V["Vendor G1 model<br/>(unmodified)"] --> T["Task-local layer:<br/>gripper, scene, controller"]
  T --> P["MuJoCo physics"]
  T --> C["Onboard RGB camera"]
```

</div>
<div>

- Audited every pinned G1 variant: **no actuated gripper or hand** in
  any of them
- A free-standing balance probe showed **0.93 m of pelvis drift in
  2 s** → fixed pelvis+torso adopted deliberately
- Built a task-local parallel-jaw gripper (2 actuated fingers, real
  collision pads, explicit TCP site), kept separate from the
  unmodified robot model

</div>
</div>

<div class="mt-4 text-sm opacity-80 border-t pt-3">
All manipulation success is produced through simulated physical
contact and actuation — no post-reset teleportation, attachment, or
artificial grasp assistance.
</div>

---

# The Failure Was Architectural, Not a Gain-Tuning Problem

```mermaid {scale: 0.65}
graph LR
  A["Torque PD"] -->|"5x actuator<br/>mismatch"| B["Per-joint<br/>gains"]
  B -->|"error moves,<br/>doesn't shrink"| C["Position servos<br/>+ waypoint IK"]
  C --> D["Deterministic<br/>grasp"]
```

<div class="mt-2 text-sm max-w-4xl">
Shoulder/elbow vs. wrist torque authority differs <b>~5×</b>; one
shared gain pair could not serve both. Re-tuning gains only <i>moved</i>
the tracking error (0.061 → 0.215 m in one variant). Replacing the
architecture converged deterministically: <b>0.108 m lift, ~3.5 s hold,
5/5</b> — two independently-budgeted tuning attempts on the same
architecture failed before I replaced the representation itself.
</div>

---

# Task 1: Reliable Inside a Measured Envelope, Not Beyond It

<div class="grid grid-cols-5 gap-8 items-center">
<div class="col-span-2">
<img src="/task1_grasp.png" class="rounded shadow max-h-52 object-contain mx-auto" />
</div>
<div class="col-span-3 text-sm">

- Deterministic nominal success, **5/5** identical reruns
- **3/3 (100%)** of the IK-reachable cube-position variants complete
  the task; 2 of 5 sampled offsets fall outside the reachable envelope
  (predicted by IK check, confirmed live)

</div>
</div>

<div class="mt-4 p-4 rounded bg-white/5 border text-sm">
<b>Limitation, stated plainly:</b> the strict engineering grasp-slip
target (≤10 mm while grasped) is <b>not met</b> — measured ~25.9 mm.
Task-level behavior is reliable inside the supported envelope; grasp
quality is below the stricter target.
</div>

---

# Task-Space Safety ≠ Trajectory Safety

<div class="grid grid-cols-2 gap-8 text-sm">
<div>

**Expected**: an 8 cm Cartesian gap to a second, nearby object looked
safe by static geometric reasoning.

**Observed**: full-episode instrumentation measured **48.7 mm** of
real displacement — ~5× the 10 mm design target.

**Diagnosis**: traced to the retreat motion's joint-space path
sweeping close to the object — a segment never Cartesian-monitored,
because the static 8 cm gap had looked sufficient.

</div>
<div>

```mermaid {scale: 0.62}
graph TD
  A["8cm task-space gap<br/>(looks safe)"] --> B["Full-episode<br/>displacement probe"]
  B --> C["Measured: 48.7mm"]
  C --> D["Root cause: joint-space<br/>retreat path"]
```

<div class="mt-3 p-3 rounded bg-white/5 border text-xs">
Fix: re-evaluated candidates on <b>reachability + measured
displacement + camera visibility</b> jointly. Final slot: <b>0.0 /
1.7 mm</b>.
</div>

</div>
</div>

---

# Policy Observations vs. Privileged State: A Clean Split

```mermaid {scale: 0.7}
graph LR
  E[Expert] --> O[Observations] --> A[Actions] --> D[Dataset] --> R[Replay] --> L["Future<br/>policy"]
```

<div class="grid grid-cols-2 gap-8 mt-6 text-sm">
<div>

**Policy-facing observations**
- RGB
- Robot joint state
- TCP pose
- Gripper state

</div>
<div>

**Privileged expert / evaluation state**
- Cube pose, target pose
- Contact state
- State-machine phase

</div>
</div>

<div class="mt-4 text-sm opacity-80 text-center">
Privileged state is recorded for evaluation but never silently exposed
as learner input — a design choice, not an afterthought.
</div>

---

# A 10 Hz Action Cannot Faithfully Represent a 500 Hz Expert

<div class="text-sm">

| Representation | What it tests | Max TCP replay error |
|---|---|---|
| Naive 10 Hz hold | action representation | 48.7 mm |
| Exact 500 Hz execution replay | simulation determinism | 3.65 × 10⁻⁸ m |
| Static per-phase goal @ 10 Hz | action representation | ~97 mm |
| **H=5 sub-action chunks @ 50 Hz (shipped)** | action representation | **8.09 mm** |

</div>

<div class="mt-4 p-3 rounded bg-white/5 border text-sm text-center">
Exact replay ruled out simulation nondeterminism — the gap was
entirely in the <b>action representation</b>. Exact-execution replay,
policy-action replay, and a future learned policy are three distinct
things; no model is involved in any row above.
</div>

---

# Object-Conditioned Task Specification, Not Language Grounding

<div class="grid grid-cols-2 gap-4">
<img src="/task2_red_done.png" class="rounded shadow max-h-48 object-contain mx-auto" />
<img src="/task2_green_done.png" class="rounded shadow max-h-48 object-contain mx-auto" />
</div>

<div class="grid grid-cols-2 gap-8 mt-4 text-sm">
<div>

Same physical scene, different task specification, different selected
object. `selected_object_id` is supplied by the task specification —
it is not parsed from text or grounded visually in the control loop.

</div>
<div>

- 4 configurations × 3 deterministic trials = **12/12**
- **0** wrong-object placements, distractor displacement **0.0–1.73 mm**
- Both instructions route to the **same** target pad — the
  instruction changes *which object*, not *where it goes*

</div>
</div>

---

# Entrance-Test Coverage

<div class="text-sm">

| Requirement | Status | Evidence |
|---|---|---|
| Unitree G1 in simulation | **COMPLETE** | Fixed-base, real contact, no teleport |
| Isaac Lab (as specified) | **DEVIATION** | MuJoCo used; Isaac Lab unavailable on dev host |
| Task 1 — pick-and-place | **COMPLETE** | 5 spatial variants, 3/3 reachable succeed |
| Task 2 — object-conditioned select | **PARTIAL** | Instruction selects object; shared single target |
| Task 3 — tool use / multi-object | **NOT ATTEMPTED** | Out of the time box |
| Model-free control, physical interaction | **COMPLETE** | Classical IK + servos, real contact/actuation |
| VLA-oriented data pipeline | **COMPLETE** | RGB, proprioception, actions, privileged split, replay |
| Existing-policy integration / inference | **NOT ATTEMPTED** | No model run in this project |

</div>

---
layout: center
class: text-center
---

# What's Next, and Why I Fit This Direction

<div class="grid grid-cols-2 gap-10 mt-6 text-left text-sm">
<div>

**What I'd study next**
1. Replace privileged object selection with a **grounded** RGB +
   instruction → object representation
2. Fit the **simplest** baseline first — behavior cloning on existing
   demonstrations before a large VLA
3. Test **real generalization**: unseen positions, paraphrases, new
   object classes — not just repeats of tested configurations

</div>
<div>

**Why I fit**
- **Robotics**: I can build, instrument, and debug a physically
  grounded manipulation pipeline
- **ML systems**: representation, sampling rates, data interfaces,
  and execution fidelity are habits I already have
- **Research**: comfortable with failed hypotheses and changing
  architecture when measurements contradict the design

</div>
</div>

<div class="mt-8 text-base max-w-2xl mx-auto">
I am still early in robotics, but I can already contribute at the
boundary between robot learning and ML systems.
</div>

---
layout: section
hideInToc: true
---

# Appendix
## Detailed Evidence for Technical Questions

---

# Appendix A1 — Environment, Vendor Boundary, and the Isaac Lab Deviation

- Host: Intel Mac, macOS 12.7.6, x86_64
- **Isaac Lab requires Linux/Windows and was unavailable on this
  host.** `unitree_mujoco` / MuJoCo was used instead. This is a
  **deviation from the literal entrance-test specification**, not a
  documented, coordinator-approved substitution — no authorization
  record exists in this repository
- `unitreerobotics/unitree_mujoco` pinned at commit `4134cb5`, a git
  submodule; vendor files are never edited
- All manipulation-specific code lives in a task-local layer, checked
  at import time against the unmodified vendor model

---

# Appendix A2 — Controller Tuning History (Full)

| Attempt | Change | Result |
|---|---|---|
| 3-1..3 (torque PD) | gain/feedforward/margin tuning | height 0.005 m of 0.08 m required |
| 3B-1 | per-joint gains by torque authority | TCP RMS 0.061→0.215 m, contact lost |
| 3B-2 | + settle-before-close gate | gate never converged |
| 3B-3 | revert gains + torque-weighted IK | height 0.0005 m of 0.08 m required |
| 3C-1 | position servos, old gripper gains | height 0.0497 m, held 0.198 s, dropped |
| 3C-2 | position servos, gripper gain only | **height 0.108 m, held 3.5 s, 5/5** |

---

# Appendix A3 — Full Task 1 State Machine

<div class="flex flex-wrap items-center gap-1 text-xs max-w-full">
<span class="px-2 py-1 border rounded">RESET</span><span>→</span>
<span class="px-2 py-1 border rounded">PREGRASP</span><span>→</span>
<span class="px-2 py-1 border rounded">APPROACH</span><span>→</span>
<span class="px-2 py-1 border rounded">CLOSE</span><span>→</span>
<span class="px-2 py-1 border rounded">VERIFY_CONTACT</span><span>→</span>
<span class="px-2 py-1 border rounded">LIFT</span><span>→</span>
<span class="px-2 py-1 border rounded">HOLD</span><span>→</span>
<span class="px-2 py-1 border rounded">TRANSPORT</span><span>→</span>
<span class="px-2 py-1 border rounded">LOWER</span><span>→</span>
<span class="px-2 py-1 border rounded">OPEN</span><span>→</span>
<span class="px-2 py-1 border rounded">VERIFY_RELEASE</span><span>→</span>
<span class="px-2 py-1 border rounded">RETREAT</span><span>→</span>
<span class="px-2 py-1 border rounded">VERIFY_SUCCESS</span><span>→</span>
<span class="px-2 py-1 border rounded">DONE / FAILED</span>
</div>

<div class="mt-6 text-sm">
Objective success detector: 8 conditions, all required continuously
through a settling dwell window. Transport aborts (reports FAILED)
rather than faking success if bilateral contact is lost.
</div>

---

# Appendix A4 — Setup-Variant Reachability (Task 1, Full Table)

| Variant | Reachable? | Outcome |
|---|---|---|
| nominal | yes | succeeds |
| x − 0.03 | yes | succeeds |
| y + 0.03 | yes | succeeds |
| x + 0.03 | no (27.1 mm IK residual) | fails at `SETTLE_APPROACH` |
| y − 0.03 | no (8.4 mm IK residual) | fails at `SETTLE_APPROACH` |

Both failures were predicted by a pre-run IK feasibility check before
any trial executed, and the live trial agreed exactly.

---

# Appendix A5 — A Slip Metric That Was Correct Math, Wrong Window

The reported slip metric (15.6 cm) accumulated displacement for the
entire rest of the trial after grasp, including phases where the cube
is *supposed* to separate from the gripper.

- Post-release separation alone: **14.9 cm** — nearly the whole old figure
- The TCP-local-frame transform itself was already correct
- Corrected, gated on the "carrying" signal: genuine grasp-phase slip
  is **3.3–5.4 cm** — no change to any pass/fail outcome

---

# Appendix A6 — Passing Tests ≠ Complete Evaluation

<div class="grid grid-cols-2 gap-6 items-center">
<div class="text-sm">

A vendor decorative hand mesh visibly clipped through the cube on
every grasp — found by human visual review after **104/104**
automated tests were already green.

The tests checked contact forces, heights, and velocities. None
checked whether an unrelated, collision-free visual mesh overlapped
the cube — zero prior coverage for that failure mode.

</div>
<img src="/decorative_hand_defect.png" class="rounded shadow max-h-56 object-contain mx-auto" />
</div>

---

# Appendix A7 — An Orientation Fix That Didn't Fix Slip

- Hypothesis: free wrist-roll drift during descent causes off-axis,
  corner-only contact, driving grasp slip
- Measured wrist orientation: jaw axis well-aligned (~4–5° off); the
  "tall" axis was **47° off vertical**
- A stronger orientation objective found a genuine **kinematic
  reachability conflict** — leveling the wrist costs 30–70 mm of
  position error
- A measured finger-mount correction improved contact offset by 17%
  but did **not** reduce overall slip

---

# Appendix A8 — Scaling Exposes a Generalization Gap

<div class="grid grid-cols-2 gap-4">
<img src="/spatial_by_split.png" class="rounded shadow max-h-56 object-contain mx-auto" />
<img src="/replay_tcp_error.png" class="rounded shadow max-h-56 object-contain mx-auto" />
</div>

<div class="text-sm mt-3">
32 configurations sampled from a continuous cube-position envelope:
the shipped decoder meets the 10 mm target on 22/29 (76%) episodes.
All 7 offenders first diverge post-release, at `RETREAT` — placement
and success are unaffected. Reported as an open gap, not fixed here.
</div>

---

# Appendix A9 — Full Limitations

**Task 1**: strict ≤10 mm slip bar not met (25.9 mm); fixed target
pad; some cube positions outside the reachable envelope; task-local
gripper only; torso-mounted camera; 7/29 scaled replay episodes
exceed 10 mm (all post-release).

**Task 2**: optional, time-boxed; object identity is privileged
metadata, not inferred; both instructions share one target pad; only
2 spatial arrangements were tested, each with 3 *deterministic
repeats*, not distinct seeds; no scaled dataset or policy integration.

**Project-wide**: no model has been trained or evaluated. Every result
above is produced by a scripted expert controller.

---

# Appendix A10 — Process Evidence: Design, Problems, and Time

- Task design and rationale, problem encounters, and results are
  logged per development phase in the project's own working notes
- Selected failures needing the most research reasoning: torque-control
  instability (Appendix A2), the decorative-hand visual defect
  (Appendix A6), and the action-representation replay gap (main deck)
- **Time commitment**: per-phase estimates exist for roughly half the
  development phases (~12 hours documented across the phases that
  logged it); a single aggregated total across the full project was
  **not tracked** — flagged here rather than estimated
- 311 regression tests, 0 unexpected failures, reproducible from a
  fixed seed and a committed config hash
- Task 1 and Task 2 demonstration videos exist and are decode-verified
