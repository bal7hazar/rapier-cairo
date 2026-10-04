#!/usr/bin/env python3
"""Generate the Rapier/Parry 2D API parity inventory.

Dependency-free and conservative: mask comments/strings, find balanced blocks, recognize public
declarations/common impls, classify `(owner, kind, name)` items, and embed the Rust inventory in
docs/API_PARITY.md so normal generation and `--check` do not need upstream checkouts.
"""

from __future__ import annotations

import argparse
import difflib
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "docs" / "API_PARITY.md"
RAPIER_VERSION, PARRY_VERSION, GOLDEN_PARRY = "0.35.3+4", "0.31.1", "0.30.2"
INVENTORY_START, INVENTORY_END = "<!-- api-parity-rust-inventory\n", "\napi-parity-rust-inventory -->"

RAPIER_DIRS, PARRY_DIRS = ("dynamics", "geometry", "pipeline", "control"), ("shape", "query", "bounding_volume", "mass_properties")

OPEN = {"{": "}", "(": ")", "[": "]"}
RUST_IMPL_TRAITS = set("""
Add AddAssign Sub SubAssign Mul MulAssign Div DivAssign Neg Not Index IndexMut From TryFrom
Into Default Clone Copy Debug PartialEq Eq PartialOrd Hash Serialize Deserialize Archive Pod
Zeroable AbsDiffEq RelativeEq UlpsEq Shape PointQuery RayCast PointQueryWithLocation SimdValue
""".split())
CAIRO_IMPL_TRAITS = set("""
Add AddAssign Sub SubAssign Mul MulAssign Div DivAssign Neg Not BitAnd BitOr BitXor BitNot
IndexView Index Default Into TryInto PartialEq Drop Copy Serde Debug PointQuery RayCast PointQueryWithLocation
""".split())

EXCLUSIONS = (
    "dim3-only", "soft bodies", "multibody", "SIMD/parallel", "debug render",
    "serde/rkyv/bytemuck", "profiling counters", "dyn hooks",
    "trimesh/voxels/3D heightfield", "EPA/GJK internals not exposed",
    "solver / island internals not exposed",
    "f32/f64 conversions and approx traits",
    "soft bodies are not part of the port",
    "Q32.32 state cannot become NaN or infinite; nothing to contain",
    "static dispatch through StepConfig / StageConfig (D10)",
    "persistent mutable graph not ported (D7): values cannot hand out mutable references into the step's storage",
    "closed value enum replaces Arc / dyn shapes (SH2a)",
    "Cairo has no IndexMut / &mut index",
    "persisted contact-graph order, no faithful Default",
    "no faithful form: fixed solver-contact array and count; anchors are offsets at the step's start",
    "construction-time decomposition, several thousand lines, no consumer; reopened on a consumer's need",
    "solver / island internals not exposed (dynamics::solver is pub(crate) upstream)",
    "contact skin changes the collider layout and the contact solver for every user; no opt-in form keeps existing steps",
    "a removal log is a field of the stepped collider set: +0.1 to +0.2 % Cairo steps per tick, +27,580 on the owner's pile10 shot, in process (WS3, commit 16ac69c)",
)

# PX1 (2026-09-27, programme decision): the reasons above this line predate PX1; the coverage
# summary reports "raw" parity against them alone, and "in scope" parity against every reason,
# so closing SOLVER_ISLAND_INTERNALS below cannot quietly raise the headline number.
SOLVER_ISLAND_REASON = "solver / island internals not exposed"

# Programme decision (2026-09-29): three more closed reasons, each item listed by exact
# `(owner, kind, name)`; like PX1's they count in "in scope" only, the raw figure ignores them.
SOFT_CONTACTS_REASON = "soft bodies are not part of the port"
QUARANTINE_REASON = "Q32.32 state cannot become NaN or infinite; nothing to contain"
DISPATCHER_REASON = "static dispatch through StepConfig / StageConfig (D10)"
# Upstream 0.35's soft-body contact detection (`geometry/narrow_phase/soft_contacts/`) and its accessor on
# `ContactPair`. `SoftEdgePass` / `SoftVolumePatch` used to match the "epa" pattern of the EPA/GJK reason.
SOFT_CONTACTS: frozenset[tuple[str, str, str]] = frozenset({
    ("ContactPair", "method", "soft"),
    ("SoftContactImpulse", "type", "SoftContactImpulse"),
    ("SoftDetectionCtx", "method", "motion_margin"), ("SoftDetectionCtx", "method", "pieces_of_one_body"),
    ("SoftEdgeCandidate", "type", "SoftEdgeCandidate"), ("SoftEdgePass", "type", "SoftEdgePass"),
    ("SoftPairContacts", "method", "disable_all"), ("SoftPairContacts", "method", "impulses"),
    ("SoftPairContacts", "method", "is_touching"), ("SoftPairContacts", "method", "vertex_pass_on"),
    ("SoftPairContacts", "type", "SoftPairContacts"), ("SoftRigidPatch", "type", "SoftRigidPatch"),
    ("SoftSelfContacts", "method", "tangles"), ("SoftVertexCandidate", "type", "SoftVertexCandidate"),
    ("SoftVertexHits", "type", "SoftVertexHits"), ("SoftVertexPass", "method", "candidates_of"),
    ("SoftVertexPass", "type", "SoftVertexPass"), ("SoftVolumePatch", "type", "SoftVolumePatch"),
    ("VolumeBin", "type", "VolumeBin"),
})
# Upstream's containment of non-finite state (`pipeline/physics_pipeline/quarantine.rs`). Q32.32 arithmetic panics
# (`'Fixed: overflow'`, `'Fixed: division by zero'`) instead of producing an invalid value:
# `crates/rapier2d/tests/finite_state.cairo` (a tiny mass and a huge impulse panic, a small mass stays finite).
QUARANTINE: frozenset[tuple[str, str, str]] = frozenset({
    ("PhysicsPipeline", "method", "quarantine"), ("PhysicsWorld", "method", "quarantine"),
    ("Quarantine", "method", "bodies"), ("Quarantine", "method", "colliders"),
    ("Quarantine", "method", "is_empty"), ("Quarantine", "type", "Quarantine"),
})
# Runtime-pluggable query dispatchers: the port chooses its dispatcher and stages at compile time.
DISPATCHERS: frozenset[tuple[str, str, str]] = frozenset({
    ("NarrowPhase", "method", "query_dispatcher"), ("NarrowPhase", "method", "with_query_dispatcher"),
    ("PersistentQueryDispatcher", "method", "contact_manifold_convex_convex"),
    ("PersistentQueryDispatcher", "method", "contact_manifolds"),
    ("PersistentQueryDispatcher", "trait", "PersistentQueryDispatcher"),
    ("QueryDispatcher", "method", "chain"), ("QueryDispatcherChain", "type", "QueryDispatcherChain"),
})
# Programme decision (2026-09-29, after CP3): the interaction graph's read-only surface is a view over the pair list
# (IG1); only the pieces that hand out the persistent graph or mutable references into it are closed.
MUTABLE_GRAPH_REASON = ("persistent mutable graph not ported (D7): values cannot hand out mutable references into the "
                        "step's storage")
MUTABLE_GRAPH: frozenset[tuple[str, str, str]] = frozenset({
    ("InteractionGraph", "method", "raw_graph"), ("InteractionGraph", "method", "interaction_pair_mut"),
    ("InteractionGraph", "method", "interactions_with_mut"), ("InteractionsWithMut", "type", "InteractionsWithMut"),
})
# Programme decision (2026-09-29, PX4): the `Arc<dyn Shape>` surface of upstream has no counterpart in a closed
# `Shape` value enum (ADR 0001 entries 35 and 37): the copy-on-write accessor, the downcast to `&mut dyn`, and the
# trait-object item itself. Its value meanings (`as_shape`, `clone_box`, `clone_dyn`, `scale_dyn`, `new`, ...) are
# ported (`shape/dyn_api.cairo`).
CLOSED_ENUM_REASON = "closed value enum replaces Arc / dyn shapes (SH2a)"
CLOSED_ENUM: frozenset[tuple[str, str, str]] = frozenset({
    ("SharedShape", "method", "make_mut"), ("Shape", "method", "as_shape_mut"), ("Shape", "trait", "Shape"),
    # The project manager's decision, 2026-10-03 (PX7): the `CompositeShape` traits and their `CompositeShapeRef` view
    # are the trait-object surface of composite shapes; the closed `Shape` enum matches on `Polyline` / `HeightField`
    # (`dispatch/composite.cairo`).
    ("CompositeShape", "trait", "CompositeShape"), ("CompositeShape", "method", "bvh"),
    ("CompositeShape", "method", "is_deformable"), ("CompositeShape", "method", "map_part_at"),
    ("TypedCompositeShape", "trait", "TypedCompositeShape"), ("TypedCompositeShape", "method", "map_typed_part_at"),
    ("TypedCompositeShape", "method", "map_untyped_part_at"), ("CompositeShapeRef", "type", "CompositeShapeRef"),
    # The project manager's decision, 2026-10-04 (PX9): `Compound::bvh` closes like `CompositeShape::bvh`; the compound
    # scans `aabbs` in `parts_in_aabb`, cheaper than a tree up to about 6 parts (ADR 36, SH2b).
    ("Compound", "method", "bvh"),
})
# The project manager's decisions, 2026-10-03 (PX7), each item listed by exact `(owner, kind, name)`; like PX1's
# reasons they count in "in scope" only.
# Cairo's `core::ops` has `Index` and `IndexView` but no `IndexMut`, and the sets hold values (`set` writes back).
NO_INDEX_MUT_REASON = "Cairo has no IndexMut / &mut index"
NO_INDEX_MUT: frozenset[tuple[str, str, str]] = frozenset({
    ("ColliderSet", "impl", "IndexMut<ColliderHandle>"), ("RigidBodySet", "impl", "IndexMut<RigidBodyHandle>"),
})
# The contact-graph order is persisted state of the solver's contact graph: a default `ContactRef` / `GraphPos` would
# be an invalid position, not a faithful value.
NO_FAITHFUL_DEFAULT_REASON = "persisted contact-graph order, no faithful Default"
NO_FAITHFUL_DEFAULT: frozenset[tuple[str, str, str]] = frozenset({
    ("ContactRef", "impl", "Default"), ("GraphPos", "impl", "Default"),
})
# Upstream's `SolverContacts` is a growable list of solver contacts, and `solver_contact_world_points` reads their
# world points after the step; the port keeps a fixed solver-contact array with a count, and its anchors are
# world-frame offsets from the centre of mass at the step's start (the bodies have moved since).
UNFAITHFUL_REASON = ("no faithful form: fixed solver-contact array and count; anchors are offsets at the step's "
                     "start")
UNFAITHFUL: frozenset[tuple[str, str, str]] = frozenset({
    ("SolverContacts", "type", "SolverContacts"), ("ContactManifoldData", "method", "solver_contact_world_points"),
})
# Programme decision (2026-09-29): V-HACD and voxelisation were in scope as low priority with no lot; closed by the
# decision of 2026-10-03 until a consumer needs them. `ColliderBuilder::voxelized_mesh` is already closed by the
# voxel pattern of `exclusion_reason`, and the builder's voxelized decompositions are not in the inventory.
DECOMPOSITION_REASON = "construction-time decomposition, several thousand lines, no consumer; reopened on a consumer's need"
DECOMPOSITION: frozenset[tuple[str, str, str]] = frozenset({
    (owner, "method", name) for owner in ("SharedShape", "ColliderBuilder")
    for name in ("convex_decomposition", "convex_decomposition_with_params", "round_convex_decomposition",
                 "round_convex_decomposition_with_params", "voxelized_convex_decomposition",
                 "voxelized_convex_decomposition_with_params", "voxelized_mesh")
    if (owner, name) != ("ColliderBuilder", "voxelized_mesh")
})
# The project manager's decisions, 2026-10-03 (PX8, after the SC2 study of the parked API families), each item listed
# by exact `(owner, kind, name)`; like PX1's reasons they count in "in scope" only.
# Upstream's `dynamics::solver` is `pub(crate)`, so no rapier user can call the solver's scalar API. The reason is new
# because PX1's text is carried by rows of the table (a longer text would change them). The gathers and scatters of
# these owners stay `SIMD/parallel`, and the `SolverVel` type stays `ported`.
SOLVER_SCALAR_REASON = "solver / island internals not exposed (dynamics::solver is pub(crate) upstream)"
SOLVER_SCALAR: frozenset[tuple[str, str, str]] = frozenset({
    ("SolverBodies", "type", "SolverBodies"),
    *(("SolverBodies", "method", n) for n in ("clear", "copy_from", "get_pose", "get_vel", "len", "resize", "set_vel")),
    *(("SolverVel", "impl", n) for n in ("AddAssign", "Sub", "SubAssign")),
    *(("SolverVel", "method", n) for n in ("as_mut_slice", "as_slice", "as_vector_slice", "as_vector_slice_mut",
                                           "zero")),
    ("SolverPose", "type", "SolverPose"), ("SolverPose", "impl", "Default"),
    *(("SolverPose", "method", n) for n in ("inverse_transform_point", "pose", "transform_point")),
    ("SolverTransform", "type", "SolverTransform"), ("SolverTransform", "method", "transform_point"),
    ("SolverPoseRepr", "method", "identity"), ("SolverVelRepr", "method", "zero"), ("VelocitySolver", "method", "new"),
})
# `contact_skin` is a collider field read in the broad phase, the narrow phase and the solver contacts; a port changes
# the collider layout and the contact solver for every user (`docs/research/parked-families.md` §3).
CONTACT_SKIN_REASON = ("contact skin changes the collider layout and the contact solver for every user; no opt-in "
                       "form keeps existing steps")
CONTACT_SKIN: frozenset[tuple[str, str, str]] = frozenset({
    ("Collider", "method", "contact_skin"), ("Collider", "method", "set_contact_skin"),
    ("ColliderBuilder", "method", "contact_skin"),
})
# The project manager's decision, 2026-10-04 (PX9): `ColliderSet::take_removed` needs a removal log, a field of the
# set that every step copies. WS3 measured the one-cell boxed list (the smallest layout a field can have, #266, not
# merged): +0.1 to +0.2 % steps per tick on the default path, +27,580 on the pile10 shot. The figure is WS3's; the
# VPS cap of 8 GiB could not compile the step probes for a second measurement (ADR 53).
REMOVAL_LOG_REASON = ("a removal log is a field of the stepped collider set: +0.1 to +0.2 % Cairo steps per tick, "
                      "+27,580 on the owner's pile10 shot, in process (WS3, commit 16ac69c)")
REMOVAL_LOG: frozenset[tuple[str, str, str]] = frozenset({("ColliderSet", "method", "take_removed")})
# Every reason added since PX1: they do not count in the raw figure.
POST_PX1_REASONS = frozenset({SOLVER_ISLAND_REASON, SOFT_CONTACTS_REASON, QUARANTINE_REASON, DISPATCHER_REASON,
                              MUTABLE_GRAPH_REASON, CLOSED_ENUM_REASON, NO_INDEX_MUT_REASON,
                              NO_FAITHFUL_DEFAULT_REASON, UNFAITHFUL_REASON, DECOMPOSITION_REASON,
                              SOLVER_SCALAR_REASON, CONTACT_SKIN_REASON, REMOVAL_LOG_REASON})

# PX1: rapier's contact/joint constraint solver internals and the persistent-island / BVH
# broad-phase internals have no Cairo counterpart by design, mirroring "EPA/GJK internals not
# exposed": Cairo's contact and joint solvers (`rapier_dynamics2d::solver::{contact,joint}`) are
# unrolled per-body kernels with no persistent constraint objects, no generic (multibody-only)
# solve path and no SIMD lanes; the island graph and the BVH broad phase are not ported (ADR 6 /
# D7: no persistent islands, no BVH broad phase — Cairo's broad phase is the swept-AABB grid).
# Anything a user of rapier calls stays in scope: Cairo's own `JointConstraint` /
# `JointConstraintHelper` (`solver/joint.cairo`, `solver/joint/helper.cairo`) cover the
# non-generic solve/warmstart/writeback/lock/new surface under the same names (`ported`), and
# `IslandManager::{active_bodies, new, num_active_bodies, wake_up}` map onto `World` (`ported`);
# only the upstream-only pieces below are internal. Listed by exact `(owner, kind, name)`, never a
# loose pattern (a previous PO1 self-test decision keeps the scalar `SolverBodies` API — `len`,
# `get_pose`, `get_vel`, `set_vel`, `clear`, `copy_from`, `resize` — and `SolverPose` / `SolverVel`
# / `VelocitySolver` out of this reason; PX8 closes them under `SOLVER_SCALAR_REASON` since the project
# manager's decision of 2026-10-03; the persisted
# contact-graph order (`ContactRef`, `GraphPos`) stays out too, but its `Default` impls are closed by
# `NO_FAITHFUL_DEFAULT` since PX7, the rest of the SO/DO class of PLAN.md being open).
SOLVER_ISLAND_INTERNALS: frozenset[tuple[str, str, str]] = frozenset({
    # Sequential-impulse contact constraints (dynamics/solver/contact_constraint/): per-contact
    # Coulomb/twist-friction builders and the warmstart state they carry across steps.
    ("ContactWithCoulombFriction", "method", "solve"),
    ("ContactWithCoulombFriction", "method", "warmstart"),
    ("ContactWithCoulombFriction", "method", "writeback_impulses"),
    ("ContactWithCoulombFrictionBuilder", "method", "apply_restitution"),
    ("ContactWithCoulombFrictionBuilder", "method", "generate"),
    ("ContactWithCoulombFrictionBuilder", "method", "has_bouncy_seed"),
    ("ContactWithCoulombFrictionBuilder", "method", "update"),
    ("ContactWithCoulombFrictionBuilder", "method", "update_rhs_wo_bias"),
    ("ContactWithTwistFriction", "method", "solve"),
    ("ContactWithTwistFriction", "method", "warmstart"),
    ("ContactWithTwistFriction", "method", "writeback_impulses"),
    ("ContactWithTwistFrictionBuilder", "method", "apply_restitution"),
    ("ContactWithTwistFrictionBuilder", "method", "generate"),
    ("ContactWithTwistFrictionBuilder", "method", "has_bouncy_seed"),
    ("ContactWithTwistFrictionBuilder", "method", "update"),
    ("ContactWithTwistFrictionBuilder", "method", "update_rhs_wo_bias"),
    ("CoulombContactPointInfos", "impl", "Default"),
    ("CoulombContactPointInfos", "type", "CoulombContactPointInfos"),
    ("TwistContactPointInfos", "impl", "Default"),
    ("TwistContactPointInfos", "type", "TwistContactPointInfos"),
    # The normal/tangent constraint parts and the free solve/warmstart kernels they call
    # (`total_impulse` stays `ported`: Cairo's contact row exposes the same accessor).
    ("ContactConstraintNormalPart", "method", "generic_solve"),
    ("ContactConstraintNormalPart", "method", "generic_warmstart"),
    ("ContactConstraintNormalPart", "method", "solve"),
    ("ContactConstraintNormalPart", "method", "solve_pair"),
    ("ContactConstraintNormalPart", "method", "solve_restitution"),
    ("ContactConstraintNormalPart", "method", "warmstart"),
    ("ContactConstraintNormalPart", "method", "zero"),
    ("ContactConstraintTangentPart", "method", "generic_solve"),
    ("ContactConstraintTangentPart", "method", "generic_warmstart"),
    ("ContactConstraintTangentPart", "method", "solve"),
    ("ContactConstraintTangentPart", "method", "warmstart"),
    ("ContactConstraintTangentPart", "method", "zero"),
    ("ContactConstraintsSet", "method", "new"),
    ("dynamics", "function", "solve"),
    ("dynamics", "function", "solve_pair"),
    ("dynamics", "function", "solve_restitution"),
    ("dynamics", "function", "warmstart"),
    ("dynamics", "function", "joint_data_num_constraints"),
    ("dynamics", "function", "joint_num_constraints"),
    ("dynamics", "function", "reset_buffer"),
    ("dynamics", "function", "reset_buffer_reusing"),
    # The generic (multibody-attached) contact/joint constraint variants: Cairo has no multibody
    # solve path (nalgebra-cairo, out of scope), so these never gain a counterpart.
    ("GenericContactConstraint", "method", "generic_solve_group"),
    ("GenericContactConstraint", "method", "generic_warmstart_group"),
    ("GenericContactConstraint", "method", "invalid"),
    ("GenericContactConstraint", "method", "remove_cfm_and_bias_from_rhs"),
    ("GenericContactConstraint", "method", "solve"),
    ("GenericContactConstraint", "method", "warmstart"),
    ("GenericContactConstraint", "method", "writeback_impulses"),
    ("GenericContactConstraintBuilder", "method", "apply_restitution"),
    ("GenericContactConstraintBuilder", "method", "generate"),
    ("GenericContactConstraintBuilder", "method", "has_bouncy_seed"),
    ("GenericContactConstraintBuilder", "method", "invalid"),
    ("GenericContactConstraintBuilder", "method", "update"),
    ("GenericJointConstraint", "impl", "Default"),
    ("GenericJointConstraint", "method", "invalid"),
    ("GenericJointConstraint", "method", "lock_axes"),
    ("GenericJointConstraint", "method", "remove_bias_from_rhs"),
    ("GenericJointConstraint", "method", "solve"),
    ("GenericJointConstraint", "method", "writeback_impulses"),
    ("GenericJointConstraint", "type", "GenericJointConstraint"),
    ("GenericJointConstraintBuilder", "type", "GenericJointConstraintBuilder"),
    ("JointGenericExternalConstraintBuilder", "method", "generate"),
    ("JointGenericExternalConstraintBuilder", "method", "update"),
    ("JointGenericExternalConstraintBuilder", "type", "JointGenericExternalConstraintBuilder"),
    ("JointGenericInternalConstraintBuilder", "method", "generate"),
    ("JointGenericInternalConstraintBuilder", "method", "num_constraints"),
    ("JointGenericInternalConstraintBuilder", "method", "update"),
    ("JointGenericInternalConstraintBuilder", "type", "JointGenericInternalConstraintBuilder"),
    ("JointSolverBody", "method", "fill_jacobians"),
    ("JointSolverBody", "method", "invalid"),
    ("JointSolverBody", "type", "JointSolverBody"),
    ("LinkOrBodyRef", "type", "LinkOrBodyRef"),
    # The persistent joint-constraint builders and the dyn-dispatch mutable view; Cairo's own
    # `JointConstraint` / `JointConstraintHelper` cover the non-generic solve surface (`ported`).
    ("AnyJointConstraintMut", "method", "writeback_impulses"),
    ("AnyJointConstraintMut", "type", "AnyJointConstraintMut"),
    ("JointConstraint", "method", "remove_bias_from_rhs"),
    ("JointConstraint", "method", "solve_generic"),
    ("JointConstraint", "method", "update"),
    ("JointConstraint", "method", "warmstart_generic"),
    ("JointConstraintBuilder", "method", "generate"),
    ("JointConstraintBuilder", "method", "update"),
    ("JointConstraintBuilder", "method", "update_warmstart_seeds"),
    ("JointConstraintBuilder", "type", "JointConstraintBuilder"),
    ("JointConstraintHelper", "method", "finalize_constraints"),
    ("JointConstraintHelper", "method", "finalize_generic_constraints"),
    ("JointConstraintHelper", "method", "limit_angular"),
    ("JointConstraintHelper", "method", "limit_angular_generic"),
    ("JointConstraintHelper", "method", "limit_linear"),
    ("JointConstraintHelper", "method", "limit_linear_coupled"),
    ("JointConstraintHelper", "method", "limit_linear_generic"),
    ("JointConstraintHelper", "method", "lock_angular_generic"),
    ("JointConstraintHelper", "method", "lock_jacobians_generic"),
    ("JointConstraintHelper", "method", "lock_linear_generic"),
    ("JointConstraintHelper", "method", "motor_angular"),
    ("JointConstraintHelper", "method", "motor_angular_generic"),
    ("JointConstraintHelper", "method", "motor_linear"),
    ("JointConstraintHelper", "method", "motor_linear_coupled"),
    ("JointConstraintHelper", "method", "motor_linear_generic"),
    ("JointConstraintHelper", "method", "recentered_angle"),
    ("JointConstraintsSet", "method", "iter_constraints_mut"),
    ("JointConstraintsSet", "method", "new"),
    ("JointConstraintsSet", "method", "writeback_impulses"),
    ("JointConstraintsSet", "type", "JointConstraintsSet"),
    ("AngularLimitParams", "method", "new"),
    ("AngularLimitParams", "type", "AngularLimitParams"),
    ("MotorParameters", "impl", "Default"),
    ("MotorParameters", "type", "MotorParameters"),
    ("WritebackId", "type", "WritebackId"),
    # The staged island solver (the per-island sweep over the constraint graph).
    ("StagedIslandSolver", "method", "init_and_solve"),
    ("StagedIslandSolver", "method", "new"),
    # Persistent islands (ADR 6 / D7: no persistent island graph — `World::active_bodies` scans
    # awake bodies each step) and the BVH broad phase (ADR 6 / D7: swept-AABB grid instead).
    ("Island", "method", "bodies"),
    ("Island", "method", "len"),
    ("Island", "method", "singleton"),
    ("IslandManager", "method", "persistent_island_of"),
    ("IslandManager", "type", "IslandManager"),
    ("PersistentIslands", "method", "apply_impulse_joint_event"),
    ("PersistentIslands", "method", "assert_consistent"),
    ("PersistentIslands", "method", "begin_sleep_scan"),
    ("PersistentIslands", "method", "body_island"),
    ("PersistentIslands", "method", "bootstrap"),
    ("PersistentIslands", "method", "clear_pending_split_of"),
    ("PersistentIslands", "method", "contact_edge_removed"),
    ("PersistentIslands", "method", "contact_link_loc"),
    ("PersistentIslands", "method", "ensure_body"),
    ("PersistentIslands", "method", "finish_sleep_scan"),
    ("PersistentIslands", "method", "link_contact"),
    ("PersistentIslands", "method", "link_joint"),
    ("PersistentIslands", "method", "mark_island_sleeping"),
    ("PersistentIslands", "method", "observe_body_for_sleep"),
    ("PersistentIslands", "method", "remove_body"),
    ("PersistentIslands", "method", "remove_body_raw"),
    ("PersistentIslands", "method", "run_pending_split"),
    ("PersistentIslands", "method", "schedule_split"),
    ("PersistentIslands", "method", "split_allowed"),
    ("PersistentIslands", "method", "split_island_now"),
    ("PersistentIslands", "method", "unlink_contact"),
    ("PersistentIslands", "method", "unlink_joint"),
    ("BroadPhaseBvh", "method", "as_query_pipeline"),
    ("BroadPhaseBvh", "method", "as_query_pipeline_mut"),
    ("BroadPhaseBvh", "method", "new"),
    ("BroadPhaseBvh", "method", "set_aabb"),
    ("BroadPhaseBvh", "method", "update"),
    ("BroadPhaseBvh", "method", "with_optimization_strategy"),
    ("BroadPhaseBvh", "type", "BroadPhaseBvh"),
    ("BvhOptimizationStrategy", "type", "BvhOptimizationStrategy"),
    # Multi-manifold contact workspaces: persisted state matching sub-shape ids across steps
    # (SH2a's decision, `MISSING_REASONS` above, is the same "no workspace" shape for the
    # composite/heightfield pairs already ported; the workspace machinery itself is internal).
    ("CompositeShapeCompositeShapeContactManifoldsWorkspace", "method", "new"),
    ("CompositeShapeCompositeShapeContactManifoldsWorkspace", "type",
     "CompositeShapeCompositeShapeContactManifoldsWorkspace"),
    ("CompositeShapeShapeContactManifoldsWorkspace", "method", "new"),
    ("CompositeShapeShapeContactManifoldsWorkspace", "type", "CompositeShapeShapeContactManifoldsWorkspace"),
    ("ContactManifoldsWorkspace", "impl", "Clone"),
    ("ContactManifoldsWorkspace", "impl", "From<T>"),
    ("ContactManifoldsWorkspace", "type", "ContactManifoldsWorkspace"),
    ("HeightFieldCompositeShapeContactManifoldsWorkspace", "method", "new"),
    ("HeightFieldCompositeShapeContactManifoldsWorkspace", "type",
     "HeightFieldCompositeShapeContactManifoldsWorkspace"),
    ("HeightFieldShapeContactManifoldsWorkspace", "method", "new"),
    ("HeightFieldShapeContactManifoldsWorkspace", "type", "HeightFieldShapeContactManifoldsWorkspace"),
    ("TypedWorkspaceData", "type", "TypedWorkspaceData"),
    ("WorkspaceData", "method", "as_typed_workspace_data"),
    ("WorkspaceData", "method", "clone_dyn"),
    ("WorkspaceData", "trait", "WorkspaceData"),
})


@dataclass(frozen=True, order=True)
class Item:
    owner: str
    kind: str
    name: str
    module: str = ""
    source: str = ""
    flags: str = ""

    @property
    def key(self) -> tuple[str, str, str]:
        return (self.owner, self.kind, self.name)

    def as_json(self) -> dict[str, str]:
        data = {
            "owner": self.owner,
            "kind": self.kind,
            "name": self.name,
            "module": self.module,
            "source": self.source,
        }
        if self.flags:
            data["flags"] = self.flags
        return data


def mask_comments(text: str) -> str:
    """Blank comments and string/char literals while preserving offsets and newlines."""
    out = list(text)
    i, state, depth = 0, "code", 0
    while i < len(text):
        pair = text[i:i + 2]
        if state == "code":
            if pair == "//":
                state = "line"
                out[i] = out[i + 1] = " "
                i += 2
                continue
            if pair == "/*":
                state, depth = "block", 1
                out[i] = out[i + 1] = " "
                i += 2
                continue
            if text[i] == '"':
                state = "string"
                out[i] = " "
                i += 1
                continue
            m = re.match(r"'(?:\\.|[^\\'\n])'", text[i:i + 4]) if text[i] == "'" else None
            if m:
                for k in range(i, i + len(m.group(0))):
                    out[k] = " "
                i += len(m.group(0))
                continue
            i += 1
            continue
        if state == "line":
            if text[i] == "\n":
                state = "code"
            else:
                out[i] = " "
            i += 1
            continue
        if state == "block":
            if pair == "/*":
                depth += 1
                out[i] = out[i + 1] = " "
                i += 2
                continue
            if pair == "*/":
                depth -= 1
                out[i] = out[i + 1] = " "
                i += 2
                if depth == 0:
                    state = "code"
                continue
            if text[i] != "\n":
                out[i] = " "
            i += 1
            continue
        if text[i] == "\\" and i + 1 < len(text):
            out[i] = out[i + 1] = " "
            i += 2
            continue
        if text[i] == '"':
            state = "code"
        if text[i] != "\n":
            out[i] = " "
        i += 1
    return "".join(out)


def closing(text: str, opening: int) -> int:
    stack = []
    for i in range(opening, len(text)):
        c = text[i]
        if c in OPEN:
            stack.append(OPEN[c])
        elif c in ")}]":
            if not stack or stack[-1] != c:
                raise ValueError(f"unbalanced {c!r} at byte {i}")
            stack.pop()
            if not stack:
                return i
    raise ValueError(f"unclosed bracket at byte {opening}")


def blocks(text: str, pattern: re.Pattern[str]) -> list[tuple[re.Match[str], int, int]]:
    found = []
    for match in pattern.finditer(text):
        opening = text.find("{", match.start(), match.end() + 3)
        if opening >= 0:
            found.append((match, opening, closing(text, opening)))
    return found


def split_top(value: str, seps: str = ",") -> list[str]:
    parts, start, depth = [], 0, 0
    for i, c in enumerate(value):
        if c in "<([{":
            depth += 1
        elif c in ">)]}" and not (c == ">" and i > 0 and value[i - 1] in "-="):
            depth -= 1
        elif depth == 0 and c in seps:
            parts.append(value[start:i].strip())
            start = i + 1
    parts.append(value[start:].strip())
    return [p for p in parts if p]


def skip_generics(value: str, start: int) -> int:
    if start >= len(value) or value[start] != "<":
        return start
    depth = 0
    for i in range(start, len(value)):
        if value[i] == "<":
            depth += 1
        elif value[i] == ">" and value[i - 1] not in "-=":
            depth -= 1
            if depth == 0:
                return i + 1
    return len(value)


def squash(value: str) -> str:
    return re.sub(r"\s+", " ", value).strip()


def unique_items(items: set[Item] | list[Item]) -> list[Item]:
    result: dict[tuple[str, str, str], Item] = {}
    for item in sorted(items, key=lambda it: (it.key, it.module, it.source)):
        result.setdefault(item.key, item)
    return sorted(result.values())


def clean_type(value: str, self_type: str = "") -> str:
    value = re.sub(r"'[_A-Za-z0-9]+", "", value)
    value = re.sub(r"\b(?:crate|super|self|std|core|alloc|na|parry|rapier)::", "", value)
    value = re.sub(r"\bSelf\b", self_type, value)
    value = value.replace("&", " ").replace("mut ", " ").replace("dyn ", " ")
    value = re.sub(r"\b(?:Real|f32|f64)\b", "Real", value)
    value = re.sub(r"\s+", "", value).strip(",")
    return value


def type_head(value: str, fallback: str = "") -> tuple[str, list[str]]:
    value = clean_type(value, fallback)
    value = re.sub(r"^(?:Option|Box|Arc|Vec|Array|Cow)<(.+)>$", r"\1", value)
    m = re.match(r"([A-Za-z_][A-Za-z0-9_]*)(?:<(.*)>)?$", value, re.S)
    if not m:
        return value, []
    return m.group(1), split_top(m.group(2) or "")


OWNER_ALIASES = {name: (name,) for name in """
ShapeType HalfSpace Cuboid Ball Capsule Segment ConvexPolygon Aabb Ray RayIntersection
""".split()}
OWNER_ALIASES.update({
    "PhysicsWorld": ("World",), "PhysicsPipeline": ("World", "pipeline", "PhysicsPipeline"),
    # QY2: `QueryPipeline` is the filter-bundle view of `queries/pipeline.cairo` (the world is
    # passed with each call); its methods are also the `World` and `queries` free functions.
    "QueryPipeline": ("World", "queries", "QueryPipeline"),
    # QY1: the struct and `QueryDispatcher` trait live in `query/dispatcher.cairo`.
    "DefaultQueryDispatcher": ("dispatch", "DefaultQueryDispatcher"),
    "PersistentQueryDispatcher": ("dispatch",), "RigidBodyHandle": ("Handle",),
    "ColliderHandle": ("Handle",), "ImpulseJointHandle": ("Handle",),
    "MultibodyJointHandle": ("Handle",), "IslandManager": ("pipeline::islands", "World"),
    # SH1: Parry's generic `RoundShape<S>` is instantiated as three aliases; its `impl Shape` and
    # `RayCast` are the aliases' (`From<RoundCuboid> for Shape`, `RoundCuboidRayCast`, ...).
    "RoundShape": ("RoundShape", "RoundShapeTrait", "RoundCuboid", "RoundTriangle", "RoundConvexPolygon"),
    "BroadPhaseBvh": ("broad_phase",), "NarrowPhase": ("NarrowPhase", "NarrowPhaseContactPairs", "NarrowPhaseInteractionGraph",
                    # PX5: stage 1 of the step is the free function `pipeline/user_changes.cairo::handle_user_changes` (no facade method).
                    "UserChanges"),
    "Halfspace": ("HalfSpace",),
    # PX7: upstream's free `geometry::contact_pair` items (`NEW_CONTACT_BIT`, `is_bouncy`) are the free items of
    # `rapier_geometry2d/src/contact.cairo`.
    "geometry": ("geometry", "Contact"),
    "MassProperties": ("MassProperties", "RigidBodyMassProps", "ColliderMassProps"),
    "RigidBodyMassProps": ("RigidBodyMassProps", "RigidBody"),
    # CC2: the CCD members of `RigidBody` are `RigidBodyCcdApiTrait` (`rigid_body_set/ccd_api.cairo`:
    # `body_api.cairo` is at the file budget).
    "RigidBody": ("RigidBody", "RigidBodyCcdApi"),
    "RigidBodyColliders": ("RigidBodyColliders", "RigidBodySet"), "ColliderShape": ("Shape",),
    # QY1: Parry's free `query::` functions live in `rapier_geometry2d::query` (top level, and the
    # per-pair kernel files `query/{ball,cuboid,segment,halfspace,support_map}.cairo`), next to
    # the older `closest_points` / `dispatch` / `ray` kernels whose files carry those owners.
    # SH2b: the compound pairs are free functions of `{point,ray,query/composite,dispatch/composite}/
    # compound.cairo` (owner `Compound`).
    # After CN1: the contact-manifold generators (`contact_generators/*.cairo`, owner
    # `ContactGenerators`) and the clipping kernels (`clip.cairo`) are Parry's `query::` free functions too.
    "parry::query": ("Query", "Ball", "Cuboid", "Segment", "Halfspace", "SupportMap",
                     "ClosestPoints", "dispatch", "Intersection", "Composite", "Compound",
                     "ContactGenerators", "Clip", "LineLine"),
    # PX3: the geometry utilities off the step path. `Aabb` and `Segment` get their clipping,
    # utility and split methods from extension traits (`aabb/{clip,utils}.cairo`,
    # `query/split.cairo`); Parry's line-line closest points are `closest_points/line_line.cairo`;
    # `parry::mass_properties::convex_polygon_area_and_center_of_mass` is `mass/convex_polygon.cairo`.
    "Aabb": ("Aabb", "AabbClip", "AabbUtils"), "Segment": ("Segment", "SegmentSplit"),
    "parry::mass_properties": ("ConvexPolygon",),
    # Parry's `ContactManifold` persistence methods are the `ManifoldTrait` of `manifold.cairo`.
    "ContactManifold": ("ContactManifold", "Manifold"),
    # MH1: the frozen `FeatureId` struct is Parry's packed `PackedFeatureId` (one `u32`); Parry's
    # free `bounding_volume::` builders live in `aabb/bounding_volume.cairo`.
    "PackedFeatureId": ("FeatureId",),
    "parry::bounding_volume": ("BoundingVolume",),
    "TypedShape": ("Shape",),
    # CP3: upstream's `ContactPair` (one pair, all its manifolds) is `ContactPairView`
    # (`narrow_phase/contact_pairs.cairo`; the pair list keeps one `ContactPair` entry per manifold,
    # ADR 0001 entry 35); the `NarrowPhase` reads are `NarrowPhaseContactPairsTrait`. The graph reads are `NarrowPhaseInteractionGraphTrait` (IG1).
    "ContactPair": ("ContactPair", "ContactPairView"),
    # SH2a: the composite queries are free functions of `{point,ray,query,dispatch}/composite.cairo`
    # and `query/sweep/composite.cairo` (owner `Composite`), over `Shape::{Polyline, HeightField}`.
    "Heightfield": ("HeightField", "Heightfield", "Composite"),
    "Polyline": ("Polyline", "Composite"),
    "CompositeShapeRef": ("Composite",),
    # PX1: `SharedShape` (an `Arc<dyn Shape>` wrapper) is the closed `Shape` enum's role. PX2 gave
    # `Shape` the same-named constructors (`ShapeTrait::ball`, `::cuboid`, …, `shape.cairo`), so
    # those now match too; only `new`, `make_mut` and `convex_polyline_unmodified` stay `missing`
    # (`MISSING_REASONS`), the `Arc<dyn Shape>` / copy-on-write / unmodified-polyline surface a
    # closed value enum has no counterpart for.
    "SharedShape": ("Shape", "ShapeDyn"),
    # PX4: the value meanings of the `dyn Shape` API (`as_shape`, `clone_box`, `scale_dyn`, `ccd_thickness`, ...)
    # are `ShapeDynTrait` (`shape/dyn_api.cairo`); `Index` on the sets is `core::ops::Index` (`collider_set/access.cairo`,
    # `rigid_body_set/index.cairo`), and `take_modified` is `ColliderSetChangesTrait`. Upstream's `RigidPairContacts`
    # is CP3's `ContactPairView` (`narrow_phase/contact_pairs/types.cairo`).
    "Shape": ("Shape", "ShapeDyn"),
    "ColliderSet": ("ColliderSet", "ColliderSetIndex", "ColliderSetChanges"),
    "RigidBodySet": ("RigidBodySet", "RigidBodySetIndex"),
    "RigidPairContacts": ("RigidPairContacts", "ContactPairView"),
})

METHOD_RENAMES: dict[tuple[str, str], tuple[str, ...]] = {
    # PX4: `data::Index` and the typed handles are all `Handle`; the sets implement `core::ops::Index<Set, Handle>`.
    ("ColliderSet", "Index<ColliderHandle>"): ("Index<Handle>",), ("ColliderSet", "Index<data::Index>"): ("Index<Handle>",),
    ("RigidBodySet", "Index<RigidBodyHandle>"): ("Index<Handle>",), ("RigidBodySet", "Index<data::Index>"): ("Index<Handle>",),
    # SH2a: one free function per query serves both composite kinds and both argument orders
    # (`*_composite`, the composite shape first or second); `CompositeShapeRef`'s methods are
    # the same functions; `*_mut` accessors are the copy-out reads (values, not references).
    **{("parry::query", f"{q}_{pair}"): (f"{q}_composite",) for q in (
        "cast_shapes", "closest_points", "contact", "distance", "intersection_test")
        for pair in ("composite_shape_shape", "shape_composite_shape", "heightfield_shape",
                     "shape_heightfield")},
    **{("parry::query", f"cast_shapes_nonlinear_{pair}"): ("cast_shapes_nonlinear_composite",)
       for pair in ("composite_shape_shape", "shape_composite_shape")},
    **{("parry::query", n): ("contact_manifolds_composite",) for n in (
        "contact_manifolds_composite_shape_shape", "contact_manifolds_heightfield_shape",
        "contact_manifolds_heightfield_shape_shapes")},
    ("CompositeShapeRef", "cast_local_ray"): ("cast_local_ray_polyline", "cast_local_ray_heightfield"),
    ("CompositeShapeRef", "cast_local_ray_and_get_normal"): (
        "cast_local_ray_and_get_normal_polyline", "cast_local_ray_and_get_normal_heightfield"),
    ("CompositeShapeRef", "cast_shape"): ("cast_shapes_composite",),
    ("CompositeShapeRef", "cast_shape_nonlinear"): ("cast_shapes_nonlinear_composite",),
    ("CompositeShapeRef", "closest_points_to_shape"): ("closest_points_composite",),
    ("CompositeShapeRef", "contact_with_shape"): ("contact_composite",),
    ("CompositeShapeRef", "distance_to_shape"): ("distance_composite",),
    ("CompositeShapeRef", "intersects_shape"): ("intersection_test_composite",),
    ("CompositeShapeRef", "contains_local_point"): ("contains_local_point_composite",),
    ("CompositeShapeRef", "project_local_point"): ("project_local_point_composite",),
    ("CompositeShapeRef", "project_local_point_and_get_feature"): (
        "project_local_point_and_get_feature_composite",),
    ("CompositeShapeRef", "project_local_point_and_get_location"): (
        "project_local_point_and_get_location_polyline",),
    ("Heightfield", "map_elements_in_local_aabb"): ("elements_in_local_aabb",),
    ("Polyline", "update_vertices"): ("set_vertices",),
    ("Shape", "as_polyline_mut"): ("as_polyline",), ("Shape", "as_heightfield_mut"): ("as_heightfield",),
    ("Shape", "as_composite_shape"): ("is_composite",),
    # SH2b: the compound pairs of the manifold dispatcher (`dispatch/composite/compound.cairo`).
    ("parry::query", "contact_manifolds_composite_shape_composite_shape"): ("contact_manifolds_composite_pair",),
    ("parry::query", "contact_manifolds_heightfield_composite_shape"): ("contact_manifolds_composite_pair",),
    ("Shape", "as_compound_mut"): ("as_compound",),
    ("PhysicsWorld", "new"): ("new",), ("PhysicsWorld", "step"): ("step",),
    ("PhysicsWorld", "contact_pair"): ("contact_pair",), ("PhysicsPipeline", "step"): ("step",),
    ("QueryPipeline", "cast_ray"): ("cast_ray",),
    ("QueryPipeline", "cast_ray_and_get_normal"): ("cast_ray_and_get_normal",),
    ("QueryPipeline", "intersect_ray"): ("intersect_ray",),
    ("QueryPipeline", "project_point"): ("project_point",),
    ("QueryPipeline", "intersection_with_shape"): ("intersect_shape",),
    ("Collider", "parent"): ("parent", "parent_handle"), ("Shape", "as_typed_shape"): ("shape_type",),
    # Values, not references: `*_mut` accessors are the copy-out reads (write back with `set`).
    ("Collider", "shape_mut"): ("shape",), ("Collider", "shared_shape"): ("shape",),
    ("ColliderSet", "get_mut"): ("get",), ("ColliderSet", "iter_mut"): ("iter",),
    ("ColliderSet", "iter_enabled_mut"): ("iter_enabled",),
    ("ColliderSet", "get_unknown_gen_mut"): ("get_unknown_gen",),
    ("PhysicsWorld", "rigid_bodies_mut"): ("rigid_bodies",),
    ("PhysicsWorld", "all_colliders_mut"): ("all_colliders",),
    ("PhysicsWorld", "step_with_events"): ("step_with_force_events",),
    ("PhysicsWorld", "PhysicsWorld"): ("World",), ("ColliderShape", "ColliderShape"): ("Shape",),
    ("ColliderHandle", "ColliderHandle"): ("Handle",), ("ColliderHandle", "from_raw_parts"): ("new",),
    # LO1: `RigidBodyHandle` is the same generational `Handle` as `ColliderHandle`.
    ("RigidBodyHandle", "RigidBodyHandle"): ("Handle",), ("RigidBodyHandle", "from_raw_parts"): ("new",),
    # LO1: a pose is the only type that converts to a `RigidBodyPosition` (`Into<Pose2, _>`).
    ("RigidBodyPosition", "From<T>"): ("From<Pose2>",),
    ("ImpulseJointHandle", "ImpulseJointHandle"): ("Handle",),
    ("ImpulseJointHandle", "from_raw_parts"): ("new",),
    # JA1: joints are values, so upstream's `*_mut` accessors are the copy-out reads (write back
    # with `set`, or `World::set_impulse_joint` for the wake-up flag).
    ("ImpulseJointSet", "get_mut"): ("get",), ("ImpulseJointSet", "iter_mut"): ("iter",),
    ("ImpulseJointSet", "get_unknown_gen_mut"): ("get_unknown_gen",),
    ("ColliderPosition", "From<T>"): ("From<Pose2>",),
    # QY1: the exact intersection kernels of `dispatch/intersection.cairo` with upstream's
    # signatures (`(center12, b1, b2)`, `(pos12, c1, c2)`); the other `intersection_test_*` differ.
    ("parry::query", "intersection_test_ball_ball"): ("ball_ball",),
    ("parry::query", "intersection_test_cuboid_cuboid"): ("cuboid_cuboid",),
    # MH1: the packed id type carries the frozen name `FeatureId`; upstream's `FeatureId` enum is
    # `UnpackedFeatureId`. A `Span` is the by-reference form, so the `*_ref` point-cloud builders
    # are the `Span` ones.
    ("PackedFeatureId", "PackedFeatureId"): ("FeatureId",),
    ("PackedFeatureId", "From<FeatureId>"): ("From<UnpackedFeatureId>",),
    ("Aabb", "from_points_ref"): ("from_points",),
    ("parry::bounding_volume", "local_point_cloud_aabb_ref"): ("local_point_cloud_aabb",),
    ("parry::bounding_volume", "point_cloud_aabb_ref"): ("point_cloud_aabb",),
    # PX3: the support-map AABB is generic over any `SupportMap`.
    ("parry::bounding_volume", "local_aabb"): ("local_support_map_aabb",),
    # PO1: `dyn Shape` downcasts to a mutable reference; a `Shape` is a value, so the `*_mut`
    # accessors are the copy-out `as_*` reads (build a new `Shape` to change it), as `Collider::shape_mut`.
    ("Shape", "as_ball_mut"): ("as_ball",), ("Shape", "as_capsule_mut"): ("as_capsule",),
    ("Shape", "as_convex_polygon_mut"): ("as_convex_polygon",), ("Shape", "as_cuboid_mut"): ("as_cuboid",),
    ("Shape", "as_halfspace_mut"): ("as_halfspace",), ("Shape", "as_segment_mut"): ("as_segment",),
    # SH1: the same copy-out reads for the triangle and the round shapes.
    ("Shape", "as_triangle_mut"): ("as_triangle",), ("Shape", "as_round_cuboid_mut"): ("as_round_cuboid",),
    ("Shape", "as_round_triangle_mut"): ("as_round_triangle",),
    ("Shape", "as_round_convex_polygon_mut"): ("as_round_convex_polygon",),
    # PO1: Parry's `TypedShape` (the tagged enum over the concrete shapes) is the closed `Shape` enum.
    ("TypedShape", "TypedShape"): ("Shape",),
    # PX1: the `SharedShape` type itself is the `Shape` enum (see `OWNER_ALIASES`).
    ("SharedShape", "SharedShape"): ("Shape",),
}


def owner_candidates(owner: str) -> tuple[str, ...]:
    base = OWNER_ALIASES.get(owner, (owner,))
    extra = []
    for candidate in base:
        if candidate and candidate[0].isupper():
            extra.append(candidate + "Trait")
    return tuple(dict.fromkeys((*base, *extra)))


def normalize_owner(self_type: str, fallback: str = "") -> str:
    head, _ = type_head(self_type, fallback)
    if head in ("Self", "") and fallback:
        return fallback
    return {
        "Halfspace": "HalfSpace",
        "HeightField": "Heightfield",
        "HeightField2": "Heightfield",
        "TOI": "ShapeCastHit",
    }.get(head, head)


def impl_name(trait_expr: str, target: str, fallback: str = "") -> tuple[str, str] | None:
    trait_expr = clean_type(trait_expr, target)
    target = clean_type(target, fallback)
    head, args = type_head(trait_expr, target)
    head = head.split("::")[-1]
    if head not in RUST_IMPL_TRAITS:
        return None
    owner = normalize_owner(target, fallback)
    arg = args[0] if args else ""
    if head in ("Add", "AddAssign", "Sub", "SubAssign", "Mul", "MulAssign", "Div", "DivAssign",
                "Index", "IndexMut", "From", "TryFrom", "Into") and arg:
        return owner, f"{head}<{normalize_rhs(arg)}>"
    return owner, head


def normalize_rhs(value: str) -> str:
    value = clean_type(value)
    if value.startswith("["):
        return "[T; N]"
    if value.startswith("("):
        return "(" + ", ".join(normalize_rhs(p) for p in split_top(value[1:-1])) + ")"
    head, _ = type_head(value)
    # PX7: the port's scalar is `Fixed`, upstream's `Real`.
    return {"Real": "Real", "Fixed": "Real", "Self": "Self", "Halfspace": "HalfSpace"}.get(head, head)


def cfg_attrs_before(text: str, pos: int) -> str:
    start = max(text.rfind("}", 0, pos), text.rfind(";", 0, pos), text.rfind("\n\n", 0, pos))
    return text[start + 1:pos]


def dim3_only(text: str, pos: int, rel: str) -> bool:
    attrs = cfg_attrs_before(text, pos)
    if "dim3" in attrs and "dim2" not in attrs:
        return True
    return any(part in rel for part in (
        "spherical_joint", "convex_polyhedron", "polygonal_feature3d", "heightfield3",
        "epa3", "voronoi_simplex3", "tetrahedron", "cone.rs", "cylinder.rs",
    ))


def cfg_test_spans(text: str) -> list[tuple[int, int]]:
    spans = []
    for m in re.finditer(r"#\s*\[\s*cfg\s*\(\s*test\s*\)\s*\]\s*(?:pub\s+)?mod\s+\w+\s*\{", text):
        spans.append((m.start(), closing(text, text.find("{", m.start()))))
    return spans


def inside(pos: int, spans: list[tuple[int, int]]) -> bool:
    return any(s < pos < e for s, e in spans)


def parse_impl_header(text: str, at: int) -> tuple[str | None, str, int] | None:
    i = at + 4
    while i < len(text) and text[i].isspace():
        i += 1
    i = skip_generics(text, i)
    depth, j = 0, i
    while j < len(text):
        c = text[j]
        if c in "<([":
            depth += 1
        elif c in ">)]" and not (c == ">" and text[j - 1] in "-="):
            depth -= 1
        elif c == "{" and depth <= 0:
            break
        elif c == ";" and depth <= 0:
            return None
        j += 1
    if j >= len(text):
        return None
    header = re.split(r"(?<![\w$])where(?![\w$])", text[i:j])[0]
    parts = [squash(p) for p in re.split(r"(?<![\w$])for(?![\w$])", header)]
    if len(parts) >= 2:
        return squash(" for ".join(parts[:-1])), parts[-1], j
    return None, squash(header), j


def module_for(crate: str, rel: str) -> str:
    parts = rel.split("/")
    if crate == "rapier":
        return parts[0]
    return "parry::" + parts[0]


def module_tree(src: Path) -> set[Path]:
    """The source files reachable from `src/lib.rs` through `mod name;` declarations: a file nobody declares (Parry's
    `shape/polygon.rs`) is dead code and not part of the API."""
    decl = re.compile(r"(?m)^\s*((?:#\[[^\]]*\]\s*)*)(?:pub(?:\([^)]*\))?\s+)?mod\s+([A-Za-z_][A-Za-z0-9_]*)\s*;")
    seen: set[Path] = set()
    todo = [src / "lib.rs"]
    while todo:
        path = todo.pop()
        if path in seen or not path.is_file():
            continue
        seen.add(path)
        here = path.parent if path.name in ("lib.rs", "mod.rs") else path.parent / path.stem
        raw = path.read_text()
        for m in decl.finditer(mask_comments(raw)):
            # A module declared behind a dim3-only cfg (rapier's `ray_cast_vehicle_controller`) is not 2D API; the
            # attribute is read from the raw text (masking blanks its string literals).
            attrs = raw[m.start(1):m.end(1)]
            if "dim3" in attrs and "dim2" not in attrs:
                continue
            for child in (here / f"{m.group(2)}.rs", here / m.group(2) / "mod.rs"):
                if child.is_file():
                    todo.append(child)
                    break
    return seen


def rust_files(root: Path, dirs: tuple[str, ...]) -> list[tuple[str, Path]]:
    src = root / "src"
    if not src.is_dir():
        raise SystemExit(f"missing source directory: {src}")
    reachable = module_tree(src)
    paths = []
    for d in dirs:
        base = src / d
        if base.is_file():
            paths.append(base)
        elif base.is_dir():
            paths.extend(base.rglob("*.rs"))
    return [(str(p.relative_to(src)), p) for p in sorted(paths) if p in reachable]


def add_rust_decl_items(items: set[Item], text: str, raw: str, rel: str, module: str, crate: str,
                        skip_spans: list[tuple[int, int]], impl_spans: list[tuple[int, int]]) -> None:
    type_re = re.compile(r"\bpub\s+(struct|enum|trait|type)\s+([A-Za-z_][A-Za-z0-9_]*)")
    for m in type_re.finditer(text):
        if inside(m.start(), skip_spans + impl_spans) or dim3_only(raw, m.start(), rel):
            continue
        kind = "trait" if m.group(1) == "trait" else "type"
        items.add(Item(m.group(2), kind, m.group(2), module, f"{crate}/src/{rel}"))
    fn_re = re.compile(r"\bpub\s+(?:const\s+)?(?:unsafe\s+)?fn\s+([A-Za-z_][A-Za-z0-9_]*)")
    for m in fn_re.finditer(text):
        if inside(m.start(), skip_spans + impl_spans) or dim3_only(raw, m.start(), rel):
            continue
        items.add(Item(module, "function", m.group(1), module, f"{crate}/src/{rel}"))
    const_re = re.compile(r"\bpub\s+const\s+([A-Z][A-Z0-9_]*)\s*:")
    for m in const_re.finditer(text):
        if inside(m.start(), skip_spans + impl_spans) or dim3_only(raw, m.start(), rel):
            continue
        items.add(Item(module, "const", m.group(1), module, f"{crate}/src/{rel}"))


def parse_rust_file(raw: str, rel: str, crate: str, items: set[Item]) -> None:
    text = mask_comments(raw)
    module = module_for(crate, rel)
    skip_spans = cfg_test_spans(text)
    impl_blocks = []
    for m in re.finditer(r"(?<![\w$])impl\b", text):
        if inside(m.start(), skip_spans) or dim3_only(raw, m.start(), rel):
            continue
        header = parse_impl_header(text, m.start())
        if not header:
            continue
        trait, self_type, opening = header
        try:
            end = closing(text, opening)
        except ValueError:
            continue
        impl_blocks.append((trait, self_type, m.start(), opening, end))
    impl_spans = [(b, e) for _, _, _, b, e in impl_blocks]
    trait_blocks = blocks(text, re.compile(r"\bpub\s+trait\s+([A-Za-z_][A-Za-z0-9_]*)[^{]*\{"))
    trait_spans = [(b, e) for _, b, e in trait_blocks]

    add_rust_decl_items(items, text, raw, rel, module, crate, skip_spans, impl_spans + trait_spans)

    for trait, self_type, _start, opening, end in impl_blocks:
        body = text[opening + 1:end]
        if trait is None:
            owner = normalize_owner(self_type)
            # A method or const behind `#[cfg(feature = "dim3")]` inside a shared impl is dim3-only too.
            for m in re.finditer(r"\bpub\s+(?:const\s+)?(?:unsafe\s+)?fn\s+([A-Za-z_][A-Za-z0-9_]*)", body):
                if not dim3_only(raw, opening + 1 + m.start(), rel):
                    items.add(Item(owner, "method", m.group(1), module, f"{crate}/src/{rel}"))
            for m in re.finditer(r"\bpub\s+const\s+([A-Z][A-Z0-9_]*)\s*:", body):
                if not dim3_only(raw, opening + 1 + m.start(), rel):
                    items.add(Item(owner, "const", m.group(1), module, f"{crate}/src/{rel}"))
        else:
            tracked = impl_name(trait, self_type)
            if tracked:
                items.add(Item(tracked[0], "impl", tracked[1], module, f"{crate}/src/{rel}"))

    for m, opening, end in trait_blocks:
        if inside(m.start(), skip_spans) or dim3_only(raw, m.start(), rel):
            continue
        owner = m.group(1)
        body = text[opening + 1:end]
        for fn in re.finditer(r"(?m)^\s*fn\s+([A-Za-z_][A-Za-z0-9_]*)", body):
            if not dim3_only(raw, opening + 1 + fn.start(1), rel):
                items.add(Item(owner, "method", fn.group(1), module, f"{crate}/src/{rel}"))


def parse_rust(rapier_root: Path, parry_root: Path) -> list[Item]:
    items: set[Item] = set()
    for rel, path in rust_files(rapier_root, RAPIER_DIRS):
        parse_rust_file(path.read_text(), rel, "rapier", items)
    for rel, path in rust_files(parry_root, PARRY_DIRS):
        parse_rust_file(path.read_text(), rel, "parry", items)
    return unique_items(items)


def cairo_owner_from_path(path: Path) -> str:
    rel = path.relative_to(ROOT / "crates")
    crate, _, *parts = rel.parts
    stem = path.stem
    if stem == "lib":
        return crate
    # CC1: the free functions of the shape casts and sweeps (`query/{shape_cast,
    # nonlinear_shape_cast,sweep}.cairo` and their kernel files) are Parry's `query::` ones.
    if "query" in parts[:-1] and stem in (
            "shape_cast", "nonlinear_shape_cast", "sweep", "proxy", "ball_ball"):
        return "Query"
    if "contact_generators" in parts[:-1]:
        return "ContactGenerators"
    return {
        "aabb": "Aabb",
        "world": "World",
        "queries": "queries",
        "pipeline": "pipeline",
        "dispatch": "dispatch",
        "shape": "Shape",
        "mass": "MassProperties",
        "ray": "Ray",
        "rigid_body": "RigidBody",
        "rigid_body_set": "RigidBodySet",
        "collider": "Collider",
        "collider_set": "ColliderSet",
        "narrow_phase": "NarrowPhase",
        "joint": "GenericJoint",
        "events": "Events",
    }.get(stem, "".join(p.capitalize() for p in stem.split("_")))


def cairo_impl_owner(impl_name: str, trait_expr: str, fallback: str) -> str:
    trait_head = clean_type(trait_expr).split("<", 1)[0]
    if trait_head.endswith("Trait"):
        return trait_head[:-5]
    for suffix in ("Impl", "Default"):
        if impl_name.endswith(suffix):
            return impl_name[:-len(suffix)]
    return fallback


def cairo_impl_item(impl_name: str, trait_expr: str, body: str, fallback: str) -> Item | None:
    trait_expr = clean_type(trait_expr)
    head, args = type_head(trait_expr)
    if head not in CAIRO_IMPL_TRAITS:
        return None
    owner = cairo_impl_owner(impl_name, trait_expr, fallback)
    if head in ("Into", "TryInto") and len(args) >= 2:
        return Item(normalize_owner(args[1]), "impl", f"{'From' if head == 'Into' else 'TryFrom'}<{normalize_rhs(args[0])}>")
    if head in ("PointQuery", "RayCast", "PointQueryWithLocation") and args:
        # Parry's shape query traits are generic over the shape in Cairo (`impl BallPointQuery of PointQuery<Ball>`).
        return Item(normalize_owner(args[0]), "impl", head)
    if head in ("IndexView", "Index") and len(args) >= 2:
        return Item(owner, "impl", f"Index<{normalize_rhs(args[1])}>")
    if head in ("Add", "AddAssign", "Sub", "SubAssign", "Mul", "MulAssign", "Div", "DivAssign") and args:
        return Item(owner, "impl", f"{head}<{normalize_rhs(args[-1])}>")
    return Item(owner, "impl", head)


# Crates that are not part of the engine's API (CS1: `rapier_sink` holds contract fixtures).
SKIPPED_CRATES = {"rapier_sink"}


def parse_cairo() -> list[Item]:
    items: set[Item] = set()
    # SH1: an impl may carry generic parameters (`impl RoundShapePointQuery<T, +Drop<T>, ...> of
    # PointQuery<RoundShape<T>>`, possibly wrapped over lines by `scarb fmt`).
    impl_re = re.compile(
        r"\bpub\s+impl\s+([A-Za-z_][A-Za-z0-9_]*)(?:\s*<[^{;]*?>)?\s+of\s+([^{]+?)\s*\{"
    )
    trait_re = re.compile(r"\bpub\s+trait\s+([A-Za-z_][A-Za-z0-9_]*)[^{]*\{")
    mod_re = re.compile(r"\bpub\s+mod\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{")
    for path in sorted((ROOT / "crates").glob("*/src/**/*.cairo")):
        if any(part in {"tests", "benches", "alternatives", "fixtures", "generated", "probes"} for part in path.parts):
            continue
        # Measurement fixtures (never published), not engine API.
        if path.relative_to(ROOT / "crates").parts[0] in SKIPPED_CRATES:
            continue
        source = str(path.relative_to(ROOT))
        fallback = cairo_owner_from_path(path)
        text = mask_comments(path.read_text())
        skip = cfg_test_spans(text)
        impls = [(m, b, e) for m, b, e in blocks(text, impl_re) if not inside(m.start(), skip)]
        traits = [(m, b, e) for m, b, e in blocks(text, trait_re) if not inside(m.start(), skip)]
        excluded = skip + [(b, e) for _, b, e in impls + traits]

        for m in re.finditer(r"\bpub\s+(struct|enum|trait|type)\s+([A-Za-z_][A-Za-z0-9_]*)", text):
            if not inside(m.start(), skip):
                items.add(Item(m.group(2), "trait" if m.group(1) == "trait" else "type", m.group(2), source, source))
        # After CP3: a derived `Default` / `Debug` is the same impl as upstream's written one.
        for m in re.finditer(r"#\[derive\(([^)]*)\)\]\s*(?:#\[[^\]]*\]\s*)*pub\s+(?:struct|enum)\s+"
                             r"([A-Za-z_][A-Za-z0-9_]*)", text):
            if inside(m.start(), skip):
                continue
            for derived in ("Default", "Debug"):
                if re.search(rf"\b{derived}\b", m.group(1)):
                    items.add(Item(m.group(2), "impl", derived, source, source))
        for m, opening, end in traits:
            owner = m.group(1)[:-5] if m.group(1).endswith("Trait") else m.group(1)
            body = text[opening + 1:end]
            for fn in re.finditer(r"\bfn\s+([A-Za-z_][A-Za-z0-9_]*)", body):
                items.add(Item(owner, "method", fn.group(1), source, source))
        for m, opening, end in impls:
            body = text[opening + 1:end]
            owner = cairo_impl_owner(m.group(1), m.group(2), fallback)
            for fn in re.finditer(r"\bfn\s+([A-Za-z_][A-Za-z0-9_]*)", body):
                items.add(Item(owner, "method", fn.group(1), source, source))
            for const in re.finditer(r"\bconst\s+([A-Z][A-Z0-9_]*)\s*:", body):
                items.add(Item(owner, "const", const.group(1), source, source))
            impl = cairo_impl_item(m.group(1), m.group(2), body, fallback)
            if impl:
                items.add(Item(impl.owner, impl.kind, impl.name, source, source))
                # PO1: Parry's `impl Shape for X` (its trait is the closed `Shape` enum here) is
                # `Into<X, Shape>`: it is what makes `X` usable as a `Shape`.
                if impl.owner == "Shape" and impl.name.startswith("From<"):
                    items.add(Item(impl.name[5:-1], "impl", "Shape", source, source))

        modules = [(m.group(1), b, e) for m, b, e in blocks(text, mod_re)]
        for m in re.finditer(r"\bpub\s+fn\s+([A-Za-z_][A-Za-z0-9_]*)", text):
            if inside(m.start(), excluded):
                continue
            nested = [name for name, b, e in modules if b < m.start() < e]
            owner = "::".join([fallback, *nested]) if nested else fallback
            items.add(Item(owner, "function" if owner in ("pipeline", "queries", "dispatch") else "method", m.group(1), source, source))
        for m in re.finditer(r"\bpub\s+const\s+([A-Z][A-Z0-9_]*)\s*:", text):
            if not inside(m.start(), excluded):
                items.add(Item(fallback, "const", m.group(1), source, source))
    return unique_items(items)


def load_inventory(path: Path) -> list[Item]:
    if not path.exists():
        raise SystemExit(f"{path} does not exist; run --refresh first")
    text = path.read_text()
    start = text.find(INVENTORY_START)
    end = text.find(INVENTORY_END, start + len(INVENTORY_START))
    if start < 0 or end < 0:
        raise SystemExit(f"{path} has no embedded inventory; run --refresh")
    return sorted(Item(**entry) for entry in json.loads(text[start + len(INVENTORY_START):end]))


def exclusion_reason(item: Item) -> str:
    # Programme decision (2026-09-29): exact lists first (they override the loose patterns below).
    if item.key in SOFT_CONTACTS:
        return SOFT_CONTACTS_REASON
    if item.key in QUARANTINE:
        return QUARANTINE_REASON
    if item.key in DISPATCHERS:
        return DISPATCHER_REASON
    if item.key in MUTABLE_GRAPH:
        return MUTABLE_GRAPH_REASON
    if item.key in CLOSED_ENUM:
        return CLOSED_ENUM_REASON
    if item.key in NO_INDEX_MUT:
        return NO_INDEX_MUT_REASON
    if item.key in NO_FAITHFUL_DEFAULT:
        return NO_FAITHFUL_DEFAULT_REASON
    if item.key in UNFAITHFUL:
        return UNFAITHFUL_REASON
    if item.key in DECOMPOSITION:
        return DECOMPOSITION_REASON
    if item.key in SOLVER_SCALAR:
        return SOLVER_SCALAR_REASON
    if item.key in CONTACT_SKIN:
        return CONTACT_SKIN_REASON
    if item.key in REMOVAL_LOG:
        return REMOVAL_LOG_REASON
    blob = " ".join((item.owner, item.kind, item.name, item.module, item.source)).lower()
    impl = item.kind == "impl"
    # PO1: the FEM / soft-constraint solver files hold the soft bodies' linear algebra (`BlockMatrix`,
    # `SkylineCholesky`, `ConjugateGradient`), constraints (`NeoHookeanConstraint`, `SoftAttachmentConstraint`,
    # ...) and sets; checked first so that `prepare` is not mistaken for `epa`.
    if any(x in blob for x in ("soft_body", "softbody", "softelastic", "deformable_mesh", "insert_deformable",
                               "soft_fem", "soft_constraint")) \
            or item.name == "soft_bodies" and item.owner in ("PhysicsWorld", "Quarantine") \
            or (item.owner, item.name) == ("RigidBodyType", "is_soft_frame"):
        return "soft bodies"
    if "multibody" in blob:
        return "multibody"
    if any(x in blob for x in ("simd", "parallel", "coloring", "graph_col", "thread_pool", "num_threads")):
        return "SIMD/parallel"
    # PO1: `solver/interaction_groups.rs` is the SIMD-lane grouping of the contact solver: its
    # `InteractionGroups` (not the collision-filter type of `geometry/interaction_groups.rs`) has
    # `clear_groups` / `group_manifold_refs`. Its `new` stays as is: it stands in for the filter
    # type's `pub const fn new`, which the inventory does not list. The SIMD gathers/scatters of
    # `solver/solver_body.rs` read `SIMD_WIDTH` bodies at once.
    if item.source.endswith("solver/interaction_groups.rs") \
            and item.name in ("clear_groups", "group_manifold_refs") \
            or item.source.endswith("solver/solver_body.rs") \
            and any(x in item.name for x in ("gather", "scatter", "assert_ids_in_range")):
        return "SIMD/parallel"
    if "debug_render" in blob:
        return "debug render"
    if any(x in blob for x in ("counter", "timer")) and "controller" not in blob:
        return "profiling counters"
    if item.owner in ("PhysicsHooks", "EventHandler", "ChannelEventCollector") or "physics_hooks" in blob:
        return "dyn hooks"
    # QY2: the predicate is a `&dyn Fn(ColliderHandle, &Collider) -> bool` closure.
    if (item.owner, item.name) == ("QueryFilter", "predicate"):
        return "dyn hooks"
    if any(x in blob for x in ("trimesh", "voxels", "heightfield3", "height_field3", "mesh_converter")) \
            or (item.owner, item.name) == ("ColliderBuilder", "voxelized_mesh"):
        return "trimesh/voxels/3D heightfield"
    # LO1: the `*_support_map_with_params` variants take the GJK `VoronoiSimplex` (and a warm-start
    # direction) of the algorithm Cairo replaces by analytic and SAT kernels.
    if any(x in blob for x in ("epa", "gjk", "simplex")) \
            or item.owner == "parry::query" and item.name.endswith("_support_map_with_params"):
        return "EPA/GJK internals not exposed"
    # PX1: exact `(owner, kind, name)` list only — see `SOLVER_ISLAND_INTERNALS` above.
    if item.key in SOLVER_ISLAND_INTERNALS:
        return SOLVER_ISLAND_REASON
    if impl and any(x in item.name for x in ("Serialize", "Deserialize", "Archive", "Pod", "Zeroable")):
        return "serde/rkyv/bytemuck"
    # PO1: `DeserializableTypedShape` exists only behind `serde-serialize` (its `into_shared_shape`).
    if item.owner.startswith("Deserializable"):
        return "serde/rkyv/bytemuck"
    if any(x in blob for x in ("serde", "rkyv", "bytemuck")):
        return "serde/rkyv/bytemuck"
    if impl and any(x in item.name for x in ("AbsDiffEq", "RelativeEq", "UlpsEq")):
        return "f32/f64 conversions and approx traits"
    if any(x in blob for x in ("f32", "f64", "approx")):
        return "f32/f64 conversions and approx traits"
    # PX8: `CompoundEdgeCone` is parry 0.31's 2D type (the element of `CompoundPseudoNormals::boundary_edges`), not a
    # 3D cone; it is ported by lot CE.
    if item.owner == "CompoundEdgeCone":
        return ""
    if "Spherical" in item.owner or any(x in blob for x in (
        "dim3", "polyhedron", "tetrahedron", "cone", "cylinder", "spherical_joint")) \
            or (item.owner, item.name) in (("ColliderBuilder", "capsule_z"), ("Cuboid", "vid")):
        # PO1: `Cuboid::vid` is a nested helper of the 3D-only `support_face`, not a method.
        return "dim3-only"
    return ""


def item_names(item: Item) -> tuple[str, ...]:
    renamed = METHOD_RENAMES.get((item.owner, item.name), ())
    return tuple(dict.fromkeys((item.name, *renamed)))


def find_matches(item: Item, cairo: set[tuple[str, str, str]]) -> list[tuple[str, str, str]]:
    matches = []
    for owner in owner_candidates(item.owner):
        for name in item_names(item):
            kinds = (item.kind,)
            if item.kind == "method":
                kinds = ("method", "function")
            if item.kind == "function":
                kinds = ("function", "method")
            for kind in kinds:
                key = (owner, kind, name)
                if key in cairo:
                    matches.append(key)
    return matches


# Items knowingly left missing, with the lot that owns them (instead of the generic "not found").
MISSING_REASONS: dict[tuple[str, str], str] = {
    # CP3 / programme (2026-09-29): the interaction graph stays in scope; a read-only view over the pair list can
    # answer most of it (derived `Default` / `Debug` are matched since the CP3 follow-up).
    # SH2a: composite–composite pairs are unsupported (`None`), see `dispatch/composite.cairo`.
    # SH2a: no persistent workspace: the previous manifolds are matched by sub-shape ids.
    **{(t, n): "SH2a: no workspace; previous manifolds are matched by sub-shape ids." for t in (
        "CompositeShapeShapeContactManifoldsWorkspace",
        "CompositeShapeCompositeShapeContactManifoldsWorkspace",
        "HeightFieldShapeContactManifoldsWorkspace",
        "HeightFieldCompositeShapeContactManifoldsWorkspace") for n in (t, "new")},
    # SH2b: parry 0.31's `CompoundFlags::FIX_INTERNAL_EDGES` (pseudo-normals of the parts' outlines)
    # postdates the golden pin (parry2d-f64 0.30.2): no reference to port it against.
    **{(o, n): "SH2b: parry 0.31 `FIX_INTERNAL_EDGES`, after the golden pin (parry2d-f64 0.30.2)." for o, n in (
        ("Compound", "DEFAULT_WELD_TOLERANCE"), ("Compound", "flags"),
        ("Compound", "part_normal_constraints"), ("Compound", "set_flags"),
        ("Compound", "with_flags"), ("CompoundFlags", "CompoundFlags"),
        ("CompoundPseudoNormals", "CompoundPseudoNormals"))},
    # PX2 / PX4: `SharedShape` -> `Shape` (`OWNER_ALIASES`); every constructor upstream builds through a
    # concrete shape has a same-named `ShapeTrait` counterpart, and PX4's `ShapeDynTrait` holds the value meanings
    # of `new`, `convex_polyline_unmodified`, `as_shape`, `clone_box`, `clone_dyn`, `scale_dyn` and the CCD
    # thicknesses. `make_mut`, `as_shape_mut` and the trait object are closed by `CLOSED_ENUM`.
    # CC1: upstream compiles no nonlinear half-space kernel (commented out of its `mod.rs` and of
    # `DefaultQueryDispatcher::cast_shapes_nonlinear`, which answers `Unsupported`, as the port).
    **{("parry::query", n): "Not compiled upstream (commented out); the pair is unsupported, as upstream." for n in (
        "cast_shapes_nonlinear_halfspace_support_map", "cast_shapes_nonlinear_support_map_halfspace")},
}


def classify(rust: list[Item], cairo: list[Item]) -> tuple[dict[Item, tuple[str, str]], list[Item]]:
    cairo_keys = {item.key for item in cairo}
    consumed: set[tuple[str, str, str]] = set()
    statuses: dict[Item, tuple[str, str]] = {}
    for item in rust:
        reason = exclusion_reason(item)
        if reason:
            statuses[item] = ("excluded", reason)
            continue
        matches = find_matches(item, cairo_keys)
        if matches:
            consumed.update(matches)
            detail = "Same public name."
            if item.owner not in owner_candidates(item.owner) or item_names(item) != (item.name,):
                detail = "Mapped to " + ", ".join(f"{o}.{n}" for o, _, n in matches[:3])
            statuses[item] = ("ported", detail)
        elif (item.owner, item.name) in MISSING_REASONS:
            statuses[item] = ("missing", MISSING_REASONS[(item.owner, item.name)])
        else:
            candidates = ", ".join(owner_candidates(item.owner))
            statuses[item] = ("missing", f"Not found on Cairo candidate(s): {candidates}.")
    extras = [item for item in cairo if item.key not in consumed]
    return statuses, extras


def anchor(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")


def display(item: Item) -> str:
    return f"{item.kind} `{item.name}`"


WORK_PACKAGES = (
    ("Sensors and intersection events", re.compile(r"sensor|intersection|intersect|ActiveEvents|ContactEvent", re.I), "standard", "SE sensors"),
    ("CCD and shape casts", re.compile(r"ccd|toi|shape_cast|sweep|cast_shape|nonlinear", re.I), "hard", "QP queries"),
    ("Character controller", re.compile(r"character_controller|KinematicCharacter|Character", re.I), "standard", "phase 3"), ("Vehicle and PID controllers", re.compile(r"vehicle|pid|Controller", re.I), "standard", "control crate policy"),
    ("Additional 2D shapes", re.compile(r"Triangle|Round|Polyline|Compound|Heightfield|heightfield2|SharedShape|Scaled", re.I), "standard", "shape interface"), ("Query completion", re.compile(r"query|distance|closest|contact|project|ray|cast|dispatcher", re.I), "standard", "QP queries"),
    ("Collider API completion", re.compile(r"Collider|ActiveCollision|Coefficient|CollisionGroups", re.I), "mechanical", "DB/EV"), ("Rigid-body API completion", re.compile(r"RigidBody|Damping|Dominance|LockedAxes|MassProps", re.I), "mechanical", "KD/SL"),
    ("Joint API completion", re.compile(r"Joint|Motor|Limit|Rope|Spring|Prismatic|Revolute|Fixed", re.I), "standard", "JL/RJ"), ("Pipeline and world facade", re.compile(r"Pipeline|World|Island|NarrowPhase|BroadPhase|Event", re.I), "standard", "P1/SL/EV"),
    ("Mass, AABB, and shape helpers", re.compile(r"Mass|Aabb|Bounding|support|feature|clip|sat", re.I), "standard", "geometry"),
)


def package_for(item: Item) -> tuple[str, str, str]:
    blob = " ".join((item.owner, item.name, item.source))
    for title, pat, tier, depends in WORK_PACKAGES:
        if pat.search(blob):
            return title, tier, depends
    return "API polish and miscellaneous parity", "mechanical", "AP triage"


def render(rust: list[Item], cairo: list[Item]) -> str:
    statuses, extras = classify(rust, cairo)
    modules = sorted(set(item.module for item in rust))
    lines = [
        f"# API parity with rapier-rs {RAPIER_VERSION} and parry2d subset",
        "",
        "<!-- Generated by scripts/api_parity.py: do not edit by hand. -->",
        "",
        "Generated by `python3 scripts/api_parity.py` (`--check` fails when stale; "
        "`--refresh --rapier <checkout> --parry <checkout>` re-reads upstream). The Rust "
        "inventory is embedded at the end of this file, so normal regeneration does not need a "
        "Rust checkout.",
        "",
        f"Upstream target: rapier-rs `{RAPIER_VERSION}` 2D plus parry `{PARRY_VERSION}` public "
        f"items Rapier exposes or users need. Golden vectors still pin `parry2d-f64 {GOLDEN_PARRY}`.",
        "",
        "Statuses: `ported` means the same public name or a documented owner/name mapping exists "
        "in Cairo; `partial` is reserved for split owners; `missing` is the default; `excluded` "
        "uses only the closed reasons below.",
        "",
        "Closed exclusion reasons: " + ", ".join(f"`{r}`" for r in EXCLUSIONS) + ".",
        "",
        "Of these, the project manager's decisions of 2026-10-03 are: " + ", ".join(f"`{r}`" for r in (
            NO_INDEX_MUT_REASON, NO_FAITHFUL_DEFAULT_REASON, UNFAITHFUL_REASON, DECOMPOSITION_REASON,
            SOLVER_SCALAR_REASON, CONTACT_SKIN_REASON)) + " (the first four from PX7, the last two from PX8);"
        " the closed-enum reason also covers the composite traits of PX7 and `Compound::bvh` (PX9, 2026-10-04), and "
        f"`{REMOVAL_LOG_REASON}` is `ColliderSet::take_removed` (PX9).",
        "",
        "## Coverage summary",
        "",
        "Two coverage figures (PX1, 2026-09-27), so closing an exclusion never quietly raises the "
        "headline number: **raw** = ported / (items − excluded by the reasons that predate PX1); "
        "**in scope** = ported / (items − every excluded item, including the reasons added since PX1: "
        + ", ".join(f"`{r}`" for r in sorted(POST_PX1_REASONS)) + ").",
        "",
        "| Module | Ported | Partial | Missing | Excluded | Items | Raw | In scope |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    total = {k: 0 for k in ("ported", "partial", "missing", "excluded", "excluded_new")}
    for module in modules:
        owned = [item for item in rust if item.module == module]
        counts = {k: sum(statuses[item][0] == k for item in owned) for k in ("ported", "partial", "missing", "excluded")}
        counts["excluded_new"] = sum(
            statuses[item][0] == "excluded" and statuses[item][1] in POST_PX1_REASONS for item in owned)
        for k, v in counts.items():
            total[k] += v
        raw_denom = len(owned) - (counts["excluded"] - counts["excluded_new"])
        scope_denom = len(owned) - counts["excluded"]
        raw_cov = "—" if raw_denom <= 0 else f"{100.0 * counts['ported'] / raw_denom:.1f}%"
        scope_cov = "—" if scope_denom <= 0 else f"{100.0 * counts['ported'] / scope_denom:.1f}%"
        lines.append(f"| {module} | {counts['ported']} | {counts['partial']} | {counts['missing']} | {counts['excluded']} | {len(owned)} | {raw_cov} | {scope_cov} |")
    raw_denom = len(rust) - (total["excluded"] - total["excluded_new"])
    scope_denom = len(rust) - total["excluded"]
    raw_cov = "—" if raw_denom <= 0 else f"{100.0 * total['ported'] / raw_denom:.1f}%"
    scope_cov = "—" if scope_denom <= 0 else f"{100.0 * total['ported'] / scope_denom:.1f}%"
    lines.append(f"| **total** | **{total['ported']}** | **{total['partial']}** | **{total['missing']}** | **{total['excluded']}** | **{len(rust)}** | **{raw_cov}** | **{scope_cov}** |")
    lines += ["", f"Cairo-only public items not matched to upstream: **{len(extras)}**.", ""]

    owners = sorted(set(item.owner for item in rust))
    for owner in owners:
        owned = [item for item in rust if item.owner == owner]
        lines += [f"## {owner}", "", "| Item | Module | Status | Detail | Source |", "|---|---|---|---|---|"]
        for item in owned:
            status, detail = statuses[item]
            lines.append(f"| {display(item)} | {item.module} | {status} | {detail} | `{item.source}` |")
        lines.append("")

    missing = [item for item in rust if statuses[item][0] in ("missing", "partial")]
    groups: dict[tuple[str, str, str], list[Item]] = {}
    for item in missing:
        groups.setdefault(package_for(item), []).append(item)
    lines += ["## Missing work packages", "", "| Package | Items | Tier | Depends on / context |", "|---|---:|---|---|"]
    for key, items in sorted(groups.items(), key=lambda kv: (-len(kv[1]), kv[0][0])):
        title, tier, depends = key
        lines.append(f"| [{title}](#{anchor('wp-' + title)}) | {len(items)} | {tier} | {depends} |")
    for key, items in sorted(groups.items(), key=lambda kv: (-len(kv[1]), kv[0][0])):
        title, tier, depends = key
        lines += ["", f"### WP: {title}", "", f"Tier: {tier}. Depends/context: {depends}. Estimate: {len(items)} public items.", ""]
        for item in sorted(items)[:80]:
            lines.append(f"- **{item.owner}** {display(item)} (`{item.source}`)")
        if len(items) > 80:
            lines.append(f"- ... {len(items) - 80} more")

    if extras:
        lines += ["", "## Cairo public items without upstream match", ""]
        for item in extras[:200]:
            lines.append(f"- **{item.owner}** {display(item)} (`{item.source}`)")
        if len(extras) > 200:
            lines.append(f"- ... {len(extras) - 200} more")

    inventory = json.dumps([item.as_json() for item in rust], indent=2, sort_keys=True)
    lines += [
        "",
        "## Embedded Rust inventory",
        "",
        "Updated only by `python3 scripts/api_parity.py --refresh --rapier <checkout> --parry <checkout>`.",
        "",
        INVENTORY_START + inventory + INVENTORY_END,
        "",
    ]
    return "\n".join(lines)


def write_or_check(generated: str, check: bool) -> int:
    current = OUTPUT.read_text() if OUTPUT.exists() else ""
    if check:
        if current == generated:
            print(f"{OUTPUT.relative_to(ROOT)} is up to date")
            return 0
        print(f"{OUTPUT.relative_to(ROOT)} is stale; run python3 scripts/api_parity.py", file=sys.stderr)
        diff = difflib.unified_diff(current.splitlines(), generated.splitlines(), fromfile=str(OUTPUT), tofile="generated", lineterm="")
        for line in list(diff)[:120]:
            print(line, file=sys.stderr)
        return 1
    OUTPUT.write_text(generated)
    print(f"wrote {OUTPUT.relative_to(ROOT)}")
    return 0


def self_test() -> int:
    sample = """
    pub struct Shared;
    #[cfg(feature = "dim3")]
    pub struct Only3;
    pub struct Builder;
    impl Builder {
        pub fn new() -> Self { Self }
        pub fn density(self, d: Real) -> Self { self }
    }
    impl Add<Real> for Builder { fn add(self, rhs: Real) -> Self { self } }
    pub trait Demo { fn hook(&self); }
    """
    items: set[Item] = set()
    parse_rust_file(sample, "dynamics/sample.rs", "rapier", items)
    keys = {i.key for i in items}
    assert ("Shared", "type", "Shared") in keys
    assert ("Only3", "type", "Only3") not in keys
    assert ("Builder", "method", "density") in keys
    assert ("Builder", "impl", "Add<Real>") in keys
    assert ("Demo", "method", "hook") in keys
    reasons = {
        ("SoftFemSet", "new", "rapier/src/dynamics/solver/soft_fem/soft_fem_set.rs"): "soft bodies",
        ("SoftFemSystem", "prepare", "rapier/src/dynamics/solver/soft_fem/system/soft_fem_system_prepare.rs"): "soft bodies",
        ("BlockMatrix", "mul", "rapier/src/dynamics/solver/soft_fem/soft_fem_sparse.rs"): "soft bodies",
        ("MeshConverter", "convert", "rapier/src/geometry/mesh_converter.rs"): "trimesh/voxels/3D heightfield",
        ("DeserializableTypedShape", "into_shared_shape", "parry/src/shape/shape.rs"): "serde/rkyv/bytemuck",
        ("InteractionGroups", "clear_groups", "rapier/src/dynamics/solver/interaction_groups.rs"): "SIMD/parallel",
        ("SolverBodies", "gather_vels", "rapier/src/dynamics/solver/solver_body.rs"): "SIMD/parallel",
        ("Cuboid", "vid", "parry/src/shape/cuboid.rs"): "dim3-only",
        # Not excluded: the collision-filter `InteractionGroups`. The scalar solver-body API is closed since PX8.
        ("InteractionGroups", "test", "rapier/src/geometry/interaction_groups.rs"): "",
        ("SolverBodies", "len", "rapier/src/dynamics/solver/solver_body.rs"): SOLVER_SCALAR_REASON,
    }
    for (owner, name, source), reason in reasons.items():
        assert exclusion_reason(Item(owner, "method", name, source=source)) == reason, (owner, name)
    # PX7: the closed lists answer by exact `(owner, kind, name)`; the matcher maps `Fixed` to `Real`.
    for key, reason in ((("CompositeShape", "method", "bvh"), CLOSED_ENUM_REASON),
                        (("ColliderSet", "impl", "IndexMut<ColliderHandle>"), NO_INDEX_MUT_REASON),
                        (("GraphPos", "impl", "Default"), NO_FAITHFUL_DEFAULT_REASON),
                        (("SolverContacts", "type", "SolverContacts"), UNFAITHFUL_REASON),
                        (("SharedShape", "method", "round_convex_decomposition"), DECOMPOSITION_REASON)):
        assert exclusion_reason(Item(*key)) == reason, key
    # PX8: the solver's scalar API and `contact_skin` are closed; `SolverVel` the type, the SIMD gathers and
    # `CompoundEdgeCone` (parry 0.31's 2D type) are not.
    for key, reason in ((("SolverBodies", "method", "len"), SOLVER_SCALAR_REASON),
                        (("SolverVel", "impl", "SubAssign"), SOLVER_SCALAR_REASON),
                        (("VelocitySolver", "method", "new"), SOLVER_SCALAR_REASON),
                        (("Collider", "method", "set_contact_skin"), CONTACT_SKIN_REASON),
                        (("Compound", "method", "bvh"), CLOSED_ENUM_REASON),
                        (("ColliderSet", "method", "take_removed"), REMOVAL_LOG_REASON),
                        (("SolverVel", "type", "SolverVel"), ""),
                        (("CompoundEdgeCone", "type", "CompoundEdgeCone"), "")):
        assert exclusion_reason(Item(*key)) == reason, key
    assert len(SOLVER_SCALAR) == 26 and len(CONTACT_SKIN) == 3
    assert exclusion_reason(Item("SolverBodies", "method", "gather_vels",
                                 source="rapier/src/dynamics/solver/solver_body.rs")) == "SIMD/parallel"
    assert exclusion_reason(Item("ContactRef", "method", "new")) == ""
    assert normalize_rhs("Fixed") == normalize_rhs("Real")
    print("self-test passed")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_true", help="fail if docs/API_PARITY.md is stale")
    modes.add_argument("--refresh", action="store_true", help="refresh embedded Rust inventory")
    modes.add_argument("--self-test", action="store_true", help="run parser self-tests")
    parser.add_argument("--rapier", type=Path, default=Path("/home/claude/git/refs/rapier"), help="rapier-rs checkout")
    parser.add_argument("--parry", type=Path, default=Path("/home/claude/git/refs/parry"), help="parry checkout")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        return self_test()
    rust = parse_rust(args.rapier, args.parry) if args.refresh else load_inventory(OUTPUT)
    cairo = parse_cairo()
    return write_or_check(render(rust, cairo), args.check)


if __name__ == "__main__":
    raise SystemExit(main())
