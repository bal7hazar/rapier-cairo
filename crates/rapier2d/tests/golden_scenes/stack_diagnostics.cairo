//! SO: why `box_stack3` diverges from upstream. The engine is untouched: one step is replayed
//! with the public stage functions (`pipeline::{handle_user_changes, detect_collisions,
//! touching_manifolds, scatter_touching, advance_to_final_positions}`, `solve_island` over a
//! `SolverBodyStore`), the only free parameter being the order in which the touching manifolds
//! are handed to the solver. `Order::Pair` is the port (D8, ascending pair order) and reproduces
//! `World::step` bit for bit; `Order::Colour` is upstream's order (Rapier 0.35.3 solves the
//! persistent colour buckets in ascending colour, dynamic–dynamic pairs from colour 0 up and
//! pairs with a fixed body from colour 127 down, so the ground contact comes last);
//! `Order::Reversed` is upstream's contact-graph edge order in this scene, a control showing that
//! the edge order is not what matters.
use core::dict::{Felt252Dict, Felt252DictTrait};
use rapier2d::pipeline;
use rapier2d::prelude::{Handle, RigidBody, RigidBodyTrait, World, WorldTrait};
use rapier_core::rigid_body::RigidBodyType;
use rapier_dynamics2d::joint::ImpulseJoint;
use rapier_dynamics2d::narrow_phase::ContactPair;
use rapier_dynamics2d::rigid_body_set::RigidBodySetTrait;
use rapier_dynamics2d::solver::body_store::SolverBodyStoreTrait;
use rapier_dynamics2d::solver::island::solve_island;
use rapier_geometry2d::contact::{ContactData, ContactManifold};
use rapier_golden::scenes;
use rapier_testing::opaque;
use super::builder::{body_handle, build_world, f, reseed};
use super::{Stats, compare};

/// Order in which the touching manifolds of a step are solved.
#[derive(Copy, Drop, PartialEq, Debug)]
enum Order {
    /// Ascending pair order (the port, D8).
    Pair,
    /// Descending pair order: upstream's contact-graph edge order in `box_stack3`.
    Reversed,
    /// Upstream's solver colours, ascending (see [`colour_order`]).
    Colour,
}

/// Upstream colour of each manifold (`narrow_phase/mod.rs`, `assign_pair_solver_color`), assigned
/// greedily in ascending `(min, max)` body index as `apply_deferred_solver_coloring` does:
/// pairs of two non-fixed bodies take the lowest colour below 120 free on both, pairs with one
/// fixed body the highest colour below 128 free on the other one. Every manifold must have two
/// parent bodies (the stack's do). Upstream colours a pair once,
/// when it starts touching, and keeps the colour while it touches; this recomputes the colours
/// every step, which is the same thing as long as the set of touching pairs does not change (all
/// three stack pairs touch from step 1 on, upstream and port). Returns the manifold indices in
/// ascending colour, ties in colouring order.
fn colour_order(
    manifolds: Span<ContactManifold>, entries: Span<(Handle, RigidBody)>,
) -> Array<u32> {
    // Colouring order: ascending (min, max) body index (insertion sort, n is tiny).
    let mut keyed: Array<(u32, u32)> = array![];
    let mut i = 0;
    while i != manifolds.len() {
        let (a, b) = body_indices(*manifolds.at(i));
        let key = if a < b {
            a * 0x10000 + b
        } else {
            b * 0x10000 + a
        };
        keyed = insert_sorted(keyed.span(), (key, i));
        i += 1;
    }
    let mut used: Felt252Dict<bool> = Default::default();
    let mut coloured: Array<(u32, u32)> = array![];
    for (_, m) in keyed.span() {
        let manifold = *manifolds.at(*m);
        let (a, b) = body_indices(manifold);
        let dyn_a = is_coloured(entries, a);
        let dyn_b = is_coloured(entries, b);
        let colour = if dyn_a && dyn_b {
            let mut c = 0_u32;
            while used.get(slot(a, c)) || used.get(slot(b, c)) {
                c += 1;
            }
            assert!(c < 120, "colour overflow");
            used.insert(slot(a, c), true);
            used.insert(slot(b, c), true);
            c
        } else {
            let d = if dyn_a {
                a
            } else {
                b
            };
            let mut c = 127_u32;
            while used.get(slot(d, c)) {
                c -= 1;
            }
            used.insert(slot(d, c), true);
            c
        };
        coloured = insert_sorted(coloured.span(), (colour, *m));
    }
    let mut out = array![];
    for (_, m) in coloured.span() {
        out.append(*m);
    }
    out
}

fn slot(body: u32, colour: u32) -> felt252 {
    (body * 256 + colour).into()
}

fn body_indices(m: ContactManifold) -> (u32, u32) {
    (m.data.rigid_body1.unwrap().index, m.data.rigid_body2.unwrap().index)
}

/// Whether body `index` takes part in upstream's colouring: every body but a fixed one
/// (`rb.is_fixed()` upstream, so kinematic bodies colour like dynamic ones); `entries` is the
/// dense `(handle, body)` walk of the set.
fn is_coloured(entries: Span<(Handle, RigidBody)>, index: u32) -> bool {
    let (_, body) = entries.at(index);
    *body.body_type != RigidBodyType::Fixed
}

/// Engine candidate for upstream's order without colouring: the touching manifolds in pair
/// order, those with a fixed side moved after the others (stable). Same permutation as
/// [`colour_order`] whenever the dynamic–dynamic colours ascend in pair order (the stack).
fn fixed_last(
    pairs: Span<ContactPair>, entries: Span<(Handle, RigidBody)>,
) -> Array<ContactManifold> {
    let mut first = array![];
    let mut last = array![];
    for pair in pairs {
        let m = *pair.manifold;
        if m.data.num_solver_contacts != 0 {
            let (a, b) = body_indices(m);
            if is_coloured(entries, a) && is_coloured(entries, b) {
                first.append(m);
            } else {
                last.append(m);
            }
        }
    }
    first.append_span(last.span());
    first
}

/// `sorted` with `item` inserted after every entry of key `<=` its key (stable).
fn insert_sorted(sorted: Span<(u32, u32)>, item: (u32, u32)) -> Array<(u32, u32)> {
    let (key, _) = item;
    let mut out = array![];
    let mut placed = false;
    for entry in sorted {
        let (k, _) = *entry;
        if !placed && k > key {
            out.append(item);
            placed = true;
        }
        out.append(*entry);
    }
    if !placed {
        out.append(item);
    }
    out
}

/// The permutation of `order` over `n` touching manifolds.
fn permutation(
    order: Order, manifolds: Span<ContactManifold>, entries: Span<(Handle, RigidBody)>,
) -> Array<u32> {
    let n = manifolds.len();
    match order {
        Order::Pair => {
            let mut out = array![];
            let mut i = 0;
            while i != n {
                out.append(i);
                i += 1;
            }
            out
        },
        Order::Reversed => {
            let mut out = array![];
            let mut i = n;
            while i != 0 {
                i -= 1;
                out.append(i);
            }
            out
        },
        Order::Colour => colour_order(manifolds, entries),
    }
}

/// One `World::step` (no joints, no events kept) with the touching manifolds solved in `order`;
/// returns the solve permutation (manifold indices in solve order).
fn ordered_step(ref world: World, order: Order) -> Array<u32> {
    let params = world.integration_parameters;
    pipeline::handle_user_changes(ref world.bodies, ref world.colliders);
    let _ = pipeline::detect_collisions(
        params, ref world.bodies, ref world.colliders, ref world.narrow_phase,
    );
    let touching = pipeline::touching_manifolds(world.narrow_phase.pairs.span());
    let entries = world.bodies.iter().span();
    let perm = permutation(order, touching.span(), entries);
    let mut ordered = array![];
    for k in perm.span() {
        ordered.append(*touching.at(*k));
    }
    let mut joints: Array<ImpulseJoint> = array![];
    let (mut store, _) = SolverBodyStoreTrait::from_bodies(ref world.bodies, world.gravity, params);
    solve_island(params, ref store, ref ordered, ref joints);
    store.to_bodies(ref world.bodies);
    // Back to pair order.
    let mut solved = array![];
    let mut i = 0;
    while i != touching.len() {
        let mut k = 0;
        while *perm.at(k) != i {
            k += 1;
        }
        solved.append(*ordered.at(k));
        i += 1;
    }
    if !solved.is_empty() {
        world
            .narrow_phase
            .pairs = pipeline::scatter_touching(world.narrow_phase.pairs.span(), solved.span());
    }
    pipeline::advance_to_final_positions(ref world.bodies, ref world.colliders);
    perm
}

/// The port's contact pairs, in the layout of the `box_stack3` diagnostics of `scenes.json`
/// (upstream prints its pairs in edge order, the port in pair order).
fn print_pairs(ref world: World, step: u32) {
    println!("STEP {}", step);
    for pair in world.narrow_phase.pairs.span() {
        let m = *pair.manifold;
        println!(
            " pair {} {} n1 {} {} n2 {} {} normal {} {}",
            *pair.collider1.index,
            *pair.collider2.index,
            m.local_n1.x.raw,
            m.local_n1.y.raw,
            m.local_n2.x.raw,
            m.local_n2.y.raw,
            m.data.normal.x.raw,
            m.data.normal.y.raw,
        );
        let mut i = 0;
        for p in m.points.span() {
            if i == m.num_points.into() {
                break;
            }
            println!(
                "  p1 {} {} p2 {} {} dist {} fid {} {} imp {} {} ws {} {}",
                p.local_p1.x.raw,
                p.local_p1.y.raw,
                p.local_p2.x.raw,
                p.local_p2.y.raw,
                p.dist.raw,
                p.fid1.packed,
                p.fid2.packed,
                p.data.impulse.raw,
                p.data.tangent_impulse.raw,
                p.data.warmstart_impulse.raw,
                p.data.warmstart_tangent_impulse.raw,
            );
            i += 1;
        }
        let mut i = 0;
        for sc in m.data.solver_contacts.span() {
            if i == m.data.num_solver_contacts.into() {
                break;
            }
            println!(
                "  sc {} dist {} arms {} {} / {} {}",
                sc.contact_id,
                sc.dist.raw,
                sc.anchor1.x.raw,
                sc.anchor1.y.raw,
                sc.anchor2.x.raw,
                sc.anchor2.y.raw,
            );
            i += 1;
        }
    }
}

/// `Order::Pair` through the stage functions is `World::step`, bit for bit (bodies and the
/// narrow-phase cache), over the landing (steps 1–10).
#[test]
fn test_stack_pair_order_is_the_port() {
    let scene = scenes::BOX_STACK3;
    let mut staged = build_world(scene);
    let mut fused = build_world(scene);
    let mut step = 0_u32;
    while step != 10 {
        let perm = ordered_step(ref staged, Order::Pair);
        assert_eq!(perm.span(), array![0, 1, 2].span());
        let _ = fused.step();
        let mut i = 1;
        while i != 4 {
            let a = staged.body(body_handle(i)).unwrap();
            let b = fused.body(body_handle(i)).unwrap();
            assert_eq!(a.position(), b.position());
            assert_eq!(a.linvel(), b.linvel());
            assert_eq!(a.vels.angvel, b.vels.angvel);
            i += 1;
        }
        assert!(staged.narrow_phase.pairs.span() == fused.narrow_phase.pairs.span());
        step += 1;
    }
}

/// Upstream's colouring of the stack: `(box0, box1)` colour 0, `(box1, box2)` colour 1, the
/// ground pair colour 127, i.e. pair indices `[1, 2, 0]`, every step.
#[test]
fn test_stack_colour_order() {
    let mut world = build_world(scenes::BOX_STACK3);
    let mut step = 0_u32;
    while step != 3 {
        let perm = ordered_step(ref world, Order::Colour);
        assert_eq!(perm.span(), array![1, 2, 0].span());
        step += 1;
    }
}

/// Steps 1–10 per solve order, the port's contact pairs printed at the first diverging steps.
/// Returns nothing; asserts the violations of each order and that steps 1–3 match whatever the
/// order (only the ground pair carries impulses before box1 lands at step 4).
#[test]
fn test_stack_first_steps() {
    let scene = scenes::BOX_STACK3;
    for (order, want) in array![(Order::Pair, 7_u32), (Order::Reversed, 6), (Order::Colour, 0)] {
        println!("order {:?}", order);
        let mut world = build_world(scene);
        let mut stats: Stats = Default::default();
        let mut step = 1_u32;
        while step != 11 {
            let _ = ordered_step(ref world, order);
            if step >= 3 && step <= 5 {
                print_pairs(ref world, step);
            }
            stats = compare(ref world, scene, *scene.samples.span().at(step), stats);
            if step == 3 {
                assert_eq!(stats.violations, 0, "steps 1-3 within tolerance");
            }
            step += 1;
        }
        println!("order {:?}: violations {}", order, stats.violations);
        assert_eq!(stats.violations, want);
    }
}

/// Upstream's impulses of one contact point after step 60 (`scenes.json`, `box_stack3`
/// `contact_diagnostics`), matched on the port's point by feature ids.
#[derive(Copy, Drop)]
struct UpstreamPoint {
    collider1: u32,
    collider2: u32,
    fid1: u32,
    fid2: u32,
    impulse: i64,
    tangent_impulse: i64,
    warmstart_impulse: i64,
    warmstart_tangent_impulse: i64,
}

const UPSTREAM_60: [UpstreamPoint; 6] = [
    UpstreamPoint {
        collider1: 2,
        collider2: 3,
        fid1: 3221225524,
        fid2: 1073741826,
        impulse: 350989599,
        tangent_impulse: -19775632,
        warmstart_impulse: 87746830,
        warmstart_tangent_impulse: -4944772,
    },
    UpstreamPoint {
        collider1: 2,
        collider2: 3,
        fid1: 1073741825,
        fid2: 3221225534,
        impulse: 351261819,
        tangent_impulse: 19646692,
        warmstart_impulse: 87815881,
        warmstart_tangent_impulse: 4911676,
    },
    UpstreamPoint {
        collider1: 1,
        collider2: 2,
        fid1: 1073741824,
        fid2: 3221225534,
        impulse: 702044178,
        tangent_impulse: 861150,
        warmstart_impulse: 175509253,
        warmstart_tangent_impulse: 213969,
    },
    UpstreamPoint {
        collider1: 1,
        collider2: 2,
        fid1: 3221225524,
        fid2: 1073741827,
        impulse: 702456565,
        tangent_impulse: -98861,
        warmstart_impulse: 175615661,
        warmstart_tangent_impulse: -24728,
    },
    UpstreamPoint {
        collider1: 0,
        collider2: 1,
        fid1: 3221225524,
        fid2: 1073741826,
        impulse: 1053289081,
        tangent_impulse: 116898,
        warmstart_impulse: 263319068,
        warmstart_tangent_impulse: 27796,
    },
    UpstreamPoint {
        collider1: 0,
        collider2: 1,
        fid1: 3221225524,
        fid2: 1073741827,
        impulse: 1053440364,
        tangent_impulse: -82082,
        warmstart_impulse: 263363023,
        warmstart_tangent_impulse: -20521,
    },
];

/// How a window after step 0 starts.
#[derive(Copy, Drop, PartialEq, Debug)]
enum Seed {
    /// Upstream poses and velocities, empty contact cache (as `test_box_stack3_second_window`).
    Cold,
    /// Upstream poses and velocities, then the manifolds of that state with upstream's impulses
    /// ([`UPSTREAM_60`]) written on the points of equal feature ids: the next step warm-starts
    /// from upstream's solver state.
    Warm,
}

/// Writes [`UPSTREAM_60`] into the port's manifolds of the current state; returns the number of
/// points matched.
fn seed_impulses(ref world: World) -> u32 {
    pipeline::handle_user_changes(ref world.bodies, ref world.colliders);
    let _ = pipeline::detect_collisions(
        world.integration_parameters, ref world.bodies, ref world.colliders, ref world.narrow_phase,
    );
    let mut matched = 0;
    let mut pairs = array![];
    for pair in world.narrow_phase.pairs.span() {
        let mut pair = *pair;
        let [mut p0, mut p1] = pair.manifold.points;
        for up in UPSTREAM_60.span() {
            if *up.collider1 != pair.collider1.index || *up.collider2 != pair.collider2.index {
                continue;
            }
            println!(
                "seed pair {} {}: port fids {} {} / {} {}, upstream {} {}",
                *up.collider1,
                *up.collider2,
                p0.fid1.packed,
                p0.fid2.packed,
                p1.fid1.packed,
                p1.fid2.packed,
                *up.fid1,
                *up.fid2,
            );
            if p0.fid1.packed == *up.fid1 && p0.fid2.packed == *up.fid2 {
                p0.data = data_of(*up);
                matched += 1;
            } else if p1.fid1.packed == *up.fid1 && p1.fid2.packed == *up.fid2 {
                p1.data = data_of(*up);
                matched += 1;
            }
        }
        pair.manifold.points = [p0, p1];
        pairs.append(pair);
    }
    world.narrow_phase.pairs = pairs;
    matched
}

fn data_of(up: UpstreamPoint) -> ContactData {
    ContactData {
        impulse: f(up.impulse),
        tangent_impulse: f(up.tangent_impulse),
        warmstart_impulse: f(up.warmstart_impulse),
        warmstart_tangent_impulse: f(up.warmstart_tangent_impulse),
    }
}

/// Replays the window `[start, end]` of `box_stack3` with the solve `order`, re-seeded from the
/// upstream sample at `start` when non-zero; returns the maxima and violations.
fn window(order: Order, start: u32, end: u32, seed: Seed) -> Stats {
    let scene = scenes::BOX_STACK3;
    let mut world = build_world(scene);
    let samples = scene.samples.span();
    let mut next = 0;
    while *samples.at(next).step < start {
        next += 1;
    }
    if start != 0 {
        reseed(ref world, scene, *samples.at(next));
        if seed == Seed::Warm {
            assert!(start == 60, "upstream impulses are recorded at step 60");
            assert_eq!(seed_impulses(ref world), 6, "every upstream point matched");
        }
    }
    next += 1;
    let mut stats: Stats = Default::default();
    let mut step = start;
    while step != end {
        let _ = ordered_step(ref world, order);
        step += 1;
        if *samples.at(next).step == step {
            stats = compare(ref world, scene, *samples.at(next), stats);
            next += 1;
        }
    }
    println!(
        "{:?} {:?} [{}..{}] max ulps (step): tx {} ({}) ty {} ({}) re {} ({}) im {} ({}) vx {} ({}) vy {} ({}) w {} ({}); violations {}",
        order,
        seed,
        start,
        end,
        stats.tx.ulps,
        stats.tx.step,
        stats.ty.ulps,
        stats.ty.step,
        stats.re.ulps,
        stats.re.step,
        stats.im.ulps,
        stats.im.step,
        stats.vx.ulps,
        stats.vx.step,
        stats.vy.ulps,
        stats.vy.step,
        stats.w.ulps,
        stats.w.step,
        stats.violations,
    );
    stats
}

/// Upstream's solve order over the first window: every sample within tolerance.
#[test]
fn test_stack_colour_order_first_window() {
    let stats = window(Order::Colour, 0, 60, Seed::Cold);
    assert_eq!(stats.violations, 0);
}

/// Second window per order and seed: the violations of each.
#[test]
fn test_stack_second_window_cold_colour() {
    assert_eq!(window(Order::Colour, 60, 120, Seed::Cold).violations, 2);
}

#[test]
fn test_stack_second_window_warm_colour() {
    assert_eq!(window(Order::Colour, 60, 120, Seed::Warm).violations, 0);
}

#[test]
fn test_stack_second_window_warm_pair() {
    assert_eq!(window(Order::Pair, 60, 120, Seed::Warm).violations, 0);
}

/// The stack's contact pairs at step 1 (all three touching) and the body walk: the probe inputs.
fn probe_inputs() -> (Span<ContactPair>, Span<(Handle, RigidBody)>) {
    let mut world = build_world(scenes::BOX_STACK3);
    pipeline::handle_user_changes(ref world.bodies, ref world.colliders);
    let _ = pipeline::detect_collisions(
        world.integration_parameters, ref world.bodies, ref world.colliders, ref world.narrow_phase,
    );
    (opaque(world.narrow_phase.pairs.span()), opaque(world.bodies.iter().span()))
}

/// Setup only: subtract from the `gas_order_*` probes.
#[test]
fn gas_order_baseline() {
    let (pairs, entries) = probe_inputs();
    assert_eq!(pairs.len() + entries.len(), 7);
}

/// Today's solver input (D8): `pipeline::touching_manifolds`.
#[test]
fn gas_order_pair() {
    let (pairs, entries) = probe_inputs();
    let m = pipeline::touching_manifolds(pairs);
    assert_eq!(m.len() + entries.len(), 7);
}

/// Stateless fixed-last partition (candidate): same permutation as upstream on the stack.
#[test]
fn gas_order_fixed_last() {
    let (pairs, entries) = probe_inputs();
    let m = fixed_last(pairs, entries);
    assert_eq!(m.len() + entries.len(), 7);
}

/// On the stack the fixed-last partition is upstream's colour order.
#[test]
fn test_stack_fixed_last_is_colour_order() {
    let (pairs, entries) = probe_inputs();
    let touching = pipeline::touching_manifolds(pairs);
    let perm = colour_order(touching.span(), entries);
    let m = fixed_last(pairs, entries);
    assert_eq!(perm.len(), m.len());
    let mut k = 0;
    while k != m.len() {
        assert!(*m.at(k) == *touching.at(*perm.at(k)));
        k += 1;
    }
}

/// Stateless greedy colouring recomputed every step (upper bound of faithful parity without
/// persisted colours): touching manifolds, colours, then the permuted copy.
#[test]
fn gas_order_colour() {
    let (pairs, entries) = probe_inputs();
    let touching = pipeline::touching_manifolds(pairs);
    let perm = colour_order(touching.span(), entries);
    let mut m = array![];
    for k in perm.span() {
        m.append(*touching.at(*k));
    }
    assert_eq!(m.len() + entries.len(), 7);
}
