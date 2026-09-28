"""CS3 throwaway lever builds (prototype branch only, never merged): cumulative stubs on a
`git archive HEAD` copy of the prototype in `.cs3-throwaway/copy` (git-excluded), each followed by
a release build of rapier_sink and the Sierra felts, CASM felts, Sierra and CASM class bytes of the
given classes (default `Split4Step BasicGameStep`). A lever "off" is a panicking stub on a branch
the game never takes; a stage "out" is replaced by a wrapper that serialises its inputs and
deserialises its result (what a library-call wrapper compiles, without the syscall), so that the
rest of the step stays reachable. Sizes only: the stubbed builds do not step.

usage: python3 scripts/cs3_levers.py [CLASS ...]   (see docs/research/class-split.md, section 5)"""
import json, os, re, shutil, subprocess, sys
from pathlib import Path

WT = Path(__file__).resolve().parent.parent
COPY = WT / ".cs3-throwaway" / "copy"
sys.path.insert(0, str(WT / "scripts"))
import bytecode_size as bs  # noqa

def wrapper(*inputs):
    """A library-call wrapper's code without the syscall: the inputs serialised, the result
    deserialised (from felts the compiler cannot see through)."""
    ser = "".join(f"    {x}.serialize(ref cs3_a);\n" for x in inputs)
    return ("{\n    let mut cs3_a: Array<felt252> = array![];\n" + ser
            + "    let mut cs3_s = cs3_a.span();\n    Serde::deserialize(ref cs3_s).expect('cs3 stub')\n}")

def stub_fn(path, name, nth=0, body=None):
    p = COPY / path
    s = p.read_text()
    idx = [m.start() for m in re.finditer(r"fn " + re.escape(name) + r"\s*[<(]", s)][nth]
    # skip the signature: first '{' after the closing ')' of the parameter list at depth 0
    i = s.index("(", idx)
    depth = 0
    while True:
        c = s[i]
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                break
        i += 1
    b = s.index("{", i)
    depth, j = 0, b
    while True:
        if s[j] == "{":
            depth += 1
        elif s[j] == "}":
            depth -= 1
            if depth == 0:
                break
        j += 1
    s = s[:b] + (body or "{\n    core::panic_with_felt252('cs3 stub')\n}") + s[j + 1:]
    p.write_text(s)

def stub_block(path, opener):
    p = COPY / path
    s = p.read_text()
    b = s.index(opener) + len(opener) - 1
    assert s[b] == "{"
    depth, j = 0, b
    while True:
        if s[j] == "{":
            depth += 1
        elif s[j] == "}":
            depth -= 1
            if depth == 0:
                break
        j += 1
    s = s[:b] + "{\n        core::panic_with_felt252('cs3 stub')\n    }" + s[j + 1:]
    p.write_text(s)

LEVERS = [
    ("free_path (pair-free fast path) off", lambda: (
        stub_block("crates/rapier2d/src/pipeline.cairo",
                   "let (scratch, sleeping, force_events, pairs) = if free_candidate {"))),
    ("+ non-basic shape arms (SH1 / SH2 AABB, mass, Serde helpers) off", lambda: [
        stub_fn("crates/rapier_geometry2d/src/shape.cairo", n) for n in [
            "deserialize_sh1", "serialize_triangle", "serialize_round_cuboid",
            "serialize_round_triangle", "serialize_round_convex_polygon", "serialize_polyline",
            "serialize_heightfield", "serialize_compound", "deserialize_sh2a", "sh1_aabb",
            "sh1_mass_properties"]]),
    ("+ mass properties out (per-shape `mass_properties` behind a wrapper)", lambda: (
        stub_fn("crates/rapier_geometry2d/src/shape.cairo", "mass_properties",
                body=wrapper("self", "density")))),
    ("+ islands out (`update_islands`, `islands_after_insertions` behind wrappers)", lambda: (
        stub_fn("crates/rapier2d/src/pipeline/islands.cairo", "update_islands",
                body=wrapper("pairs", "dormant", "entries")),
        stub_fn("crates/rapier2d/src/pipeline/sleeping.cairo", "islands_after_insertions",
                body=wrapper("pairs", "dormant", "entries", "fresh")))),
    ("+ broad phase out (`find_pairs`, `find_pairs_sparse`, `near_statics` behind wrappers)", lambda: (
        stub_fn("crates/rapier_geometry2d/src/broad_phase.cairo", "find_pairs",
                body=wrapper("proxies")),
        stub_fn("crates/rapier_geometry2d/src/broad_phase/sparse.cairo", "find_pairs_sparse",
                body=wrapper("statics", "dynamic")),
        stub_fn("crates/rapier2d/src/pipeline/active_set.cairo", "near_statics",
                body=wrapper("statics", "dynamic")))),
    ("+ solve and advance out (fused stage behind a wrapper that writes the moved bodies and colliders back)", lambda: (
        (COPY / "crates/rapier2d/src/pipeline/fused.cairo").write_text(
            (COPY / "crates/rapier2d/src/pipeline/fused.cairo").read_text().replace(
                "use rapier_dynamics2d::narrow_phase::NarrowPhase;",
                "use rapier_dynamics2d::narrow_phase::{ContactPair, NarrowPhase};")),
        stub_fn("crates/rapier2d/src/pipeline/fused.cairo", "solve_and_advance_sleeping_with", body="""{
    let mut cs3_a: Array<felt252> = array![];
    entries.serialize(ref cs3_a);
    snapshot.serialize(ref cs3_a);
    narrow_phase.pairs.span().serialize(ref cs3_a);
    let mut cs3_s = cs3_a.span();
    let (moved, moved_colliders, pairs): (
        Span<(Handle, RigidBody)>, Span<(Handle, Collider)>, Array<ContactPair>,
    ) = Serde::deserialize(ref cs3_s).expect('cs3 stub');
    for (h, b) in moved {
        let _ = bodies.set_internal(*h, *b);
    }
    for (h, c) in moved_colliders {
        let _ = colliders.set_internal(*h, *c);
    }
    narrow_phase.pairs = pairs;
    bodies.mark_modified();
    colliders.mark_modified();
}"""))),
    ("+ narrow-phase pair loop out (batched, behind a wrapper)", lambda: (
        stub_fn("crates/rapier_dynamics2d/src/narrow_phase.cairo", "compute_contacts_from_scratch_with", body="""{
    let mut cs3_a: Array<felt252> = array![];
    self.pairs.span().serialize(ref cs3_a);
    prediction.serialize(ref cs3_a);
    pairs.serialize(ref cs3_a);
    for co in scratch {
        co.handle.serialize(ref cs3_a);
        co.shape.serialize(ref cs3_a);
        co.pose.serialize(ref cs3_a);
    }
    let mut cs3_s = cs3_a.span();
    let (current, events): (Array<ContactPair>, Array<CollisionEvent>) = Serde::deserialize(
        ref cs3_s,
    )
        .expect('cs3 stub');
    self.pairs = current;
    events
}"""))),
]

def build():
    env = dict(os.environ, SCARB_PROFILE="release")
    p = subprocess.run([str(WT / "scripts/build-shims/scarb"), "build", "-p", "rapier_sink"],
                       cwd=COPY, capture_output=True, text=True, env=env)
    if p.returncode != 0:
        print((p.stdout + p.stderr)[-4000:])
        sys.exit("build failed")
    return bs.measure(COPY / "target" / "release", "rapier_sink")

def main():
    if COPY.exists():
        shutil.rmtree(COPY)
    COPY.mkdir(parents=True)
    subprocess.run(f"git -C {WT} archive HEAD | tar -x -C {COPY}", shell=True, check=True)
    classes = sys.argv[1:] or ["Split4Step", "BasicGameStep"]
    rows = []
    for label, apply in LEVERS:
        apply()
        m = build()
        rows.append((label, {c: m[c] for c in classes}))
        print(label, {c: (m[c]["sierra_felts"], m[c]["casm_felts"], m[c]["sierra_bytes"], m[c]["casm_bytes"]) for c in classes}, flush=True)

main()
