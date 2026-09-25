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
IndexView Default Into TryInto PartialEq Drop Copy Serde Debug
""".split())

EXCLUSIONS = (
    "dim3-only", "soft bodies", "multibody", "SIMD/parallel", "debug render",
    "serde/rkyv/bytemuck", "profiling counters", "dyn hooks",
    "trimesh/voxels/3D heightfield", "EPA/GJK internals not exposed",
    "f32/f64 conversions and approx traits",
)


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
    "QueryPipeline": ("World", "queries"), "DefaultQueryDispatcher": ("dispatch",),
    "PersistentQueryDispatcher": ("dispatch",), "RigidBodyHandle": ("Handle",),
    "ColliderHandle": ("Handle",), "ImpulseJointHandle": ("Handle",),
    "MultibodyJointHandle": ("Handle",), "IslandManager": ("pipeline::islands", "World"),
    "BroadPhaseBvh": ("broad_phase",), "NarrowPhase": ("NarrowPhase",),
    "Halfspace": ("HalfSpace",),
    "MassProperties": ("MassProperties", "RigidBodyMassProps", "ColliderMassProps"),
    "RigidBodyMassProps": ("RigidBodyMassProps", "RigidBody"),
    "RigidBodyColliders": ("RigidBodyColliders", "RigidBodySet"), "ColliderShape": ("Shape",),
})

METHOD_RENAMES: dict[tuple[str, str], tuple[str, ...]] = {
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
    ("ColliderPosition", "From<T>"): ("From<Pose2>",),
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
    return {"Real": "Real", "Self": "Self", "Halfspace": "HalfSpace"}.get(head, head)


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


def rust_files(root: Path, dirs: tuple[str, ...]) -> list[tuple[str, Path]]:
    src = root / "src"
    if not src.is_dir():
        raise SystemExit(f"missing source directory: {src}")
    paths = []
    for d in dirs:
        base = src / d
        if base.is_file():
            paths.append(base)
        elif base.is_dir():
            paths.extend(base.rglob("*.rs"))
    return [(str(p.relative_to(src)), p) for p in sorted(paths)]


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
            for m in re.finditer(r"\bpub\s+(?:const\s+)?(?:unsafe\s+)?fn\s+([A-Za-z_][A-Za-z0-9_]*)", body):
                items.add(Item(owner, "method", m.group(1), module, f"{crate}/src/{rel}"))
            for m in re.finditer(r"\bpub\s+const\s+([A-Z][A-Z0-9_]*)\s*:", body):
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
    if head == "IndexView" and len(args) >= 2:
        return Item(owner, "impl", f"Index<{normalize_rhs(args[1])}>")
    if head in ("Add", "AddAssign", "Sub", "SubAssign", "Mul", "MulAssign", "Div", "DivAssign") and args:
        return Item(owner, "impl", f"{head}<{normalize_rhs(args[-1])}>")
    return Item(owner, "impl", head)


def parse_cairo() -> list[Item]:
    items: set[Item] = set()
    impl_re = re.compile(r"\bpub\s+impl\s+([A-Za-z_][A-Za-z0-9_]*)\s+of\s+([^{\n]+)\s*\{")
    trait_re = re.compile(r"\bpub\s+trait\s+([A-Za-z_][A-Za-z0-9_]*)[^{]*\{")
    mod_re = re.compile(r"\bpub\s+mod\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{")
    for path in sorted((ROOT / "crates").glob("*/src/**/*.cairo")):
        if any(part in {"tests", "benches", "alternatives", "fixtures", "generated", "probes"} for part in path.parts):
            continue
        source = str(path.relative_to(ROOT))
        fallback = cairo_owner_from_path(path)
        text = mask_comments(path.read_text())
        skip = cfg_test_spans(text)
        impls = [(m, b, e) for m, b, e in blocks(text, impl_re) if not inside(m.start(), skip)]
        traits = [(m, b, e) for m, b, e in blocks(text, trait_re) if not inside(m.start(), skip)]
        excluded = skip + [(b, e) for _, b, e in impls + traits]

        for m in re.finditer(r"\bpub\s+(struct|enum|trait)\s+([A-Za-z_][A-Za-z0-9_]*)", text):
            if not inside(m.start(), skip):
                items.add(Item(m.group(2), "trait" if m.group(1) == "trait" else "type", m.group(2), source, source))
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
    blob = " ".join((item.owner, item.kind, item.name, item.module, item.source)).lower()
    impl = item.kind == "impl"
    if any(x in blob for x in ("soft_body", "softbody", "softelastic", "deformable_mesh", "insert_deformable")) \
            or item.name == "soft_bodies" and item.owner in ("PhysicsWorld", "Quarantine"):
        return "soft bodies"
    if "multibody" in blob:
        return "multibody"
    if any(x in blob for x in ("simd", "parallel", "coloring", "graph_col", "thread_pool", "num_threads")):
        return "SIMD/parallel"
    if "debug_render" in blob:
        return "debug render"
    if any(x in blob for x in ("counter", "timer")) and "controller" not in blob:
        return "profiling counters"
    if item.owner in ("PhysicsHooks", "EventHandler", "ChannelEventCollector") or "physics_hooks" in blob:
        return "dyn hooks"
    if any(x in blob for x in ("trimesh", "voxels", "heightfield3", "height_field3")) \
            or (item.owner, item.name) == ("ColliderBuilder", "voxelized_mesh"):
        return "trimesh/voxels/3D heightfield"
    if any(x in blob for x in ("epa", "gjk", "simplex")):
        return "EPA/GJK internals not exposed"
    if impl and any(x in item.name for x in ("Serialize", "Deserialize", "Archive", "Pod", "Zeroable")):
        return "serde/rkyv/bytemuck"
    if any(x in blob for x in ("serde", "rkyv", "bytemuck")):
        return "serde/rkyv/bytemuck"
    if impl and any(x in item.name for x in ("AbsDiffEq", "RelativeEq", "UlpsEq")):
        return "f32/f64 conversions and approx traits"
    if any(x in blob for x in ("f32", "f64", "approx")):
        return "f32/f64 conversions and approx traits"
    if "Spherical" in item.owner or any(x in blob for x in (
        "dim3", "polyhedron", "tetrahedron", "cone", "cylinder", "spherical_joint")) \
            or (item.owner, item.name) == ("ColliderBuilder", "capsule_z"):
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
        "## Coverage summary",
        "",
        "| Module | Ported | Partial | Missing | Excluded | Items | Coverage |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    total = {k: 0 for k in ("ported", "partial", "missing", "excluded")}
    for module in modules:
        owned = [item for item in rust if item.module == module]
        counts = {k: sum(statuses[item][0] == k for item in owned) for k in total}
        for k, v in counts.items():
            total[k] += v
        denom = len(owned) - counts["excluded"]
        cov = "—" if denom <= 0 else f"{100.0 * counts['ported'] / denom:.1f}%"
        lines.append(f"| {module} | {counts['ported']} | {counts['partial']} | {counts['missing']} | {counts['excluded']} | {len(owned)} | {cov} |")
    denom = len(rust) - total["excluded"]
    cov = "—" if denom <= 0 else f"{100.0 * total['ported'] / denom:.1f}%"
    lines.append(f"| **total** | **{total['ported']}** | **{total['partial']}** | **{total['missing']}** | **{total['excluded']}** | **{len(rust)}** | **{cov}** |")
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
