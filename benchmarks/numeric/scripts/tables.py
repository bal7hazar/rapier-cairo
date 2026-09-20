#!/usr/bin/env python3
"""Pivot tables (markdown) from results.json, as used in the report. Cell format: steps / range_check / l2_gas."""
import json, pathlib
ROOT = pathlib.Path(__file__).resolve().parent.parent
r = json.load(open(ROOT / "results.json"))
def cell(m, op):
    v = r[m]["ops"].get(op)
    if not v or not v["net"]: return "–"
    n = v["net"]; bw = n["builtins"].get("bitwise", 0)
    return f"{n['steps']} / {n['builtins'].get('range_check', 0)}{' +%dbw' % bw if bw else ''} / {n['l2_gas']}"
def pivot(title, mods, labels, order=None):
    ops = order or sorted(set(o for m in mods for o in r[m]["ops"]))
    print(f"\n**{title}**\n\n| op | " + " | ".join(labels) + " |\n|---|" + "---:|" * len(mods))
    for op in ops:
        print(f"| {op} | " + " | ".join(cell(m, op) for m in mods) + " |")
def flat(title, m):
    print(f"\n**{title}**\n\n| case | steps | range_check | bitwise | l2_gas |\n|---|---:|---:|---:|---:|")
    for op in sorted(r[m]["ops"]):
        n = r[m]["ops"][op]["net"]
        print(f"| {op} | {n['steps']} | {n['builtins'].get('range_check', 0)} | {n['builtins'].get('bitwise', 0)} | {n['l2_gas']} |")
A_ORDER = ["add_same_sign","add_mixed_sign","sub","mul","div","neg","abs","lt","lt_same_sign","sqrt","sin","sin_fast_lut","cos","cos_fast_lut","atan","atan_fast_lut","exp","floor","round","from_int","to_int_u32"]
pivot("A. Library scalar ops", ["a_cubit_f64","a_cubit_f128","a_orion_fp16x16","a_orion_fp8x23","a_orion_fp32x32"], ["cubit f64 (32.32)","cubit f128 (64.64)","orion FP16x16","orion FP8x23","orion FP32x32"], A_ORDER)
B_ORDER = ["add_same_sign","add_mixed_sign","sub","mul","div","neg","lt_mixed_sign","lt_same_sign","sqrt","dot2","dot2_fused","cross2","cross2_fused","length2","length2_fused","length2_wide_sqrt","normalize2","normalize2_fused","normalize2_wide_sqrt","normalize2_fused_rsqrt","mat3_mul_vec3","mat3_mul_vec3_fused"]
pivot("B. Representations", ["b_cubit_f64","b_sm64","b_i64n","b_i64b","b_felt","b_cubit_f128","b_sm128"], ["cubit f64","sm64","i64 naive","i64+BoundedInt","felt lazy","cubit f128","sm128"], B_ORDER)
flat("B'. mul-then-shift and division primitives", "b_mulshift")
flat("C. sqrt", "c_sqrt")
flat("D. trig cost", "d_trig")
flat("E. Orion tensors vs struct Mat3", "e_orion_linalg")
flat("F. math vs bitwise vs loop", "f_heuristic")
flat("G. pitfalls / extra kernels", "g_pitfalls")
