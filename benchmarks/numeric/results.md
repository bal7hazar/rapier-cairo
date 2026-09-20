
### a_cubit_f128

Baselines: `b1` = 67 steps / 14520 gas, `bi` = 67 steps / 14520 gas, `b2` = 71 steps / 14920 gas, `bo` = 65 steps / 14320 gas, `b2b` = 70 steps / 14820 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| abs | 0 | 0 | 0 |
| atan | 1116 | 258 | 188720 |
| atan_fast_lut | 304 | 60 | 85890 |
| cos | 3676 | 947 | 464880 |
| cos_fast_lut | 444 | 81 | 57250 |
| exp | 1091 | 262 | 157820 |
| floor | 41 | 13 | 5110 |
| neg | 5 | 0 | 500 |
| round | 55 | 14 | 7580 |
| sin | 3642 | 945 | 460230 |
| sin_fast_lut | 386 | 79 | 52600 |
| sqrt | 74 | 21 | 8880 |
| add_mixed_sign | 27 | 2 | 2960 |
| add_same_sign | 19 | 1 | 2130 |
| div | 93 | 24 | 11880 |
| mul | 91 | 24 | 11580 |
| sub | 26 | 1 | 4550 |
| lt | 0 | 0 | 0 |
| lt_same_sign | 15 | 1 | 1580 |
| from_int | 26 | 9 | 3230 |
| to_int_u32 | 16 | 6 | 2120 |

### a_cubit_f64

Baselines: `b2b` = 70 steps / 14820 gas, `b2` = 71 steps / 14920 gas, `b1` = 67 steps / 14520 gas, `bo` = 65 steps / 14320 gas, `bi` = 67 steps / 14520 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| abs | 0 | 0 | 0 |
| atan | 436 | 68 | 83740 |
| atan_fast_lut | 319 | 18 | 57290 |
| cos | 1084 | 199 | 134820 |
| cos_fast_lut | 263 | 31 | 34870 |
| exp | 398 | 63 | 64870 |
| floor | 16 | 4 | 1880 |
| neg | 5 | 0 | 500 |
| round | 31 | 5 | 3570 |
| sin | 1051 | 197 | 130270 |
| sin_fast_lut | 206 | 29 | 30320 |
| sqrt | 35 | 13 | 4410 |
| add_mixed_sign | 15 | 2 | 1640 |
| add_same_sign | 4 | 1 | 470 |
| div | 18 | 5 | 2250 |
| mul | 18 | 5 | 2250 |
| sub | 27 | 1 | 4450 |
| lt | 0 | 0 | 0 |
| lt_same_sign | 15 | 1 | 1580 |
| from_int | 4 | 1 | 470 |
| to_int_u32 | 12 | 4 | 1480 |

### a_orion_fp16x16

Baselines: `b1` = 67 steps / 14520 gas, `bo` = 65 steps / 14320 gas, `b2` = 71 steps / 14920 gas, `bi` = 67 steps / 14520 gas, `b2b` = 70 steps / 14820 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| abs | 0 | 0 | 0 |
| atan | 380 | 56 | 73680 |
| atan_fast_lut | 313 | 16 | 54940 |
| cos | 1011 | 174 | 123270 |
| cos_fast_lut | 229 | 28 | 29840 |
| exp | 326 | 48 | 53300 |
| floor | 16 | 4 | 1880 |
| neg | 5 | 0 | 500 |
| round | 31 | 5 | 3570 |
| sin | 976 | 172 | 118670 |
| sin_fast_lut | 172 | 26 | 25290 |
| sqrt | 13 | 5 | 1650 |
| add_mixed_sign | 15 | 2 | 1640 |
| add_same_sign | 4 | 1 | 470 |
| div | 15 | 4 | 1780 |
| mul | 15 | 4 | 1780 |
| sub | 27 | 1 | 4450 |
| lt | 0 | 0 | 0 |
| lt_same_sign | 15 | 1 | 1580 |
| from_int | 4 | 1 | 470 |
| to_int_u32 | 9 | 3 | 1110 |

### a_orion_fp32x32

Baselines: `b1` = 67 steps / 14520 gas, `b2` = 71 steps / 14920 gas, `bo` = 65 steps / 14320 gas, `b2b` = 70 steps / 14820 gas, `bi` = 67 steps / 14520 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| abs | 0 | 0 | 0 |
| atan | 436 | 68 | 83740 |
| atan_fast_lut | 319 | 18 | 57290 |
| cos | 1084 | 199 | 134820 |
| cos_fast_lut | 263 | 31 | 34870 |
| exp | 398 | 63 | 64870 |
| floor | 16 | 4 | 1880 |
| neg | 5 | 0 | 500 |
| round | 31 | 5 | 3570 |
| sin | 1051 | 197 | 130270 |
| sin_fast_lut | 206 | 29 | 30320 |
| sqrt | 35 | 13 | 4410 |
| add_mixed_sign | 15 | 2 | 1640 |
| add_same_sign | 4 | 1 | 470 |
| div | 18 | 5 | 2250 |
| mul | 18 | 5 | 2250 |
| sub | 27 | 1 | 4450 |
| lt | 0 | 0 | 0 |
| lt_same_sign | 15 | 1 | 1580 |
| from_int | 4 | 1 | 470 |
| to_int_u32 | 12 | 4 | 1480 |

### a_orion_fp8x23

Baselines: `b1` = 67 steps / 14520 gas, `bo` = 65 steps / 14320 gas, `bi` = 67 steps / 14520 gas, `b2` = 71 steps / 14920 gas, `b2b` = 70 steps / 14820 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| abs | 0 | 0 | 0 |
| atan | 406 | 58 | 77630 |
| atan_fast_lut | 313 | 16 | 54940 |
| cos | 1011 | 174 | 123270 |
| cos_fast_lut | 229 | 28 | 29840 |
| exp | 362 | 53 | 58830 |
| floor | 16 | 4 | 1880 |
| neg | 5 | 0 | 500 |
| round | 31 | 5 | 3570 |
| sin | 976 | 172 | 118670 |
| sin_fast_lut | 172 | 26 | 25290 |
| sqrt | 13 | 5 | 1650 |
| add_mixed_sign | 15 | 2 | 1640 |
| add_same_sign | 4 | 1 | 470 |
| div | 15 | 4 | 1780 |
| mul | 15 | 4 | 1780 |
| sub | 27 | 1 | 4450 |
| lt | 0 | 0 | 0 |
| lt_same_sign | 15 | 1 | 1580 |
| from_int | 4 | 1 | 470 |
| to_int_u32 | 9 | 3 | 1110 |

### b_cubit_f128

Baselines: `v2` = 79 steps / 15720 gas, `v1v` = 73 steps / 15120 gas, `b1` = 67 steps / 14520 gas, `m3` = 116 steps / 19420 gas, `v1` = 71 steps / 14920 gas, `b2` = 71 steps / 14920 gas, `b2b` = 70 steps / 14820 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| neg | 5 | 0 | 500 |
| sqrt | 74 | 21 | 8880 |
| add_mixed_sign | 27 | 2 | 2960 |
| add_same_sign | 19 | 1 | 2130 |
| div | 93 | 24 | 11880 |
| mul | 91 | 24 | 11580 |
| sub | 26 | 1 | 4550 |
| lt_mixed_sign | 0 | 0 | 0 |
| lt_same_sign | 15 | 1 | 1580 |
| mat3_mul_vec3 | 959 | 226 | 121890 |
| length2 | 275 | 70 | 34780 |
| normalize2 | 468 | 118 | 59240 |
| cross2 | 209 | 49 | 27810 |
| dot2 | 214 | 50 | 27710 |

### b_cubit_f64

Baselines: `b2` = 71 steps / 14920 gas, `b1` = 67 steps / 14520 gas, `b2b` = 70 steps / 14820 gas, `v1v` = 73 steps / 15120 gas, `v1` = 71 steps / 14920 gas, `m3` = 116 steps / 19420 gas, `v2` = 79 steps / 15720 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| neg | 5 | 0 | 500 |
| sqrt | 35 | 13 | 4410 |
| add_mixed_sign | 15 | 2 | 1640 |
| add_same_sign | 4 | 1 | 470 |
| div | 18 | 5 | 2250 |
| mul | 18 | 5 | 2250 |
| sub | 27 | 1 | 4450 |
| lt_mixed_sign | 0 | 0 | 0 |
| lt_same_sign | 15 | 1 | 1580 |
| mat3_mul_vec3 | 344 | 55 | 47370 |
| length2 | 109 | 24 | 14480 |
| normalize2 | 143 | 34 | 19560 |
| cross2 | 71 | 11 | 9880 |
| dot2 | 69 | 12 | 9270 |

### b_felt

Baselines: `b2` = 70 steps / 14820 gas, `b2b` = 70 steps / 14820 gas, `v1` = 70 steps / 14820 gas, `b1` = 65 steps / 14320 gas, `v2` = 78 steps / 15620 gas, `m3` = 112 steps / 19020 gas, `v1v` = 71 steps / 14920 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| neg | 1 | 0 | 100 |
| sqrt | 12 | 5 | 1550 |
| add_mixed_sign | 0 | 0 | 0 |
| add_same_sign | 0 | 0 | 0 |
| div | 61 | 11 | 7050 |
| mul | 16 | 5 | 1950 |
| sub | 0 | 0 | 0 |
| lt_mixed_sign | 11 | 2 | 1240 |
| lt_same_sign | 11 | 2 | 1240 |
| mat3_mul_vec3 | 177 | 45 | 20850 |
| mat3_mul_vec3_fused | 82 | 15 | 9810 |
| length2 | 58 | 15 | 7510 |
| length2_fused | 38 | 10 | 4620 |
| normalize2 | 178 | 37 | 20850 |
| cross2 | 42 | 10 | 5020 |
| cross2_fused | 18 | 5 | 2150 |
| dot2 | 42 | 10 | 5020 |
| dot2_fused | 18 | 5 | 2150 |

### b_i64b

Baselines: `b1` = 65 steps / 14320 gas, `m3` = 112 steps / 19020 gas, `v1v` = 71 steps / 14920 gas, `b2b` = 70 steps / 14820 gas, `b2` = 70 steps / 14820 gas, `v2` = 78 steps / 15620 gas, `v1` = 70 steps / 14820 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| neg | 3 | 0 | 300 |
| sqrt | 17 | 6 | 2120 |
| add_mixed_sign | 6 | 2 | 740 |
| add_same_sign | 6 | 2 | 740 |
| div | 44 | 7 | 5730 |
| mul | 16 | 5 | 1950 |
| sub | 6 | 2 | 740 |
| lt_mixed_sign | 6 | 1 | 670 |
| lt_same_sign | 6 | 1 | 670 |
| mat3_mul_vec3 | 202 | 57 | 24920 |
| mat3_mul_vec3_fused | 82 | 15 | 10180 |
| length2 | 65 | 18 | 8660 |
| length2_fused | 42 | 11 | 5090 |
| length2_wide_sqrt | 17 | 6 | 2120 |
| normalize2 | 151 | 32 | 19120 |
| normalize2_fused | 131 | 25 | 16630 |
| normalize2_fused_rsqrt | 87 | 25 | 11480 |
| normalize2_wide_sqrt | 112 | 20 | 14500 |
| cross2 | 51 | 12 | 6670 |
| cross2_fused | 18 | 5 | 2150 |
| dot2 | 51 | 12 | 6670 |
| dot2_fused | 18 | 5 | 2150 |

### b_i64n

Baselines: `b1` = 65 steps / 14320 gas, `b2` = 70 steps / 14820 gas, `v2` = 78 steps / 15620 gas, `b2b` = 70 steps / 14820 gas, `m3` = 112 steps / 19020 gas, `v1v` = 71 steps / 14920 gas, `v1` = 70 steps / 14820 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| neg | 3 | 0 | 300 |
| sqrt | 16 | 6 | 2020 |
| add_mixed_sign | 6 | 2 | 740 |
| add_same_sign | 6 | 2 | 740 |
| div | 88 | 20 | 12880 |
| mul | 33 | 7 | 4010 |
| sub | 6 | 2 | 740 |
| lt_mixed_sign | 6 | 1 | 670 |
| lt_same_sign | 6 | 1 | 670 |
| mat3_mul_vec3 | 270 | 75 | 34280 |
| length2 | 78 | 22 | 10640 |
| normalize2 | 254 | 62 | 36230 |
| cross2 | 66 | 16 | 8750 |
| dot2 | 66 | 16 | 8750 |

### b_mulshift

Baselines: `u3` = 74 steps / 15220 gas, `w2` = 70 steps / 14820 gas, `w1` = 65 steps / 14320 gas, `y2` = 71 steps / 14920 gas, `u2` = 70 steps / 14820 gas, `x2` = 75 steps / 15320 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| mulshift_bounded_int | 12 | 4 | 1480 |
| mulshift_div_operator_literal | 15 | 5 | 1950 |
| mulshift_divrem_const_nonzero | 15 | 5 | 1950 |
| mulshift_felt | 17 | 6 | 2220 |
| u64_checked_mul | 4 | 1 | 470 |
| u64_div_const | 8 | 3 | 1010 |
| u64_div_runtime | 8 | 3 | 1010 |
| u64_wide_mul_only | 0 | 0 | 0 |
| mulshift_div_runtime_divisor | 15 | 5 | 1950 |
| u128_div_const | 13 | 4 | 1580 |
| u128_checked_mul | 25 | 9 | 3130 |
| u128_div_runtime | 12 | 4 | 1480 |
| u256_div_runtime | 57 | 15 | 7070 |
| u128_wide_mul_only | 24 | 9 | 3030 |

### b_sm128

Baselines: `b2` = 71 steps / 14920 gas, `v1` = 71 steps / 14920 gas, `b1` = 67 steps / 14520 gas, `m3` = 116 steps / 19420 gas, `v2` = 79 steps / 15720 gas, `v1v` = 73 steps / 15120 gas, `b2b` = 70 steps / 14820 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| neg | 2 | 0 | 200 |
| sqrt | 65 | 16 | 7740 |
| add_mixed_sign | 27 | 2 | 2960 |
| add_same_sign | 19 | 1 | 2130 |
| div | 93 | 24 | 11880 |
| mul | 53 | 15 | 6570 |
| sub | 24 | 1 | 4250 |
| lt_mixed_sign | 0 | 0 | 0 |
| lt_same_sign | 15 | 1 | 1580 |
| mat3_mul_vec3 | 545 | 145 | 68520 |
| length2 | 174 | 47 | 21660 |
| normalize2 | 362 | 95 | 45620 |
| cross2 | 122 | 31 | 16350 |
| dot2 | 122 | 32 | 15850 |

### b_sm64

Baselines: `b2` = 71 steps / 14920 gas, `b1` = 67 steps / 14520 gas, `v1` = 71 steps / 14920 gas, `v1v` = 73 steps / 15120 gas, `m3` = 116 steps / 19420 gas, `v2` = 79 steps / 15720 gas, `b2b` = 70 steps / 14820 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| neg | 2 | 0 | 200 |
| sqrt | 10 | 4 | 1280 |
| add_mixed_sign | 15 | 2 | 1640 |
| add_same_sign | 4 | 1 | 470 |
| div | 18 | 5 | 2250 |
| mul | 18 | 5 | 2250 |
| sub | 25 | 1 | 4250 |
| lt_mixed_sign | 0 | 0 | 0 |
| lt_same_sign | 15 | 1 | 1580 |
| mat3_mul_vec3 | 344 | 55 | 47370 |
| length2 | 79 | 15 | 10840 |
| normalize2 | 114 | 25 | 16030 |
| cross2 | 67 | 11 | 9470 |
| dot2 | 69 | 12 | 9270 |

### c_sqrt

Baselines: `s1` = 65 steps / 14320 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| inv_sqrt_core_then_div | 26 | 9 | 3330 |
| inv_sqrt_div_then_core | 21 | 8 | 2760 |
| sqrt_core_u128_widemul | 10 | 4 | 1280 |
| sqrt_core_u64_lowprec | 14 | 5 | 1750 |
| sqrt_cubit_style_checked_mul | 35 | 13 | 4410 |
| sqrt_newton8_loop | 356 | 87 | 43510 |
| sqrt_newton8_unrolled | 231 | 78 | 31160 |

### d_sweep

Baselines: 

| op | steps |  | l2_gas |
|---|---:|---:|
| sweep_bhaskara_sin (raw, no baseline) | 425585 |  | 55330730 |
| sweep_cubit_atan (raw, no baseline) | 511197 |  | 70586960 |
| sweep_cubit_atan_fast (raw, no baseline) | 464580 |  | 65263820 |
| sweep_cubit_cos (raw, no baseline) | 632290 |  | 80762680 |
| sweep_cubit_cos_fast (raw, no baseline) | 463890 |  | 60702670 |
| sweep_cubit_sin (raw, no baseline) | 624881 |  | 79704970 |
| sweep_cubit_sin_fast (raw, no baseline) | 456652 |  | 59642980 |
| sweep_poly_atan (raw, no baseline) | 451651 |  | 58224100 |
| sweep_poly_cos7 (raw, no baseline) | 443971 |  | 57339290 |
| sweep_poly_sin5 (raw, no baseline) | 437950 |  | 56627060 |
| sweep_poly_sin7 (raw, no baseline) | 441568 |  | 57059210 |
| sweep_poly_sin9 (raw, no baseline) | 445186 |  | 57491360 |

### d_trig

Baselines: `c2` = 71 steps / 14920 gas, `c1` = 67 steps / 14520 gas, `i2` = 70 steps / 14820 gas, `i1` = 65 steps / 14320 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| cubit_atan_fast_lut_0p7 | 319 | 18 | 57290 |
| cubit_atan_poly_0p7 | 436 | 68 | 83740 |
| cubit_cos_fast_lut_0p7 | 263 | 31 | 34870 |
| cubit_cos_taylor_0p7 | 1084 | 199 | 134820 |
| cubit_sin_fast_lut_0p7 | 206 | 29 | 30320 |
| cubit_sin_fast_lut_2p5 | 221 | 30 | 30320 |
| cubit_sin_taylor_0p7 | 1051 | 197 | 130270 |
| cubit_sin_taylor_2p5 | 1051 | 197 | 130270 |
| cubit_tan_taylor_0p7 | 2169 | 401 | 268940 |
| bhaskara_sin_0p7 | 70 | 15 | 9430 |
| poly_cos7_0p7 | 155 | 38 | 19240 |
| poly_sin5_0p7 | 126 | 30 | 16250 |
| poly_sin7_0p7 | 144 | 35 | 18400 |
| poly_sin7_2p5 | 148 | 36 | 18400 |
| poly_sin9_0p7 | 162 | 40 | 20550 |
| poly_atan2_deg11 | 186 | 44 | 23110 |

### e_orion_linalg

Baselines: `tmm16` = 391 steps / 47580 gas, `smi` = 150 steps / 22940 gas, `tdot16` = 203 steps / 28660 gas, `svi` = 112 steps / 19020 gas, `tmv16` = 304 steps / 38880 gas, `smc` = 161 steps / 24040 gas, `tdot32` = 203 steps / 28660 gas, `tmm32` = 391 steps / 47580 gas, `tmv32` = 304 steps / 38880 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| struct_mat3_mul_cubit_f64 | 937 | 161 | 134250 |
| struct_mat3_mul_i64b | 574 | 171 | 70100 |
| struct_mat3_mul_i64b_fused | 214 | 45 | 25480 |
| struct_mat3_vec3_i64b | 202 | 57 | 24920 |
| struct_mat3_vec3_i64b_fused | 82 | 15 | 10180 |
| orion_tensor_dot_3_fp16x16 | 294 | 24 | 35080 |
| orion_tensor_dot_3_fp32x32 | 303 | 27 | 36490 |
| orion_tensor_matmul_3x3_fp16x16 | 2957 | 313 | 344300 |
| orion_tensor_matmul_3x3_fp32x32 | 3038 | 340 | 356990 |
| orion_tensor_matvec_3x3_3_fp16x16 | 1262 | 117 | 142660 |
| orion_tensor_matvec_3x3_3_fp32x32 | 1289 | 126 | 146890 |

### f_heuristic

Baselines: `h2` = 70 steps / 14820 gas, `r1` = 66 steps / 14420 gas, `k2` = 70 steps / 14820 gas, `t1` = 67 steps / 14520 gas, `p1` = 67 steps / 14520 gas, `q1` = 65 steps / 14320 gas

| op | steps | range_check | bitwise | l2_gas |
|---|---:|---:|---:|---:|
| pack_bitwise_or | 9 | 1 | 1 | 1453 |
| pack_math_bounded_int | 1 | 0 | 0 | 100 |
| pack_math_checked | 8 | 2 | 0 | 940 |
| pack_math_felt | 6 | 2 | 0 | 740 |
| shl13_loop_doubling | 211 | 27 | 0 | 22990 |
| shl13_math_core_pow | 127 | 23 | 0 | 14300 |
| shl13_math_table_mul | 16 | 2 | 0 | 1740 |
| split_bitwise_and | 19 | 3 | 2 | 3176 |
| split_loop_32_halvings | 1127 | 223 | 0 | 128310 |
| split_math_divrem | 8 | 3 | 0 | 1010 |
| low32_bitwise_and | 7 | 0 | 1 | 1183 |
| low32_math_divrem | 8 | 3 | 0 | 1010 |
| parity_bitwise_and | 11 | 0 | 1 | 1583 |
| parity_math_divrem | 12 | 3 | 0 | 1410 |
| split_math_bounded_int | 8 | 3 | 0 | 1010 |

### g_pitfalls

Baselines: `f1` = 65 steps / 14320 gas, `b2` = 70 steps / 14820 gas

| op | steps | range_check | l2_gas |
|---|---:|---:|---:|
| dot6_fused_single_rescale | 26 | 5 | 2950 |
| dot6_naive_6mul_5add | 134 | 40 | 16320 |
| mul_blackboxed_inputs | 16 | 5 | 1950 |
| mul_literal_inputs_no_blackbox | 0 | 0 | 0 |
| mul_x4_chain_default_inlining | 72 | 20 | 8720 |
| mul_x4_chain_inline_always | 72 | 20 | 8720 |
| mul_x4_chain_inline_never | 108 | 20 | 15120 |
| felt_storage_boundary_check | 6 | 2 | 740 |
