use core::array::ArrayTrait;
use core::integer::{u64_safe_divmod, u64_as_non_zero, u256_from_felt252};
use core::option::OptionTrait;
use core::traits::{Into, TryInto};

use cubit::f64::types::fixed::{Fixed, FixedTrait, ONE};

fn derive(seed: felt252, entropy: felt252) -> felt252 {
    return core::hash::LegacyHash::hash(seed, entropy);
}

// Returns a psuedo-random value between two values based on a seed. The returned
// value is inclusive of the low param and exclusive of the high param (low <= res < high)
fn fixed_between(seed: felt252, low: Fixed, high: Fixed) -> Fixed {
    assert(high > low, 'high !> low');
    let seed_low = u256_from_felt252(seed).low % ONE.into();
    return FixedTrait::new(seed_low.try_into().unwrap(), false) * (high - low) + low;
}

fn u64_between(seed: felt252, low: u64, high: u64) -> u64 {
    let fixed = fixed_between(
        seed, FixedTrait::new_unscaled(low, false), FixedTrait::new_unscaled(high, false)
    );
    return fixed.mag / ONE;
}

fn fixed_normal_between(seed: felt252, low: Fixed, high: Fixed) -> Fixed {
    let acc = _fixed_normal_between_loop(seed, low, high, FixedTrait::ZERO(), 5);
    return acc / FixedTrait::new_unscaled(5, false);
}

fn u64_normal_between(seed: felt252, low: u64, high: u64) -> u64 {
    let fixed_low = FixedTrait::new_unscaled(low, false);
    let fixed_high = FixedTrait::new_unscaled(high - 1, false);
    let fixed = fixed_normal_between(seed, fixed_low, fixed_high);
    return fixed.round().mag / ONE;
}

fn _fixed_normal_between_loop(
    seed: felt252, low: Fixed, high: Fixed, acc: Fixed, iter: felt252
) -> Fixed {
    if (iter == 0) {
        return acc;
    }
    let iter_seed = derive(seed, iter);
    let sample = fixed_between(iter_seed, low, high);
    return _fixed_normal_between_loop(seed, low, high, acc + sample, iter - 1);
}

