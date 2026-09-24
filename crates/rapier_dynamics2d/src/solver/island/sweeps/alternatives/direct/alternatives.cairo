//! Rejected/experimental dictionary constraint storage, dense indices only.
use core::dict::{Felt252Dict, Felt252DictTrait};
use core::nullable::{FromNullableResult, NullableTrait, match_nullable};
use super::{*, Biased, Refresh, Relax, Restitution, Update};

#[derive(Destruct)]
pub(crate) struct Contacts {
    values: Felt252Dict<Nullable<ContactConstraint>>,
    count: u32,
}
pub(crate) fn new_contacts(mut rows: Span<ContactConstraint>) -> Contacts {
    let mut values: Felt252Dict<Nullable<ContactConstraint>> = Default::default();
    let mut count = 0_u32;
    while let Some(c) = rows.pop_front() {
        values.insert(count.into(), NullableTrait::new(*c));
        count += 1;
    }
    Contacts { values, count }
}
fn get_contacts(ref rows: Contacts, id: u32) -> ContactConstraint {
    match match_nullable(rows.values.get(id.into())) {
        FromNullableResult::NotNull(c) => c.unbox(),
        FromNullableResult::Null => core::panic_with_felt252('missing constraint'),
    }
}
pub(crate) fn finish_contacts(ref rows: Contacts) -> Array<ContactConstraint> {
    let mut out = array![];
    let mut i = 0;
    while i != rows.count {
        out.append(get_contacts(ref rows, i));
        i += 1;
    }
    out
}

#[derive(Destruct)]
pub(crate) struct Joints {
    values: Felt252Dict<Nullable<JointConstraint>>,
    count: u32,
}
pub(crate) fn new_joints(mut rows: Span<JointConstraint>) -> Joints {
    let mut values: Felt252Dict<Nullable<JointConstraint>> = Default::default();
    let mut count = 0_u32;
    while let Some(c) = rows.pop_front() {
        values.insert(count.into(), NullableTrait::new(*c));
        count += 1;
    }
    Joints { values, count }
}
fn get_joints(ref rows: Joints, id: u32) -> JointConstraint {
    match match_nullable(rows.values.get(id.into())) {
        FromNullableResult::NotNull(c) => c.unbox(),
        FromNullableResult::Null => core::panic_with_felt252('missing constraint'),
    }
}
pub(crate) fn finish_joints(ref rows: Joints) -> Array<JointConstraint> {
    let mut out = array![];
    let mut i = 0;
    while i != rows.count {
        out.append(get_joints(ref rows, i));
        i += 1;
    }
    out
}

pub(crate) fn contacts<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref cs: Contacts, ref bodies: B, ms: Span<ContactManifold>, p: IntegrationParameters, stage: u8,
) {
    match stage {
        0 => contacts_stage::<Update, B>(ref cs, ref bodies, ms, p),
        1 => contacts_stage::<Biased, B>(ref cs, ref bodies, ms, p),
        2 => contacts_stage::<Refresh, B>(ref cs, ref bodies, ms, p),
        3 => contacts_stage::<Relax, B>(ref cs, ref bodies, ms, p),
        _ => contacts_stage::<Restitution, B>(ref cs, ref bodies, ms, p),
    }
}

fn contacts_stage<impl S: PairStage, B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref cs: Contacts, ref bodies: B, ms: Span<ContactManifold>, p: IntegrationParameters,
) {
    let mut id = 0_u32;
    while id != cs.count {
        let mut c = get_contacts(ref cs, id);
        if c.num_elements != 0 {
            let i = c.solver_vel1;
            let j = c.solver_vel2;
            let mut pair = BodyPair { first: bodies.get(i), second: bodies.get(j) };
            S::apply(ref c, ref pair, ms, p);
            bodies.set_pair(i, pair.first, j, pair.second);
        }
        cs.values.insert(id.into(), NullableTrait::new(c));
        id += 1;
    }
}

pub(crate) fn joints<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref rows: Joints, ref bodies: B, biased: bool, warmstart: bool,
) {
    let mut id = 0_u32;
    while id != rows.count {
        let mut c = get_joints(ref rows, id);
        if c.num_rows != 0 {
            let i = c.solver_vel1;
            let j = c.solver_vel2;
            let mut pair = BodyPair { first: bodies.get(i), second: bodies.get(j) };
            if warmstart {
                pair_joint::warmstart(c, ref pair);
            }
            pair_joint::solve(ref c, ref pair, biased);
            bodies.set_pair(i, pair.first, j, pair.second);
        }
        rows.values.insert(id.into(), NullableTrait::new(c));
        id += 1;
    }
}
