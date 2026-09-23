//! One manifold, one substep (update/warmstart, biased solve, refresh/relax), then restitution.
//! Identical setup/consumption in every candidate, measured in both tracked resources.
use rapier_core::integration_parameters::IntegrationParametersTrait;
use rapier_testing::opaque;
use super::*;
use super::super::super::body_store::DenseBodies;
use super::super::super::contact::ContactConstraintsSetTrait;

trait Sweep {
    fn run(
        ref cs: ContactConstraintsSet,
        ref bodies: DenseBodies,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        stage: u8,
    );
}
impl Original of Sweep {
    #[inline(always)]
    fn run(
        ref cs: ContactConstraintsSet,
        ref bodies: DenseBodies,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        stage: u8,
    ) {
        alternatives::contacts_original(ref cs, ref bodies, ms, p, stage);
    }
}
impl Separate of Sweep {
    #[inline(always)]
    fn run(
        ref cs: ContactConstraintsSet,
        ref bodies: DenseBodies,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        stage: u8,
    ) {
        contacts(ref cs, ref bodies, ms, p, stage);
    }
}
impl Metered of Sweep {
    #[inline(always)]
    fn run(
        ref cs: ContactConstraintsSet,
        ref bodies: DenseBodies,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        stage: u8,
    ) {
        alternatives::contacts_metered(ref cs, ref bodies, ms, p, stage);
    }
}
impl Direct of Sweep {
    #[inline(always)]
    fn run(
        ref cs: ContactConstraintsSet,
        ref bodies: DenseBodies,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        stage: u8,
    ) {
        direct::contacts(ref cs, ref bodies, ms, p, stage);
    }
}
impl Specialized of Sweep {
    #[inline(always)]
    fn run(
        ref cs: ContactConstraintsSet,
        ref bodies: DenseBodies,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        stage: u8,
    ) {
        specialized::contacts(ref cs, ref bodies, ms, p, stage);
    }
}
impl MeteredPair of Sweep {
    #[inline(always)]
    fn run(
        ref cs: ContactConstraintsSet,
        ref bodies: DenseBodies,
        ms: Span<ContactManifold>,
        p: IntegrationParameters,
        stage: u8,
    ) {
        metered_pair::contacts(ref cs, ref bodies, ms, p, stage);
    }
}
fn input(
    count: u8,
) -> (ContactConstraintsSet, DenseBodies, ContactManifold, IntegrationParameters) {
    let (bs, _, ms) = super::super::fixtures::stack(2);
    let p: IntegrationParameters = opaque(Default::default());
    let m = opaque(*ms.at(1));
    let mut cs = ContactConstraintsSetTrait::generate([m].span(), bs.span(), p, p.substep_dt());
    let mut c = cs.constraints.pop_front().unwrap();
    c.num_elements = opaque(count);
    cs.constraints.append(opaque(c));
    let bodies: DenseBodies = DenseBodiesTrait::new(opaque(bs.span()));
    (cs, bodies, m, p)
}
fn consume(cs: ContactConstraintsSet, ref bodies: DenseBodies) {
    let _ = opaque((*cs.constraints.at(0), bodies.get(0), bodies.get(1)));
}
fn probe<impl S: Sweep>(count: u8, solve: bool) {
    let (mut cs, mut bodies, m, p) = input(count);
    if solve {
        S::run(ref cs, ref bodies, [m].span(), p, 0);
        S::run(ref cs, ref bodies, [m].span(), p, 1);
        S::run(ref cs, ref bodies, [m].span(), p, 2);
        S::run(ref cs, ref bodies, [m].span(), p, 4);
    }
    consume(cs, ref bodies);
}
#[test]
fn gas_baseline() {
    probe::<Original>(2, false);
}
#[test]
fn gas_original_one() {
    probe::<Original>(1, true);
}
#[test]
fn gas_original_two() {
    probe::<Original>(2, true);
}
#[test]
fn gas_separate_one() {
    probe::<Separate>(1, true);
}
#[test]
fn gas_separate_two() {
    probe::<Separate>(2, true);
}
#[test]
fn gas_metered_one() {
    probe::<Metered>(1, true);
}
#[test]
fn gas_metered_two() {
    probe::<Metered>(2, true);
}

#[test]
fn gas_direct_one() {
    probe::<Direct>(1, true);
}
#[test]
fn gas_direct_two() {
    probe::<Direct>(2, true);
}
fn dict_probe(count: u8) {
    let (mut cs, mut bodies, m, p) = input(count);
    let mut rows = direct::alternatives::new_contacts(cs.constraints.span());
    direct::alternatives::contacts(ref rows, ref bodies, [m].span(), p, 0);
    direct::alternatives::contacts(ref rows, ref bodies, [m].span(), p, 1);
    direct::alternatives::contacts(ref rows, ref bodies, [m].span(), p, 2);
    direct::alternatives::contacts(ref rows, ref bodies, [m].span(), p, 4);
    cs.constraints = direct::alternatives::finish_contacts(ref rows);
    consume(cs, ref bodies);
}
#[test]
fn gas_dict_one() {
    dict_probe(1);
}
#[test]
fn gas_dict_two() {
    dict_probe(2);
}

#[test]
fn gas_metered_pair_one() {
    probe::<MeteredPair>(1, true);
}
#[test]
fn gas_metered_pair_two() {
    probe::<MeteredPair>(2, true);
}

#[test]
fn gas_specialized_one() {
    probe::<Specialized>(1, true);
}
#[test]
fn gas_specialized_two() {
    probe::<Specialized>(2, true);
}

fn cached_probe(count: u8) {
    let (cs, mut bodies, m, p) = input(count);
    let mut cs = cs;
    let directions = super::super::super::contact::cached::prepare(cs.constraints.span());
    cached::contacts(ref cs, ref bodies, [m].span(), p, 0, directions.span());
    cached::contacts(ref cs, ref bodies, [m].span(), p, 1, directions.span());
    cached::contacts(ref cs, ref bodies, [m].span(), p, 2, directions.span());
    cached::contacts(ref cs, ref bodies, [m].span(), p, 4, directions.span());
    consume(cs, ref bodies);
}
#[test]
fn gas_cached_one() {
    cached_probe(1);
}
#[test]
fn gas_cached_two() {
    cached_probe(2);
}

fn sparse_probe(count: u8) {
    let (cs, mut bodies, m, p) = input(count);
    let mut cs = cs;
    let directions = super::super::super::contact::cached::prepare(cs.constraints.span());
    sparse::contacts(ref cs, ref bodies, [m].span(), p, 0, directions.span());
    sparse::contacts(ref cs, ref bodies, [m].span(), p, 1, directions.span());
    sparse::contacts(ref cs, ref bodies, [m].span(), p, 2, directions.span());
    sparse::contacts(ref cs, ref bodies, [m].span(), p, 4, directions.span());
    consume(cs, ref bodies);
}
#[test]
fn gas_sparse_one() {
    sparse_probe(1);
}
#[test]
fn gas_sparse_two() {
    sparse_probe(2);
}

fn zero_probe(count: u8) {
    let (cs, mut bodies, m, p) = input(count);
    let mut cs = cs;
    let directions = super::super::super::contact::cached::prepare(cs.constraints.span());
    zero::contacts(ref cs, ref bodies, [m].span(), p, 0, directions.span());
    zero::contacts(ref cs, ref bodies, [m].span(), p, 1, directions.span());
    zero::contacts(ref cs, ref bodies, [m].span(), p, 2, directions.span());
    zero::contacts(ref cs, ref bodies, [m].span(), p, 4, directions.span());
    consume(cs, ref bodies);
}
#[test]
fn gas_zero_one() {
    zero_probe(1);
}
#[test]
fn gas_zero_two() {
    zero_probe(2);
}

mod active;
