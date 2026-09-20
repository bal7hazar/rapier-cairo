//! Test-only fixtures, shared by lifecycle/set tests and solver-cost candidates.
use fixed::{FixedTrait, HALF, ONE, ZERO};
use glam::Vec2;
use rapier_core::data::handle::Handle;
use rapier_core::integration_parameters::{IntegrationParameters, IntegrationParametersTrait};
use rapier_geometry2d::contact::{ContactManifold, SolverContact, SolverFlags};
use super::super::body::SolverBody;
use super::{ContactConstraint, ContactConstraintTrait};

pub fn fixture(count: u8) -> (ContactManifold, Array<SolverBody>, IntegrationParameters) {
    let h = Handle { index: 7, generation: 3 };
    let b = SolverBody {
        handle: h,
        im: Vec2 { x: ONE, y: ONE },
        ii: ONE,
        linvel: Vec2 { x: HALF, y: -ONE },
        ..Default::default(),
    };
    let mut m: ContactManifold = Default::default();
    m.num_points = count;
    m.data.num_solver_contacts = count;
    m.data.normal = Vec2 { x: ZERO, y: ONE };
    m.data.rigid_body2 = Some(h);
    m.data.friction = HALF;
    m.data.solver_flags = SolverFlags { bits: 1 };
    let sc = SolverContact {
        anchor1: Vec2 { x: -HALF, y: -HALF },
        anchor2: Vec2 { x: -HALF, y: -HALF },
        dist: FixedTrait::from_raw(-4294967),
        ..Default::default(),
    };
    m
        .data
        .solver_contacts =
            [
                sc,
                SolverContact {
                    anchor1: Vec2 { x: HALF, y: -HALF },
                    anchor2: Vec2 { x: HALF, y: -HALF },
                    contact_id: 1,
                    ..sc,
                },
            ];
    (m, array![b], Default::default())
}

pub fn prepared(count: u8) -> (ContactConstraint, Array<SolverBody>) {
    let (m, bs, p) = fixture(count);
    let mut c = ContactConstraintTrait::generate(m, bs.span(), p, p.substep_dt());
    c.update(p, bs.span(), m);
    (c, bs)
}
