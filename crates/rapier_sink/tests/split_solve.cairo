//! PROTOTYPE (never merged): the layouts with the solve in `SolverClass`
//! (`rapier_sink::split_solve`), against the in-process step on pile10.

use rapier2d::prelude::BasicStepConfig;
use rapier_sink::family::{BALL_CLASS_SLOT, POLYGON_CLASS_SLOT};
use rapier_sink::split::CONTACT_CLASS_SLOT;
use rapier_sink::split_solve::{
    SOLVER_CLASS_SLOT, Split3StepConfig, Split4FullSolveStepConfig, Split4StepConfig,
    SplitSolveStepConfig,
};
use rapier_testing::opaque;
use crate::pile10::{digest, run};
use crate::split::install;

fn split_solve(ticks: u32) {
    install("SolverClass", SOLVER_CLASS_SLOT);
    let (mut pile, _) = run::<SplitSolveStepConfig>(ticks);
    opaque(digest(ref pile));
}

fn split3(ticks: u32) {
    install("ContactClass", CONTACT_CLASS_SLOT);
    install("SolverClass", SOLVER_CLASS_SLOT);
    let (mut pile, _) = run::<Split3StepConfig>(ticks);
    opaque(digest(ref pile));
}

fn split4(ticks: u32) {
    install("ContactBallClass", BALL_CLASS_SLOT);
    install("ContactPolygonClass", POLYGON_CLASS_SLOT);
    install("SolverClass", SOLVER_CLASS_SLOT);
    let (mut pile, _) = run::<Split4StepConfig>(ticks);
    opaque(digest(ref pile));
}

fn split4_full(ticks: u32) {
    install("ContactBallClass", BALL_CLASS_SLOT);
    install("ContactPolygonClass", POLYGON_CLASS_SLOT);
    install("SolverClass", SOLVER_CLASS_SLOT);
    let (mut pile, _) = run::<Split4FullSolveStepConfig>(ticks);
    opaque(digest(ref pile));
}

#[test]
fn test_split4_bit_identical() {
    let ticks = 151;
    let (mut pile, events) = run::<BasicStepConfig>(ticks);
    let expected = digest(ref pile);
    install("ContactBallClass", BALL_CLASS_SLOT);
    install("ContactPolygonClass", POLYGON_CLASS_SLOT);
    install("SolverClass", SOLVER_CLASS_SLOT);
    let (mut pile, split_events) = run::<Split4StepConfig>(ticks);
    assert_eq!(split_events, events);
    assert_eq!(digest(ref pile), expected);
}

#[test]
fn test_split3_bit_identical() {
    let ticks = 151;
    let (mut pile, events) = run::<BasicStepConfig>(ticks);
    let expected = digest(ref pile);
    install("ContactClass", CONTACT_CLASS_SLOT);
    install("SolverClass", SOLVER_CLASS_SLOT);
    let (mut pile, split_events) = run::<Split3StepConfig>(ticks);
    assert_eq!(split_events, events);
    assert_eq!(digest(ref pile), expected);
}

#[test]
fn steps_split_solve_0() {
    split_solve(0);
}

#[test]
fn steps_split_solve_10() {
    split_solve(10);
}

#[test]
fn steps_split_solve_20() {
    split_solve(20);
}

#[test]
fn steps_split_solve_30() {
    split_solve(30);
}

#[test]
fn steps_split_solve_40() {
    split_solve(40);
}

#[test]
fn steps_split_solve_42() {
    split_solve(42);
}

#[test]
fn steps_split_solve_43() {
    split_solve(43);
}

#[test]
fn steps_split_solve_44() {
    split_solve(44);
}

#[test]
fn steps_split_solve_50() {
    split_solve(50);
}

#[test]
fn steps_split_solve_60() {
    split_solve(60);
}

#[test]
fn steps_split_solve_70() {
    split_solve(70);
}

#[test]
fn steps_split_solve_80() {
    split_solve(80);
}

#[test]
fn steps_split_solve_90() {
    split_solve(90);
}

#[test]
fn steps_split_solve_100() {
    split_solve(100);
}

#[test]
fn steps_split_solve_110() {
    split_solve(110);
}

#[test]
fn steps_split_solve_120() {
    split_solve(120);
}

#[test]
fn steps_split_solve_130() {
    split_solve(130);
}

#[test]
fn steps_split_solve_140() {
    split_solve(140);
}

#[test]
fn steps_split_solve_150() {
    split_solve(150);
}

#[test]
fn steps_split_solve_151() {
    split_solve(151);
}

#[test]
fn steps_split3_0() {
    split3(0);
}

#[test]
fn steps_split3_10() {
    split3(10);
}

#[test]
fn steps_split3_20() {
    split3(20);
}

#[test]
fn steps_split3_30() {
    split3(30);
}

#[test]
fn steps_split3_40() {
    split3(40);
}

#[test]
fn steps_split3_42() {
    split3(42);
}

#[test]
fn steps_split3_43() {
    split3(43);
}

#[test]
fn steps_split3_44() {
    split3(44);
}

#[test]
fn steps_split3_50() {
    split3(50);
}

#[test]
fn steps_split3_60() {
    split3(60);
}

#[test]
fn steps_split3_70() {
    split3(70);
}

#[test]
fn steps_split3_80() {
    split3(80);
}

#[test]
fn steps_split3_90() {
    split3(90);
}

#[test]
fn steps_split3_100() {
    split3(100);
}

#[test]
fn steps_split3_110() {
    split3(110);
}

#[test]
fn steps_split3_120() {
    split3(120);
}

#[test]
fn steps_split3_130() {
    split3(130);
}

#[test]
fn steps_split3_140() {
    split3(140);
}

#[test]
fn steps_split3_150() {
    split3(150);
}

#[test]
fn steps_split3_151() {
    split3(151);
}

#[test]
fn steps_split4_0() {
    split4(0);
}

#[test]
fn steps_split4_10() {
    split4(10);
}

#[test]
fn steps_split4_20() {
    split4(20);
}

#[test]
fn steps_split4_30() {
    split4(30);
}

#[test]
fn steps_split4_40() {
    split4(40);
}

#[test]
fn steps_split4_42() {
    split4(42);
}

#[test]
fn steps_split4_43() {
    split4(43);
}

#[test]
fn steps_split4_44() {
    split4(44);
}

#[test]
fn steps_split4_50() {
    split4(50);
}

#[test]
fn steps_split4_60() {
    split4(60);
}

#[test]
fn steps_split4_70() {
    split4(70);
}

#[test]
fn steps_split4_80() {
    split4(80);
}

#[test]
fn steps_split4_90() {
    split4(90);
}

#[test]
fn steps_split4_100() {
    split4(100);
}

#[test]
fn steps_split4_110() {
    split4(110);
}

#[test]
fn steps_split4_120() {
    split4(120);
}

#[test]
fn steps_split4_130() {
    split4(130);
}

#[test]
fn steps_split4_140() {
    split4(140);
}

#[test]
fn steps_split4_150() {
    split4(150);
}

#[test]
fn steps_split4_151() {
    split4(151);
}

#[test]
fn steps_split4full_0() {
    split4_full(0);
}

#[test]
fn steps_split4full_43() {
    split4_full(43);
}

#[test]
fn steps_split4full_44() {
    split4_full(44);
}

#[test]
fn steps_split4full_60() {
    split4_full(60);
}

#[test]
fn steps_split4full_100() {
    split4_full(100);
}

#[test]
fn steps_split4full_151() {
    split4_full(151);
}
