//! Joint stage dispatch outside the loop, preserving the original array body adapter.
use super::*;
/// Joint sweep in caller order, selecting warmstart once before the loop.
/// Array API rounding/overflow and endpoint restoration are unchanged.
pub(crate) fn joints<B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref rows: Array<JointConstraint>, ref bodies: B, biased: bool, warmstart: bool,
) {
    if warmstart {
        run::<Warm, B>(ref rows, ref bodies, biased);
    } else {
        run::<Cold, B>(ref rows, ref bodies, biased);
    }
}
trait Warmstart {
    fn apply(c: JointConstraint, ref pair: Array<SolverBody>);
}
impl Warm of Warmstart {
    #[inline(always)]
    fn apply(c: JointConstraint, ref pair: Array<SolverBody>) {
        c.warmstart(ref pair);
    }
}
impl Cold of Warmstart {
    #[inline(always)]
    fn apply(c: JointConstraint, ref pair: Array<SolverBody>) {}
}
pub(crate) fn run<impl W: Warmstart, B, +DenseBodiesTrait<B>, +Destruct<B>>(
    ref rows: Array<JointConstraint>, ref bodies: B, biased: bool,
) {
    let mut out = array![];
    while let Some(mut c) = rows.pop_front() {
        if c.num_rows != 0 {
            let i = c.solver_vel1;
            let j = c.solver_vel2;
            let mut pair = array![bodies.get(i), bodies.get(j)];
            c.solver_vel1 = 0;
            c.solver_vel2 = 1;
            W::apply(c, ref pair);
            c.solve(ref pair, biased);
            bodies.set_pair(i, *pair.at(0), j, *pair.at(1));
            c.solver_vel1 = i;
            c.solver_vel2 = j;
        }
        out.append(c);
    }
    rows = out;
}
