//! Family `integration_parameters`: defaults of every field plus the quantities the solver
//! derives from them. Every derived value comes from the real upstream function.

use crate::q::{jf, jq, Q};
use rapier2d_f64::dynamics::{GenericJoint, IntegrationParameters, SpringCoefficients};
use serde_json::{json, Value};

fn spring_defaults(s: &SpringCoefficients<f64>) -> Value {
    json!({
        "natural_frequency": jf(s.natural_frequency),
        "damping_ratio": jf(s.damping_ratio),
        "angular_frequency": jf(s.angular_frequency()),
    })
}

fn spring_derived(s: &SpringCoefficients<f64>, dt: f64) -> Value {
    json!({
        "erp_inv_dt": jf(s.erp_inv_dt(dt)),
        "erp": jf(s.erp(dt)),
        "cfm_coeff": jf(s.cfm_coeff(dt)),
        "cfm_factor": jf(s.cfm_factor(dt)),
    })
}

/// Quantities derived for a step length `dt`. `dt_value` is how `dt` itself is reported.
fn derived(id: &str, dt: f64, dt_value: Value, note: &str) -> Value {
    let params = IntegrationParameters {
        dt,
        ..Default::default()
    };
    let joint = GenericJoint::default().softness;

    // Same operation as `staged_island_solver/init.rs`: `params.dt /= num_solver_iterations as Real`.
    let mut substep = params;
    substep.dt /= params.num_solver_iterations as f64;

    json!({
        "id": id,
        "note": note,
        "dt": dt_value,
        "num_solver_iterations": params.num_solver_iterations,
        "inv_dt": jf(params.inv_dt()),
        "substep_dt": jf(substep.dt),
        "substep_inv_dt": jf(substep.inv_dt()),
        "allowed_linear_error": jf(params.allowed_linear_error()),
        "max_corrective_velocity": jf(params.max_corrective_velocity()),
        "prediction_distance": jf(params.prediction_distance()),
        "max_linear_velocity": jf(params.max_linear_velocity()),
        "contact_recycle_distance": jf(params.contact_recycle_distance()),
        "contact": spring_derived(&substep.contact_softness, substep.dt),
        "static_contact": spring_derived(&substep.static_contact_softness, substep.dt),
        "joint": spring_derived(&joint, substep.dt),
    })
}

pub fn generate() -> Value {
    let p = IntegrationParameters::default();
    let joint = GenericJoint::default().softness;
    let dt_q = Q::snap(1.0 / 60.0);

    json!({
        "family": "integration_parameters",
        "defaults": {
            "dt": jf(p.dt),
            "min_ccd_dt": jf(p.min_ccd_dt),
            "contact_softness": spring_defaults(&p.contact_softness),
            "static_contact_softness": spring_defaults(&p.static_contact_softness),
            "joint_softness": spring_defaults(&joint),
            "warmstart_coefficient": jf(p.warmstart_coefficient),
            "length_unit": jf(p.length_unit),
            "normalized_allowed_linear_error": jf(p.normalized_allowed_linear_error),
            "normalized_max_corrective_velocity": jf(p.normalized_max_corrective_velocity),
            "normalized_prediction_distance": jf(p.normalized_prediction_distance),
            "normalized_max_linear_velocity": jf(p.normalized_max_linear_velocity),
            "normalized_contact_recycle_distance": jf(p.normalized_contact_recycle_distance),
            "num_solver_iterations": p.num_solver_iterations,
            "num_internal_pgs_iterations": p.num_internal_pgs_iterations,
            "num_internal_stabilization_iterations": p.num_internal_stabilization_iterations,
            "max_ccd_substeps": p.max_ccd_substeps,
            "contact_clustering": p.contact_clustering,
            "contact_recycling": p.contact_recycling,
            "friction_in_bias_pass": p.friction_in_bias_pass,
            "warmstart_joints": p.warmstart_joints,
        },
        "cases": [
            derived(
                "dt_f64",
                1.0 / 60.0,
                jf(1.0 / 60.0),
                "dt = 1.0 / 60.0 as an f64 (the upstream default, not a Q32.32 value)",
            ),
            derived(
                "dt_q32",
                dt_q.f(),
                jq(dt_q),
                "dt = round(2^32 / 60) / 2^32, the step length the scenes use",
            ),
        ],
    })
}
