//! Multi-waypoint mission planning, replanning, state queries, and eclipse aggregation.

mod eclipse;
mod planning;
mod query;

pub use planning::{plan_waypoint_mission, replan_from_waypoint};
pub use query::{get_mission_state_at_time, resample_leg_trajectory};
