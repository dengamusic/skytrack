#include "skytrack/sensors/cooperative.hpp"

#include <stdexcept>

namespace skytrack {

CooperativeReceiver::CooperativeReceiver(SensorCommon common, CooperativeParams params)
    : Sensor(std::move(common)), params_(std::move(params)) {
  if (params_.protocol.empty()) throw std::invalid_argument("sensor '" + id() + "': protocol must be set");
}

Scan CooperativeReceiver::observe(double t, const World& world, Rng& rng) const {
  // The receiver collects broadcasts over the last period and sends them as one batch at t.
  Scan scan;
  scan.t_true_s = t;
  scan.t_ready_s = t;
  const double scale = common_.errors.noise_scale;

  for (const Target& target : world.targets) {
    if (!target.cooperative || target.cooperative->protocol != params_.protocol) continue;
    const double t_broadcast = t - rng.uniform(0.0, period_s());
    if (!target.trajectory.active(t_broadcast)) continue;
    const KinematicState state = target.trajectory.state(t_broadcast);
    if ((state.position - true_position()).norm() > params_.max_range_m) continue;
    if (!rng.bernoulli(params_.reception_prob)) continue;

    const geo::Geodetic g = world.frame.enu_to_geodetic(state.position + target.navigation_error(t_broadcast));
    CooperativeReport d;
    d.protocol = params_.protocol;
    d.identity = target.cooperative->identity;
    d.lat_deg = geo::rad2deg(g.lat_rad);
    d.lon_deg = geo::rad2deg(g.lon_rad);
    d.alt_m = g.alt_m;
    const double vel_std = scale * params_.velocity_std_mps;
    d.vel_east_mps = state.velocity.x() + vel_std * rng.normal();
    d.vel_north_mps = state.velocity.y() + vel_std * rng.normal();
    d.vel_up_mps = state.velocity.z() + vel_std * rng.normal();
    d.accuracy_h_m = target.cooperative->horizontal_std_m;
    d.accuracy_v_m = target.cooperative->vertical_std_m;
    scan.detections.push_back(make_detection(t_broadcast, target.id, d));
  }
  return scan;
}

}  // namespace skytrack
