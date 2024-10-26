/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_odometry.h"

#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

void OdometryStateEstimation::update(Agent *agent, World *world,
                                     EnvironmentState *state) {
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    auto &rg = world->get_random_generator();
    auto twist = agent->twist.relative(agent->pose);
    twist.velocity[0] += _longitudinal_speed_error(rg);
    twist.velocity[1] += _transversal_speed_error(rg);
    twist.angular_speed += _angular_speed_error(rg);
    ng_float_t dt = std::max<ng_float_t>(0, world->get_time() - _time);
    _time = world->get_time();
    _pose.integrate(twist, dt);
    auto buffer = get_or_init_buffer(*_state, "pose");
    if (buffer) {
      buffer->set_data(std::valarray<ng_float_t>{
          _pose.position[0], _pose.position[1], _pose.orientation});
    }
    buffer = get_or_init_buffer(*_state, "twist");
    if (buffer) {
      buffer->set_data(std::valarray<ng_float_t>{
          twist.velocity[0], twist.velocity[1], twist.angular_speed});
    }
  }
}

const std::map<std::string, core::Property>
    OdometryStateEstimation::properties =
        core::Properties{
            {"longitudinal_speed_error",
             core::make_property<ng_float_t, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_longitudinal_speed_error,
                 &OdometryStateEstimation::set_longitudinal_speed_error,
                 default_longitudinal_speed_error,
                 "Longitudinal speed standard deviation")},
            {"transversal_speed_error",
             core::make_property<ng_float_t, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_transversal_speed_error,
                 &OdometryStateEstimation::set_transversal_speed_error,
                 default_longitudinal_speed_error,
                 "Transversal speed standard deviation")},
            {"angular_speed_error",
             core::make_property<ng_float_t, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_angular_speed_error,
                 &OdometryStateEstimation::set_angular_speed_error,
                 default_angular_speed_error,
                 "Angular speed standard deviation")},
        } +
        Sensor::properties;

const std::string OdometryStateEstimation::type =
    register_type<OdometryStateEstimation>("Odometry");

} // namespace navground::sim
