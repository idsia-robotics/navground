/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_odometry.h"

#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

void OdometryStateEstimation::update(Agent *agent, World *world,
                                     EnvironmentState *state) {
  auto &rg = world->get_random_generator();
  _twist = agent->twist.relative(agent->pose);
  _twist.velocity[0] += _twist.velocity[0] * _longitudinal_speed_error(rg);
  _twist.velocity[1] += _twist.velocity[1] * _transversal_speed_error(rg);
  _twist.angular_speed += _twist.angular_speed * _angular_speed_error(rg);
  ng_float_t dt = std::max<ng_float_t>(0, world->get_time() - _time);
  _time = world->get_time();
  _pose = _pose.integrate(_twist.absolute(_pose), dt);
  Behavior *b = agent->get_behavior();
  if (get_update_ego_state() && b) {
    b->set_pose(_pose);
    b->set_twist(_twist);
  }
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    if (get_update_sensing_state()) {
      auto buffer = get_or_init_buffer(*_state, "pose");
      if (buffer) {
        buffer->set_data(std::valarray<ng_float_t>{
            _pose.position[0], _pose.position[1], _pose.orientation});
      }
      buffer = get_or_init_buffer(*_state, "twist");
      if (buffer) {
        buffer->set_data(std::valarray<ng_float_t>{
            _twist.velocity[0], _twist.velocity[1], _twist.angular_speed});
      }
    }
  }
}

const std::map<std::string, core::Property>
    OdometryStateEstimation::properties =
        core::Properties{
            {"longitudinal_speed_bias",
             core::make_property<ng_float_t, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_longitudinal_speed_bias,
                 &OdometryStateEstimation::set_longitudinal_speed_bias, 0,
                 "Longitudinal speed bias")},
            {"longitudinal_speed_std_dev",
             core::make_property<ng_float_t, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_longitudinal_speed_std_dev,
                 &OdometryStateEstimation::set_longitudinal_speed_std_dev,
                 default_longitudinal_speed_std_dev,
                 "Longitudinal speed standard deviation")},
            {"transversal_speed_bias",
             core::make_property<ng_float_t, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_transversal_speed_bias,
                 &OdometryStateEstimation::set_transversal_speed_bias, 0,
                 "Transversal speed bias")},
            {"transversal_speed_std_dev",
             core::make_property<ng_float_t, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_transversal_speed_std_dev,
                 &OdometryStateEstimation::set_transversal_speed_std_dev,
                 default_longitudinal_speed_std_dev,
                 "Transversal speed standard deviation")},
            {"angular_speed_bias",
             core::make_property<ng_float_t, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_angular_speed_bias,
                 &OdometryStateEstimation::set_angular_speed_bias, 0,
                 "Angular speed bias")},
            {"angular_speed_std_dev",
             core::make_property<ng_float_t, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_angular_speed_std_dev,
                 &OdometryStateEstimation::set_angular_speed_std_dev,
                 default_angular_speed_std_dev,
                 "Angular speed standard deviation")},
            {"update_ego_state",
             core::make_property<bool, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_update_ego_state,
                 &OdometryStateEstimation::set_update_ego_state, false,
                 "Whether to update the behavior ego state")},
            {"update_sensing_state",
             core::make_property<bool, OdometryStateEstimation>(
                 &OdometryStateEstimation::get_update_sensing_state,
                 &OdometryStateEstimation::set_update_sensing_state, true,
                 "Whether to update the behavior sensing state")},
        } +
        Sensor::properties;

const std::string OdometryStateEstimation::type =
    register_type<OdometryStateEstimation>("Odometry");

} // namespace navground::sim
