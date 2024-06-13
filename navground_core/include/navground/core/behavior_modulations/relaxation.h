/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_MODULATIONS_RELAXATION_H_
#define NAVGROUND_CORE_BEHAVIOR_MODULATIONS_RELAXATION_H_

#include "navground/core/behavior_modulation.h"
#include "navground/core/types.h"
#include "navground_core_export.h"

namespace navground::core {

ng_float_t relax(ng_float_t x0, ng_float_t x1, ng_float_t tau, ng_float_t dt);

std::vector<ng_float_t> relax(const std::vector<ng_float_t>& v0,
                              const std::vector<ng_float_t>& v1, ng_float_t tau,
                              ng_float_t dt);

Twist2 relax(const Twist2& v0, const Twist2& v1, ng_float_t tau, ng_float_t dt);

Twist2 relax(Behavior& behavior, const Twist2& current_value,
             const Twist2& value, ng_float_t tau, ng_float_t dt);

/**
 * @brief      A modulation to relax the command over time.
 *             used by \ref HLBehavior.
 *
 * *Registered properties*: tau
 */
class NAVGROUND_CORE_EXPORT RelaxationModulation : public BehaviorModulation {
 public:
  /**
   * Default \f$\eta\f$
   */
  static constexpr ng_float_t default_tau = 0.125;

  /**
   * @brief      Construct a new instance
   *
   * @param[in]  tau      The relaxation time
   */
  explicit RelaxationModulation(ng_float_t tau = default_tau)
      : BehaviorModulation(), _tau(tau), _actuated_twist() {}

  /**
   * @private
   */
  std::string get_type() const override { return type; }
  /**
   * @private
   */
  void pre(Behavior& behavior, ng_float_t time_step) override;
  /**
   * @private
   */
  Twist2 post(Behavior& behavior, ng_float_t time_step,
              const Twist2& cmd) override;

  /**
   * @brief      Gets the relaxation time \f$\tau\f$. Higher values lead to
   * lower accelerations.
   *
   * @return     \f$\eta\f$
   */
  ng_float_t get_tau() const { return _tau; }
  /**
   * @brief      Sets the relaxation time \f$\tau\f$. Higher values lead to
   * lower accelerations.
   *
   * @param[in]  value  A positive value. If zero, relaxation is disabled.
   */
  void set_tau(ng_float_t value) { _tau = value; }

  /** @private
   */
  virtual const Properties& get_properties() const override {
    return properties;
  };

  /**
   * Properties: tau
   * @private
   */
  static const std::map<std::string, Property> properties;

 private:
  static const std::string type;
  float _tau;
  Twist2 _actuated_twist;
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_MODULATIONS_RELAXATION_H_
