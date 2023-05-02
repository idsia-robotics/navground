#ifndef NAVGROUND_CORE_SOCIAL_MARGIN_H
#define NAVGROUND_CORE_SOCIAL_MARGIN_H

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>

namespace navground::core {

/**
 * @brief      This class defines a modulated social margin that may be added to
 * the safety margin around \ref Neighbor for behaviors that uses the \ref
 * GeometricState. The social margin is assigned to the neighbor's \ref
 * Neighbor::id and it is modulated by the neighbor current distance.
 */
class SocialMargin {
 public:
  enum class ModulationType { zero, constant, linear, quadratic, logistic };

  /**
   * @brief      Abstract modulation: maps a pair (margin, distance) to a
   * margin.
   */
  class Modulation {
   public:
    virtual ~Modulation() = default;
    Modulation(){};
    virtual float operator()(float margin,
                             [[maybe_unused]] float distance) const {
      return margin;
    }
    virtual float operator()(float margin) const { return margin; }
  };

  /**
   * @brief      A modulation that always return 0.
   */
  class ZeroModulation : public Modulation {
   public:
    using Modulation::Modulation;
    float operator()([[maybe_unused]] float margin,
                     [[maybe_unused]] float distance) const override {
      return 0.0f;
    }
    float operator()([[maybe_unused]] float margin) const override {
      return 0.0f;
    }
  };

  /**
   * @brief      A modulation that leaves the margin unchanged
   */
  class ConstantModulation : public Modulation {
   public:
    using Modulation::Modulation;
  };

  // returns margin above upper_distance, and a linear interpolation below

  /**
   * @brief      A modulation that linearly interpolate between 0 and the input
   * margin, returning the input margin above `upper_distance`
   */
  class LinearModulation : public Modulation {
   public:
    /**
     * @brief      Constructs a new instance.
     *
     * @param[in]  upper_distance  The upper distance
     */
    explicit LinearModulation(float upper_distance)
        : Modulation(), upper_distance(upper_distance) {}
    float operator()(float margin, float distance) const override {
      if (distance < 0) return 0.0;
      float upper = std::max(upper_distance, margin);
      if (distance > upper) return margin;
      return margin * distance / upper;
    }

    float get_upper_distance() const { return upper_distance; }

   private:
    float upper_distance;
  };

  /**
   * @brief      A modulation that quadratically interpolates between 0 and the
   * input margin, returning the input margin above `upper_distance`. The slope
   * is 0 at distance 0.
   */
  class QuadraticModulation : public Modulation {
   public:
    explicit QuadraticModulation(float upper_distance)
        : Modulation(), upper_distance(upper_distance) {}
    float operator()(float margin, float distance) const override {
      if (distance < 0) return 0.0;
      const float upper = std::max(upper_distance, 2 * margin);
      if (distance > upper) return margin;
      const float x = distance / upper;
      return -margin * x * x + 2 * margin * x;
    }

    float get_upper_distance() const { return upper_distance; }

   private:
    float upper_distance;
  };

  /**
   * @brief      A logistic modulation.
   */
  class LogisticModulation : public Modulation {
   public:
    using Modulation::Modulation;
    float operator()(float margin, float distance) const override {
      if (distance < 0) return 0.0;
      return 2 * distance + margin -
             margin * std::log2(1 + std::exp2(2 * distance / margin));
    }
  };

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  value  The default value of social margins
   */
  explicit SocialMargin(float value = 0.0f)
      : default_social_margin(std::max(0.0f, value)),
        social_margins(),
        modulation(std::make_shared<ConstantModulation>()) {}

  /**
   * @brief      Gets the modulation.
   *
   * @return     The modulation.
   */
  std::shared_ptr<Modulation> get_modulation() const { return modulation; }
  /**
   * @brief      Sets the modulation.
   *
   * @param[in]  value  The desired modulation
   */
  void set_modulation(const std::shared_ptr<Modulation>& value) {
    if (value) {
      modulation = value;
    }
  }

  /**
   * @brief      Get the value, ignoring type and distance
   *
   * @return     The social margin
   */
  float get() const { return (*modulation)(default_social_margin); }
  /**
   * @brief      Set the default value, ignoring type.
   *
   * @param[in]  value  The desired social margin
   */
  void set(float value) { default_social_margin = std::max(0.0f, value); }
  /**
   * @brief      Get the value for a specific type, ignoring distance
   *
   * @param[in]  type  The neighbor type
   *
   * @return     The social margin
   */
  float get(unsigned type) { return (*modulation)(get_value(type)); }
  /**
   * @brief      Set the value for the specified type.
   *
   * @param[in]  type   The neighbor type
   * @param[in]  value  The desired social margin
   */
  void set(unsigned type, float value) {
    social_margins[type] = std::max(0.0f, value);
  }
  /**
   * @brief      Get the value for a specific type and distance
   *
   * @param[in]  type      The neighbor type
   * @param[in]  distance  The neighbor distance
   *
   * @return     The modulated social margin
   */
  float get(unsigned type, float distance) {
    return (*modulation)(get_value(type), distance);
  }

  /**
   * @brief      Gets the stored [non-modulated] value for the specific type
   *
   * @param[in]  type  The neighbor type
   *
   * @return     The stored social margin
   */
  float get_value(unsigned type) {
    return social_margins[type].value_or(default_social_margin);
  }

  /**
   * @brief      Gets the default value
   *
   * @return     The default social margin.
   */
  float get_default_value() const { return default_social_margin; }

  /**
   * @brief      Gets the stored [non-modulated] values for the specific types
   *
   * @return     A map of type -> social margin.
   */
  const std::map<unsigned, std::optional<float>>& get_values() const {
    return social_margins;
  }

 private:
  float default_social_margin;
  std::map<unsigned, std::optional<float>> social_margins;
  std::shared_ptr<Modulation> modulation;
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_SOCIAL_MARGIN_H
