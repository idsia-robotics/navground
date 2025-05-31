/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SAMPLING_SAMPLER_H
#define NAVGROUND_SIM_SAMPLING_SAMPLER_H

#include <memory>
#include <random>
#include <type_traits>
#include <utility>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/core/types.h"
#include "navground/sim/export.h"

using navground::core::Vector2;

// std::default_random_engine

namespace navground::sim {

/**
 * @brief      What should a generator do at the end of a sequence
 */
enum class Wrap {
  /**
   * Start from scratch
   */
  loop,
  /**
   * Repeat the last entry
   */
  repeat,
  /**
   * Terminate
   */
  terminate
};

inline unsigned wrap_index(const Wrap &wrap, unsigned i, unsigned size) {
  if (wrap == Wrap::repeat) {
    i = std::min(i, size - 1);
  } else if (wrap == Wrap::loop) {
    i = i % size;
  }
  return i;
}

inline bool wrap_done(const Wrap &wrap, unsigned i, unsigned size) {
  return wrap == Wrap::terminate && i >= size;
}

inline Wrap wrap_from_string(const std::string &value) {
  if (value == "terminate") {
    return Wrap::terminate;
  }
  if (value == "repeat") {
    return Wrap::repeat;
  }
  return Wrap::loop;
}

inline std::string wrap_to_string(const Wrap &value) {
  switch (value) {
  case Wrap::terminate:
    return "terminate";
  case Wrap::repeat:
    return "repeat";
  default:
    return "loop";
  }
}

/**
 * @brief      Abstract Sampler base class.
 * that allows to sample values of type T using \ref sample.
 *
 * @tparam     T     The sampled type
 * @param[in]  once     Whether to repeat the first sample (until reset)
 */
template <typename T> struct Sampler {
  friend struct PropertySampler;

  using value_type = T;

  Sampler(bool once = false)
      : once(once), _index{0}, first_sample(std::nullopt) {}
  virtual ~Sampler() = default;
  /**
   * @brief      Sample values of type T.
   *
   * @throw std::runtime_error If the generator is exhausted (i.e., \ref done
   * returns true)
   *
   * @param[in]  rg A random generator
   *
   * @return     The sampled value.
   */
  T sample(RandomGenerator &rg) {
    if (done()) {
      throw std::runtime_error("Generator is exhausted");
    }
    T v = (once && first_sample) ? *first_sample : s(rg);
    if (once) {
      if (!first_sample) {
        first_sample = v;
        _index++;
      }
    } else {
      _index++;
    }
    return v;
  }
  /**
   * @brief      Counts the number of sampled values since reset.
   *
   * @return     The number of sampled values
   */
  virtual unsigned count() const { return _index; }
  /**
   * @brief      Returns whether the generator is exhausted and
   * if not reset, \ref sample will raise an error.
   *
   * @return     True if the generator is exhausted.
   */
  virtual bool done() const { return false; }
  // friend std::ostream& operator<<(std::ostream& os, const Sampler& c) {
  //   return c.output(os);
  // }

  /**
   * @brief      Resets the generator.
   *
   * It also resets the samples count to 0.
   */
  virtual void reset(std::optional<unsigned> index = std::nullopt) {
    // if (!once) {
    //   _index = index;
    // }
    if (!once) {
      _index = 0;
    } else if (index) {
      _index = *index;
    }
    first_sample = std::nullopt;
  }

  // virtual std::ostream& output(std::ostream& os) const {
  //   return os << "Sampler";
  // }

  /**
   * Whenever to sample only once and than output the same constant value until
   * reset.
   */
  bool once;

protected:
  virtual T s(RandomGenerator &rg) = 0;
  unsigned _index;
  std::optional<T> first_sample;
};

/**
 * @brief      An inexhaustible generator that always returns the same value.
 *
 * @tparam     T   The sampled type
 */
template <typename T> struct ConstantSampler final : public Sampler<T> {
  /**
   * @brief      Construct an instance
   *
   * @param[in]  value  The constant value
   * @param[in]  once   Whether to repeat the first sample (until reset)
   */
  explicit ConstantSampler(T value, bool once = false)
      : Sampler<T>(once), value{value} {}

  T value;

protected:
  T s(RandomGenerator &rg) override { return value; }
  // std::ostream& output(std::ostream& os) const override {
  //   return os << "ConstantSampler(" << value << ")";
  // }
};

/**
 * @brief      An generator that loops through a sequence of values.
 *
 * If wrap is not set to \ref Wrap::terminate, the generator is inexhaustible,
 * else it will be exhausted after looping once through all values once.
 *
 * @tparam     T   The sampled type
 */
template <typename T> struct SequenceSampler final : public Sampler<T> {
  using Sampler<T>::_index;
  /**
   * @brief      Construct an instance
   *
   * @param[in]  values  The values to be sampled in sequence
   * @param[in]  wrap    How it should wrap at the end of the sequence
   * @param[in]  once   Whether to repeat the first sample (until reset)
   */
  explicit SequenceSampler(const std::vector<T> &values, Wrap wrap = Wrap::loop,
                           bool once = false)
      : Sampler<T>(once), values{values}, wrap{wrap} {}

  /**
   * @private
   */
  bool done() const override {
    return wrap_done(wrap, _index, static_cast<unsigned>(values.size()));
  }

  std::vector<T> values;
  Wrap wrap;

protected:
  T s(RandomGenerator &rg) override {
    return values[wrap_index(wrap, _index,
                             static_cast<unsigned>(values.size()))];
  }

  // std::ostream& output(std::ostream& os) const override {
  //   os << "SequenceSampler({";
  //   for (const auto& item : values) {
  //     os << item << ", ";
  //   }
  //   os << "})";
  //   return os;
  // }
};

template <typename T>
inline constexpr bool is_number =
    std::is_floating_point_v<T> || std::is_same_v<T, int> ||
    std::is_same_v<T, unsigned>;

template <typename T>
inline constexpr bool is_algebra = is_number<T> || std::is_same_v<T, Vector2>;

/**
 * @brief      An generator that sample regularly,
 * adding a fixed step to the previous sample.
 *
 * Only defined if T is an algebra.
 *
 * If \ref wrap is not set to \ref Wrap::terminate, the generator is
 * inexhaustible, else it will be exhausted after looping once through all
 * values.
 *
 * @tparam     T   The sampled type
 */
template <typename T> struct RegularSampler final : public Sampler<T> {
  using Sampler<T>::_index;

  /**
   * @brief      { function_description }
   *
   * @private
   * @param[in]  from    The initial value
   * @param[in]  number  The number of samples to draw (``from`` included)
   * @param[in]  wrap    How it should wrap at the end of the interval
   * @param[in]  once   Whether to repeat the first sample (until reset)
   */
  RegularSampler(const T &from, std::optional<unsigned> number,
                 Wrap wrap = Wrap::loop, bool once = false)
      : Sampler<T>(once), from{from}, number{number}, wrap{wrap} {}

  /**
   * @brief      Construct a sampler that will samples ``number`` points between
   * ``from`` and ``to``.
   *
   * For example, ``number=3``, samples the following points:
   * ``from``, ``(from + to) / 2``, ``to``.
   *
   *
   * @param[in]  from    The first value
   * @param[in]  to      The target value to be reached after ``number``
   * samples.
   * @param[in]  number  The number of samples to draw.
   * @param[in]  wrap    How it should wrap at the end of the interval
   * (i.e., after ``number`` samples have been drawn)
   * @param[in]  once   Whether to repeat the first sample (until reset)
   *
   * @return     The sampler.
   */
  static RegularSampler make_with_interval(const T &from, const T &to,
                                           unsigned number,
                                           Wrap wrap = Wrap::loop,
                                           bool once = false) {
    RegularSampler r(from, number, wrap, once);
    r.to = to;
    if (number > 1) {
      r.step = (to - from) / (number - 1);
    }
    return r;
  }

  /**
   * @brief      Construct a sampler that will samples points, iteratively
   * adding ``step``.
   *
   * For example, ``step=1``, samples the following points:
   * ``from``, ``from + 1``, ``from + 2`, ...
   *
   * @param[in]  from    The first value
   * @param[in]  step    The step
   * @param[in]  number  The number of samples to draw
   * @param[in]  wrap    How it should wrap at the end of the interval
   * (i.e., after ``number`` samples have been drawn)
   * @param[in]  once   Whether to repeat the first sample (until reset)
   */
  static RegularSampler
  make_with_step(const T &from, const T &step,
                 std::optional<unsigned> number = std::nullopt,
                 Wrap wrap = Wrap::loop, bool once = false) {
    RegularSampler r(from, number, wrap, once);
    r.step = step;
    if (number && *number > 0) {
      r.to = from + step * (*number - 1);
    }
    return r;
  }

  /**
   * @private
   */
  bool done() const override {
    return number && wrap_done(wrap, _index, *number);
  }

  T from;
  std::optional<T> to;
  T step;
  std::optional<unsigned> number;
  Wrap wrap;

protected:
  T s(RandomGenerator &rg) override {
    unsigned i = _index;
    if (number) {
      i = wrap_index(wrap, i, *number);
    }
    return from + step * i;
  }

  // std::ostream& output(std::ostream& os) const override {
  //   return os << "RegularSampler<" << from << " + " << step << ">";
  // }
};

/**
 * @brief      RegularSamplerly sample from a grid of points.
 *
 * If wrap is not set to \ref Wrap::terminate, the generator is inexhaustible,
 * else it will be exhausted after looping once through all values once.
 */
struct GridSampler final : public Sampler<Vector2> {
  // using Sampler<Vector2>::_index;

  /**
   * @brief      Construct an instance
   *
   * @param[in]  from     One corner of the covered area
   * @param[in]  to      The opposite corner of the covered area
   * @param[in]  numbers  The size of the grid, i.e.,
   * the number of points along the x- and y-axis.
   * @param[in]  wrap     How it should wrap at end of the covered area
   * @param[in]  once   Whether to repeat the first sample (until reset)
   *
   */
  explicit GridSampler(const Vector2 &from, const Vector2 &to,
                       std::array<unsigned, 2> numbers, Wrap wrap = Wrap::loop,
                       bool once = false)
      : Sampler<Vector2>{once}, from(from), to(to), numbers(numbers),
        wrap(wrap), step() {
    const auto delta = from - to;
    for (int i = 0; i < 2; ++i) {
      if (numbers[i] > 1) {
        step[i] = delta[i] / (numbers[i] - 1);
      }
    }
  }

  /**
   * @private
   */
  bool done() const override {
    return wrap_done(wrap, _index, numbers[0] * numbers[1]);
  }

  Vector2 from;
  Vector2 to;
  std::array<unsigned, 2> numbers;
  Wrap wrap;

protected:
  Vector2 s(RandomGenerator &rg) override {
    unsigned i = wrap_index(wrap, _index, numbers[0] * numbers[1]);
    return {from[0] + (i % numbers[0]) * step[0],
            from[1] + (i / numbers[0]) * step[1]};
  }

private:
  Vector2 step;
};

template <class T>
using uniform_distribution = typename std::conditional<
    std::is_floating_point<T>::value, std::uniform_real_distribution<T>,
    typename std::conditional<std::is_integral<T>::value,
                              std::uniform_int_distribution<T>,
                              void>::type>::type;

/**
 * @brief      Sample randomly from a uniform distribution.
 *
 * Only defined if T is a number or a \ref core::Vector2.
 *
 * @tparam     T     The sampled type
 * @param[in]  once  Whether to repeat the first sample (until reset)
 */
template <typename T> struct UniformSampler final : public Sampler<T> {
  using Sampler<T>::_index;

  UniformSampler(T min, T max, bool once = false)
      : Sampler<T>(once), min{min}, max{max}, dist{min, max} {}

  T min;
  T max;

protected:
  T s(RandomGenerator &rg) override { return dist(rg); }

  uniform_distribution<T> dist;
};

/**
 * @brief      Sample vectors randomly from a uniform distribution.
 *
 * @param[in]  once  Whether to repeat the first sample (until reset)
 */
template <> struct UniformSampler<Vector2> final : public Sampler<Vector2> {
  using Sampler<Vector2>::_index;

  UniformSampler(Vector2 min, Vector2 max, bool once = false)
      : Sampler<Vector2>(once), min{min}, max{max}, dist_x{min[0], max[0]},
        dist_y{min[1], max[1]} {}

  Vector2 min;
  Vector2 max;

protected:
  Vector2 s(RandomGenerator &rg) override { return {dist_x(rg), dist_y(rg)}; }

  uniform_distribution<ng_float_t> dist_x;
  uniform_distribution<ng_float_t> dist_y;
};

/**
 * @brief      Sample randomly from a uniform distribution.
 *
 * Values are optionally clamped when min and/or max are provided.
 *
 * Only defined if T is a number.
 *
 * @tparam     T     The sampled type
 */
template <typename T> struct NormalSampler final : public Sampler<T> {
  /**
   * @brief      Construct an instance
   *
   * @param[in]  mean     The mean
   * @param[in]  std_dev  The standard deviation
   * @param[in]  min      The minimum value
   * @param[in]  max      The maximum value
   * @param[in]  once     Whether to repeat the first sample (until reset)
   * @param[in]  clamp    Whether to clamp the values in ``[min, max]``
   */
  NormalSampler(ng_float_t mean, ng_float_t std_dev,
                std::optional<T> min = std::nullopt,
                std::optional<T> max = std::nullopt, bool once = false,
                bool clamp = true)
      : Sampler<T>(once), min(min), max(max), mean(mean), std_dev(std_dev),
        clamp(clamp), dist{mean, std_dev} {}

  std::optional<T> min;
  std::optional<T> max;
  ng_float_t mean;
  ng_float_t std_dev;
  bool clamp;

protected:
  T s(RandomGenerator &rg) override {
    T value = static_cast<T>(dist(rg));
    if (min) {
      if (value < *min) {
        if (clamp) {
          value = *min;
        } else {
          return s(rg);
        }
      }
    }
    if (max) {
      if (value > *max) {
        if (clamp) {
          value = *max;
        } else {
          return s(rg);
        }
      }
    }
    return value;
  }

private:
  std::normal_distribution<ng_float_t> dist;
};

inline std::vector<double>
make_probabilities(size_t n, const std::vector<double> &values) {
  std::vector<double> ps(n);
  size_t m = std::min(values.size(), n);
  std::transform(values.begin(), values.begin() + m, ps.begin(),
                 [](double v) { return std::max(0.0, v); });
  if (m < n) {
    const auto sum = std::accumulate(ps.begin(), ps.end(), 0.0);
    const auto p = std::max(0.0, (1 - sum) / (n - m));
    std::fill_n(ps.begin() + m, n - m, p);
  }
  return ps;
}

/**
 * @brief      An inexhaustible generator that randomly draw from a
 * collection of values with replacement.
 *
 *
 * @tparam     T   The sampled type
 */
template <typename T> struct ChoiceSampler final : public Sampler<T> {
  /**
   * @brief      Construct an instance
   *
   * @param[in]  values  The values. Should have at least length 1.
   * @param[in]  probabilities: The probability weight for each value.
   *     Can but must not be normalized.
   *     Exceeding weights (with respect to the number of values) are ignored.
   *     Missing weights are assigned a uniform value, 
   *     so that the total sum is 1. 
   *     
   *     For example, if there are 4 values and 2 weights ``{0.2, 0.6}``, the
   *     weights will be completed as ``{0.2, 0.6, 0.1, 0.1}``.
   *
   *     Passing an empty vector (the default) creates a discrete
   *     uniform distribution.
   *
   * @param[in]  once    Whether to repeat the first sample (until reset)
   */
  explicit ChoiceSampler(const std::vector<T> &values,
                         const std::vector<double> &probabilities = {},
                         bool once = false)
      : Sampler<T>(once), _values{values},
        _probabilities{make_probabilities(_values.size(), probabilities)},
        _dist(_probabilities.begin(), _probabilities.end()) {}

  /**
   * @private
   */
  bool done() const override { return _values.empty(); }

  std::vector<T> _values;
  std::vector<double> _probabilities;

protected:
  T s(RandomGenerator &rg) override { return _values[_dist(rg)]; }

private:
  std::discrete_distribution<> _dist;
};

/**
 * @brief      An inexhaustible generator of iid binary values
 * sampled from a Bernoulli distribution.
 *
 *
 * @tparam     T   The sampled type
 */
template <typename T> struct BinarySampler final : public Sampler<T> {
  /**
   * @brief      Construct an instance
   *
   * @param[in]  probability: The probability of the positive value in [0, 1]
   *
   * @param[in]  once    Whether to repeat the first sample (until reset)
   */
  explicit BinarySampler(const double &probability = 0.5, bool once = false)
      : Sampler<T>(once), _dist(probability) {}

  /**
   * @private
   */
  bool done() const override { return false; }

  double get_probability() const { return _dist.p(); }

protected:
  T s(RandomGenerator &rg) override { return T(_dist(rg)); }

private:
  std::bernoulli_distribution _dist;
};

template <class T, class U> struct is_one_of;

template <class T, class... Ts>
struct is_one_of<T, std::variant<Ts...>>
    : std::bool_constant<(std::is_same_v<T, Ts> || ...)> {};

template <class T>
using allowed = is_one_of<T, navground::core::Property::Field>;

/**
 * @brief      This class wraps generic \ref Sampler<T> to generate
 * values of type \ref navground::core::Property::Field.
 */
struct PropertySampler : Sampler<navground::core::Property::Field> {
  template <typename T> using US = std::unique_ptr<Sampler<T>>;

  template <typename S>
  using ST = std::decay_t<decltype(std::declval<S>().sample())>;

  using S =
      std::variant<US<bool>, US<int>, US<ng_float_t>, US<std::string>,
                   US<Vector2>, US<std::vector<bool>>, US<std::vector<int>>,
                   US<std::vector<ng_float_t>>, US<std::vector<std::string>>,
                   US<std::vector<Vector2>>>;

  /**
   * @brief      Constructs an instance
   *
   * @param[in]  value     A sampler
   *
   * @tparam     T     Needs to be one of the type of \ref
   * navground::core::Property::Field
   *
   */
  template <typename T, typename = std::enable_if_t<allowed<T>::value>>
  PropertySampler(std::unique_ptr<Sampler<T>> value)
      : sampler{std::move(value)}, type_name(core::field_type_name<T>()) {}

  /**
   * @brief      Constructs an instance
   *
   * @param      value     A sampler
   *
   * @tparam     S     Needs to be one a subclass of Sampler<T>,
   * with T as one of the types of \ref navground::core::Property::Field
   */
  template <typename S, typename = std::enable_if_t<allowed<ST<S>>::value>>
  PropertySampler(S &&value)
      : sampler(
            static_cast<US<ST<S>>>(std::make_unique<S>(std::forward(value)))),
        type_name(core::field_type_name<S::value_type>()) {}

  /**
   * @brief      Create a new sampler
   *
   * @param[in]  args   The arguments of the constructor of ``S``
   *
   * @tparam     S      Needs to be one a subclass of Sampler<T>,
   * with T as one of the types of \ref navground::core::Property::Field
   * @tparam     Targs  The type of the arguments of the constructor of ``S``
   */
  template <typename S, typename... Targs,
            typename = std::enable_if_t<allowed<ST<S>>::value>>
  PropertySampler(Targs... args)
      : sampler{static_cast<US<ST<S>>>(std::make_unique<S>(args...))},
        type_name(core::field_type_name<S::value_type>()) {}

  S sampler;
  std::string type_name;

  /**
   * @brief      Check if the sampler is valid.
   *
   * @return     True if valid
   **/
  bool valid() const {
    return std::visit([](auto &&arg) -> bool { return bool(arg); }, sampler);
  }

  /**
   * @private
   */
  unsigned count() const override {
    return std::visit(
        [](auto &&arg) -> unsigned {
          if (arg) {
            return arg->count();
          }
          return 0;
        },
        sampler);
  }

  /**
   * @private
   */
  bool done() const override {
    return std::visit(
        [](auto &&arg) -> bool {
          if (arg) {
            return arg->done();
          }
          return true;
        },
        sampler);
  }

  /**
   * @private
   */
  void reset(std::optional<unsigned> index = std::nullopt) override {
    Sampler<navground::core::Property::Field>::reset(index);
    std::visit(
        [index](auto &&arg) -> void {
          if (arg) {
            arg->reset(index);
          }
        },
        sampler);
  }

protected:
  navground::core::Property::Field s(RandomGenerator &rg) override {
    return std::visit(
        [&rg](auto &&arg) -> navground::core::Property::Field {
          return arg->sample(rg);
        },
        sampler);
  }
};

} // namespace navground::sim

#endif // NAVGROUND_SIM_SAMPLING_SAMPLER_H
