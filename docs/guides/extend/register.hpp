#include "navground/core/property.h"

namespace core = navground::core;

struct MyComponent : public BaseComponent {
  // ... define the interface ...
};

struct MyComponent : public BaseComponent {
  // ... define the interface ...
  inline const static core::Properties properties{
      {"my_param", core::make_property<int, MyComponent>(
                       [](const MyComponent *) { return 1; }, nullptr, 1,
                       "my description")}};
};

struct MyComponent : public BaseComponent {
  // ... define the rest of the interface ...
  bool get_value() const { return _value; }
  void set_value(bool value) { _value = value; }
  inline const static core::Properties properties{
      {"value", core::make_property<bool, MyComponent>(
                    &MyComponent::get_value, &MyComponent::set_value, true,
                    "my description")}};

private:
  bool _value;
};

MyComponent c;
bool value = c.get("value").get<bool>();
c.set("value", !value);

std::string yaml = YAML::dump<BaseComponent>(&c);

struct MyComponent : public BaseComponent {

  // ... properties registration ...

  inline static const std::string type = register_type<MyComponent>("MyName");
};