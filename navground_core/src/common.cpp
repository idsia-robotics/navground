/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/common.h"

namespace navground::core {

Twist2 Twist2::relative(const Pose2 & reference) const {
  if (frame == Frame::absolute) {
    auto r_value = rotate(-reference.orientation);
    r_value.frame = Frame::relative;
    return r_value;
  }
  return *this;
}

Twist2 Twist2::absolute(const Pose2 & reference) const {
  if (frame == Frame::relative) {
    auto a_value = rotate(reference.orientation);
    a_value.frame = Frame::absolute;
    return a_value;
  }
  return *this;
}

Twist2 Twist2::to_frame(Frame frame, const Pose2 & reference) const {
  return frame == Frame::relative ? relative(reference) : absolute(reference);
}

}  // namespace navground::core
