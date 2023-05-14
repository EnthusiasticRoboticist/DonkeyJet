#include <librealsense2/rs.hpp>

namespace donkeyjet {
namespace hardware {

class RealsenseSensor {
public:
  virtual void start_stream();
  virtual void stop_stream();
  std::function<void(const rs2::frame &frame)> callback;

protected:
  virtual void default_callback(const rs2::frame &frame) = 0;
  rs2::pipeline rs_pipeline_;
  rs2::config rs_config_;
};

using RealsenseSensorPtr = std::shared_ptr<RealsenseSensor>;

class T265 : public RealsenseSensor {
public:
  static RealsenseSensorPtr create(const rs2::device &rs_device);

protected:
  T265(const rs2::device &rs_device);
  virtual void default_callback(const rs2::frame &frame);
};

class D435 : public RealsenseSensor {
public:
  static RealsenseSensorPtr create(const rs2::device &rs_device);

protected:
  D435(const rs2::device &rs_device);
  virtual void default_callback(const rs2::frame &frame);
};

} // namespace hardware
} // namespace donkeyjet