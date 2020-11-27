#include "flir_capture.h"
#include <nodelet/nodelet.h>

namespace flir {
class FlirNodelet : public nodelet::Nodelet {

public:
  FlirNodelet() {}
  ~FlirNodelet() {
    if (pubThread_) {
      pubThread_->interrupt();
      pubThread_->join();
    }
  }
  virtual void onInit();

  boost::shared_ptr<FlirCapture> inst_;
  std::shared_ptr<boost::thread> pubThread_;
};
} // namespace flir
