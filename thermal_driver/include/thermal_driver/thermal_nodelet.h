#include "ircapture.h"
#include <nodelet/nodelet.h>

namespace thermal {
class IRNodelet : public nodelet::Nodelet {

public:
  IRNodelet() {}
  ~IRNodelet() {
    if (pubThread_) {
      pubThread_->interrupt();
      pubThread_->join();
    }
  }
  virtual void onInit();

  boost::shared_ptr<IRCapture> inst_;
  std::shared_ptr<boost::thread> pubThread_;
  ros::Timer monitor_;
};
} // namespace thermal
