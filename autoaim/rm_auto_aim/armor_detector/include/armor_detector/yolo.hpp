#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace rm_auto_aim
{
class YOLOBase
{
public:
  virtual std::list<Armor> detect(const std::shared_ptr<hobotcv::Image> & hobot_img, int frame_count) = 0;
};

class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const std::shared_ptr<hobotcv::Image> & hobot_img, int frame_count = -1);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP