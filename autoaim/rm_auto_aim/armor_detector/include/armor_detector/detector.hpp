std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  // Update params (this can be kept if you still want to update parameters)
  detector_->binary_thres = get_parameter("binary_thres").as_int();
  detector_->detect_color = get_parameter("detect_color").as_int();
  detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

  // If you only need debug information, the actual armor detection can be removed:
  // auto armors = yolo11_->detect(img, frame_count_);

  // Publish debug info
  if (debug_) {
    // Publish binary image for debugging
    binary_img_pub_.publish(
      cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(
      detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
      [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
    std::sort(
      detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
      [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });

    // Publish debug lights and armors data
    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    // If you want to keep number image publishing:
    // if (!armors.empty()) {
    //   auto all_num_img = detector_->getAllNumbersImage();
    //   number_img_pub_.publish(
    //     *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    // }

    // Draw results (for debugging purposes)
    detector_->drawResults(img);

    // Draw camera center (for visualization)
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    
    // Optionally, draw latency if you need it
    // std::stringstream latency_ss;
    // latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    // auto latency_s = latency_ss.str();
    // cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    // Publish final result image with debug info
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  // Return an empty vector or debug armors (depending on your needs)
  // Since we're removing the actual detection, we can return an empty vector here:
  return std::vector<Armor>();
}

