/// \file ROS Node for converting data from Lepton 3.1R to ROS messages
/// \brief Converts raw data received from ethernet to ros messages
///
/// PARAMETERS:
///     \param None
///
/// PUBLISHES:
///     \param thermal_image (sensor_msgs::msg::Image) Thermal image with custom colormap
///     \param raw_thermal_temperature (thermal_network::msg::ThermalData) Raw temperature data
///
/// SUBSCRIBES:
///     \param None
///
/// SERVERS:
///     \param None
///
/// CLIENTS:
///     \param None

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "thermal_network/msg/thermal_data.hpp"

using namespace std::chrono_literals;

class ThermalData : public rclcpp::Node
{
/// \brief Node that receives data from UDP network and converts it to ROS messages

public:
  /// \brief Main constructor
  ThermalData()
  : Node("ThermalData")
  {
    // Socket settings intialization
    if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Socket creation failed");
      rclcpp::shutdown();
    }

    memset(&servaddr_, 0, sizeof(servaddr_));
    memset(&cliaddr_, 0, sizeof(cliaddr_));

    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(8080);
    servaddr_.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd_, (const struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Bind failed");
      close(sockfd_);
      rclcpp::shutdown();
    }

    // Publishers
    thermal_pub_ = create_publisher<thermal_network::msg::ThermalData>(
      "raw_thermal_tempature", 10);
    img_pub_ = create_publisher<sensor_msgs::msg::Image>("thermal_image", 10);

    // Running thread to receive thermal data
    received_thread_ = std::thread(&ThermalData::temp_data, this);
  }

  /// \brief Main destructor that closes the function and merges with the thread
  ~ThermalData()
  {
    close(sockfd_);
    if (received_thread_.joinable()) {
      received_thread_.join();
    }
  }

private:
  // Variables
  sensor_msgs::msg::Image thermal_image_msg_;
  thermal_network::msg::ThermalData temp_msg_;

  int sockfd_;
  struct sockaddr_in servaddr_, cliaddr_;
  std::thread received_thread_;

  bool autoRangeMin_ = false;
  bool autoRangeMax_ = false;
  uint16_t rangeMin_ = 27300; // Minimum temperture range (temp / 100 - 273) celsius
  uint16_t rangeMax_ = 31500; // Maximum temperture range
  int myImageWidth_ = 160;
  int myImageHeight_ = 120;
  uint16_t minValue_ = rangeMin_;
  uint16_t maxValue_ = rangeMax_;
  float diff_ = maxValue_ - minValue_;
  float scale_ = 255 / diff_;
  uint16_t n_zero_value_drop_frame_ = 0;
  std::array<uint8_t, 9840> result_;
  std::array<std::array<uint8_t, 9840>, 4> shelf_;
  std::vector<float> temperature_data_ = std::vector<float>(myImageWidth_ * myImageHeight_, 0.0);
  std::vector<uint8_t> image_data_ = std::vector<uint8_t>(myImageWidth_ * myImageHeight_ * 3);

  // Create objects
  rclcpp::Publisher<thermal_network::msg::ThermalData>::SharedPtr thermal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;

  // Custom colorpalette
  std::vector<int> colormap_ironblack_ =
  {255, 255, 255, 253, 253, 253, 251, 251, 251, 249, 249, 249,
    247, 247, 247, 245, 245, 245, 243, 243, 243, 241, 241, 241, 239, 239, 239, 237, 237, 237, 235,
    235, 235, 233, 233, 233, 231, 231, 231, 229, 229, 229, 227, 227, 227, 225, 225, 225, 223, 223,
    223, 221, 221, 221, 219, 219, 219, 217, 217, 217, 215, 215, 215, 213, 213, 213, 211, 211, 211,
    209, 209, 209, 207, 207, 207, 205, 205, 205, 203, 203, 203, 201, 201, 201, 199, 199, 199, 197,
    197, 197, 195, 195, 195, 193, 193, 193, 191, 191, 191, 189, 189, 189, 187, 187, 187, 185, 185,
    185, 183, 183, 183, 181, 181, 181, 179, 179, 179, 177, 177, 177, 175, 175, 175, 173, 173, 173,
    171, 171, 171, 169, 169, 169, 167, 167, 167, 165, 165, 165, 163, 163, 163, 161, 161, 161, 159,
    159, 159, 157, 157, 157, 155, 155, 155, 153, 153, 153, 151, 151, 151, 149, 149, 149, 147, 147,
    147, 145, 145, 145, 143, 143, 143, 141, 141, 141, 139, 139, 139, 137, 137, 137, 135, 135, 135,
    133, 133, 133, 131, 131, 131, 129, 129, 129, 126, 126, 126, 124, 124, 124, 122, 122, 122, 120,
    120, 120, 118, 118, 118, 116, 116, 116, 114, 114, 114, 112, 112, 112, 110, 110, 110, 108, 108,
    108, 106, 106, 106, 104, 104, 104, 102, 102, 102, 100, 100, 100, 98, 98, 98, 96, 96, 96, 94,
    94, 94, 92, 92, 92, 90, 90, 90, 88, 88, 88, 86, 86, 86, 84, 84, 84, 82, 82, 82, 80, 80, 80,
    78, 78, 78, 76, 76, 76, 74, 74, 74, 72, 72, 72, 70, 70, 70, 68, 68, 68, 66, 66, 66, 64, 64,
    64, 62, 62, 62, 60, 60, 60, 58, 58, 58, 56, 56, 56, 54, 54, 54, 52, 52, 52, 50, 50, 50, 48,
    48, 48, 46, 46, 46, 44, 44, 44, 42, 42, 42, 40, 40, 40, 38, 38, 38, 36, 36, 36, 34, 34, 34,
    32, 32, 32, 30, 30, 30, 28, 28, 28, 26, 26, 26, 24, 24, 24, 22, 22, 22, 20, 20, 20, 18, 18,
    18, 16, 16, 16, 14, 14, 14, 12, 12, 12, 10, 10, 10, 8, 8, 8, 6, 6, 6, 4, 4, 4, 2, 2, 2, 0, 0,
    0, 0, 0, 9, 2, 0, 16, 4, 0, 24, 6, 0, 31, 8, 0, 38, 10, 0, 45, 12, 0, 53, 14, 0, 60, 17, 0,
    67, 19, 0, 74, 21, 0, 82, 23, 0, 89, 25, 0, 96, 27, 0, 103, 29, 0, 111, 31, 0, 118, 36, 0,
    120, 41, 0, 121, 46, 0, 122, 51, 0, 123, 56, 0, 124, 61, 0, 125, 66, 0, 126, 71, 0, 127, 76,
    1, 128, 81, 1, 129, 86, 1, 130, 91, 1, 131, 96, 1, 132, 101, 1, 133, 106, 1, 134, 111, 1, 135,
    116, 1, 136, 121, 1, 136, 125, 2, 137, 130, 2, 137, 135, 3, 137, 139, 3, 138, 144, 3, 138,
    149, 4, 138, 153, 4, 139, 158, 5, 139, 163, 5, 139, 167, 5, 140, 172, 6, 140, 177, 6, 140,
    181, 7, 141, 186, 7, 141, 189, 10, 137, 191, 13, 132, 194, 16, 127, 196, 19, 121, 198, 22,
    116, 200, 25, 111, 203, 28, 106, 205, 31, 101, 207, 34, 95, 209, 37, 90, 212, 40, 85, 214,
    43, 80, 216, 46, 75, 218, 49, 69, 221, 52, 64, 223, 55, 59, 224, 57, 49, 225, 60, 47, 226,
    64, 44, 227, 67, 42, 228, 71, 39, 229, 74, 37, 230, 78, 34, 231, 81, 32, 231, 85, 29, 232,
    88, 27, 233, 92, 24, 234, 95, 22, 235, 99, 19, 236, 102, 17, 237, 106, 14, 238, 109, 12, 239,
    112, 12, 240, 116, 12, 240, 119, 12, 241, 123, 12, 241, 127, 12, 242, 130, 12, 242, 134, 12,
    243, 138, 12, 243, 141, 13, 244, 145, 13, 244, 149, 13, 245, 152, 13, 245, 156, 13, 246, 160,
    13, 246, 163, 13, 247, 167, 13, 247, 171, 13, 248, 175, 14, 248, 178, 15, 249, 182, 16, 249,
    185, 18, 250, 189, 19, 250, 192, 20, 251, 196, 21, 251, 199, 22, 252, 203, 23, 252, 206, 24,
    253, 210, 25, 253, 213, 27, 254, 217, 28, 254, 220, 29, 255, 224, 30, 255, 227, 39, 255, 229,
    53, 255, 231, 67, 255, 233, 81, 255, 234, 95, 255, 236, 109, 255, 238, 123, 255, 240, 137,
    255, 242, 151, 255, 244, 165, 255, 246, 179, 255, 248, 193, 255, 249, 207, 255, 251, 221, 255,
    253, 235, 255, 255, 24, -1};

  /// \brief Main function that receives data from udp
  void temp_data()
  {
    socklen_t len = sizeof(cliaddr_);
    while (rclcpp::ok()) {
      for (int i = 0; i < 4; ++i) {
        ssize_t received_bytes =
          recvfrom(
          sockfd_,
          shelf_[i].data(), sizeof(shelf_[i]), 0, (struct sockaddr *)&cliaddr_, &len);
        if (received_bytes < 0) {
          RCLCPP_ERROR_STREAM(get_logger(), "Receive failed");
          close(sockfd_);
          return;
        }
      }
      process_data();
    }
  }

  /// \brief Processes data received and publishes the temperature data and image
  void process_data()
  {
    const int * selectedColormap_ = colormap_ironblack_.data();
    int selectedColormapSize_ = colormap_ironblack_.size();

    if (autoRangeMin_ || autoRangeMax_) {
      if (autoRangeMin_) {
        maxValue_ = 65535;
      }
      if (autoRangeMax_) {
        minValue_ = 0;
      }
      for (int iSegment = 1; iSegment <= 4; iSegment++) {
        for (int i = 0; i < 4920; i++) {
          if (i % 82 < 2) {
            continue;
          }
          uint16_t value = (shelf_[iSegment - 1][i * 2] << 8) + shelf_[iSegment - 1][i * 2 + 1];
          if (value == 0) {
            continue;
          }
          if (autoRangeMax_ && (value > maxValue_)) {
            maxValue_ = value;
          }
          if (autoRangeMin_ && (value < minValue_)) {
            minValue_ = value;
          }
        }
      }
      diff_ = maxValue_ - minValue_;
      scale_ = 255 / diff_;
    }

    int row, column;
    uint16_t value;
    uint16_t valueFrameBuffer;
    for (int iSegment = 1; iSegment <= 4; iSegment++) {
      int ofsRow = 30 * (iSegment - 1);
      for (int i = 0; i < 4920; i++) {
        if (i % 82 < 2) {
          continue;
        }
        valueFrameBuffer = (shelf_[iSegment - 1][i * 2] << 8) + shelf_[iSegment - 1][i * 2 + 1];
        if (valueFrameBuffer == 0) {
          n_zero_value_drop_frame_++;
          break;
        }
        if (!autoRangeMax_ && (valueFrameBuffer > maxValue_)) {
          value = maxValue_;
        } else if (!autoRangeMin_ && (valueFrameBuffer <= minValue_)) {
          value = minValue_;
        } else {
          value = (valueFrameBuffer - minValue_) * scale_;
        }
        float temperature = (valueFrameBuffer / 100.0) - 273.0;
        int ofs_r = 3 * value + 0; if (selectedColormapSize_ <= ofs_r) {
          ofs_r = selectedColormapSize_ - 1;
        }
        int ofs_g = 3 * value + 1; if (selectedColormapSize_ <= ofs_g) {
          ofs_g = selectedColormapSize_ - 1;
        }
        int ofs_b = 3 * value + 2; if (selectedColormapSize_ <= ofs_b) {
          ofs_b = selectedColormapSize_ - 1;
        }
        column = (i % 82) - 2 + (myImageWidth_ / 2) * ((i % (82 * 2)) / 82);
        row = i / 82 / 2 + ofsRow;
        if (row < myImageHeight_ && column < myImageWidth_) {
          image_data_[(row * myImageWidth_ + column) * 3 + 0] = selectedColormap_[ofs_b];
          image_data_[(row * myImageWidth_ + column) * 3 + 1] = selectedColormap_[ofs_g];
          image_data_[(row * myImageWidth_ + column) * 3 + 2] = selectedColormap_[ofs_r];
          temperature_data_[row * myImageWidth_ + column] = temperature;
        }
      }
    }
    if (n_zero_value_drop_frame_ != 0) {
      n_zero_value_drop_frame_ = 0;
    }
    temp_msg_.temp = temperature_data_;
    temp_msg_.height = myImageHeight_;
    temp_msg_.width = myImageWidth_;
    thermal_pub_->publish(temp_msg_);
    thermal_image_msg_.header.stamp = get_clock()->now();
    thermal_image_msg_.header.frame_id = "thermal_image";
    thermal_image_msg_.height = myImageHeight_;
    thermal_image_msg_.width = myImageWidth_;
    thermal_image_msg_.encoding = sensor_msgs::image_encodings::RGB8;
    thermal_image_msg_.is_bigendian = false;
    thermal_image_msg_.step = myImageWidth_ * 3;
    thermal_image_msg_.data = image_data_;
    img_pub_->publish(thermal_image_msg_);
  }
};

/// \brief Main function
int main(int argc, char * argv[])
{
  /// \brief Spinning the node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalData>());
  rclcpp::shutdown();
  return 0;
}
