#include <gmock/gmock.h>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestRustySystem : public ::testing::Test {
 protected:
  void SetUp() override {
    mock_system_ =
        R"(
  <ros2_control name="MockRustySystemHardware" type="system">
    <hardware>
      <plugin>rusty_control/RustySystemHardware</plugin>
      <param name="pin_enable_0">1</param>
      <param name="pin_pos_0">2</param>
      <param name="pin_neg_0">3</param>
      <param name="pin_enable_1">4</param>
      <param name="pin_pos_1">5</param>
      <param name="pin_neg_1">6</param>
    </hardware>
  </ros2_control>
)";
  }

  std::string mock_system_;
};

class TestableResourceManager : public hardware_interface::ResourceManager {
 public:
  friend TestRustySystem;
  TestableResourceManager() : hardware_interface::ResourceManager() {}
  TestableResourceManager(const std::string& urdf,
                          bool validate_interfaces = true,
                          bool activate_all = false)
      : hardware_interface::ResourceManager(urdf, validate_interfaces,
                                            activate_all) {}
};

TEST_F(TestRustySystem, load_generic_system) {
  auto urdf = ros2_control_test_assets::urdf_head + mock_system_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(TestableResourceManager rm(urdf));
}