#include <rviz/default_plugin/tools/point_tool.h>
#include <rviz/properties/property.h>
#include <pluginlib/class_list_macros.h>
namespace smartcar_region_tool {
class DrawRegion : public rviz::PointTool {
public:
  DrawRegion() { shortcut_key_ = 'b'; }
  void onInitialize() override {
    rviz::PointTool::onInitialize();
    setName("Draw Region");
  }
};
class SetLinePoint : public rviz::PointTool {
public:
  SetLinePoint() {
    shortcut_key_ = 'l';
    getPropertyContainer()->subProp("Single click")->setValue(false);
    getPropertyContainer()->subProp("Topic")->setValue("/single_nav/line_point");
  }
  void onInitialize() override { rviz::PointTool::onInitialize(); setName("Draw Low-Cost Lines"); }
};
}
PLUGINLIB_EXPORT_CLASS(smartcar_region_tool::DrawRegion, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(smartcar_region_tool::SetLinePoint, rviz::Tool)
