#include "detection/bounding_boxes.h"
#include "geometry_msgs/Point.h"

#include <ros/ros.h>

#include <gtest/gtest.h>
#include <thread>

// terminal: catkin build --verbose --catkin-make-args run_tests | sed -n '/\[==========\]/,/\[==========\]/p'

class MyTestSuite : public ::testing::Test {
   public:
    MyTestSuite() {
    }
    ~MyTestSuite() {}
};

TEST_F(MyTestSuite, floatValue) {
    BoundingBoxes bb;
    float x1, y1, z1, x2, y2, z2;
    x1 = y1 = z1 = 1;
    x2 = y2 = z2 = 10;
    float dist = bb.calculateDistance2Points(x1, y1, z1, x2, y2, z2);
    EXPECT_EQ(dist, (float)sqrt(243));
}

TEST_F(MyTestSuite, geometryValue) {
    BoundingBoxes bb;
    geometry_msgs::Point p1, p2;
    p1.x = p1.y = p1.z = 1;
    p2.x = p2.y = p2.z = 11;
    float dist = bb.calculateDistance2Points(p1, p2);
    EXPECT_EQ(dist, (float)sqrt(243));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([] {while(ros::ok()) ros::spin(); });

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
