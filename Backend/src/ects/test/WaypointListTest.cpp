#include "../plugins/waypoint/WaypointList.hpp"
#include "../plugins/waypoint/WaypointListFileMessages.hpp"
#include "gtest/gtest.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace ects::plugins::waypoints;

TEST(Waypoints, waypoint_list) {
    json wp = {{"heading", {{"accuracy", 2.0}, {"heading", 0.0}}},
               {"name", "1"},
               {"position", {{"radius", 1.0}, {"x", 49.01515}, {"y", 8.42657}}},
               {"wait_time", 5.0}};
    Waypoint w1 = wp;
    wp["name"] = "2";
    Waypoint w2 = wp;
    wp["name"] = "3";
    Waypoint w3 = wp;
    wp["name"] = "4";
    Waypoint w4 = wp;

    WaypointList w{};
    ASSERT_EQ(w.size(), 0) << "initialize with non-zero size";

    ASSERT_NO_THROW(w.add_waypoint(w2, 0));
    ASSERT_EQ(w.size(), 1) << "failed adding exactly one waypoint";
    ASSERT_EQ(w.get_waypoint(0).get_name(), w2.get_name());

    ASSERT_ANY_THROW(w.add_waypoint(w4, 2));

    ASSERT_NO_THROW(w.add_waypoint(w4, 1));
    ASSERT_EQ(w.size(), 2) << "failed adding exactly two waypoints";
    ASSERT_EQ(w.get_waypoint(1).get_name(), w4.get_name());

    ASSERT_NO_THROW(w.add_waypoint(w1, 0));
    ASSERT_EQ(w.size(), 3) << "failed adding exactly three waypoints";
    ASSERT_EQ(w.get_waypoint(0).get_name(), w1.get_name());
    ASSERT_EQ(w.get_waypoint(1).get_name(), w2.get_name());
    ASSERT_EQ(w.get_waypoint(2).get_name(), w4.get_name());

    ASSERT_NO_THROW(w.reverse_waypoints());
    ASSERT_EQ(w.size(), 3) << "reversing waypoints changed size";
    ASSERT_EQ(w.get_waypoint(0).get_name(), w4.get_name());
    ASSERT_EQ(w.get_waypoint(1).get_name(), w2.get_name());
    ASSERT_EQ(w.get_waypoint(2).get_name(), w1.get_name());

    ASSERT_NO_THROW(w.replace_waypoint(0, w3));
    ASSERT_EQ(w.size(), 3) << "replace waypoint changed size";
    ASSERT_EQ(w.get_waypoint(0).get_name(), w3.get_name());
    ASSERT_EQ(w.get_waypoint(1).get_name(), w2.get_name());
    ASSERT_EQ(w.get_waypoint(2).get_name(), w1.get_name());

    ASSERT_ANY_THROW(w.replace_waypoint(3, w2));

    ASSERT_NO_THROW(w.reorder_waypoints({1, 2, 0}));
    ASSERT_EQ(w.size(), 3) << "reorder waypoint changed size";
    ASSERT_EQ(w.get_waypoint(0).get_name(), w1.get_name());
    ASSERT_EQ(w.get_waypoint(1).get_name(), w3.get_name());
    ASSERT_EQ(w.get_waypoint(2).get_name(), w2.get_name());

    ASSERT_ANY_THROW(w.reorder_waypoints({0, 1, 0}));
    ASSERT_ANY_THROW(w.reorder_waypoints({0, 1, 3}));
    ASSERT_ANY_THROW(w.reorder_waypoints({0, 1}));

    auto wx = WaypointList::from_ros(WaypointList::to_ros(w));
    ASSERT_EQ(wx.size(), 3);
    ASSERT_EQ(wx.get_waypoint(0).get_name(), w1.get_name());
    ASSERT_EQ(wx.get_waypoint(1).get_name(), w3.get_name());
    ASSERT_EQ(wx.get_waypoint(2).get_name(), w2.get_name());

    ASSERT_EQ(w.size(), 3);
    ASSERT_EQ(w.get_waypoint(0).get_name(), w1.get_name());
    ASSERT_EQ(w.get_waypoint(1).get_name(), w3.get_name());
    ASSERT_EQ(w.get_waypoint(2).get_name(), w2.get_name());

    ASSERT_NO_THROW(w.remove_waypoint(1));
    ASSERT_EQ(w.size(), 2);
    ASSERT_EQ(w.get_waypoint(0).get_name(), w1.get_name());
    ASSERT_EQ(w.get_waypoint(1).get_name(), w2.get_name());
    ASSERT_NEAR(w.total_length(), 0.0, 0.01);
}

TEST(Waypoints, FileListResponse) {
    FileListResponse response{"something went wrong"};
    auto r = FileListResponse::to_ros(response);
    ASSERT_TRUE(r.filenames.empty());
    ASSERT_FALSE(r.success);
    ASSERT_EQ(r.error_message, "something went wrong");
}
