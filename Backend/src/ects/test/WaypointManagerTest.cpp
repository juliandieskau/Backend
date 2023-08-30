#include "../plugins/waypoint/WaypointManager.hpp"
#include "TestUtil.hpp"
#include "ects/ForceRetransmit.h"
#include "gtest/gtest.h"
#include <nlohmann/json.hpp>
#include <thread>

using namespace ects::plugins::waypoints;

const auto demo_waypoints =
    R"({"cyclic":true,"waypoints":[{"heading":{"accuracy":0.5890486240386963,"heading":2.35619449019234},"name":"Wiese","position":{"radius":10.0,"x":458114.6331091485,"y":5429324.024971707},"wait_time":0.0},{"heading":null,"name":"ROBDEKON","position":{"radius":7.0,"x":458074.6841011973,"y":5429384.169504865},"wait_time":0.0},{"heading":{"accuracy":0.5890486240386963,"heading":3.14159265358979},"name":"Parkplatz","position":{"radius":5.0,"x":458120.1476794424,"y":5429249.08108047},"wait_time":0.0},{"heading":null,"name":"Pforte","position":{"radius":13.0,"x":458029.2282057639,"y":5429234.709122923},"wait_time":0.0},{"heading":null,"name":"IOSB","position":{"radius":1.0,"x":458008.3351209665,"y":5429324.8282689415},"wait_time":0.0},{"heading":null,"name":"Smart Control Room","position":{"radius":3.0,"x":458044.381144582,"y":5429371.100157436},"wait_time":0.0}]})";

const auto waypoint_test_config =
    "{\n"
    "  \"core\": {\n"
    "    \"robot_name\": \"waypoints_test\",\n"
    "    \"load_plugins\": [\n"
    "      \"waypoints\"\n"
    "    ]\n"
    "  },\n"
    "  \"waypoints\": {\n"
    "    \"waypointlist_directory\": \"./test_waypoints_dir\",\n"
    "    \"start_execution_topic\": \"/test/start\",\n"
    "    \"stop_execution_topic\": \"/test/stop\",\n"
    "    \"current_waypoint_topic\": \"/test/current_waypoint\"\n"
    "  }"
    "}\n";

TEST(EctsPlugins, waypoints) {
    auto test_dir = std::filesystem::path("./test_waypoints_dir");
    std::filesystem::remove_all(test_dir);
    std::filesystem::create_directory(test_dir);

    auto ects = ects_with_config(waypoint_test_config);
    load_test_plugin(ects, "waypoints");
    ros::NodeHandle nh;

    bool received_waypoint_list = false, received_waypoint_count = false;
    WaypointList::ros_t new_waypoints;
    size_t waypoint_count;

    auto add_waypoint = nh.advertise<AddWaypointMessage::ros_t>(
        "/ects/waypoints/add_waypoint", 0);
    auto remove_waypoint = nh.advertise<RemoveWaypointMessage::ros_t>(
        "/ects/waypoints/remove_waypoint", 0);
    auto replace_waypoint = nh.advertise<ReplaceWaypointMessage::ros_t>(
        "/ects/waypoints/replace_waypoint", 0);
    auto reorder_waypoints = nh.advertise<ReorderWaypointsMessage::ros_t>(
        "/ects/waypoints/reorder_waypoints", 0);
    auto repeat_waypoints = nh.advertise<RepeatWaypointsMessage::ros_t>(
        "/ects/waypoints/repeat_waypoints", 0);
    auto reverse_waypoints = nh.advertise<ReverseWaypointsMessage::from_ros_t>(
        "/ects/waypoints/reverse_waypoints", 0);
    auto waypoint_list_subscriber = nh.subscribe<WaypointList::ros_t>(
        "/ects/waypoints/waypoint_list", 0,
        [&](const WaypointList::ros_t::ConstPtr &msg) {
            new_waypoints = *msg;
            received_waypoint_list = true;
        });
    auto waypoint_count_subscriber =
        nh.subscribe<NumberOfWaypointsMessage::ros_t>(
            "/ects/waypoints/number_of_waypoints", 0,
            [&](const NumberOfWaypointsMessage::ros_t::ConstPtr &msg) {
                waypoint_count = msg->data;
                received_waypoint_count = true;
            });

    auto pub_retransmit =
        nh.advertise<ects::ForceRetransmit>("/ects/retransmit", 0);
    auto retransmit_topic = [&](const std::string &topic) {
        ects::ForceRetransmit retransmit;
        retransmit.topic = topic;
        retransmit.reload_all = topic.empty();
        pub_retransmit.publish(retransmit);
    };
    json wp = {{"heading", {{"accuracy", 2.0}, {"heading", 0.0}}},
               {"name", "1"},
               {"position",
                {{"radius", 1.0}, {"x", 458074.68410}, {"y", 5429384.16950}}},
               {"wait_time", 5.0}};
    Waypoint w1 = wp;
    wp["name"] = "2";
    Waypoint w2 = wp;
    wp["name"] = "3";
    Waypoint w3 = wp;

    auto spin = [&](std::vector<std::string> names) {
        spin_predicate(
            [&] { return received_waypoint_list && received_waypoint_count; },
            1000);
        ASSERT_TRUE(received_waypoint_list);
        ASSERT_TRUE(received_waypoint_count);
        ASSERT_EQ(names.size(), waypoint_count);
        ASSERT_EQ(names.size(), new_waypoints.waypoints.size());
        for (size_t i = 0; i < waypoint_count; ++i)
            ASSERT_EQ(new_waypoints.waypoints[i].name, names[i]);
        received_waypoint_list = false;
        received_waypoint_count = false;
    };

    AddWaypointMessage::ros_t add;
    add.index = 0;
    add.waypoint = Waypoint::to_ros(w2);
    add_waypoint.publish(add);
    spin({"2"});

    add.waypoint = Waypoint::to_ros(w1);
    add_waypoint.publish(add);
    spin({"1", "2"});

    ReplaceWaypointMessage::ros_t replace;
    replace.index_to_replace = 0;
    replace.replacement_waypoint = Waypoint::to_ros(w3);
    replace_waypoint.publish(replace);
    spin({"3", "2"});

    ReorderWaypointsMessage::ros_t reorder;
    reorder.new_indices = {1, 0};
    reorder_waypoints.publish(reorder);
    spin({"2", "3"});

    retransmit_topic("");
    spin({"2", "3"});

    RepeatWaypointsMessage::ros_t repeat;
    repeat.data = true;
    repeat_waypoints.publish(repeat);
    spin({"2", "3"});
    ASSERT_EQ(new_waypoints.cyclic, true);

    reverse_waypoints.publish(ReverseWaypointsMessage::to_ros_t{});
    spin({"3", "2"});

    RemoveWaypointMessage::ros_t remove;
    remove.index = 0;
    remove_waypoint.publish(remove);
    spin({"2"});

    retransmit_topic("/ects/waypoints/waypoint_list");
    retransmit_topic("/ects/waypoints/number_of_waypoints");
    spin({"2"});

    {
        auto call = [&](std::vector<std::string> files) {
            bool received = false;
            std::thread call_thread([&]() {
                using file_list_service =
                    ects::server_traits<FileListService>::ros_t;
                auto files_caller = nh.serviceClient<file_list_service>(
                    "/ects/waypoints/saved_lists");
                file_list_service list;
                ASSERT_TRUE(files_caller.call(list));
                received = true;
                ASSERT_TRUE(list.response.success);
                ASSERT_TRUE(std::is_permutation(list.response.filenames.begin(),
                                                list.response.filenames.end(),
                                                files.begin(), files.end()));
            });
            spin_predicate([&]() { return received; }, 1000);
            call_thread.join();
            EXPECT_TRUE(received);
        };

        call({});
        {
            std::ofstream demo_file(test_dir / "demo.waypoints");
            demo_file << demo_waypoints;
        }
        call({"demo.waypoints"});
        {
            bool received = false;
            std::thread call_thread([&]() {
                using waypoint_list_service =
                    ects::server_traits<WaypointListFileService>::ros_t;
                auto load_caller = nh.serviceClient<waypoint_list_service>(
                    "/ects/waypoints/load_waypoint_list");
                waypoint_list_service list;
                list.request.filename = "nonexistent.waypoints";
                ASSERT_TRUE(load_caller.call(list));
                ASSERT_FALSE(list.response.success);

                list.request.filename = "demo.waypoints";
                ASSERT_TRUE(load_caller.call(list));
                ASSERT_TRUE(list.response.success);
                received = true;
            });
            spin_predicate([&]() { return received; }, 1000);
            call_thread.join();
            EXPECT_TRUE(received);
        }
        spin({"Wiese", "ROBDEKON", "Parkplatz", "Pforte", "IOSB",
              "Smart Control Room"});
        {
            bool received = false;
            std::thread call_thread([&]() {
                using waypoint_list_service =
                    ects::server_traits<WaypointListFileService>::ros_t;
                auto save_caller = nh.serviceClient<waypoint_list_service>(
                    "/ects/waypoints/save_waypoint_list");
                waypoint_list_service list;
                std::filesystem::permissions(
                    test_dir / "demo.waypoints",
                    std::filesystem::perms::owner_write |
                        std::filesystem::perms::others_write |
                        std::filesystem::perms::group_write,
                    std::filesystem::perm_options::remove);
                list.request.filename = "demo.waypoints";
                ASSERT_TRUE(save_caller.call(list));
                ASSERT_FALSE(list.response.success);

                list.request.filename = "other.waypoints";
                ASSERT_TRUE(save_caller.call(list));
                ASSERT_TRUE(list.response.success);
                received = true;
            });
            spin_predicate([&]() { return received; }, 1000);
            call_thread.join();
            EXPECT_TRUE(received);
        }
        call({"demo.waypoints", "other.waypoints"});
        std::filesystem::remove_all(test_dir);
    }
    auto expect = [](auto f) {
        spin_predicate(f, 1000);
        ASSERT_TRUE(f());
    };
    {
        using empty = std_msgs::Empty;
        bool received_start = false, received_stop = false,
             received_is_executing = false;
        bool is_executing;
        auto start_execution_subscriber = nh.subscribe<IOSBWaypointList::ros_t>(
            "/test/start", 0,
            [&](const IOSBWaypointList::ros_t::ConstPtr &start) {
                received_start = true;
            });
        auto stop_execution_subscriber = nh.subscribe<empty>(
            "/test/stop", 0,
            [&](const empty::ConstPtr &empty) { received_stop = true; });
        auto is_executing_subscriber = nh.subscribe<std_msgs::Bool>(
            "/ects/waypoints/is_executing", 0,
            [&](const std_msgs::Bool::ConstPtr &b) {
                received_is_executing = true;
                is_executing = b->data;
            });
        auto start_execution_publisher =
            nh.advertise<empty>("/ects/waypoints/execute", 0);
        auto stop_execution_publisher =
            nh.advertise<empty>("/ects/waypoints/stop", 0);
        auto start = [&] { start_execution_publisher.publish(empty{}); };
        auto stop = [&] { stop_execution_publisher.publish(empty{}); };
        auto reset = [&] {
            received_start = false;
            received_stop = false;
            received_is_executing = false;
        };
        auto spin_a_bit = [] { spin_predicate([] { return false; }, 50); };
        spin_a_bit();

        // start
        start();
        expect([&] { return received_start && received_is_executing; });
        ASSERT_TRUE(is_executing);
        ASSERT_FALSE(received_stop);
        reset();

        // start while executing
        start();
        spin_a_bit();
        ASSERT_FALSE(received_start);
        ASSERT_FALSE(received_stop);
        ASSERT_FALSE(received_is_executing);
        reset();

        // retransmit
        retransmit_topic("/ects/waypoints/is_executing");
        expect([&] { return received_is_executing; });
        ASSERT_TRUE(is_executing);
        reset();

        // stop
        stop();
        expect([&] { return received_stop && received_is_executing; });
        ASSERT_FALSE(is_executing);
        ASSERT_FALSE(received_start);
        reset();

        // retransmit all
        retransmit_topic("");
        expect([&] { return received_is_executing; });
        ASSERT_FALSE(is_executing);
        reset();

        // stop while not executing
        stop();
        spin_a_bit();
        ASSERT_FALSE(received_start || received_stop || received_is_executing);
        reset();
    }
    {
        bool received_current_waypoint = false;
        size_t current_waypoint;
        auto current_waypoint_subscriber = nh.subscribe<std_msgs::UInt32>(
            "/ects/waypoints/current_waypoint", 0,
            [&](const std_msgs::UInt32::ConstPtr &b) {
                received_current_waypoint = true;
                current_waypoint = b->data;
            });
        auto current_waypoint_publisher =
            nh.advertise<std_msgs::UInt32>("/test/current_waypoint", 0);

        std_msgs::UInt32 cur;
        cur.data = 4;
        current_waypoint_publisher.publish(cur);
        expect([&] { return received_current_waypoint; });
        ASSERT_EQ(current_waypoint, 4);
    }
}
