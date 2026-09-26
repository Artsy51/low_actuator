#include <exception>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "motor_driver/dds/dds_module.hpp"
#include "motor_driver/runtime/motor_driver_runtime.hpp"
#include "motor_driver/config/param_parser.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;

    try
    {
        const auto package_share =
            ament_index_cpp::get_package_share_directory("motor_driver");
        MotorConfigParser motor_parser;
        ControlConfigParser control_parser;
        if (!motor_parser.load_from_file(package_share + "/config/motor.json") ||
            !control_parser.load_from_file(package_share + "/config/control.json"))
        {
            rclcpp::shutdown();
            return 1;
        }

        const auto &control = control_parser.get_config();
        for (const auto &joint : control.joints)
        {
            if (!motor_parser.has_joint(joint))
            {
                RCLCPP_ERROR(rclcpp::get_logger("motor_driver"),
                             "Control joint '%s' has no motor configuration", joint.c_str());
                rclcpp::shutdown();
                return 1;
            }
        }

        auto dds = std::make_shared<DdsModule>(control);
        CanModule can(motor_parser, control_parser);
        MotorDriverRuntime runtime(can, *dds, control);
        if (!runtime.start())
        {
            rclcpp::shutdown();
            return 1;
        }

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(dds->node());
        std::thread ros_thread([&executor]() { executor.spin(); });

        runtime.run();
        executor.cancel();
        if (ros_thread.joinable())
        {
            ros_thread.join();
        }
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_driver"), "Startup/runtime error: %s",
                     error.what());
        result = 1;
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return result;
}
