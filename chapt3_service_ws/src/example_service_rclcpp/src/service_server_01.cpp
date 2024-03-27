#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp" //导入消息接口

/*
    创建一个类节点，名字叫做ServiceServer01 ,继承自Node.
*/
class ServiceServer01  : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    ServiceServer01 (std::string name) : Node(name)
    {
        // 打印一句
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.",name.c_str());

        // 创建服务。两个形参
        add_ints_server_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints_srv", //服务的名字
            std::bind(&ServiceServer01::handle_add_two_ints, this, std::placeholders::_1, std::placeholders::_2));//callback
    }

private:  
    // 声明一个服务
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr add_ints_server_;

    // 收到请求的处理函数
    void handle_add_two_ints(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                             std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) 
    {
        RCLCPP_INFO(this->get_logger(), "收到a: %ld b: %ld", request->a, request->b);

        response->sum = request->a + request->b;
    };
   
};

int main(int argc, char **argv)
{
    /* 1. 初始化rclcpp */
    rclcpp::init(argc, argv);

    //* 2. 产生一个service_server_01的节点*/
    auto node = std::make_shared<ServiceServer01 >("service_server_01");

    /* 3. 运行节点，并检测退出信号 Ctrl+C*/ 
    rclcpp::spin(node);

    /* 4.停止运行,关闭rclcpp */
    rclcpp::shutdown();
    return 0;
}
