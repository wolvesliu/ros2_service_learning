#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"//导入消息接口

/*
    创建一个类节点，名字叫做ServiceClient01 ,继承自Node.
*/
class ServiceClient01  : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    ServiceClient01 (std::string name) : Node(name)
    {
        // 打印一句
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.",name.c_str());

        // 创建客户端
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints_srv");
    }

    //客户端发送请求
    void send_request(int a, int b) {
        RCLCPP_INFO(this->get_logger(), "计算%d+%d", a, b);

        // 1.等待服务端上线
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            //等待时检测rclcpp的状态
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
        }

        // 2.构造请求的
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
        request->a = a;
        request->b = b;

        // 3.发送异步请求，然后等待返回，返回时调用回调函数
        // 第一个参数：请求的消息，这里用于放a，b两个数。第二个参数：CallBack，回调函数，异步接收服务端的返回的函数。
        client_->async_send_request(request, std::bind(&ServiceClient01::result_callback_, this, std::placeholders::_1));
    }



private:
    // 声明客户端
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
   
    void result_callback_(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future) {
        auto response = result_future.get();//get()返回结果

        RCLCPP_INFO(this->get_logger(), "计算结果：%ld", response->sum);
    }
};

int main(int argc, char **argv)
{
    /* 1. 初始化rclcpp */
    rclcpp::init(argc, argv);

    //* 2. 产生一个service_client_01的节点*/
    auto node = std::make_shared<ServiceClient01 >("service_client_01");

    //增加这一行，node->send_request(5, 6);，计算5+6结果
    //客户端发送请求
    node->send_request(5, 6);

    /* 3. 运行节点，并检测退出信号 Ctrl+C*/ 
    rclcpp::spin(node);

    /* 4.停止运行,关闭rclcpp */
    rclcpp::shutdown();
    return 0;
}
