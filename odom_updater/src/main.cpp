# include<memory>
# include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]){
    // initiate
    rclcpp::init(argc,argv);
    // create and spin node
    rclcpp::spin(std::make_shared<OdomUpdater>());
    // shut down
    rclcpp::shutdown();
    return 0;
}