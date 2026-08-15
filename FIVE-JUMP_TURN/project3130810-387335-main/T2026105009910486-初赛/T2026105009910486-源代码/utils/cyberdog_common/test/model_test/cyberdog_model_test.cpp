// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "cyberdog_common/cyberdog_model.hpp"
#include "protocol/msg/connector_status.hpp"
#include "rclcpp/rclcpp.hpp"

// #define SPECIAL_VERSION 1
#define NEWEST_VERSION 2
// #define SPECIAL_PATH 3
struct DownloadControl
{
  bool thread_work;
  std::mutex mtx;
  std::condition_variable cond;
  DownloadControl()
  {
    thread_work = false;
  }
};

class model_test
{
  public:
  model_test(const std::string & name)
  {
    this->node_ptr_ = rclcpp::Node::make_shared(name);
    INFO("hello,model update test");
    this->download_thread_ = std::make_shared<std::thread>(&model_test::Download, this);
    this->connector_sub_ = this->node_ptr_->create_subscription<protocol::msg::ConnectorStatus>(
      "connector_state", rclcpp::SystemDefaultsQoS(),
      std::bind(&model_test::SignalCallback, this, std::placeholders::_1));
  }
  void Run()
  {
    INFO("test node spin,wait for request");
    this->executor_.add_node(node_ptr_);
    this->executor_.spin();
    INFO("test node rclcpp close");
    rclcpp::shutdown();
  }
  ~model_test()
  {}
  private:
  std::shared_ptr<std::thread> download_thread_;
  DownloadControl connector_state;
  rclcpp::Node::SharedPtr node_ptr_{nullptr};
  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Subscription<protocol::msg::ConnectorStatus>::SharedPtr connector_sub_{nullptr};
  void SignalCallback(const protocol::msg::ConnectorStatus::SharedPtr msg)
  {
    if(this->connector_state.thread_work){
      return;
    }
    INFO("hello,SignalCallback");
    if (msg->is_internet){
      INFO("internet is ok!!!");
      std::unique_lock<std::mutex> inference_lock(this->connector_state.mtx);
      this->connector_state.thread_work = true;
      this->connector_state.cond.notify_one();
    }

  }
  void Download(){
    INFO("hello,Download thread");

    std::unique_lock<std::mutex> inference_lock(this->connector_state.mtx);
    this->connector_state.cond.wait(
      inference_lock, [&] {
        return this->connector_state.thread_work;
      });
    // 指定版本下载
#ifdef SPECIAL_VERSION
    std::string name = "body_gesture";
    cyberdog::common::cyberdog_model test(name,true,"2.0");
    test.SetTimeout(300);
    int32_t code =  test.UpdateModels();
    if(code ==0){
      INFO("update model from Fds successfully"); 
    }
    // 添加模型替换判断
    INFO("Post_Process!!!!!!!!");
    if(test.Load_Model_Check()){
      test.Post_Process();
      INFO("replace and remove temp model prepare load new model");
    } else {
      INFO("load old model");
    }
#endif
  //最新版本
#ifdef NEWEST_VERSION
    std::string name = "gesture_action";
    cyberdog::common::cyberdog_model test(name);
    test.SetTimeout(300);
    int32_t code =  test.UpdateModels();
    if(code ==0){
      INFO("update model from Fds successfully"); 
    }
    // 添加模型替换判断
    INFO("Post_Process!!!!!!!!");
    if(test.Load_Model_Check()){
      test.Post_Process();
      INFO("replace and remove temp model prepare load new model");
    } else {
      INFO("load old model");
    }
    
#endif 
// 指定下载路径
#ifdef SPECIAL_PATH
    std::string name = "body_gesture";
    cyberdog::common::cyberdog_model test(name,true,"2.0","/SDCARD/","vision");
    test.SetTimeout(300);
    int32_t code =  test.UpdateModels();
    if(code ==0){
      INFO("update model from Fds successfully"); 
    }
    // 添加模型替换判断
    INFO("Post_Process!!!!!!!!");
    if(test.Load_Model_Check()){
      test.Post_Process();
      INFO("replace and remove temp model prepare load new model");
    } else {
      INFO("load old model");
    }
#endif 
  }



};


int main(int argc, char ** argv)
{

  // LOGGER_MAIN_INSTANCE("cyberdog_model");
  INFO("model_test node started");
  rclcpp::init(argc, argv);
  auto test_ptr = std::make_shared<model_test>("download_test");
  test_ptr->Run();
  return 0;
}