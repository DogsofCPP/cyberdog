# Cyberdog基础功能API封装

**具体包括：**

- json
- toml
- log



## Json

1. 采用 [rapidjson](https://github.com/Tencent/rapidjson) , V1.1.0

2.  代码中使用json，只需引入头文件即可，封装的API位于该头文件中：

   cmakelsit.txt:

   ```cmake
   find_package(rapidjson REQUIRED)
   
   ament_target_dependencies(${your_executable} rapidjson)
   ```

   package.xml:

   ```xml
   <depend>rapidjson</depend>
   ```

   source code:

   ```c++
   #include "cyberdog_common/cyberdog_json.hpp"
   
   # 另外，全部功能位于命名空间cyberdog::common之内.
   ```

3. 该封装只做了序列化场景的封装，并非全部API，这点要注意。主要包括Add(修改值), Get(获取值), ToString（序列化）, FromString(反序列化), ToFile(存储), FromFile(读取).

4. 若要进一步或直接使用rapidjson, 其源码位于"cyberdog_ws/third_party/rapidjson"，但不推荐这么做。如果发生第三方库的更换，其回归工作量可能较大。

5. 为了做基本的隔离，请使用"json"代替"rapidjson"命名空间

   ```c++
   # cyberdog_json.hpp
   namespace cyberdog
   {
   namespace common
   {
   /* 要求使用json代替rapidjson命名空间 */
   namespace json = rapidjson;
   }  // namespace common
   }  // namespace cyberdog
   
   # 使用时
   cyberdog::common::json Document d(cyberdog::common::json::kObjectType);
   # 当然你可以使用自己的别名，例如：
   namespace json = cyberdog::common::json;
   ```

6. 基本应用请参考json_test.cpp.

7. 更多参考：

   [rapidjson教程](http://rapidjson.org/zh-cn/index.html)

   [json协议规范](https://www.rfc-editor.org/info/rfc7159)



## Toml

1. 采用 [toml11](https://github.com/ToruNiina/toml11) , V3.7.0;

2. 引入方式同json：

3.  代码中使用json，只需引入头文件即可，封装的API位于该头文件中：

   cmakelsit.txt:

   ```cmake
   find_package(toml REQUIRED)
   
   ament_target_dependencies(${your_executable} toml)
   ```

   package.xml:

   ```xml
   <depend>toml</depend>
   ```

   source code:

   ```c++
   #include "cyberdog_common/cyberdog_toml.hpp"
   
   # 另外，全部功能位于命名空间cyberdog::common之内.
   ```

4. 若要进一步或直接使用toml11, 其源码位于"cyberdog_ws/third_party/toml"，但不推荐这么做。如果发生第三方库的更换，其回归工作量可能较大。

5. 基本应用请参考toml_test.cpp.

6. 更多参考：

   [toml11](https://github.com/ToruNiina/toml11)

   [toml协议](https://github.com/toml-lang/toml)

   [toml配置入门](https://mojotv.cn/2018/12/26/what-is-toml)


 
## Log

1、package.xml文件中添加：
``` xml
<build_depend>cyberdog_common</build_depend>
<exec_depend>cyberdog_common</exec_depend>
```
2、CMakeLists.txt文件添加： 
``` cmake
# log_test为要生成的节点，需要链接${cyberdog_log_LIBRARIES}
add_executable(log_test src/log_test.cpp)
target_link_libraries(log_test
    ${cyberdog_log_LIBRARIES}
)
target_include_directories(log_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
```
3、节点引用日志宏
``` cpp
#include "rclcpp/rclcpp.hpp"
// 包含日志封装头文件
#include "cyberdog_common/cyberdog_log.hpp"
#include "std_msgs/msg/string.hpp"

class LoggerUsage : public rclcpp::Node
{
public:
  LoggerUsage(): Node("logger_usage_demo"), count_(0)
  { //生成节点相关名称空间“logger_usage_demo”
    pub_ = create_publisher<std_msgs::msg::String>("logging_demo_count", 10);
    timer_ = create_wall_timer(500ms, std::bind(&LoggerUsage::on_timer, this));
    debug_function_to_evaluate_ = std::bind(&LoggerUsage::is_divisor_of_twelve, this, std::cref(count_));
  }
protected:
  void on_timer()
  {
    // 打印一次
    INFO_ONCE("Timer callback called (this will only log once)");
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Current count: " + std::to_string(count_);
    // 输出日志，INFO级别
    INFO("Publishing: '%s'", msg->data.c_str());
    pub_->publish(std::move(msg));
    // 函数返回true打印
    DEBUG_FUNCTION(
      &debug_function_to_evaluate_,
      "Count divides into 12 (function evaluated to true)");
    // 表达式返回true打印
    DEBUG_EXPRESSION(
      (count_ % 2) == 0, "Count is even (expression evaluated to true)");
    if (count_++ >= 15) {
      WARN("Reseting count to 0");
      count_ = 0;
    }    
  }
  bool is_divisor_of_twelve(size_t val)
  {
    if (val == 0) {
      ERROR("Modulo divisor cannot be 0");
      return false;
    }
    return (12 % val) == 0;
  }  
private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr one_shot_timer_, timer_;
  std::function<bool()> debug_function_to_evaluate_;
};

int main(int argc, char ** argv)
{
  //生成全局名称空间“Global_Name”
  LOGGER_MAIN_INSTANCE("Global_Name");
  // 输出日志，INFO级别
  INFO("hello world");
  auto node1 = std::make_shared<LoggerConfig>();
}
```   

