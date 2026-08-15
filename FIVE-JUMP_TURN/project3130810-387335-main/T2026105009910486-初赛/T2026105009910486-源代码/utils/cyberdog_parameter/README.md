<center> <font size='9' color='green'> How to use cyberdog_parameter </font>  </center>

## 1 CMake文件

添加cmake依赖

```cmake
# 添加依赖
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(cyberdog_common REQUIRED) 
find_package(cyberdog_parameter REQUIRED) 
find_package(toml REQUIRED)

ament_register_extension("ament_package" "cyberdog_parameter"
  "functions.cmake"
)

project_initialize_config_parameter()
set(dependencies
  PythonInterp
  PythonLibs
  cyberdog_parameter
  toml
  ament_index_cpp
)

include_directories(include ${PYTHON_INCLUDE_DIRS})

ament_target_dependencies(
  test_sodemo # 工程的名字
  ${dependencies}
)
```



<font color='red'>**Note**</font>

具体细节请移步 [test_readdynamiclibrary](https://git.n.xiaomi.com/MiRoboticsLab/rop/utils/-/tree/dev/test_readdynamiclibrary)工程



## 2 使用案例

```c++
void TestCase5()
{
  std::string path = ament_index_cpp::get_package_prefix("test_readdynamiclibrary") + 
    "/lib/test_readdynamiclibrary/";
  std::string filename = "test_toml.so";
  ParameterParser parser(path, filename);

  // toml format
  // a = 10
  // b = "cyberdog"
  // c = true
  // d = 0.1
  // aa = ["foo", "bar", "baz"]
  // bb = [1.1, 0.5, 880.123]
  // cc = [1, 5, 8]
  int a = parser.GetInt("a");
  std::string b = parser.GetString("b");
  bool c = parser.GetBool("c");
  double d = parser.GetDouble("d");
  std::vector<std::string> aa = parser.GetArrayValuesAsStrings("aa");
  std::cout << "a = " << a << std::endl
            << "b = " << b << std::endl
            << "c = " << c << std::endl
            << "d = " << d << std::endl;
  for (auto str : aa) {
    std::cout << str << " ";
  }

  std::cout << std::endl;
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

	TestCase5();
    return 0;
}
```

<font color='red'>**Note**</font>

请使用 [test_readdynamiclibrary](https://git.n.xiaomi.com/MiRoboticsLab/rop/utils/-/tree/dev/test_readdynamiclibrary)工程的TestCase5()案例方式，使用cyberdog_parameter库获取参数



toml文件

```toml
a = 10
b = "cyberdog"
c = true
d = 0.1
aa = ["foo", "bar", "baz"]
bb = [1.1, 0.5, 880.123]
cc = [1, 5, 8]

[A]
e = 2
f = "cyberdog_A"
g = true
h = 0.2

[A.c]
e = 22222
f = "cyberdog_c"
g = false
h = 0.2222

[[B]]
a = 3
b = "cyberdog3"
c = true
d = 33.6

[[B]]
a = 4
b = "robotlab4"
c = false
d = 44.2

[[C]]
a = 5555
b = "cyberdogCCCC"
c = true
d = 555.6

[[C]]
a = 666
b = "robotlabCCCCC"
c = true
d = 666.203
```



输出结果

----

```bash
# 运行测试案例
ros2 run test_readdynamiclibrary test_sodemo                      

a = 10
b = cyberdog
c = 1
d = 0.1
foo bar baz 
```



## 3 cyberdog_parameter库解析原始toml

如果cyberdog_parameter封装的API不支持toml源码解析，那么使用原始的toml的API进行解析。

那么，只是目前的配置文件依然是动态库so格式

```c++
void TestCase6()
{
  // toml format
  // [A.c]
  // e = 22222
  // f = "cyberdog_c"
  // g = false
  // h = 0.2222

  std::string path = ament_index_cpp::get_package_prefix("test_readdynamiclibrary") + 
    "/lib/test_readdynamiclibrary/";
  std::string filename = "test_toml.so";
  ParameterParser parser(path, filename);

  auto toml_parser = parser.GetParameterHandler();

  int e = toml::find<int>(toml_parser, "A", "c", "e");
  std::string f = toml::find<std::string>(toml_parser, "A","c",  "f");
  bool g = toml::find<bool>(toml_parser, "A", "c", "g");
  double h = toml::find<double>(toml_parser, "A", "c", "h");

  std::cout << "e = " << e << std::endl
            << "f = " << f << std::endl
            << "g = " << g << std::endl
            << "h = " << h << std::endl;

}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    // cyberdog::parameter::TestCase1();
    // cyberdog::parameter::TestCase2();
    // cyberdog::parameter::TestCase3();
    // cyberdog::parameter::TestCase4();
    // cyberdog::parameter::TestCase5();
    cyberdog::parameter::TestCase6();
    return 0;
}
```

toml文件

```toml
[A.c]
e = 22222
f = "cyberdog_c"
g = false
h = 0.2222
```

输出结果

```bash
# 运行测试案例
ros2 run test_readdynamiclibrary test_sodemo  

e = 22222
f = cyberdog_c
g = 0
h = 0.2222
```





