cmake_minimum_required(VERSION 3.5)
project(odin_ros2_driver)

# ===== 平台架构检测 =====
# 自动检测当前主机架构（x86_64 或 ARM），用于选择对应的预编译 SDK 库
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(ARCH STREQUAL "x86_64")
    set(TARGET_PLATFORM "x86")
    message(STATUS "检测到 x86_64 架构")
elseif(ARCH MATCHES "arm|aarch64")
    set(TARGET_PLATFORM "arm")
    message(STATUS "检测到 ARM 架构: ${ARCH}")
else()
    message(WARNING "不支持的架构: ${ARCH}，使用默认设置")
    set(TARGET_PLATFORM "unknown")
endif()

# ===== 预编译 SDK 库路径 =====
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")
message(STATUS "SDK 库目录: ${LIB_DIR}")

# 根据平台选择对应库文件
if(TARGET_PLATFORM STREQUAL "arm")
    set(LYD_HOST_API_LIB_NAME "lydHostApi_arm")
else()
    set(LYD_HOST_API_LIB_NAME "lydHostApi_amd")
endif()

# 查找预编译 SDK 库
find_library(LYD_HOST_API_LIB
    NAMES
        ${LYD_HOST_API_LIB_NAME}
        lib${LYD_HOST_API_LIB_NAME}.a
        lib${LYD_HOST_API_LIB_NAME}.so
    PATHS ${LIB_DIR}
    NO_DEFAULT_PATH
)

if(LYD_HOST_API_LIB)
    message(STATUS "找到 lydHostApi 库: ${LYD_HOST_API_LIB}")
else()
    file(GLOB LIB_FILES "${LIB_DIR}/lib${LYD_HOST_API_LIB_NAME}.*")
    if(LIB_FILES)
        message(STATUS "找到库文件: ${LIB_FILES}")
        set(LYD_HOST_API_LIB ${LIB_FILES})
    else()
        message(FATAL_ERROR "在 ${LIB_DIR} 中未找到预编译的 lydHostApi 库")
    endif()
endif()

# ===== 编译选项 =====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# ===== 公共依赖 =====
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# 公共头文件目录
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIR}
    ${LIBUSB_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 公共链接库
set(COMMON_LIBS
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
    ${OPENSSL_LIBRARIES}
    ${LIBUSB_LIBRARIES}
    pthread
    rt
    ${CMAKE_DL_LIBS}
    ${LYD_HOST_API_LIB}
)

# ===== ROS2 Humble 配置 =====
message(STATUS "配置 ROS2 (Ubuntu 22.04 / Humble) 构建")

# 声明 ROS2 宏，用于源码中的条件编译
add_definitions(-DROS2)

# 查找所有必要的 ROS2 包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ===== 主驱动节点：host_sdk_sample =====
# 负责与 Odin1 设备通信、发布点云/图像/IMU/里程计等话题
add_executable(host_sdk_sample
    src/host_sdk_sample.cpp
    src/yaml_parser.cpp
    src/rawCloudRender.cpp
    src/camera_pose_visualization.cpp
)

target_link_libraries(host_sdk_sample
    ${COMMON_LIBS}
    yaml-cpp
    usb-1.0
)

ament_target_dependencies(host_sdk_sample
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    cv_bridge
    image_transport
    tf2
    tf2_ros
    geometry_msgs
    tf2_geometry_msgs
)

# ===== 深度图生成模块 =====
# pointcloud_depth_converter_ros2：点云转深度图的核心算法库
add_library(pointcloud_depth_converter_ros2 src/pointcloud_depth_converter.cpp)
target_link_libraries(pointcloud_depth_converter_ros2
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

# depth_image_ros2_node_lib：深度图 ROS2 节点逻辑库
add_library(depth_image_ros2_node_lib src/depth_image_ros2_node.cpp)
target_link_libraries(depth_image_ros2_node_lib
    pointcloud_depth_converter_ros2
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

ament_target_dependencies(depth_image_ros2_node_lib
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    pcl_conversions
    message_filters
)

# pcd2depth_ros2_node：深度图生成节点入口
add_executable(pcd2depth_ros2_node src/pcd2depth_ros2.cpp)
target_link_libraries(pcd2depth_ros2_node
    depth_image_ros2_node_lib
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp
)

ament_target_dependencies(pcd2depth_ros2_node
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    pcl_conversions
    message_filters
)

# ===== 点云重投影模块 =====
# cloud_reprojector_ros2：SLAM 点云投影到相机图像的核心算法库
add_library(cloud_reprojector_ros2 src/cloud_reprojector.cpp)
target_compile_definitions(cloud_reprojector_ros2 PRIVATE ROS2)
target_link_libraries(cloud_reprojector_ros2
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

# cloud_reprojection_ros2_node：点云重投影节点入口
add_executable(cloud_reprojection_ros2_node src/cloud_reprojection_ros.cpp)
target_compile_definitions(cloud_reprojection_ros2_node PRIVATE ROS2)
target_link_libraries(cloud_reprojection_ros2_node
    cloud_reprojector_ros2
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp
)
ament_target_dependencies(cloud_reprojection_ros2_node
    rclcpp
    sensor_msgs
    nav_msgs
    cv_bridge
    image_transport
    pcl_conversions
    message_filters
)

# ===== 安装规则 =====
# 安装可执行文件
install(TARGETS
    host_sdk_sample
    pcd2depth_ros2_node
    cloud_reprojection_ros2_node
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装 package.xml
install(FILES package.xml
    DESTINATION share/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
    DESTINATION include
)

# 安装 launch 文件
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 config 配置文件
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)


ament_export_targets(export_${PROJECT_NAME})

# 声明对外依赖
ament_export_dependencies(
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    cv_bridge
    image_transport
    pcl_conversions
    message_filters
    tf2
    tf2_ros
    geometry_msgs
    tf2_geometry_msgs
)

ament_package()

# ===== ARM 平台特定链接选项 =====
# ARM 设备（如 Jetson、树莓派）需要特殊的链接标志和 RPATH
if(TARGET_PLATFORM STREQUAL "arm")
    set_target_properties(host_sdk_sample PROPERTIES
        LINK_FLAGS "-Wl,--no-as-needed -Wl,--rpath=${LIB_DIR}"
    )
    message(STATUS "添加 ARM 专用链接选项和 RPATH")
endif()

# ===== 构建信息汇总 =====
message(STATUS "=======================================")
message(STATUS "项目: ${PROJECT_NAME}")
message(STATUS "目标平台: ${TARGET_PLATFORM}")
message(STATUS "lydHostApi 库: ${LYD_HOST_API_LIB}")
message(STATUS "libusb 库: ${LIBUSB_LIBRARIES}")
message(STATUS "=======================================")
