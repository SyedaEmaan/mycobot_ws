cmake_minimum_required(VERSION 3.16)
project(mycobot_csp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# SOEM (built as shared lib, see README)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(SOEM_PC QUIET soem)
endif()

if(SOEM_PC_FOUND)
  set(SOEM_INCLUDE_DIRS ${SOEM_PC_INCLUDE_DIRS})
  set(SOEM_LIBRARIES    ${SOEM_PC_LIBRARIES})
  set(SOEM_LIBRARY_DIRS ${SOEM_PC_LIBRARY_DIRS})
else()
  find_path(SOEM_INCLUDE_DIR NAMES soem/soem.h
    HINTS ${SOEM_ROOT}/include $ENV{SOEM_ROOT}/include
          /usr/local/include /usr/include)
  find_library(SOEM_LIBRARY NAMES soem
    HINTS ${SOEM_ROOT}/lib $ENV{SOEM_ROOT}/lib
          /usr/local/lib /usr/lib)
  if(NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIBRARY)
    message(FATAL_ERROR
      "SOEM not found. Pass -DSOEM_ROOT=/path/to/install when invoking cmake.")
  endif()
  set(SOEM_INCLUDE_DIRS ${SOEM_INCLUDE_DIR})
  set(SOEM_LIBRARIES    ${SOEM_LIBRARY})
endif()

message(STATUS "SOEM include: ${SOEM_INCLUDE_DIRS}")
message(STATUS "SOEM library: ${SOEM_LIBRARIES}")

add_library(${PROJECT_NAME} SHARED src/mycobot_csp.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SOEM_INCLUDE_DIRS}
)

if(SOEM_LIBRARY_DIRS)
  target_link_directories(${PROJECT_NAME} PUBLIC ${SOEM_LIBRARY_DIRS})
endif()
target_link_libraries(${PROJECT_NAME} ${SOEM_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(
  hardware_interface mycobot_csp_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)
install(FILES mycobot_csp_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
