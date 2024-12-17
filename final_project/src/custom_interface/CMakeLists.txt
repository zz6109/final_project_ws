cmake_minimum_required(VERSION 3.8)
project(custom_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # 서비스/액션 생성기
find_package(builtin_interfaces REQUIRED)        # 기본 메시지 타입
find_package(rosidl_default_runtime REQUIRED)    # 런타임 종속성

# 서비스 파일 및 액션 파일 정의 추가
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmService.srv"            # 서비스 파일 경로
  "action/StopOnObstacle.action"  # 액션 파일 경로
  DEPENDENCIES builtin_interfaces # 종속성 추가
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
