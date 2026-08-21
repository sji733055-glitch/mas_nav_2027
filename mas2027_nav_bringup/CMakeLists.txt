cmake_minimum_required(VERSION 3.8)
project(mas2027_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()

  # ROG-Map 的同一套参数必须在 rog_map_params.yaml（独立节点）和 nav2_params.yaml
  # 的 MincoPlanner.rog_map 段（导航实际生效）里保持一致。ROS 2 参数文件没有
  # 包含/引用语法，两份无法合并，所以用测试把"改了一份忘了另一份"变成构建失败。
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rog_map_params_consistency
    test/test_rog_map_params_consistency.py
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  pcd
  config
  behavior_trees
  rviz
)
