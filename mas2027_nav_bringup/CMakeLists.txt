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

  # ROG-Map 参数只有一份：nav2_params.yaml 的 MincoPlanner.rog_map 段。该段是从上游
  # navi_minco_bit 整段搬来的 80 项，其中几项按本仓库链路改过（frame_id、话题名）。
  # 这个测试把它们钉住，防止以后同步上游时静默覆盖回 camera_init / /aft_mapped_to_init。
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
