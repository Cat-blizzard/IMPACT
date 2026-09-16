cmake_minimum_required(VERSION 3.8)
project(xq_gz_assets)

find_package(ament_cmake REQUIRED)

add_library(impact_egl_terminate_guard SHARED
  src/impact_egl_terminate_guard.c
)
target_link_libraries(impact_egl_terminate_guard PRIVATE ${CMAKE_DL_LIBS})

install(
  TARGETS impact_egl_terminate_guard
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY config models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/xq_gz_assets.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/xq_gz_assets.sh.in"
)

ament_package()
