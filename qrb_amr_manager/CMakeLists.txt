cmake_minimum_required(VERSION 3.8)
project(qrb_amr_manager VERSION 1.0.4)

set(library_name qrb_amr_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)


include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${library_name} SHARED
  src/amr_manager.cpp
  src/amr_state_machine.cpp
  src/low_power_manager.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
)

install(TARGETS qrb_amr_manager
  EXPORT export_qrb_amr_manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_qrb_amr_manager)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_auto_package()

