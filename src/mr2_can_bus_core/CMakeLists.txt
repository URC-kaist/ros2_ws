cmake_minimum_required(VERSION 3.10)
project(mr2_can_bus_core LANGUAGES CXX)

# --- compiler flags / C++17 --------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- dependencies (only pthread for std::thread) -----------------------------
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

# --- header-only interface library -------------------------------------------
add_library(${PROJECT_NAME} INTERFACE)

# expose headers
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# link pthread (interface so consumers inherit it)
target_link_libraries(${PROJECT_NAME} INTERFACE Threads::Threads)

# ensure inline implementations are available to consumers
target_compile_definitions(${PROJECT_NAME} INTERFACE CAN_BUS_CORE_IMPL)

# --- installation ------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME})

# --- export so downstream packages can find the target -----------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
