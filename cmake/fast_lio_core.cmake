find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
add_library(fast_lio_core SHARED
  ${FAST_LIO_ROOT}/src/core/mapper.cpp
  ${FAST_LIO_ROOT}/src/core/input_buffer.cpp
  ${FAST_LIO_ROOT}/src/core/local_map.cpp
  ${FAST_LIO_ROOT}/src/core/registration.cpp
  ${FAST_LIO_ROOT}/src/core/runtime_log.cpp
  ${FAST_LIO_ROOT}/src/imu_process_initialization.cpp
  ${FAST_LIO_ROOT}/src/imu_process_deskew.cpp
  ${FAST_LIO_ROOT}/src/preprocess.cpp
  ${FAST_LIO_ROOT}/src/preprocess_livox.cpp
  ${FAST_LIO_ROOT}/src/preprocess_rotating.cpp
  ${FAST_LIO_ROOT}/src/preprocess_features.cpp
  ${FAST_LIO_ROOT}/src/preprocess_planes.cpp
  ${FAST_LIO_ROOT}/include/ikd-Tree/ikd_Tree.cpp)
target_compile_features(fast_lio_core PUBLIC cxx_std_17)
target_compile_definitions(fast_lio_core PRIVATE ROOT_DIR="${FAST_LIO_ROOT}/")
target_include_directories(fast_lio_core PUBLIC
  $<BUILD_INTERFACE:${FAST_LIO_ROOT}/include> $<INSTALL_INTERFACE:include>
  PRIVATE ${FAST_LIO_ROOT}/src)
target_link_libraries(fast_lio_core PUBLIC Eigen3::Eigen pcl_common
  PRIVATE ${PCL_LIBRARIES} OpenMP::OpenMP_CXX Threads::Threads)
set_target_properties(fast_lio_core PROPERTIES CXX_EXTENSIONS OFF)
install(TARGETS fast_lio_core EXPORT fast_lio_coreTargets LIBRARY DESTINATION lib)
install(DIRECTORY ${FAST_LIO_ROOT}/include/fast_lio DESTINATION include)

include(CMakePackageConfigHelpers)
configure_package_config_file(${FAST_LIO_ROOT}/cmake/fast_lio_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/fast_lio_coreConfig.cmake
  INSTALL_DESTINATION lib/cmake/fast_lio_core)
install(EXPORT fast_lio_coreTargets NAMESPACE fast_lio:: DESTINATION lib/cmake/fast_lio_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/fast_lio_coreConfig.cmake DESTINATION lib/cmake/fast_lio_core)

if(NOT DEFINED FAST_LIO_DEFAULT_MATCH_THREADS)
  set(FAST_LIO_DEFAULT_MATCH_THREADS 1)
endif()
set(FAST_LIO_MATCH_THREADS ${FAST_LIO_DEFAULT_MATCH_THREADS} CACHE STRING "Point matching workers")
if(NOT FAST_LIO_MATCH_THREADS MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "FAST_LIO_MATCH_THREADS must be a positive integer")
endif()
target_compile_definitions(fast_lio_core PRIVATE FAST_LIO_MATCH_THREADS=${FAST_LIO_MATCH_THREADS})
