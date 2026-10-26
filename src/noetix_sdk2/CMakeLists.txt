cmake_minimum_required(VERSION 3.16)
project(noetix_sdk2 VERSION 0.3.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(NOETIX_YAML_TARGET yaml-cpp::yaml-cpp)
else()
  set(NOETIX_YAML_TARGET yaml-cpp)
endif()
include(GNUInstallDirs)

# Select vendor archives from the target architecture. "auto" is the normal
# setting for native packages. An explicit value is useful for a cross-build
# with a matching CMake toolchain file.
set(NOETIX_TARGET_ARCH "auto" CACHE STRING
    "Target vendor architecture: auto, x86_64 or aarch64")
set_property(CACHE NOETIX_TARGET_ARCH PROPERTY STRINGS auto x86_64 aarch64)
option(NOETIX_REQUIRE_N3_HW
    "Fail configuration when the N3 real-hardware backend is unavailable" OFF)

if(NOETIX_TARGET_ARCH STREQUAL "auto")
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" NOETIX_PROCESSOR_LOWER)
  if(NOETIX_PROCESSOR_LOWER MATCHES "^(x86_64|amd64)$")
    set(NOETIX_TARGET_ARCH_RESOLVED "x86_64")
  elseif(NOETIX_PROCESSOR_LOWER MATCHES "^(aarch64|arm64)$")
    set(NOETIX_TARGET_ARCH_RESOLVED "aarch64")
  else()
    set(NOETIX_TARGET_ARCH_RESOLVED "unsupported")
  endif()
elseif(NOETIX_TARGET_ARCH STREQUAL "x86_64" OR
       NOETIX_TARGET_ARCH STREQUAL "aarch64")
  set(NOETIX_TARGET_ARCH_RESOLVED "${NOETIX_TARGET_ARCH}")
else()
  message(FATAL_ERROR
    "Invalid NOETIX_TARGET_ARCH='${NOETIX_TARGET_ARCH}'. "
    "Expected auto, x86_64 or aarch64.")
endif()

set(NOETIX_X86_64 FALSE)
set(NOETIX_AARCH64 FALSE)
if(NOETIX_TARGET_ARCH_RESOLVED STREQUAL "x86_64")
  set(NOETIX_X86_64 TRUE)
elseif(NOETIX_TARGET_ARCH_RESOLVED STREQUAL "aarch64")
  set(NOETIX_AARCH64 TRUE)
endif()
message(STATUS
  "Noetix target architecture: ${NOETIX_TARGET_ARCH_RESOLVED} "
  "(CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR})")

set(NOETIX_VENDOR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third-party/vendor)
set(N3_HW_BASE ${NOETIX_VENDOR_ROOT}/hw_controller/n3)
set(N3_HW_ROOT ${N3_HW_BASE}/${NOETIX_TARGET_ARCH_RESOLVED})
set(N3_HW_INCLUDE_DIR ${N3_HW_BASE}/include)
set(BUMI_HW_BASE ${NOETIX_VENDOR_ROOT}/hw_controller/bumi)
set(BUMI_HW_ROOT ${BUMI_HW_BASE}/${NOETIX_TARGET_ARCH_RESOLVED})
# The new Bumi hw uses one common header/API for x86_64 and aarch64.
set(BUMI_HW_INCLUDE_DIR ${BUMI_HW_BASE}/include)
# Parallel-ankle solvers are family-specific and must not be mixed.
set(N3_ANKLE_ROOT ${NOETIX_VENDOR_ROOT}/parallel_ankle/n3)
set(BUMI_ANKLE_ROOT ${NOETIX_VENDOR_ROOT}/parallel_ankle/bumi)

set(N3_HW_LIBRARY ${N3_HW_ROOT}/lib/libhw_controller.a)
set(BUMI_HW_LIBRARY ${BUMI_HW_ROOT}/lib/libhw_controller.a)
set(BUMI_VENDOR_SOEM_LIBRARY ${BUMI_HW_ROOT}/lib/libsoem.a)
set(PARALLEL_LIBRARY
    ${BUMI_ANKLE_ROOT}/lib/${NOETIX_TARGET_ARCH_RESOLVED}/libparallel_ankle_cpp.a)

set(N3_REAL_HW FALSE)
set(BUMI_REAL_HW FALSE)

# N3 uses its own vendor API on both architectures. An aarch64 archive must
# be supplied separately and must match the N3 headers, not the Bumi API.
set(N3_HW_UNAVAILABLE_REASON "")
if(NOT (NOETIX_X86_64 OR NOETIX_AARCH64))
  set(N3_HW_UNAVAILABLE_REASON
      "unsupported target architecture '${CMAKE_SYSTEM_PROCESSOR}'. ")
else()
  foreach(N3_REQUIRED_FILE IN ITEMS
      "${N3_HW_LIBRARY}"
      "${N3_HW_INCLUDE_DIR}/hw_controller.h"
      "${N3_HW_INCLUDE_DIR}/common_type.h")
    if(NOT EXISTS "${N3_REQUIRED_FILE}")
      string(APPEND N3_HW_UNAVAILABLE_REASON
          "missing ${N3_REQUIRED_FILE} (target=${NOETIX_TARGET_ARCH_RESOLVED}). ")
    endif()
  endforeach()
endif()

if(N3_HW_UNAVAILABLE_REASON STREQUAL "")
  set(N3_REAL_HW TRUE)
elseif(NOETIX_REQUIRE_N3_HW)
  message(FATAL_ERROR
    "N3 real-hardware backend is required but unavailable: "
    "${N3_HW_UNAVAILABLE_REASON}"
    "Provide the matching N3 vendor archive and headers. Bumi archives are not compatible.")
endif()

# All Bumi models use the same new hw API. Only the binary archive changes
# with the target CPU architecture.
if((NOETIX_X86_64 OR NOETIX_AARCH64)
   AND EXISTS ${BUMI_HW_LIBRARY}
   AND EXISTS ${BUMI_HW_INCLUDE_DIR}/hw_controller.h
   AND EXISTS ${BUMI_HW_INCLUDE_DIR}/common_type.h
   AND EXISTS ${PARALLEL_LIBRARY}
   AND EXISTS ${BUMI_ANKLE_ROOT}/include/parallel_solve/parallel_ankle.h)
  set(BUMI_REAL_HW TRUE)
endif()

# Prefer the SOEM archive shipped with the selected new Bumi hw package.
# If the x86_64 package only provides libhw_controller.a, use the in-tree SOEM.
set(BUMI_USE_VENDOR_SOEM FALSE)
if(BUMI_REAL_HW AND EXISTS ${BUMI_VENDOR_SOEM_LIBRARY})
  set(BUMI_USE_VENDOR_SOEM TRUE)
endif()

if(N3_REAL_HW OR (BUMI_REAL_HW AND NOT BUMI_USE_VENDOR_SOEM))
  add_subdirectory(third-party/SOEM)
  set_target_properties(soem PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# N3 and Bumi vendor archives export overlapping legged::HwController symbols
# with different ABIs. Keep them isolated in separate hardware DSOs.
if(N3_REAL_HW)
  add_library(noetix_n3_hardware SHARED
    src/n3/n3_controller.cpp
    src/n3/n3_backend.cpp
    ${N3_ANKLE_ROOT}/FK.cpp
    ${N3_ANKLE_ROOT}/IK.cpp
    ${N3_ANKLE_ROOT}/Jacobian.cpp
    ${N3_ANKLE_ROOT}/coeffs.cpp
    ${N3_ANKLE_ROOT}/parallel_ankle.cpp
  )
  target_include_directories(noetix_n3_hardware PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${N3_HW_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/n3
    ${N3_ANKLE_ROOT}
  )
  target_link_libraries(noetix_n3_hardware PRIVATE
    ${NOETIX_YAML_TARGET}
    ${N3_HW_LIBRARY}
    soem
  )
  target_link_options(noetix_n3_hardware PRIVATE
    "-Wl,--exclude-libs,ALL"
    "-Wl,--no-undefined"
  )
  target_compile_features(noetix_n3_hardware PUBLIC cxx_std_17)
  message(STATUS
    "N3 real-hardware backend enabled: arch=${NOETIX_TARGET_ARCH_RESOLVED}, "
    "vendor=${N3_HW_LIBRARY}")
else()
  message(STATUS
    "N3 real-hardware backend not built: ${N3_HW_UNAVAILABLE_REASON}"
    "Use -DNOETIX_REQUIRE_N3_HW=ON for N3 deployments to fail early.")
endif()

if(BUMI_REAL_HW)
  if(BUMI_USE_VENDOR_SOEM)
    set(BUMI_SOEM_LINK ${BUMI_VENDOR_SOEM_LIBRARY})
  else()
    set(BUMI_SOEM_LINK soem)
  endif()

  add_library(noetix_bumi_hardware SHARED
    src/bumi/bumi_controller.cpp
    src/bumi/bumi_backend.cpp
    src/bumi/usb_imu_reader.cpp
  )
  target_include_directories(noetix_bumi_hardware PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${BUMI_HW_INCLUDE_DIR}
    ${BUMI_ANKLE_ROOT}/include
  )
  target_link_libraries(noetix_bumi_hardware PRIVATE
    ${NOETIX_YAML_TARGET}
    ${BUMI_HW_LIBRARY}
    ${PARALLEL_LIBRARY}
    ${BUMI_SOEM_LINK}
    pthread
    rt
  )
  target_link_options(noetix_bumi_hardware PRIVATE
    "-Wl,--exclude-libs,ALL"
    "-Wl,--no-undefined"
  )
  target_compile_features(noetix_bumi_hardware PUBLIC cxx_std_17)
  message(STATUS
    "Bumi real-hardware backend enabled: arch=${NOETIX_TARGET_ARCH_RESOLVED}, "
    "vendor=${BUMI_HW_ROOT}")
else()
  if(NOETIX_X86_64 OR NOETIX_AARCH64)
    message(STATUS
      "Bumi real-hardware backend not built. Expected new unified hw archive: "
      "${BUMI_HW_LIBRARY}")
  else()
    message(STATUS "Bumi real-hardware backend not built; unsupported target architecture.")
  endif()
endif()

add_library(noetix_sdk2 SHARED
  src/robot_factory.cpp
  src/robot/mapped_robot.cpp
)
add_library(noetix::noetix_sdk2 ALIAS noetix_sdk2)

target_compile_features(noetix_sdk2 PUBLIC cxx_std_17)
target_include_directories(noetix_sdk2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(noetix_sdk2
  PRIVATE
    ${NOETIX_YAML_TARGET}
)

if(N3_REAL_HW)
  target_link_libraries(noetix_sdk2 PRIVATE noetix_n3_hardware)
  target_compile_definitions(noetix_sdk2 PRIVATE NOETIX_HAS_N3_HW=1)
else()
  target_compile_definitions(noetix_sdk2 PRIVATE
    NOETIX_HAS_N3_HW=0
    NOETIX_N3_HW_UNAVAILABLE_REASON="${N3_HW_UNAVAILABLE_REASON}"
  )
endif()

if(BUMI_REAL_HW)
  target_link_libraries(noetix_sdk2 PRIVATE noetix_bumi_hardware)
  target_compile_definitions(noetix_sdk2 PRIVATE NOETIX_HAS_BUMI_HW=1)
else()
  target_compile_definitions(noetix_sdk2 PRIVATE NOETIX_HAS_BUMI_HW=0)
endif()

set(NOETIX_INSTALL_TARGETS noetix_sdk2)
if(N3_REAL_HW)
  list(APPEND NOETIX_INSTALL_TARGETS noetix_n3_hardware)
endif()
if(BUMI_REAL_HW)
  list(APPEND NOETIX_INSTALL_TARGETS noetix_bumi_hardware)
endif()

install(TARGETS ${NOETIX_INSTALL_TARGETS}
  EXPORT noetix_sdk2Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/noetix/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/noetix
)

install(EXPORT noetix_sdk2Targets
  FILE noetix_sdk2Targets.cmake
  NAMESPACE noetix::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/noetix_sdk2
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/noetix_sdk2Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/noetix_sdk2Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/noetix_sdk2
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/noetix_sdk2ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/noetix_sdk2Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/noetix_sdk2ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/noetix_sdk2
)

ament_export_include_directories(include)
ament_export_targets(noetix_sdk2Targets HAS_LIBRARY_TARGET)
ament_package()
