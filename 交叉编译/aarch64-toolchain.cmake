set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_SYSROOT /opt/aarch64-sysroot)

set(tools /usr/bin)
set(CMAKE_C_COMPILER ${tools}/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${tools}/aarch64-linux-gnu-g++)

get_filename_component(TOOLCHAIN_FILE_PATH "${CMAKE_CURRENT_LIST_FILE}" ABSOLUTE)
get_filename_component(TOOLCHAIN_DIR "${TOOLCHAIN_FILE_PATH}" DIRECTORY)

# message(STATUS "Toolchain file path: ${TOOLCHAIN_FILE_PATH}")
# message(STATUS "Toolchain dir: ${TOOLCHAIN_DIR}")

# NOTE `install_aarch64` is the install folder in the current workspace
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT} ${TOOLCHAIN_DIR}/install_aarch64)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# list(APPEND CMAKE_PREFIX_PATH )

# ---- pkg-config ----
set(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_SYSROOT})
set(ENV{PKG_CONFIG_LIBDIR} ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${CMAKE_SYSROOT}/usr/share/pkgconfig)
set(ENV{PKG_CONFIG_PATH} "")