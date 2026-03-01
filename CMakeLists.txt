cmake_minimum_required(VERSION 3.17)

#include(${CMAKE_CURRENT_LIST_DIR}/../../../hw/bsp/family_support.cmake)

include(pico_sdk_import.cmake)
project(picowebserverspi)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(BROWSERIO_XTERM_SOURCE "CDN" CACHE STRING "xterm.js source: CDN or LOCAL")
set_property(CACHE BROWSERIO_XTERM_SOURCE PROPERTY STRINGS CDN LOCAL)

pico_sdk_init()

add_executable(picowebserverspi)

# gets PROJECT name for the example (e.g. <BOARD>-<DIR_NAME>)
#family_get_project_name(PROJECT ${CMAKE_CURRENT_LIST_DIR})

# Prefer the tinyusb lwip
#set(LWIP ${TOP}/lib/lwip)
set(LWIP ${PICO_SDK_PATH}/lib/lwip)
set(TOP ${PICO_SDK_PATH}/lib/tinyusb)

# If we can't find one from tinyusb then check cmake var before giving up
if (NOT EXISTS ${LWIP}/src)
  set(LWIP ${TINYUSB_LWIP_PATH})
endif()

#if (NOT EXISTS ${LWIP}/src)
#  family_example_missing_dependency(${PROJECT} "lib/lwip")
#  return()
#endif()

#project(${PROJECT} C CXX ASM)

# Checks this example is valid for the family and initializes the project
#family_initialize_project(${PROJECT} ${CMAKE_CURRENT_LIST_DIR})
#add_executable(${PROJECT})

# Make sure TinyUSB can find tusb_config.h
target_include_directories(picowebserverspi PUBLIC
${CMAKE_CURRENT_LIST_DIR}/src)

# Example source
target_sources(picowebserverspi PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/src/main.c
  ${CMAKE_CURRENT_LIST_DIR}/src/usb_descriptors.c
  ${CMAKE_CURRENT_LIST_DIR}/src/web_server.c
  ${CMAKE_CURRENT_LIST_DIR}/src/app_router.c
  ${CMAKE_CURRENT_LIST_DIR}/src/serial_app.c
  ${CMAKE_CURRENT_LIST_DIR}/src/proto_uart.c
  ${CMAKE_CURRENT_LIST_DIR}/src/proto_spi.c
  ${CMAKE_CURRENT_LIST_DIR}/src/proto_i2c.c
  ${CMAKE_CURRENT_LIST_DIR}/src/proto_emmc.c
  ${CMAKE_CURRENT_LIST_DIR}/src/http_custom_files.c
  ${CMAKE_CURRENT_LIST_DIR}/src/xterm_assets.c
  )

if(BROWSERIO_XTERM_SOURCE STREQUAL "LOCAL")
  set(BROWSERIO_XTERM_JS_PRIMARY "/xterm.js")
  set(BROWSERIO_XTERM_JS_FALLBACK "/xterm.js")
  set(BROWSERIO_XTERM_CSS "/xterm.css")
else()
  set(BROWSERIO_XTERM_JS_PRIMARY "https://unpkg.com/xterm@5.3.0/lib/xterm.js")
  set(BROWSERIO_XTERM_JS_FALLBACK "https://unpkg.com/xterm/lib/xterm.js")
  set(BROWSERIO_XTERM_CSS "https://unpkg.com/xterm@5.3.0/css/xterm.css")
endif()

set(BROWSERIO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(BROWSERIO_INDEX_HDR ${BROWSERIO_GEN_DIR}/index_html_data.h)

add_custom_command(
  OUTPUT ${BROWSERIO_INDEX_HDR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${BROWSERIO_GEN_DIR}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/tools/embed_html.py
          --input ${CMAKE_CURRENT_LIST_DIR}/src/web/index.html
          --output ${BROWSERIO_INDEX_HDR}
          --define XTERM_JS_PRIMARY=${BROWSERIO_XTERM_JS_PRIMARY}
          --define XTERM_JS_FALLBACK=${BROWSERIO_XTERM_JS_FALLBACK}
          --define XTERM_CSS_URL=${BROWSERIO_XTERM_CSS}
  DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/src/web/index.html
    ${CMAKE_CURRENT_LIST_DIR}/tools/embed_html.py
  VERBATIM
)

add_custom_target(generate_web_assets DEPENDS ${BROWSERIO_INDEX_HDR})
add_dependencies(picowebserverspi generate_web_assets)

if(BROWSERIO_XTERM_SOURCE STREQUAL "LOCAL")
  target_compile_definitions(picowebserverspi PUBLIC
    BROWSERIO_XTERM_SOURCE_LOCAL=1
    BROWSERIO_XTERM_JS_URL_PRIMARY=\"${BROWSERIO_XTERM_JS_PRIMARY}\"
    BROWSERIO_XTERM_JS_URL_FALLBACK=\"${BROWSERIO_XTERM_JS_FALLBACK}\"
    BROWSERIO_XTERM_CSS_URL=\"${BROWSERIO_XTERM_CSS}\")
else()
  target_compile_definitions(picowebserverspi PUBLIC
    BROWSERIO_XTERM_SOURCE_LOCAL=0
    BROWSERIO_XTERM_JS_URL_PRIMARY=\"${BROWSERIO_XTERM_JS_PRIMARY}\"
    BROWSERIO_XTERM_JS_URL_FALLBACK=\"${BROWSERIO_XTERM_JS_FALLBACK}\"
    BROWSERIO_XTERM_CSS_URL=\"${BROWSERIO_XTERM_CSS}\")
endif()

# Example include
target_include_directories(picowebserverspi PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/src
  ${BROWSERIO_GEN_DIR}
  ${LWIP}/src/include
  ${LWIP}/src/include/ipv4
  ${LWIP}/src/include/lwip/apps
  ${TOP}/lib/networking
  )

# lib/networking sources
target_sources(picowebserverspi PUBLIC
  ${TOP}/lib/networking/dhserver.c
  ${TOP}/lib/networking/dnserver.c
  ${TOP}/lib/networking/rndis_reports.c
  )

# lwip sources
target_sources(picowebserverspi PUBLIC
  ${LWIP}/src/core/altcp.c
  ${LWIP}/src/core/altcp_alloc.c
  ${LWIP}/src/core/altcp_tcp.c
  ${LWIP}/src/core/def.c
  ${LWIP}/src/core/dns.c
  ${LWIP}/src/core/inet_chksum.c
  ${LWIP}/src/core/init.c
  ${LWIP}/src/core/ip.c
  ${LWIP}/src/core/mem.c
  ${LWIP}/src/core/memp.c
  ${LWIP}/src/core/netif.c
  ${LWIP}/src/core/pbuf.c
  ${LWIP}/src/core/raw.c
  ${LWIP}/src/core/stats.c
  ${LWIP}/src/core/sys.c
  ${LWIP}/src/core/tcp.c
  ${LWIP}/src/core/tcp_in.c
  ${LWIP}/src/core/tcp_out.c
  ${LWIP}/src/core/timeouts.c
  ${LWIP}/src/core/udp.c
  ${LWIP}/src/core/ipv4/autoip.c
  ${LWIP}/src/core/ipv4/dhcp.c
  ${LWIP}/src/core/ipv4/etharp.c
  ${LWIP}/src/core/ipv4/icmp.c
  ${LWIP}/src/core/ipv4/igmp.c
  ${LWIP}/src/core/ipv4/ip4.c
  ${LWIP}/src/core/ipv4/ip4_addr.c
  ${LWIP}/src/core/ipv4/ip4_frag.c
  ${LWIP}/src/netif/ethernet.c
  ${LWIP}/src/netif/slipif.c
  ${LWIP}/src/apps/http/httpd.c
  ${LWIP}/src/apps/http/fs.c
  ${LWIP}/src/apps/lwiperf/lwiperf.c
  )

target_link_libraries(picowebserverspi PUBLIC pico_stdlib hardware_spi tinyusb_device tinyusb_board)

# due to warnings from other net source, we need to prevent error from some of the warnings options
#if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
  #target_compile_options(${PROJECT} PUBLIC
    #-Wno-error=null-dereference
    #-Wno-error=conversion
    #-Wno-error=sign-conversion
#    -Wno-error=sign-compare
 #   )
#elseif (CMAKE_C_COMPILER_ID STREQUAL "IAR")
#endif ()

# Configure compilation flags and libraries for the example without RTOS.
# See the corresponding function in hw/bsp/FAMILY/family.cmake for details.
#family_configure_device_example(${PROJECT} noos)

pico_add_extra_outputs(picowebserverspi)
