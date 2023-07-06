cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(linux-example-btstack-a2dp-source VERSION 1.0.0 DESCRIPTION "Linux CE A2DP Source Application")

# specify the C++ standard
set (CMAKE_C_STANDARD 99)
set (CMAKE_C_STANDARD_REQUIRED True)

set (BUILD_SHARED_LIBS ON)
set (BTSTACK_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../btstack/wiced_include)
set (BTSTACK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../btstack/stack/COMPONENT_WICED_DUALMODE/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)
set (PORTING_LAYER ${CMAKE_CURRENT_SOURCE_DIR}/../bluetooth-linux)
set (BT_PROFILE_A2DP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../bt-audio-profiles/COMPONENT_a2dp_source_profile)
set (BT_AUDIO_PROFILES_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../bt-audio-profiles/profiles_include)
set (BT_AUDIO_PROFILES_UTILS ${CMAKE_CURRENT_SOURCE_DIR}/../bt-audio-profiles/utils)
set (SBC_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../bt-audio-profiles/sbc/COMPONENT_ARMv8_LINUX/COMPONENT_GCC)
set (SBC_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../bt-audio-profiles/sbc/include)

add_definitions(-DPLATFORM_LED_DISABLED -DLINUX_PLATFORM)

# control where the static and shared libraries are built so that on windows
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${BTSTACK_INCLUDE}/)
include_directories(${PORTING_LAYER}/)
include_directories(${BT_AUDIO_PROFILES_INCLUDE}/)
include_directories(${BT_AUDIO_PROFILES_UTILS}/)
include_directories(${BT_PROFILE_A2DP_SRC}/)
include_directories(${SBC_INCLUDE}/)
include_directories(${PORTING_LAYER}/wiced_hal/)

link_directories(${BTSTACK_LIB}/)
link_directories(${SBC_LIB}/)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/app_bt_config/wiced_bt_cfg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_bt_utils/app_bt_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/a2dp_source.c
    ${PORTING_LAYER}/patch_download.c
    ${PORTING_LAYER}/wiced_bt_app.c
    ${PORTING_LAYER}/hci_uart_linux.c
    ${PORTING_LAYER}/linux_main.c
    ${PORTING_LAYER}/spy_ux.c
    ${PORTING_LAYER}/linux_tcp_server.c
    ${PORTING_LAYER}/wiced_hal/wiced_hal_nvram.c
    ${PORTING_LAYER}/nvram.c
    ${PORTING_LAYER}/utils_arg_parser.c 
    ${BT_PROFILE_A2DP_SRC}/wiced_bt_a2dp_src_sbc.c
    ${BT_PROFILE_A2DP_SRC}/wiced_bt_a2dp_src_main.c
    ${BT_PROFILE_A2DP_SRC}/wiced_bt_a2dp_src_api.c
    ${BT_PROFILE_A2DP_SRC}/wiced_bt_a2dp_src_ssm.c
    ${BT_PROFILE_A2DP_SRC}/wiced_bt_a2dp_src_act.c
    ${BT_PROFILE_A2DP_SRC}/wiced_bt_a2dp_src_cfg.c
    ${BT_AUDIO_PROFILES_UTILS}/wiced_bt_utils.c
)

target_link_libraries(${PROJECT_NAME} PRIVATE btstack)
target_link_libraries(${PROJECT_NAME} PRIVATE pthread rt)
target_link_libraries(${PROJECT_NAME} PRIVATE sbc)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})
