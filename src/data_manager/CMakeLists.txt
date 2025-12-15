file(GLOB_RECURSE
    data_src 
        ${CMAKE_SOURCE_DIR}/src/data_manager/*.cpp
)

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/json json_binary_dir)
set(JSON_LIBS 
        nlohmann_json::nlohmann_json
        -lpthread
)

# 海康SDK路径
set(MV_SDK_PATH /opt/MVS)

add_library(data_manager
    SHARED
        ${data_src}    
)

target_include_directories(data_manager PUBLIC ${MV_SDK_PATH}/include)
target_link_directories(data_manager PUBLIC ${MV_SDK_PATH}/lib/aarch64)

target_link_libraries(data_manager
    PUBLIC
        ${OpenCV_LIBS}
        ${JSON_LIBS}
        ${OpenRM_LIBS}
        MvCameraControl
        usb-1.0
)
