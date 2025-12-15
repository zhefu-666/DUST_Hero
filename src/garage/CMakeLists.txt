file(GLOB_RECURSE
    garage_src
        ${CMAKE_SOURCE_DIR}/src/garage/*.cpp      
)

add_library(garage
    STATIC
        ${garage_src}
)

# 添加海康SDK路径（ARM64架构）
set(MV_SDK_PATH /opt/MVS)
include_directories(${MV_SDK_PATH}/include)
link_directories(${MV_SDK_PATH}/lib/aarch64)

target_link_libraries(garage
    PUBLIC
        ${OpenRM_LIBS}
        data_manager
        MvCameraControl
        usb-1.0
)
