file(GLOB_RECURSE
    threads_src 
        ${CMAKE_SOURCE_DIR}/src/threads/*.cpp
)

add_library(threads
    STATIC
        ${threads_src}
)

set(JSON_LIBS 
        nlohmann_json::nlohmann_json
        -hikvision  # 替换成海康相机的库文件
        -lpthread
)

target_link_libraries(threads
    PUBLIC
        ${JSON_LIBS}
        ${OpenCV_LIBS}
        ${OpenRM_LIBS}
        data_manager
)
