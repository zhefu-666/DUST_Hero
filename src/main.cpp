#include "data_manager/base.h"
#include "data_manager/param.h"
#include "threads/pipeline.h"
#include "threads/control.h"
#include "garage/garage.h"

#include <condition_variable>
#include <mutex>
#include <thread>
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cstdlib>

// X11 thread support
#include <X11/Xlib.h>

std::mutex hang_up_mutex;
std::condition_variable hang_up_cv;

// 程序退出标志 - 全局可访问
std::atomic<bool> g_running(true);
std::atomic<bool> g_cleanup_done(false);
std::atomic<int> g_signal_received(0);

// 强制退出线程
void force_exit_thread() {
    // 等待1.5秒后强制退出
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    std::cerr << "\n[ForceExit] Timeout, forcing exit..." << std::endl;
    _exit(0);
}

// 清理函数
void cleanup() {
    // 防止重复清理
    bool expected = false;
    if (!g_cleanup_done.compare_exchange_strong(expected, true)) {
        return;
    }
    
    std::cerr << "\n[Cleanup] Shutting down..." << std::endl;
    
    // 设置退出标志
    g_running = false;
    
    // 等待其他线程响应退出信号
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // 关闭相机
    std::cerr << "[Cleanup] Releasing camera..." << std::endl;
    try {
        deinit_camera();
    } catch (...) {
        std::cerr << "[Cleanup] Exception during camera release" << std::endl;
    }
    
    std::cerr << "[Cleanup] Done." << std::endl;
}

// 信号处理函数
void signal_handler(int signum) {
    static std::atomic<int> signal_count(0);
    int count = ++signal_count;
    
    // 第一次收到信号
    if (count == 1) {
        g_signal_received = signum;
        g_running = false;
        
        // 启动强制退出线程
        std::thread(force_exit_thread).detach();
        
        // 唤醒等待中的主线程
        hang_up_cv.notify_all();
    }
    // 第二次收到信号时立即退出
    else {
        std::cerr << "\n[Signal] Force exit!" << std::endl;
        _exit(128 + signum);
    }
}

int main(int argc, char** argv) {
    // 注册信号处理
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    
    // 注册退出时清理
    std::atexit(cleanup);
    
    // X11线程支持
    XInitThreads();
    
    auto param = Param::get_instance();
    auto pipeline = Pipeline::get_instance();
    auto garage = Garage::get_instance();
    auto control = Control::get_instance();

    int option;
    while ((option = getopt(argc, argv, "hs")) != -1) {
        switch (option) {
            case 's':
                Data::imshow_flag = true;
                break;
            case 'h':
                std::cout << "Usage: " << argv[0] << " [-h] [-s]" << std::endl;
                return 0;
        }
    }
    
    // 初始化相机
    while(g_running && !init_camera()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    if (!g_running) {
        std::cout << "[Main] Interrupted during init" << std::endl;
        return 1;
    }

    rm::message_init("autoaim");
    init_debug();
    init_attack();
    if (Data::serial_flag) init_serial();
    control->autoaim();

    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE)
    pipeline->autoaim_combine();  
    #endif

    #if defined(TJURM_SENTRY) || defined(TJURM_DRONSE) || defined(TJURM_HERO)
    pipeline->autoaim_baseline();
    #endif
    
    // 显示线程
    std::thread display_t(&Pipeline::display_thread, pipeline);
    display_t.detach();

    // 手动开火模式
    while(Data::manu_fire && g_running) {
        std::cin.get();
        if (!g_running) break;
        Data::auto_fire = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        Data::auto_fire = false;
    }

    rm::message("Main thread hang up!", rm::MSG_OK);
    
    // 主线程等待退出信号
    {
        std::unique_lock<std::mutex> lock(hang_up_mutex);
        while (g_running) {
            hang_up_cv.wait_for(lock, std::chrono::milliseconds(500));
        }
    }
    
    if (g_signal_received != 0) {
        std::cout << "\n[Main] Signal " << g_signal_received.load() << " received" << std::endl;
    }
    
    // cleanup() 会在 exit 时自动调用
    return 0;
}
