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

// 程序退出标志
std::atomic<bool> g_running(true);
std::atomic<bool> g_cleanup_done(false);

// 清理函数 - 在正常退出时调用
void cleanup() {
    // 使用原子操作防止重复清理
    bool expected = false;
    if (!g_cleanup_done.compare_exchange_strong(expected, true)) {
        return;  // 已经清理过了
    }
    
    std::cout << "\n[Cleanup] Shutting down camera and resources..." << std::endl;
    
    // 关闭相机
    deinit_camera();
    
    std::cout << "[Cleanup] Cleanup completed." << std::endl;
}

// 信号处理函数 - 只设置标志，不做复杂操作
volatile sig_atomic_t g_signal_received = 0;

void signal_handler(int signum) {
    g_signal_received = signum;
    g_running = false;
    
    // 对于SIGINT和SIGTERM，尝试优雅退出
    if (signum == SIGINT || signum == SIGTERM) {
        // 唤醒主线程
        hang_up_cv.notify_all();
    }
}

int main(int argc, char** argv) {
    // 注册信号处理函数
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    
    sigaction(SIGINT, &sa, nullptr);   // Ctrl+C
    sigaction(SIGTERM, &sa, nullptr);  // kill
    
    // 注册atexit清理函数（正常退出时）
    std::atexit(cleanup);
    
    // 初始化X11线程支持 - 必须在任何X11调用之前
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
                std::cout << "Usage: " << argv[0] << " [-h] [-s] " << std::endl;
                break;
        }
    }
    
    while(g_running && !init_camera()) {
        // 等待相机初始化成功
    }
    
    if (!g_running) {
        std::cout << "[Main] Interrupted before camera init completed" << std::endl;
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
    
    // 启动实时画面显示线程
    std::thread display_t(&Pipeline::display_thread, pipeline);
    display_t.detach();

    while(Data::manu_fire && g_running) {
        std::cin.get();
        if (!g_running) break;
        Data::auto_fire = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        Data::auto_fire = false;
    }

    rm::message("Main thread hang up!", rm::MSG_OK);
    
    // 等待退出信号
    {
        std::unique_lock<std::mutex> lock(hang_up_mutex);
        hang_up_cv.wait(lock, []{ return !g_running.load(); });
    }
    
    // 检查是否收到信号
    if (g_signal_received != 0) {
        std::cout << "\n[Main] Received signal " << g_signal_received 
                  << ", performing cleanup..." << std::endl;
    }
    
    // 正常退出 - cleanup会在atexit中被调用
    return 0;
}
