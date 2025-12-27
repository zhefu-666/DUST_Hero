#include "threads/pipeline.h"
#include <atomic>
extern std::atomic<bool> g_running;
#include "threads/control.h"

// 外部声明 - 更新显示线程的检测结果
extern void update_global_detections(const std::vector<rm::YoloRect>& detections);
extern void set_classifier_enabled(bool enabled);

using namespace rm;

void Pipeline::tracker_baseline_thread(
    std::mutex& mutex_in, bool& flag_in, std::shared_ptr<rm::Frame>& frame_in
) {
    auto garage = Garage::get_instance();
    auto param = Param::get_instance();
    auto control = Control::get_instance();

    // 获取装甲板长宽
    float big_width    = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Width"];
    float big_height   = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Height"];
    float small_width  = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Width"];
    float small_height = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Height"];

    std::string small_path = (*param)["Debug"]["SmallDecal"];
    std::string big_path   = (*param)["Debug"]["BigDecal"];

    init_pointer();
    init_locater();
    init_updater();
    init_classifier();
    
    // 通知显示线程分类器状态
    bool classifier_enable = (*param)["Model"]["Classifier"]["Enable"];
    set_classifier_enabled(classifier_enable);
    std::cout << "[TRACKER] Classifier enabled: " << (classifier_enable ? "YES" : "NO") << std::endl;
    std::cout << "[TRACKER] Reprojection flag: " << (Data::reprojection_flag ? "YES" : "NO") << std::endl;
    std::cout << "[TRACKER] UI flag: " << (Data::ui_flag ? "YES" : "NO") << std::endl;
    std::cout << "[TRACKER] Image flag: " << (Data::image_flag ? "YES" : "NO") << std::endl;
    
    if(Data::reprojection_flag) {
        initReprojection(small_width, small_height, big_width, big_height, small_path, big_path);
    }

    rm::CycleQueue<double> delay_list(100);
    TimePoint tp0, tp1, tp2;

    std::mutex mutex;
    while (g_running) {
        // 等待 armor_mode 激活
        if (!Data::armor_mode) {
            std::unique_lock<std::mutex> lock(mutex);
            // 使用wait_for带超时，定期检查g_running
            while (!Data::armor_mode && g_running) {
                armor_cv_.wait_for(lock, std::chrono::milliseconds(100));
            }
            if (!g_running) break;
        }

        // 等待输入帧
        std::unique_lock<std::mutex> lock_in(mutex_in);
        // 使用wait_for带超时，定期检查g_running
        while (!flag_in && g_running) {
            tracker_in_cv_.wait_for(lock_in, std::chrono::milliseconds(100));
        }
        if (!g_running) {
            lock_in.unlock();
            break;
        }
        if (!flag_in) {
            lock_in.unlock();
            continue;  // 超时但没有数据，继续循环
        }

        std::shared_ptr<rm::Frame> frame = frame_in;
        flag_in = false;
        lock_in.unlock();

        tp1 = getTime();
        bool track_flag = true;
        if (track_flag) track_flag = pointer(frame);
        if (track_flag) track_flag = classifier(frame);  // 使用 tiny_resnet 分类
        
        // ============ 关键修复：分类器处理后更新显示数据 ============
        // 此时 yolo_list 中的 class_id 和 color_id 已经被分类器更新
        update_global_detections(frame->yolo_list);
        // ===========================================================
        
        if (track_flag) track_flag = locater(frame);
        if (track_flag) track_flag = updater(frame);
        tp2 = getTime();

        if (Data::pipeline_delay_flag) rm::message("tracker time", getDoubleOfS(tp1, tp2) * 1000);
        if (track_flag) delay_list.push(getDoubleOfS(tp0, tp2));

        tp0 = tp2;
        double fps = 1.0 / delay_list.getAvg();
        rm::message("fps", fps);
        
        if (Data::image_flag) {
            if (Data::ui_flag) UI(frame);
            imshow(frame);
        }
    }
    std::cout << "[tracker_thread] Exiting..." << std::endl;
}
