#include "garage/garage.h"
#include "threads/control.h"
#include "threads/pipeline.h"
#include <thread>
#include <cmath>
#include <fstream>
#include <mutex>
#include <vector>
using namespace rm;

// 外部声明全局检测结果
extern std::mutex g_detection_mutex;
extern std::vector<rm::YoloRect> g_detections;

static double shoot_speed, shoot_delay;
static double start_fire_delay;
static int iteration_num;
static double base_to_far_dist, far_to_base_dist;

static bool fire = false;
static double target_yaw, target_pitch, fly_delay, delay;
static double rotate_delay, rotate_delay_outpost, rotate_delay_rune;
static Eigen::Vector4d pose;

static TimePoint start_autoaim;
static bool last_autoaim = false;

static std::ofstream speed_file;
static bool speed_write_flag;
static rm::SpeedQueue<float> speed_queue(3, 15.75f, {0.5, 0.3, 0.2});


static void init_send() {
    auto param = Param::get_instance();
    shoot_speed = (*param)["Car"]["ShootSpeed"];
    shoot_delay = (*param)["Car"]["ShootDelay"];
    rotate_delay = (*param)["Car"]["RotateDelay"];
    rotate_delay_outpost = (*param)["Car"]["RotateDelayOutpost"];
    rotate_delay_rune = (*param)["Car"]["RotateDelayRune"];
    start_fire_delay = (*param)["Car"]["StartFireDelay"];
    iteration_num = (*param)["Kalman"]["IterationNum"];
    base_to_far_dist = (*param)["Camera"]["Switch"]["BaseToFarDist"];
    far_to_base_dist = (*param)["Camera"]["Switch"]["FarToBaseDist"];
    speed_write_flag = (*param)["Debug"]["SaveSpeed"]["SpeedWrite"];

    std::string speed_save_path = (*param)["Debug"]["SaveSpeed"]["SavePath"];
    if (speed_write_flag) {
        speed_file.open(speed_save_path, std::ios_base::app);
    }
}

void Control::message() {
    rm::message_send();
    rm::message("system state", Data::state);
    rm::message("system yaw", get_yaw());
    rm::message("system pit", get_pitch());
    rm::message("system rol", get_roll());
    rm::message("system omg", get_yaw_omega());
    rm::message("target id", Data::target_id);
    rm::message("target yaw", target_yaw);
    rm::message("target pit", target_pitch);
    rm::message("target -x-", pose(0, 0));
    rm::message("target -y-", pose(1, 0));
    rm::message("target -z-", pose(2, 0));
    rm::message("target -0-", pose(3, 0) * 180 / M_PI);
    rm::message("target fire", fire);
    rm::message("enemy color", Data::enemy_color);
    rm::message("camera id", Data::camera_index);
}

void Control::state() {
    auto pipeline = Pipeline::get_instance();
    auto garage = Garage::get_instance();

    // 通过电控获取敌方颜色
    Data::enemy_color = get_enemy();
    Data::self_color = (Data::enemy_color == rm::ARMOR_COLOR_BLUE) ? rm::ARMOR_COLOR_RED : rm::ARMOR_COLOR_BLUE;

    // 确定自瞄状态，开始录制
    if (Data::auto_capture && !get_autoaim()) pipeline->start_record();
    else if (!Data::auto_capture && Data::manu_capture) pipeline->start_record();
    else pipeline->stop_record();

    // 确定自瞄状态，记录开始自瞄时间点
    if (!last_autoaim && get_autoaim()) {
        start_autoaim = getTime();
        Data::attack->clear();
    }
    last_autoaim = get_autoaim();

    // 获取攻击目标
    if(Data::armor_mode) Data::target_id = Data::attack->pop();
    else if (Data::rune_mode) Data::target_id = rm::ARMOR_ID_RUNE;
    else Data::target_id = rm::ARMOR_ID_UNKNOWN;
    
    Data::state = get_state();

    #ifdef TJURM_SENTRY
    if (Data::target_id != rm::ARMOR_ID_TOWER) {
        Data::camera_index = Data::camera_base;
        rm::message("camera type", 'B');
    } else if (Data::target_dist > base_to_far_dist) {
        Data::camera_index = Data::camera_far;
        rm::message("camera type", 'F');
    } else if (Data::target_dist < far_to_base_dist) {
        Data::camera_index = Data::camera_base;
        rm::message("camera type", 'B');
    }
    #endif

    #ifdef TJURM_SENTRY
    rm::message("shoot config", (int)get_shoot_config());
    Data::attack->setValidID(get_shoot_config());
    #endif

    // 更新自瞄状态
    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE)
    if (Data::auto_rune) {
        if (Data::state == 0 || Data::state == 1) pipeline->switch_rune_to_armor();
        else if (Data::state == 2 || Data::state == 3) pipeline->switch_armor_to_rune();
        else pipeline->switch_rune_to_armor();
    } else if (Data::manu_rune) {
        pipeline->switch_armor_to_rune();
    } else {
        pipeline->switch_rune_to_armor();
    }
    #endif

    #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE)
    if (Data::rune_mode)       rotate_delay = rotate_delay_rune;
    else if (Data::state == 1) rotate_delay = rotate_delay_outpost;
    else                       rotate_delay = rotate_delay;
    #endif
}

void Control::shootspeed() {
    
    #ifdef TJURM_HERO
    
    float curr_speed = 15.75f;
    float last_speed = speed_queue.back();

    float avg_speed = speed_queue.pop();
    avg_speed = std::clamp(avg_speed, 14.0f, 16.0f);

    operate_bytes_.output_data.avg_speed = avg_speed;
    shoot_speed = avg_speed;
    operate_bytes_.output_data.food = 0x01;

    rm::message("shoot speed", avg_speed);

    #endif
}

// 检查是否有装甲板检测结果，并计算其相对云台的 YAW 和 PITCH
bool Control::check_armor_detection(double& armor_yaw, double& armor_pitch, rm::ArmorID& armor_id) {
    static int call_count = 0;
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    
    if (g_detections.empty()) {
        armor_id = rm::ARMOR_ID_UNKNOWN;
        return false;
    }
    
    const auto& yolo_rect = g_detections[0];
    
    // 计算装甲板中心点
    int img_center_x = yolo_rect.box.x + yolo_rect.box.width / 2;
    int img_center_y = yolo_rect.box.y + yolo_rect.box.height / 2;
    
    // 获取摄像机内参
    rm::Camera* camera = Data::camera[Data::camera_index];
    if (!camera) {
        std::cout << "[ERROR-CAMERA-NULL] camera pointer is null, camera_index=" << Data::camera_index 
                  << " camera array size=" << Data::camera.size() << "\n";
        armor_id = rm::ARMOR_ID_UNKNOWN;
        return false;
    }
    
    // Debug: verify intrinsic matrix is valid
    if (camera->intrinsic_matrix.empty() || camera->intrinsic_matrix.rows != 3 || camera->intrinsic_matrix.cols != 3) {
        std::cout << "[ERROR-INTRINSIC-INVALID] matrix dims: " << camera->intrinsic_matrix.rows 
                  << "x" << camera->intrinsic_matrix.cols << ", empty=" << camera->intrinsic_matrix.empty() << "\n";
    }
    double fx = camera->intrinsic_matrix.at<double>(0, 0);
    double fy = camera->intrinsic_matrix.at<double>(1, 1);
    double cx = camera->intrinsic_matrix.at<double>(0, 2);
    double cy = camera->intrinsic_matrix.at<double>(1, 2);
    
    // 计算像素偏移
    double pixel_x = img_center_x - cx;
    double pixel_y = img_center_y - cy;
    
    // 转换为角度偏移（使用atan2公式）
    double offset_yaw = std::atan2(pixel_x, fx) * 180.0 / M_PI;
    double offset_pitch = std::atan2(pixel_y, fy) * 180.0 / M_PI;
    
    // 获取当前云台角度
    double gimbal_yaw = get_yaw();
    double gimbal_pitch = get_pitch();
    
    // 计算目标角度
    armor_yaw = gimbal_yaw + offset_yaw;
    armor_pitch = gimbal_pitch + offset_pitch;
    
    // 设置装甲板ID
    static std::vector<int> armor_class_map;
    static bool map_initialized = false;
    if (!map_initialized) {
        auto param = Param::get_instance();
        std::string yolo_type = (*param)["Model"]["YoloArmor"]["Type"];
        auto json_class_map = (*param)["Model"]["YoloArmor"][yolo_type]["ClassMap"];
        armor_class_map.clear();
        for (const auto& item : json_class_map) {
            armor_class_map.push_back((int)item);
        }
        map_initialized = true;
    }
    
    if (yolo_rect.class_id >= 0 && yolo_rect.class_id < armor_class_map.size()) {
        armor_id = (rm::ArmorID)armor_class_map[yolo_rect.class_id];
    } else {
        armor_id = rm::ARMOR_ID_UNKNOWN;
    }
    
    // 每10次调用输出详细诊断信息
    if (call_count % 10 == 0) {
        std::cout << "[DIAG-#" << call_count << "]"
                  << " BOX:(" << yolo_rect.box.x << "," << yolo_rect.box.y 
                  << ") " << yolo_rect.box.width << "x" << yolo_rect.box.height
                  << " | CENTER:(" << img_center_x << "," << img_center_y << ")"
                  << " | INTRINSIC: fx=" << std::fixed << std::setprecision(1) << fx 
                  << " fy=" << fy << " cx=" << cx << " cy=" << cy
                  << " | PIXEL_OFFSET:(" << std::setprecision(2) << pixel_x 
                  << "," << pixel_y << ")"
                  << " | ANGLE_OFFSET: yaw=" << std::setprecision(4) << offset_yaw 
                  << "° pitch=" << offset_pitch << "°"
                  << " | GIMBAL: yaw=" << gimbal_yaw << "° pitch=" << gimbal_pitch << "°"
                  << " | TARGET: yaw=" << armor_yaw << "° pitch=" << armor_pitch << "°\n";
    }
    call_count++;
    
    return true;
}





void Control::send_thread() {
    auto garage = Garage::get_instance();
    auto pipeline = Pipeline::get_instance();

    init_send();
    
    std::mutex mutex;
    double detected_yaw = 0, detected_pitch = 0;
    rm::ArmorID detected_armor_id = rm::ARMOR_ID_UNKNOWN;
    int frame_count = 0;
    
    while(true) {
        if(!send_flag_) {
            std::unique_lock<std::mutex> lock(mutex);
            send_cv_.wait(lock, [this]{return send_flag_;});
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(Data::send_wait_time));

        this->state();
        this->shootspeed();

        frame_count++;
        bool has_armor = check_armor_detection(detected_yaw, detected_pitch, detected_armor_id);

        // ★ 新策略：如果检测到装甲板，优先发送装甲板位置
        #ifdef TJURM_HERO
        if (has_armor) {
            send_single(detected_yaw, detected_pitch, false, detected_armor_id);
            continue;
        }
        #endif

        // 如果没有装甲板，继续原有逻辑
        if(Data::target_id == rm::ARMOR_ID_UNKNOWN) {
            #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_DRONSE)
            continue;
            #endif

            #ifdef TJURM_HERO
            std::cout << "[TRACE] 执行: target_id==UNKNOWN分支 -> send_single(gimbal_yaw, gimbal_pitch)\n";
            std::cout << "[TRACE] 执行: target_id==UNKNOWN分支 -> send_single(gimbal_yaw, gimbal_pitch)\n";
            send_single(get_yaw(), get_pitch(), false);
            continue;
            #endif

            #ifdef TJURM_SENTRY
            float camsense_x = this->state_bytes_.input_data.target_pose[0];
            float camsense_y = this->state_bytes_.input_data.target_pose[1];
            float camsense_z = this->state_bytes_.input_data.target_pose[2];
            if(abs(camsense_x) < 1e-2 && abs(camsense_y) < 1e-2) continue;

            getFlyDelay(target_yaw, target_pitch, shoot_speed, camsense_x, camsense_y, camsense_z);
            std::cout << "[TRACE] 执行: 迭代法分支 -> send_single(target_yaw, target_pitch)\n";
            std::cout << "[TRACE] 执行: 迭代法分支 -> send_single(target_yaw, target_pitch)\n";
            send_single(target_yaw, target_pitch, false);
            continue;
            #endif
        }

        // 迭代法求解击打 yaw, pitch
        auto objptr = garage->getObj(Data::target_id);
        objptr->getTarget(pose, 0.0, 0.0, 0.0);
        for(int i = 0; i < iteration_num; i++) {
            fly_delay = getFlyDelay(target_yaw, target_pitch, shoot_speed, pose(0, 0), pose(1, 0), pose(2, 0));
            fire = objptr->getTarget(pose, fly_delay, rotate_delay, shoot_delay);
        }
        rm::message("target pitch b", target_pitch);
        Data::target_dist = sqrt(pow(pose(0, 0), 2) + pow(pose(1, 0), 2) + pow(pose(2, 0), 2));
        
        // 如果返回坐标为0, 确定控制信号
        if ((std::abs(pose[0]) < 1e-2) && (std::abs(pose[1]) < 1e-2)) {
            #if defined(TJURM_INFANTRY) || defined(TJURM_BALANCE) || defined(TJURM_DRONSE)
            continue;
            #endif

            #ifdef TJURM_HERO
            std::cout << "[TRACE] 执行: target_id==UNKNOWN分支 -> send_single(gimbal_yaw, gimbal_pitch)\n";
            std::cout << "[TRACE] 执行: target_id==UNKNOWN分支 -> send_single(gimbal_yaw, gimbal_pitch)\n";
            send_single(get_yaw(), get_pitch(), false);
            continue;
            #endif

            #ifdef TJURM_SENTRY
            float camsense_x = this->state_bytes_.input_data.target_pose[0];
            float camsense_y = this->state_bytes_.input_data.target_pose[1];
            float camsense_z = this->state_bytes_.input_data.target_pose[2];
            if(abs(camsense_x) < 1e-2 && abs(camsense_y) < 1e-2) continue;

            getFlyDelay(target_yaw, target_pitch, shoot_speed, camsense_x, camsense_y, camsense_z);
            std::cout << "[TRACE] 执行: 迭代法分支 -> send_single(target_yaw, target_pitch)\n";
            std::cout << "[TRACE] 执行: 迭代法分支 -> send_single(target_yaw, target_pitch)\n";
            send_single(target_yaw, target_pitch, false);
            continue;
            #endif
        }

        // 控制发弹
        bool start_delay_flag = (getDoubleOfS(start_autoaim, getTime()) > start_fire_delay);
        bool autoaim_flag = get_autoaim();

        fire = (fire && start_delay_flag && autoaim_flag && Data::auto_fire);
        std::cout << "[TRACE] 执行: 最后分支 -> send_single(target_yaw=" << target_yaw << ", target_pitch=" << target_pitch << ", fire, target_id)\n";
        std::cout << "[TRACE] 执行: 最后分支 -> send_single(target_yaw=" << target_yaw << ", target_pitch=" << target_pitch << ", fire, target_id)\n";
        send_single(target_yaw, target_pitch, fire, Data::target_id);
    }
}
