#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 电机转动45度
constructor_args:
  - motor_yaw: '@&motor_yaw'
  - position_pid_param:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: true
template_args: []
required_hardware: []
depends:
  - Motor
=== END MANIFEST === */
// clang-format on

#include <cstdlib>
#include <cstring>

#include "Motor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"


static constexpr float GIMBAL_MAX_SPEED = static_cast<float>(M_2PI) * 1.5f;

class Action : public LibXR::Application {
 public:
  /**
   *@brief 构造函数初始化数据成员
   *
   *@param hw 硬件容器
   *@param app 应用管理器
   *@param motor_yaw 电机实例化
   */
  Action(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
         Motor *motor_yaw, LibXR::PID<float>::Param position_pid_param)
      : motor(motor_yaw), pid_position(position_pid_param) {
    UNUSED(hw);
    UNUSED(app);

    thread_.Create(this, ThreadFunc, "ActionThread", 2048,
                   LibXR::Thread::Priority::MEDIUM);
  }

  /**
   * @brief 线程函数
   *
   * @param action action实例指针
   */
  static void ThreadFunc(Action *action) {
    LibXR::Thread::Sleep(2);

    action->cmd.mode = ControlMode::MODE_CURRENT;

    action->target_angle = action->now_angle + M_PI / 4.0f;

    while (true) {
      // 更新电机反馈及状态
      action->Update();
      // 发送电流控制命令
      action->Control();
      LibXR::Thread::Sleep(2);
    }
  }

  /**
   * @brief 更新电机反馈及状态
   */
  void Update() {
    this->motor->Update();
    this->feedback_1 = this->motor->GetFeedback();

    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now;
    this->now_angle = this->feedback_1.abs_angle;
    this->now_omega = this->feedback_1.omega;
  }
  /**
   * @brief 核心控制函数
   */
  void Control() {
    float output = 0.0f;
    Solve(output, target_angle, dt_);
    this->cmd.velocity = output;
    this->motor->Control(this->cmd);
  }
  void OnMonitor() override {}

 private:
  LibXR::Thread thread_;
  // 电机实例化
  Motor *motor;
  // 统一命令结构体
  MotorCmd cmd;
  // 电机反馈
  Feedback feedback_1;

  // 串级PID参数
  LibXR::PID<float> pid_position;
  float now_angle = 0.0f;     // 当前角度
  float target_angle = 0.0f;  // 目标角度
  float now_omega = 0.0f;     // 当前角速度
  float target_omega = 0.0f;  // 目标角速度
  float j = 0.0f;             // 转动惯量
  float last_omega = 0.0f;    // 上一次目标角速度

  float dt_ = 0.0f;  ///< 控制周期时间间隔
  LibXR::MicrosecondTimestamp last_online_time_;

  /**
   * @brief 控制核心解算
   * @param output 输出量
   * @param target_angle 目标角度
   * @param dt 时间间隔
   */
  void Solve(float &output, const LibXR::CycleValue<float> &target_angle,
             float dt) {
    float error = target_angle - now_angle;
    target_omega = pid_position.Calculate(error, 0.0f, dt);

    // 前馈
    float ff = JFeedforward(target_omega, last_omega, dt, j);
    last_omega = target_omega;
    UNUSED(ff);

    // 输出量
    output = target_omega;
  }
  /**
   * @brief 转动惯量前馈计算
   * @param target_omega  本次目标角速度
   * @param last_omega    上一次目标角速度
   * @param dt_           时间间隔
   * @param J             转动惯量
   * @return              需要补偿的力矩
   */
  static float JFeedforward(float target_omega, float last_omega, float dt_,
                            float J) {
    float delta_omega = target_omega - last_omega;  // 角加速度近似
    return (J * delta_omega / dt_);                 // τ = J·α
  }
};
