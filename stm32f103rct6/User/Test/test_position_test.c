/**
 ******************************************************************************
 * @file test_position_test.c
 * @brief 验证有限位移、丢应答、拒绝命令与停止失败的控制边界。
 * @author muhuangan
 * @attention Copyright 2026 muhuangan. MIT License; see project LICENSE.
 ******************************************************************************
 */
/* 共用模拟对象字典，测试入口独立；每个场景在新进程中启动。 */
#define main TEST_UnusedVelocitySuite
#include "test_motor_startup.c"
#undef main

/** @brief 用真实启动模块和位置点动模块检查成功及错误路径。 */
int main(int argc, char **argv)
{
  uint32_t    loop;
  uint8_t     node;
  const char *scenario = (argc > 1) ? argv[1] : "normal";
  Test_Reset();
  s_drives[0].actualPosition = 1000;
  s_drives[1].actualPosition = -1000;
  if (strcmp(scenario, "zero_ramp") == 0)
  {
    s_drives[0].acceleration = 0U;
    Test_Run(TEST_LOOPS);
    Test_Check(!APP_MotorStartup_IsReady(1U), "zero ramp prevents enabling test");
    Test_Check(g_motorStartupDebug[0].lastFailure.step == APP_MOTOR_STEP_ACCELERATION, "zero ramp failure retained");
    for (loop = 0U; loop < TEST_LOOPS; loop++)
    {
      Test_Run(1U);
      APP_MotorStartup_PositionTestProcess();
    }
    Test_Check(s_drives[0].triggerCount == 0U, "zero ramp never triggers position");
    Test_Check(s_drives[0].rampWriteCount == 0U, "zero ramp is diagnosed without writing it");
    printf("PASS: %lu position checks (%s).\n", (unsigned long) s_checks, scenario);
    return EXIT_SUCCESS;
  }
  Test_Run(TEST_LOOPS);
  Test_Check(APP_MotorStartup_IsReady(1U) && APP_MotorStartup_IsReady(2U), "position startup ready");
  Test_Check(s_drives[0].mode == MID_LISAN_EC_MODE_PROFILE_POSITION, "mode 1 selected");
  Test_Check(s_drives[0].profileSpeed == CHASSIS_LINK_POSITION_SPEED_PPS, "position speed limited");
  Test_Check(APP_MotorStartup_RequestVelocity(1U, TEST_SPEED_PPS) == MID_CANOPEN_INVALID_PARAM,
             "velocity requests rejected during position test");
  if (strcmp(scenario, "no_motion") == 0)
  {
    s_drives[0].freezePosition = true;
  }
  if (strcmp(scenario, "lost_ack") == 0)
  {
    s_drives[0].loseTriggerAck = true;
  }
  if (strcmp(scenario, "rejected") == 0)
  {
    s_drives[0].rejectTarget = true;
  }
  if (strcmp(scenario, "read_failure") == 0)
  {
    s_drives[0].failPositionRead = true;
  }
  if (strcmp(scenario, "halt_failure") == 0)
  {
    s_drives[0].freezePosition = true;
    s_drives[0].ignoreHalt = true;
    s_drives[0].actualVelocity = TEST_SPEED_PPS;
  }
  for (loop = 0U; loop < TEST_LOOPS; loop++)
  {
    Test_Run(1U);
    APP_MotorStartup_PositionTestProcess();
  }
  for (node = 0U; node < TEST_NODE_COUNT; node++)
  {
    Test_Check(s_drives[node].velocityWriteCount == 0U, "no 60FF velocity write in position test");
    Test_Check(s_drives[node].rampWriteCount == 0U, "existing ramps are never rewritten");
    Test_Check(s_drives[node].acceleration == 3855U && s_drives[node].deceleration == 4000U,
               "existing positive ramps preserved");
  }
  if (strcmp(scenario, "normal") == 0)
  {
    for (node = 0U; node < TEST_NODE_COUNT; node++)
    {
      Test_Check(g_motorPositionDebug[node].result == APP_MOTOR_RESULT_VERIFIED, "both moves verified");
      Test_Check(s_drives[node].triggerCount == APP_POSITION_MOVE_COUNT, "only two triggers per node");
      Test_Check(g_motorPositionDebug[node].moves[0].actualPosition ==
                     g_motorPositionDebug[node].moves[0].startPosition + CHASSIS_LINK_POSITION_DISTANCE_PULSES,
                 "forward displacement measured");
    }
    Test_Check(s_drives[0].actualPosition == 1000 && s_drives[1].actualPosition == -1000, "return to initial position");
  }
  else
  {
    Test_Check(g_motorPositionDebug[0].result == APP_MOTOR_RESULT_FAILED, "failure retained");
    Test_Check(s_drives[0].triggerCount <= 1U, "uncertain relative move never retriggered");
    Test_Check(g_motorPositionDebug[0].moves[1].result == APP_MOTOR_RESULT_NOT_RUN, "no reverse after failed forward");
    if (strcmp(scenario, "halt_failure") == 0)
    {
      Test_Check(!g_motorPositionDebug[0].haltConfirmed, "moving drive is not confirmed stopped");
      Test_Check(s_drives[1].triggerCount == 0U, "other wheel blocked until stop confirmed");
    }
    else
    {
      Test_Check(g_motorPositionDebug[0].haltConfirmed, "failed motion followed by verified halt");
      Test_Check(g_motorPositionDebug[1].result == APP_MOTOR_RESULT_VERIFIED, "healthy peer can test after halt");
    }
    if ((strcmp(scenario, "rejected") == 0) || (strcmp(scenario, "read_failure") == 0))
    {
      Test_Check(s_drives[0].triggerCount == 0U, "invalid target or position read blocks trigger");
    }
  }
  printf("PASS: %lu position checks (%s).\n", (unsigned long) s_checks, scenario);
  return EXIT_SUCCESS;
}
