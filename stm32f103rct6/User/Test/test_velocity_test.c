/**
 ******************************************************************************
 * @file test_velocity_test.c
 * @brief 验证真实速度测试状态机的正反转顺序、实际反馈和停止失败边界。
 * @author muhuangan
 * @attention Copyright 2026 muhuangan. MIT License; see project LICENSE.
 ******************************************************************************
 */
#define main TEST_UnusedStartupSuite
#include "test_motor_startup.c"
#undef main

/** @defgroup TEST_VELOCITY_Config 模拟节点列表
 * @{
 */
static const uint8_t s_nodeIds[TEST_NODE_COUNT] = {1U, 2U};
/** @} */

/** @brief 推进真实速度测试和启动通信验证，时钟由共用模拟器推进。 */
static void TEST_VelocityRun(uint32_t loops)
{
  uint32_t index;
  for (index = 0U; index < loops; index++)
  {
    Test_Run(1U);
    APP_MotorStartup_VelocityTestProcess(s_nodeIds, TEST_NODE_COUNT);
  }
}

/** @brief 检查四条指令都实际验证完成且目标顺序正确。 */
static void TEST_CheckNode(uint8_t node)
{
  uint8_t       command;
  const int32_t targets[APP_VELOCITY_COMMAND_COUNT] = {CHASSIS_LINK_SELF_TEST_SPEED_PPS, 0,
                                                       -CHASSIS_LINK_SELF_TEST_SPEED_PPS, 0};
  Test_Check(g_motorVelocityDebug[node].result == APP_MOTOR_RESULT_VERIFIED, "full sequence verified");
  Test_Check(s_drives[node].mode == MID_LISAN_EC_MODE_PROFILE_VELOCITY, "mode 3 selected");
  Test_Check(s_drives[node].nonzeroVelocityWrites == 2U, "forward and reverse only once");
  Test_Check(s_drives[node].negativeVelocityWrites == 1U, "one reverse request");
  Test_Check(s_drives[node].positionWriteCount == 0U, "no position target writes");
  Test_Check(s_drives[node].actualVelocity == 0, "final measured velocity zero");
  for (command = 0U; command < APP_VELOCITY_COMMAND_COUNT; command++)
  {
    Test_Check(g_motorVelocityDebug[node].commands[command].result == APP_MOTOR_RESULT_VERIFIED,
               "each command retains actual verification");
    Test_Check(g_motorVelocityDebug[node].commands[command].commandValue == targets[command], "correct command order");
    Test_Check(g_motorVelocityDebug[node].commands[command].actualVelocity == targets[command],
               "actual velocity captured");
  }
  Test_Check((g_motorVelocityDebug[node].commands[APP_VELOCITY_COMMAND_STOP_FORWARD].startTick -
              g_motorVelocityDebug[node].commands[APP_VELOCITY_COMMAND_FORWARD].endTick) >=
                 CHASSIS_LINK_SELF_TEST_ON_MS,
             "forward hold duration respected");
  Test_Check((g_motorVelocityDebug[node].commands[APP_VELOCITY_COMMAND_REVERSE].startTick -
              g_motorVelocityDebug[node].commands[APP_VELOCITY_COMMAND_STOP_FORWARD].endTick) >=
                 CHASSIS_LINK_SELF_TEST_OFF_MS,
             "stop gap respected before reverse");
}

/** @brief 每个场景使用独立进程，模拟上电后的一次完整速度测试。 */
int main(int argc, char **argv)
{
  const char *scenario = (argc > 1) ? argv[1] : "normal";
  uint32_t    writes;
  Test_Reset();
  if (strcmp(scenario, "missing_node") == 0)
  {
    s_drives[0].isMissing = true;
    s_drives[0].node.hasHeartbeat = false;
  }
  Test_Run(TEST_LOOPS);
  if (strcmp(scenario, "missing_node") == 0)
  {
    Test_Check(!APP_MotorStartup_IsReady(1U) && APP_MotorStartup_IsReady(2U),
               "missing node does not block startup peer");
    TEST_VelocityRun(TEST_LOOPS);
    Test_Check(g_motorVelocityDebug[0].result == APP_MOTOR_RESULT_NOT_RUN, "missing node never starts motion");
    TEST_CheckNode(1U);
  }
  else
  {
    Test_Check(APP_MotorStartup_IsReady(1U) && APP_MotorStartup_IsReady(2U), "both nodes ready");
    if (strcmp(scenario, "no_motion") == 0)
    {
      s_drives[0].freezeVelocity = true;
    }
    if (strcmp(scenario, "stop_failure") == 0)
    {
      s_drives[0].dropStop = true;
    }
    if (strcmp(scenario, "fault") == 0)
    {
      s_drives[0].statusWord = TEST_FAULT_STATUS;
      s_drives[0].errorCode = TEST_ERROR_CODE;
    }
    if (strcmp(scenario, "read_failure") == 0)
    {
      s_drives[0].failStatusRead = true;
    }
    TEST_VelocityRun(TEST_LOOPS);
    if (strcmp(scenario, "normal") == 0)
    {
      TEST_CheckNode(0U);
      TEST_CheckNode(1U);
      Test_Check(g_motorVelocityDebug[1].commands[APP_VELOCITY_COMMAND_FORWARD].startTick >
                     g_motorVelocityDebug[0].commands[APP_VELOCITY_COMMAND_STOP_REVERSE].endTick,
                 "second wheel starts only after first wheel stops");
      writes = s_drives[0].velocityWriteCount + s_drives[1].velocityWriteCount;
      TEST_VelocityRun(TEST_LOOPS);
      Test_Check(writes == s_drives[0].velocityWriteCount + s_drives[1].velocityWriteCount,
                 "completed test never repeats");
    }
    else
    {
      Test_Check(g_motorVelocityDebug[0].result == APP_MOTOR_RESULT_FAILED, "failed test retained");
      Test_Check(g_motorVelocityDebug[0].hasFailure &&
                     g_motorVelocityDebug[0].firstFailure.result == APP_MOTOR_RESULT_FAILED,
                 "first failure preserved");
      Test_Check(s_drives[0].negativeVelocityWrites == 0U, "failed node never reverses");
      Test_Check(s_drives[0].nonzeroVelocityWrites == 1U, "failed start never repeated");
      if (strcmp(scenario, "no_motion") == 0)
      {
        Test_Check(g_motorVelocityDebug[0].haltConfirmed, "stopped after false motion ACK");
        TEST_CheckNode(1U);
      }
      else
      {
        Test_Check(!g_motorVelocityDebug[0].haltConfirmed, "stop not falsely confirmed");
        Test_Check(s_drives[1].nonzeroVelocityWrites == 0U, "other node blocked until stop confirmed");
        if (strcmp(scenario, "fault") != 0)
        {
          s_drives[0].dropStop = false;
          s_drives[0].failStatusRead = false;
          TEST_VelocityRun(TEST_LOOPS);
          Test_Check(g_motorVelocityDebug[0].haltConfirmed, "stop retry succeeds after communication recovery");
          Test_Check(g_motorVelocityDebug[0].firstFailure.result == APP_MOTOR_RESULT_FAILED,
                     "stop retry does not overwrite failure");
          TEST_CheckNode(1U);
        }
      }
    }
  }
  printf("PASS: %lu velocity checks (%s).\n", (unsigned long) s_checks, scenario);
  return EXIT_SUCCESS;
}
