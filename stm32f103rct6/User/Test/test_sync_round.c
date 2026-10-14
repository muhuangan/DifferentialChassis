/**
 ******************************************************************************
 * @file test_sync_round.c
 * @brief 模拟速度斜坡和位置积分，验证双轮同转一圈及共同停机边界。
 * @author muhuangan
 * @attention Copyright 2026 muhuangan. MIT License; see project LICENSE.
 ******************************************************************************
 */
#define main TEST_UnusedStartupSuite
#include "test_motor_startup.c"
#undef main

/** @defgroup TEST_SYNC_Config 双轮模拟配置
 * @{
 */
#define TEST_SYNC_LOOPS 7000U
static const uint8_t s_syncNodeIds[TEST_NODE_COUNT] = {1U, 2U};
/** @} */

/** @brief 推进带加减速的位置反馈模拟，并可在起转后注入读取失败。 */
static void TEST_SyncRun(uint32_t loops, bool injectReadFailure)
{
  uint32_t loop;
  for (loop = 0U; loop < loops; loop++)
  {
    Test_Run(1U);
    if (injectReadFailure && g_motorSyncDebug.started)
    {
      s_drives[0].failPositionRead = true;
    }
    APP_MotorStartup_SyncRoundProcess(s_syncNodeIds, TEST_NODE_COUNT);
  }
}

/** @brief 在起转后一次性注入超速反馈，随后仍由斜坡模型模拟实际停止。 */
static void TEST_SyncOverspeed(void)
{
  uint32_t loop;
  bool     injected = false;
  for (loop = 0U; loop < TEST_SYNC_LOOPS; loop++)
  {
    Test_Run(1U);
    if (!injected && g_motorSyncDebug.started)
    {
      s_drives[0].actualVelocity = CHASSIS_LINK_SELF_TEST_SPEED_PPS * 2;
      injected = true;
    }
    APP_MotorStartup_SyncRoundProcess(s_syncNodeIds, TEST_NODE_COUNT);
  }
}

/** @brief 检查电机累计脉冲数乘 10:1 减速比后等于轮子一圈，并确认最终停稳。 */
static void TEST_SyncCheckWheel(uint8_t node)
{
  int64_t distance = (int64_t) s_drives[node].actualPosition - g_motorSyncDebug.wheels[node].startPosition;
  int64_t error =
      distance - (int64_t) s_drives[node].pulsesPerRev * CHASSIS_LINK_GEAR_NUMERATOR / CHASSIS_LINK_GEAR_DENOMINATOR;
  Test_Check(g_motorSyncDebug.wheels[node].done, "wheel confirmed done");
  Test_Check(g_motorSyncDebug.wheels[node].targetPulses == s_drives[node].pulsesPerRev * 10U,
             "wheel revolution uses motor pulses times gearbox ratio");
  Test_Check(error <= CHASSIS_LINK_SYNC_POSITION_TOLERANCE_PULSES &&
                 error >= -CHASSIS_LINK_SYNC_POSITION_TOLERANCE_PULSES,
             "one revolution within position tolerance");
  Test_Check(s_drives[node].actualVelocity == 0 && s_drives[node].targetVelocity == 0,
             "final target and actual speed zero");
  Test_Check(g_motorSyncDebug.wheels[node].peakVelocity == CHASSIS_LINK_SELF_TEST_SPEED_PPS, "fourfold speed reached");
  Test_Check(s_drives[node].negativeVelocityWrites == 0U, "no automatic reverse correction");
  Test_Check(s_drives[node].mode == MID_LISAN_EC_MODE_PROFILE_VELOCITY && s_drives[node].positionWriteCount == 0U,
             "keeps velocity mode and does not issue position commands");
}

/** @brief 每个进程模拟一次上电，验证同转、减速、故障和停止恢复。 */
int main(int argc, char **argv)
{
  const char *scenario = (argc > 1) ? argv[1] : "normal";
  uint8_t     node;
  uint32_t    writes;
  Test_Reset();
  if (strcmp(scenario, "missing_node") == 0)
  {
    s_drives[0].isMissing = true;
    s_drives[0].node.hasHeartbeat = false;
  }
  Test_Run(TEST_LOOPS);
  for (node = 0U; node < TEST_NODE_COUNT; node++)
  {
    s_drives[node].simulateRamp = true;
    s_drives[node].actualPosition = (node == 0U) ? 1000 : -1000;
  }
  if (strcmp(scenario, "different_ppr") == 0)
  {
    s_drives[1].pulsesPerRev = 2000U;
  }
  if (strcmp(scenario, "bad_ppr") == 0)
  {
    s_drives[0].pulsesPerRev = 0U;
  }
  if (strcmp(scenario, "overflow_ppr") == 0)
  {
    s_drives[0].pulsesPerRev = UINT32_MAX;
  }
  if (strcmp(scenario, "start_failure") == 0)
  {
    s_drives[1].failNonzeroVelocity = true;
  }
  if (strcmp(scenario, "stall") == 0 || strcmp(scenario, "stop_failure") == 0)
  {
    s_drives[0].freezePosition = true;
  }
  if (strcmp(scenario, "stop_failure") == 0)
  {
    s_drives[1].dropStop = true;
  }
  if (strcmp(scenario, "overspeed") == 0)
  {
    TEST_SyncOverspeed();
  }
  else
  {
    TEST_SyncRun(TEST_SYNC_LOOPS, strcmp(scenario, "read_failure") == 0);
  }
  if (strcmp(scenario, "normal") == 0 || strcmp(scenario, "different_ppr") == 0)
  {
    Test_Check(g_motorSyncDebug.result == APP_MOTOR_RESULT_VERIFIED, "both wheels complete a revolution");
    TEST_SyncCheckWheel(0U);
    TEST_SyncCheckWheel(1U);
    Test_Check(g_motorSyncDebug.wheels[1].startCommandTick - g_motorSyncDebug.wheels[0].startCommandTick <=
                   2U * TEST_TICK_MS,
               "start commands closely spaced instead of sequential full tests");
    writes = s_drives[0].velocityWriteCount + s_drives[1].velocityWriteCount;
    TEST_SyncRun(TEST_LOOPS, false);
    Test_Check(writes == s_drives[0].velocityWriteCount + s_drives[1].velocityWriteCount,
               "completed joint test does not repeat");
  }
  else
  {
    Test_Check(g_motorSyncDebug.result == APP_MOTOR_RESULT_FAILED, "failure is not reported as completion");
    if (strcmp(scenario, "bad_ppr") == 0 || strcmp(scenario, "overflow_ppr") == 0 ||
        strcmp(scenario, "missing_node") == 0)
    {
      Test_Check(s_drives[0].nonzeroVelocityWrites == 0U && s_drives[1].nonzeroVelocityWrites == 0U,
                 "both must be ready with valid pulses per revolution before motion");
    }
    if (strcmp(scenario, "start_failure") == 0)
    {
      Test_Check(g_motorSyncDebug.firstFailure.nodeId == 2U &&
                     g_motorSyncDebug.firstFailure.operationStatus[APP_SYNC_PHASE_START] == MID_CANOPEN_TIMEOUT,
                 "failed second start retains its transaction result");
    }
    if (strcmp(scenario, "stall") == 0 || strcmp(scenario, "stop_failure") == 0)
    {
      Test_Check(g_motorSyncDebug.failureReason == APP_SYNC_FAILURE_STALL, "frozen position feedback detected");
    }
    if (strcmp(scenario, "read_failure") == 0)
    {
      Test_Check(g_motorSyncDebug.failureReason == APP_SYNC_FAILURE_SDO &&
                     g_motorSyncDebug.firstFailure.operationStatus[APP_SYNC_PHASE_POSITION] == MID_CANOPEN_TIMEOUT,
                 "failed runtime position read recorded");
    }
    if (strcmp(scenario, "overspeed") == 0)
    {
      Test_Check(g_motorSyncDebug.failureReason == APP_SYNC_FAILURE_DRIVE, "overspeed stops the joint test");
    }
    if (strcmp(scenario, "missing_node") == 0 || strcmp(scenario, "stop_failure") == 0)
    {
      Test_Check(!g_motorSyncDebug.haltConfirmed, "unacknowledged stop is not falsely confirmed");
      if (strcmp(scenario, "stop_failure") == 0)
      {
        s_drives[1].dropStop = false;
        TEST_SyncRun(TEST_LOOPS, false);
        Test_Check(g_motorSyncDebug.haltConfirmed, "both stop after communication recovery");
        Test_Check(g_motorSyncDebug.failureReason == APP_SYNC_FAILURE_STALL, "stop recovery retains original failure");
      }
    }
    else
    {
      Test_Check(g_motorSyncDebug.haltConfirmed, "failure followed by confirmed stop of both motors");
    }
    if (strcmp(scenario, "missing_node") != 0)
    {
      Test_Check(s_drives[0].actualVelocity == 0 && s_drives[1].actualVelocity == 0, "both motors end stationary");
    }
  }
  printf("PASS: %lu joint-round checks (%s).\n", (unsigned long) s_checks, scenario);
  return EXIT_SUCCESS;
}
