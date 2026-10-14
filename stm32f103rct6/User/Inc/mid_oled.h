/**
  ******************************************************************************
  * @file    mid_oled.h
  * @brief   SSD1306 128x32 点阵屏驱动，软件 I2C 时序加页地址帧缓冲。
  * @author  muhuangan
  * @date    2026-10-14
  * @version V1.0.0
  ******************************************************************************
  * @attention
  *
  * Copyright 2026 muhuangan.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  * 屏幕为 128 列 x 4 页，每页 8 行像素，因此整帧 512 字节。
  * 所有绘制都先落在帧缓冲里，MID_Oled_Flush 才按页一次性推给屏幕，
  * 避免逐字节直写在翻页时出现撕裂与闪烁。
  * 软件 I2C 使用 CubeMX 已配置的 IIC_SDA / IIC_SCL 引脚，不占用硬件 I2C 外设。
  *
  ******************************************************************************
  */

#ifndef __MID_OLED_H
#define __MID_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define MID_OLED_COLUMNS            128U /**< 屏幕列数，单位像素。 */
#define MID_OLED_PAGES              4U   /**< 屏幕页数，每页 8 行像素。 */
#define MID_OLED_PAGE_ROWS          8U   /**< 单页行数，单位像素。 */
#define MID_OLED_FRAME_BYTES        (MID_OLED_COLUMNS * MID_OLED_PAGES) /**< 帧缓冲字节数。 */
#define MID_OLED_I2C_ADDRESS        0x3CU /**< 7 位从机地址，SSD1306 常用 0x3C 或 0x3D。 */
#define MID_OLED_I2C_ADDRESS_ALT    0x3DU /**< 备用 7 位从机地址，模块 SA0 接高时为该值，初始化时自动探测。 */

/**
  * @brief  初始化屏幕并清空帧缓冲。
  * @param  None
  * @retval None
  * @note   上电后需等待屏幕自身复位完成再调用；若初始化早于屏幕就绪，表现为全黑无显示，
  *         重新上电即可恢复。调用后帧缓冲全零，但不会立即推送到屏幕，需再调 MID_Oled_Flush。
  */
void MID_Oled_Init(void);

/**
  * @brief  把帧缓冲整帧推送到屏幕。
  * @param  None
  * @retval None
  * @note   按页写入，每页先送页内列地址再送 128 字节数据；传输期间不做其它 I2C 操作。
  */
void MID_Oled_Flush(void);

/**
  * @brief  清空帧缓冲。
  * @param  None
  * @retval None
  * @note   只动帧缓冲，不改变屏幕内容，需配合 MID_Oled_Flush 才能看到清屏效果。
  */
void MID_Oled_Clear(void);

/**
  * @brief  在指定页绘制字符串。
  * @param  column 起始列，取值范围 0 至 MID_OLED_COLUMNS 减字宽。
  * @param  page   目标页，取值范围 0 至 MID_OLED_PAGES 减 1。
  * @param  pText  待显示字符串，不可为 NULL，遇不可显示字符按空格处理。
  * @retval None
  * @note   字宽固定 6 列，超出右边界部分被裁掉，不会回绕到下一页。
  */
void MID_Oled_DrawString(uint8_t column, uint8_t page, const char *pText);

/**
  * @brief  在指定页绘制无符号整数。
  * @param  column     起始列，取值范围 0 至 MID_OLED_COLUMNS 减字宽。
  * @param  page       目标页，取值范围 0 至 MID_OLED_PAGES 减 1。
  * @param  value      待显示数值。
  * @param  minDigits  最小位数，不足时左侧补零；为 0 表示不补零。
  * @retval None
  * @note   十进制显示，不带符号；数值较大时按实际位数绘制并在右边界处裁掉。
  */
void MID_Oled_DrawUint(uint8_t column, uint8_t page, uint32_t value, uint8_t minDigits);

/**
  * @brief  绘制一个填充比例条。
  * @param  column  起始列，取值范围 0 至 MID_OLED_COLUMNS 减 width。
  * @param  page   目标页，取值范围 0 至 MID_OLED_PAGES 减 1。
  * @param  width  进度条总宽，单位列。
  * @param  permille 填充比例，单位千分之一，0 至 1000，超出时按边界处理。
  * @retval None
  * @note   用第 7 行作底边框、第 0 至 6 行作填充，因此占用一页高度。
  */
void MID_Oled_DrawBar(uint8_t column, uint8_t page, uint8_t width, uint16_t permille);

/**
  * @brief  直接读写帧缓冲中的一个字节。
  * @param  column 列号，取值范围 0 至 MID_OLED_COLUMNS 减 1。
  * @param  page   页号，取值范围 0 至 MID_OLED_PAGES 减 1。
  * @param  value  写入的字节数据，每一位对应页内一行，最低位在屏幕上方。
  * @retval None
  * @note   供需要自绘图形的调用方使用，越界时静默丢弃。
  */
void MID_Oled_SetByte(uint8_t column, uint8_t page, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* __MID_OLED_H */
