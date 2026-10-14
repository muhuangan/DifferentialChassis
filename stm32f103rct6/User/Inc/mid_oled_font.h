/**
  ******************************************************************************
  * @file    mid_oled_font.h
  * @brief   SSD1306 128x32 点阵屏 6x8 点阵字模，覆盖 ASCII 0x20 至 0x7B。
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
  * 每字形 6 字节，按列组织、最低位在屏幕上方，因此一个字形恰好占一页中的 6 列。
  * 表下标等于字符码减 MID_OLED_FONT_FIRST_CHAR，越界字符由驱动层替换为空格。
  *
  ******************************************************************************
  */

#ifndef __MID_OLED_FONT_H
#define __MID_OLED_FONT_H

#include <stdint.h>

#define MID_OLED_FONT_WIDTH_BYTES  6U   /**< 单字形宽度，单位字节，一字节对应一列 8 个像素。 */
#define MID_OLED_FONT_FIRST_CHAR    0x20U /**< 字模表首个字符的 ASCII 码，即可显示字符下限。 */
#define MID_OLED_FONT_LAST_CHAR     0x7BU /**< 字模表末个字符的 ASCII 码，即可显示字符上限。 */
#define MID_OLED_FONT_CHAR_COUNT    ((MID_OLED_FONT_LAST_CHAR - MID_OLED_FONT_FIRST_CHAR) + 1U) /**< 字模表字形个数。 */

extern const uint8_t g_oledFont6x8[MID_OLED_FONT_CHAR_COUNT][MID_OLED_FONT_WIDTH_BYTES];

#ifdef __cplusplus
}
#endif

#endif /* __MID_OLED_FONT_H */
