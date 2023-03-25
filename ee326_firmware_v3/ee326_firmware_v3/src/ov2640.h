/**
 * \file
 *
 * \brief API driver for OV2640 CMOS image sensor.
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef OV2640_H_INCLUDED
#define OV2640_H_INCLUDED

#include "compiler.h"
#include "board.h"
#include "twi.h"
#include "delay.h"
#include "pio.h"
#include <asf.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Slave address of OMNIVISION chip. */
#define OV_I2C_SENSOR_ADDRESS          0x30 //(0x42u >> 1)   /* OV2640 -> 0x42 */

/** Terminating list entry for register in configuration file. */
#define OV_REG_TERM                    (0xffu)

/** Terminating list entry for value in configuration file. */
#define OV_VAL_TERM                    (0xffu)

/** Register definitions. */
/* -------- OV2640_GAIN : (Address: 0x00) AGC Gain Control LSBs -------- */
#define OV2640_GAIN                    (0x00u) /**< \brief (OV2640_GAIN) AGC Gain control LSBs (MSBs in REG15[1:0]) */

/* -------- OV2640_BGAIN : (Address: 0x01) AWB - Blue channel gain setting -------- */
#define OV2640_BLUE_GAIN               (0x01u)

/* -------- OV2640_RGAIN : (Address: 0x02) AWB - Red channel gain setting -------- */
#define OV2640_RED_GAIN                (0x02u)

/* -------- OV2640_GGAIN : (Address: 0x03) AWB - Green channel gain setting -------- */
#define OV2640_GREEN_GAIN              (0x03u)

/* -------- OV2640_REG04 : (Address: 0x04) Analog setting -------- */
#define OV2640_REG04                   (0x04u)

/* -------- OV2640_BAVG : (Address: 0x05) B Channel Average -------- */
#define OV2640_BLUE_AVG                (0x05u)

/* -------- OV2640_GAVG : (Address: 0x06) G Channel Average -------- */
#define OV2640_GREEN_AVG               (0x06u)

/* -------- OV2640_RAVG : (Address: 0x07) R Channel Average -------- */
#define OV2640_RED_AVG                 (0x07u)

/* -------- OV2640_PIDH : (Address: 0x0a) Product ID number MSB -------- */
#define OV2640_PIDH                    (0x0au)
#define OV2640_PIDH_DEFAULT            (0x26u << 0)

/* -------- OV2640_PIDL : (Address: 0x0b) Product ID number LSB -------- */
#define OV2640_PIDL                    (0x0bu)
#define OV2640_PIDL_DEFAULT            (0x41u << 0)

/* -------- OV2640_REG0C : (Address: 0x0b) -------- */
#define OV2640_REG0C                   (0x0c)
#define OV2640_REG0C_MAX_EXPOSURE_Pos    (1)
#define OV2640_REG0C_MAX_EXPOSURE_Msk    (0x3u << OV2640_REG0C_MAX_EXPOSURE_Pos) /**< \brief (OV2640_REG0C) Max exposure = frame length - limit x 2 */
#define OV2640_REG0C_MAX_EXPOSURE(value) ((OV2640_REG0C_MAX_EXPOSURE_Msk & ((value) << OV2640_REG0C_MAX_EXPOSURE_Pos)))
#define OV2640_REG0C_BYTE_SWAP_Msk       (0x1u << 3) /**< \brief (OV2640_REG0C) High 8-bit MSB and LSB swap */
#define OV2640_REG0C_BYTE_SWAP_DISABLE   (0x0u << 3) /**< \brief (OV2640_REG0C) output Y9,Y8...Y3,Y2,Y1,Y0 */
#define OV2640_REG0C_BYTE_SWAP_ENABLE    (0x1u << 3) /**< \brief (OV2640_REG0C) output Y3,Y2...Y8,Y9,Y1,Y0 */
#define OV2640_REG0C_YUV_SWAP_Msk        (0x1u << 4) /**< \brief (OV2640_REG0C) YUV output, Y <-> UV swap */
#define OV2640_REG0C_YUV_SWAP_DISABLE    (0x0u << 4) /**< \brief (OV2640_REG0C) output YUYVYUYV */
#define OV2640_REG0C_YUV_SWAP_ENABLE     (0x1u << 4) /**< \brief (OV2640_REG0C) output UYVYUYVY */
#define OV2640_REG0C_MIRROR_ENABLE       (0x1u << 6) /**< \brief (OV2640_REG0C) Mirror enable */
#define OV2640_REG0C_FLIP_ENABLE         (0x1u << 7) /**< \brief (OV2640_REG0C) Flip enable */

/* -------- OV2640_REG0D : (Address: 0x0d) Analog setting -------- */
#define OV2640_REG0D                   (0x0du)

/* -------- OV2640_REG0E : (Address: 0x0e) Analog setting -------- */
/* default value: OV2640_REG0E_BLC_BOTH|OV2640_REG0E_BLC_OPTICAL */
#define OV2640_REG0E                   (0x0eu)
#define OV2640_REG0E_OUTPUT_Pos          (0)
#define OV2640_REG0E_OUTPUT_Msk          (0x3u << OV2640_REG0E_OUTPUT_Pos) /**< \brief (OV2640_REG0E) Output driving capability */
#define   OV2640_REG0E_OUTPUT_1X         (0x0u << OV2640_REG0E_OUTPUT_Pos) /**< \brief (OV2640_REG0E) 1x */
#define   OV2640_REG0E_OUTPUT_2X         (0x1u << OV2640_REG0E_OUTPUT_Pos) /**< \brief (OV2640_REG0E) 2x */
#define   OV2640_REG0E_OUTPUT_3X         (0x2u << OV2640_REG0E_OUTPUT_Pos) /**< \brief (OV2640_REG0E) 3x */
#define   OV2640_REG0E_OUTPUT_4X         (0x3u << OV2640_REG0E_OUTPUT_Pos) /**< \brief (OV2640_REG0E) 4x */
#define OV2640_REG0E_SLEEP_MODE          (0x1u << 3) /**< \brief (OV2640_REG0E) Sleep mode */
#define OV2640_REG0E_BLC_Pos             (5)
#define OV2640_REG0E_BLC_Msk             (0x3u << OV2640_REG0E_BLC_Pos) /**< \brief (OV2640_REG0E) BLC line selection */
#define OV2640_REG0E_BLC_BOTH0           (0x0u << OV2640_REG0E_BLC_Pos) /**< \brief (OV2640_REG0E) Select both blue line and red line as BLC line. */
#define OV2640_REG0E_BLC_RED             (0x1u << OV2640_REG0E_BLC_Pos) /**< \brief (OV2640_REG0E) Select red line as BLC line. */
#define OV2640_REG0E_BLC_BLUE            (0x2u << OV2640_REG0E_BLC_Pos) /**< \brief (OV2640_REG0E) Select blue line as BLC line. */
#define OV2640_REG0E_BLC_BOTH            (0x3u << OV2640_REG0E_BLC_Pos) /**< \brief (OV2640_REG0E) Select both blue line and red line as BLC line. */
#define OV2640_REG0E_BLC_LINE_Msk        (0x1u << 7) /**< \brief (OV2640_REG0E) BLC line selection */
#define OV2640_REG0E_BLC_LINE_ELECTRICAL (0x0u << 7) /**< \brief (OV2640_REG0E) Electrical BLC */
#define OV2640_REG0E_BLC_LINE_OPTICAL    (0x1u << 7) /**< \brief (OV2640_REG0E) Optical BLC */

/* -------- OV2640_HAEC : (Address: 0x0f) Automatic exposure control bit [15:8] -------- */
#define OV2640_HAEC                    (0x0fu)

/* -------- OV2640_AEC : (Address: 0x10) Automatic exposure control bit [7:0] -------- */
#define OV2640_AEC                     (0x10u)

/* -------- OV2640_CLK : (Address: 0x11) Clock settings -------- */
#define OV2640_CLK                     (0x11u) /**< \brief (OV2640_CLK) sysclk=XVCLK1 x PLLDIV / [(CLK[5:0]+1) x2 xPreDiv] */
#define OV2640_CLK_DIVIDER_Pos           (0)
#define OV2640_CLK_DIVIDER_Msk           (0x3fu << OV2640_CLK_DIVIDER_Pos) /**< \brief (OV2640_CLK) Clock divider */
#define OV2640_CLK_DIVIDER(value)        ((OV2640_CLK_DIVIDER_Msk & ((value) << OV2640_CLK_DIVIDER_Pos)))
#define OV2640_CLK_PLL_Pos               (6)
#define OV2640_CLK_PLL_Msk               (0x3u << OV2640_CLK_PLL_Pos) /**< \brief (OV2640_CLK) PLL setting - Changing this value is not recommanded */
#define OV2640_CLK_PLL(value)            ((OV2640_CLK_PLL_Msk & ((value) << OV2640_CLK_PLL_Pos)))

/* -------- OV2640_REG12 : (Address: 0x12) -------- */
#define OV2640_REG12                   (0x12u)
#define OV2640_REG12_RAW_RGB             (0x1u << 0)
#define OV2640_REG12_SENSOR_RAW          (0x1u << 4)
#define OV2640_REG12_CC656_MODE          (0x1u << 5)
#define OV2640_REG12_VSKIP               (0x1u << 6)
#define OV2640_REG12_RESET               (0x1u << 7)

/* -------- OV2640_REG13 : (Address: 0x13) -------- */
#define OV2640_REG13                   (0x13u)
#define OV2640_REG13_EXPOSURE_Msk        (0x01u << 0) /**< \brief (OV2640_REG13) Exposure auto/manual control selection */
#define OV2640_REG13_EXPOSURE_MANUAL     (0x0u << 0)
#define OV2640_REG13_EXPOSURE_AUTO       (0x1u << 0)
#define OV2640_REG13_WBAL_Msk            (0x1u << 1) /**< \brief (OV2640_REG13) Auto white balance control selection */
#define OV2640_REG13_WBAL_MANUAL         (0x0u << 1)
#define OV2640_REG13_WBAL_AUTO           (0x1u << 1)
#define OV2640_REG13_AGC_Msk             (0x1u << 2) /**< \brief (OV2640_REG13) AGC auto/manual control selection */
#define OV2640_REG13_AGC_MANUAL          (0x0u << 2)
#define OV2640_REG13_AGC_AUTO            (0x1u << 2)
#define OV2640_REG13_LAEC_Msk            (0x1u << 3) /**< \brief (OV2640_REG13) LAEC enable */
#define OV2640_REG13_LAEC_DISABLE        (0x0u << 3)
#define OV2640_REG13_LAEC_ENABLE         (0x1u << 3)
#define OV2640_REG13_BANDING_OPT_Msk     (0x1u << 4) /**< \brief (OV2640_REG13) Banding option */
#define OV2640_REG13_BANDING_OPT_LIMITED (0x0u << 4) /**< \brief (OV2640_REG13) Minimum exposure is limited to 1/120 or 1/100 second when banding filter is enabled */
#define OV2640_REG13_BANDING_OPT_ENABLE  (0x1u << 4) /**< \brief (OV2640_REG13) Minimum exposure is allowed to be less than 1/120 or 1/100 second when banding filter is enabled */
#define OV2640_REG13_BANDING_Mask        (0x1u << 5) /**< \brief (OV2640_REG13) Banding enable */
#define OV2640_REG13_BANDING_DISABLE     (0x0u << 5)
#define OV2640_REG13_BANDING_ENABLE      (0x1u << 5)
#define OV2640_REG13_FRAME_DROP_Mask     (0x1u << 6) /**< \brief (OV2640_REG13) Enable frame drop function */
#define OV2640_REG13_FRAME_DROP_DISABLE  (0x0u << 6)
#define OV2640_REG13_FRAME_DROP_ENABLE   (0x1u << 6)
#define OV2640_REG13_AEC_Mask            (0x1u << 7) /**< \brief (OV2640_REG13) AEC speed selection */
#define OV2640_REG13_AEC_NORMAL          (0x0u << 7) /**< \brief (OV2640_REG13) Normal */
#define OV2640_REG13_AEC_FASTER          (0x1u << 7) /**< \brief (OV2640_REG13) Faster AEC correction */

/* -------- OV2640_REG14 : (Address: 0x14) -------- */
#define OV2640_REG14                   (0x14u)

/* -------- OV2640_REG15 : (Address: 0x15) -------- */
#define OV2640_REG15                   (0x15u)
#define OV2640_REG15_GAIN_Pos          (0)
#define OV2640_REG15_GAIN_Msk          (0x3u << OV2640_REG15_GAIN_Pos) /**< \brief (OV2640_REG15) AGC MSBs (digital gain) (LSBs in GAIN[7:0]) */
#define OV2640_REG15_GAIN(value)       ((OV2640_REG15_GAIN_Msk & ((value) << OV2640_REG15_GAIN_Pos)))
#define OV2640_REG15_NIGHT_Mask        (0x3u << 2) /**< \brief (OV2640_REG15) Night mode triggering point */
#define OV2640_REG15_NIGHT_2X_GAIN     (0x0u << 2) /**< \brief (OV2640_REG15) 2x gain */
#define OV2640_REG15_NIGHT_4X_GAIN     (0x1u << 2) /**< \brief (OV2640_REG15) 4x gain */
#define OV2640_REG15_NIGHT_8X_GAIN     (0x2u << 2) /**< \brief (OV2640_REG15) 8x gain */
#define OV2640_REG15_NIGHT_16X_GAIN    (0x3u << 2) /**< \brief (OV2640_REG15) 16x gain */
#define OV2640_REG15_CEIL_Mask         (0x3u << 4) /**< \brief (OV2640_REG15) Ceiling of inserting frames */
#define OV2640_REG15_CEIL_0            (0x0u << 4) /**< \brief (OV2640_REG15) Up to 0 frames */
#define OV2640_REG15_CEIL_1            (0x1u << 4) /**< \brief (OV2640_REG15) Up to 1 frames */
#define OV2640_REG15_CEIL_2            (0x2u << 4) /**< \brief (OV2640_REG15) Up to 2 frames */
#define OV2640_REG15_CEIL_3            (0x3u << 4) /**< \brief (OV2640_REG15) Up to 3 frames */
#define OV2640_REG15_CEIL_7            (0x7u << 4) /**< \brief (OV2640_REG15) Up to 7 frames */
#define OV2640_REG15_ENABLE_NIGHT      (0x1u << 7)  /**< \brief (OV2640_REG15) Enable inserting frames in night mode */

/* -------- OV2640_REG16 : (Address: 0x16) -------- */
#define OV2640_REG16                   (0x16u)

/* -------- OV2640_AHSTART : (Address: 0x17) Sensor Horizontal output start point 8 MSBs (LSBs in REG16[1:0])-------- */
#define OV2640_AHSTART                 (0x17u)

/* -------- OV2640_AHSIZE : (Address: 0x18) Sensor Horizontal output size 8 MSBs (LSBs in REG16[4:3])-------- */
#define OV2640_AHSIZE                  (0x18u)

/* -------- OV2640_AVSTART : (Address: 0x19) Sensor Vertical output start point 8 MSBs (LSBs in REG16[2])-------- */
#define OV2640_AVSTART                 (0x19u)

/* -------- OV2640_AVSIZE : (Address: 0x1a) Sensor Vertical output size 8 MSBs (LSBs in REG16[5])-------- */
#define OV2640_AVSIZE                  (0x1au)

/* -------- OV2640_PIXEL_SHIFT : (Address: 0x1b) Pixel shift -------- */
#define OV2640_PIXEL_SHIFT             (0x1bu)

/* -------- OV2640_MIDH : (Address: 0x1c) Manufacturer ID Byte - High -------- */
#define OV2640_MIDH                    (0x1cu)
#define OV2640_MIDH_DEFAULT              (0x7fu << 0)

/* -------- OV2640_MIDL : (Address: 0x1d) Manufacturer ID Byte - Low -------- */
#define OV2640_MIDL                    (0x1du)
#define OV2640_MIDL_DEFAULT              (0xa2u << 0)

/* -------- OV2640_REG1E : (Address: 0x1e) -------- */
#define OV2640_REG1E                   (0x1eu)

/* -------- OV2640_REG1F : (Address: 0x1f) -------- */
#define OV2640_REG1F                   (0x1fu)

/* -------- OV2640_REG1E : (Address: 0x1e) -------- */
#define OV2640_REG1E                   (0x1eu)

/* -------- OV2640_REG20 : (Address: 0x20) -------- */
#define OV2640_REG20                   (0x20u)

/* -------- OV2640_REG21 : (Address: 0x21) -------- */
#define OV2640_REG21                   (0x21u)

/* -------- OV2640_REG21 : (Address: 0x24) Luminance signal high range for AEC/AGC operation -------- */
#define OV2640_WPT                     (0x24u)

/* -------- OV2640_REG21 : (Address: 0x25) Luminance signal low range for AEC/AGC operation -------- */
#define OV2640_BPT                     (0x25u)

/* -------- OV2640_VPT : (Address: 0x26) effective only in AEG/AGC fast mode -------- */
#define OV2640_VPT                     (0x26u)

/* -------- OV2640_REG27 : (Address: 0x27) -------- */
#define OV2640_REG27                   (0x27u)
#define OV2640_REG27_BLACKSUN            (0x1u << 7) /**< \brief (OV2640_REG27) Black sun cancellation enable */

/* -------- OV2640_REG28 : (Address: 0x28) -------- */
#define OV2640_REG28                   (0x28u)
#define OV2640_REG28_VSYNC_Msk           (0x1u << 1) /**< \brief (OV2640_REG28) VSYNC polarity */
#define OV2640_REG28_VSYNC_POSITIVE      (0x1u << 0) /**< \brief (OV2640_REG28) Positive */
#define OV2640_REG28_VSYNC_NEGATIVE      (0x1u << 1) /**< \brief (OV2640_REG28) Negative */
#define OV2640_REG28_VSYNC_OUTPUT_Msk    (0x1u << 3) /**< \brief (OV2640_REG28) No VSYNC output option */
#define OV2640_REG28_VSYNC_OUTPUT_STILL  (0x0u << 3) /**< \brief (OV2640_REG28) Still output VSYNC when frame drop */
#define OV2640_REG28_VSYNC_OUTPUT_NONE   (0x1u << 3) /**< \brief (OV2640_REG28) No VSYNC output when frame drop */
#define OV2640_REG28_HREF_Msk            (0x1u << 4) /**< \brief (OV2640_REG28) HREF polarity */
#define OV2640_REG28_HREF_POSITIVE       (0x0u << 4) /**< \brief (OV2640_REG28) Output positive HREF */
#define OV2640_REG28_HREF_NEGATIVE       (0x1u << 4) /**< \brief (OV2640_REG28) Output negative HREF for data valid */
#define OV2640_REG28_HSYNC_Msk           (0x1u << 5) /**< \brief (OV2640_REG28) HSYNC polarity */
#define OV2640_REG28_HSYNC_POSITIVE      (0x0u << 5) /**< \brief (OV2640_REG28) Positive */
#define OV2640_REG28_HSYNC_NEGATIVE      (0x1u << 5) /**< \brief (OV2640_REG28) Negative */
#define OV2640_REG28_HREF_OUTPUT_Msk     (0x1u << 6) /**< \brief (OV2640_REG28) HREF pin output swap */
#define OV2640_REG28_HREF_OUTPUT_HREF    (0x0u << 6) /**< \brief (OV2640_REG28) HREF */
#define OV2640_REG28_HREF_OUTPUT_HSYNC   (0x1u << 6) /**< \brief (OV2640_REG28) HSYNC */
#define OV2640_REG28_OUTPUT_REVERSE      (0x1u << 7) /**< \brief (OV2640_REG28) Output data bit reverse option */

/* -------- OV2640_REG65 : (Address: 0x65) -------- */
#define OV2640_REG65                  (0x65u)
#define OV2640_REG65_BIT_SWAP_Msk       (0x1u << 3) /**< \brief (OV2640_REG65) Output data bit swap option */
#define OV2640_REG65_BIT_SWAP_NORMAL    (0x0u << 3) /**< \brief (OV2640_REG65) Output DATA[9:0] */
#define OV2640_REG65_BIT_SWAP_REVERSE   (0x1u << 3) /**< \brief (OV2640_REG65) Output DATA[0:9] */

/* -------- OV2640_YUV422CTRL : (Address: 0xd9) -------- */
#define OV2640_YUV422CTRL             (0xd9u)

#define OV2640_YUV422CTRL_CNV_OPT_Msk   (0x1u << 0) /**< \brief (OV2640_YUV422CTRL) cnv_opt */
#define OV2640_YUV422CTRL_CNV_OPT_AVERAGE (0x0u << 0) /**< \brief (OV2640_YUV422CTRL) Average mode */
#define OV2640_YUV422CTRL_CNV_OPT_DROP    (0x1u << 0) /**< \brief (OV2640_YUV422CTRL) Drop mode */

#define OV2640_YUV422CTRL_V_FIRST_Msk   (0x1u << 1) /**< \brief (OV2640_YUV422CTRL) v_first */
#define OV2640_YUV422CTRL_V_FIRST_YUYV  (0x0u << 1) /**< \brief (OV2640_YUV422CTRL) Output line will be YUYV... */
#define OV2640_YUV422CTRL_V_FIRST_YVYU  (0x1u << 1) /**< \brief (OV2640_YUV422CTRL) Output line will be YVYU... (it will affect definition of U/V in SDE. If it is set, all registers in SDE about U/V must be swapped */

/**
 * OV2640 register struture.
 */
typedef struct _ov_reg {
	uint8_t reg; /*!< Register to be written */
	uint8_t val; /*!< Value to be written in the register */
} ov_reg;

/**
 * Enumeration of different format which could be used by the 0V7740 sensor.
 * These enumeration allow to configure the OV2640 registers using
 * the OV2640_table_registre.c Array.
 */
typedef enum _e_OV2640_format {
	JPEG_INIT,
	YUV422,
	JPEG,
	JPEG_320x240,
	JPEG_640x480,
	JPEG_800x600,
	JPEG_1024x768,
	JPEG_1280x1024,
	JPEG_1600x1200,
	QVGA_YUV422_10FPS, /*!< Register to be written  */
	QVGA_YUV422_15FPS, /*!< Register to be written */
	QVGA_YUV422_20FPS, /*!< Register to be written */
	QVGA_YUV422_30FPS, /*!< Register to be written */
	QVGA_RGB888, /*!< Register to be written */
	QQVGA_YUV422, /*!< Register to be written */
	QQVGA_RGB888, /*!< Register to be written */
	TEST_PATTERN, /*!< Register to be written */
	VGA_YUV422_20FPS /*!< Register to be written */
} e_OV2640_format;

extern const ov_reg OV2640_JPEG_INIT[];
extern const ov_reg OV2640_YUV422[];
extern const ov_reg OV2640_JPEG[];
extern const ov_reg OV2640_JPEG_320x240[];
extern const ov_reg OV2640_JPEG_640x480[];
extern const ov_reg OV2640_JPEG_800x600[];
extern const ov_reg OV2640_JPEG_1024x768[];
extern const ov_reg OV2640_JPEG_1280x1024[];
extern const ov_reg OV2640_JPEG_1600x1200[];
extern const ov_reg OV2640_QVGA_YUV422_10FPS[];
extern const ov_reg OV2640_QVGA_YUV422_15FPS[];
extern const ov_reg OV2640_QVGA_YUV422_20FPS[];
extern const ov_reg OV2640_QVGA_YUV422_30FPS[];
extern const ov_reg OV2640_QVGA_RGB888[];
extern const ov_reg OV2640_QQVGA_YUV422[];
extern const ov_reg OV2640_QQVGA_RGB888[];
extern const ov_reg OV2640_TEST_PATTERN[];
extern const ov_reg OV2640_VGA_YUV422_20FPS[];

void ov_power(uint32_t on_off, Pio* const p_pio, const uint32_t ul_mask);
void ov_reset(Pio* const p_pio, const uint32_t ul_mask);
uint32_t ov_read_reg(Twi* const p_twi, twi_packet_t *p_packet);
uint32_t ov_write_reg(Twi* const p_twi, twi_packet_t* const p_packet);
uint32_t ov_write_regs(Twi* const p_twi, const ov_reg *p_reg_list);
void ov_dump_registers(Twi* const p_twi, ov_reg *p_regs);
uint32_t ov_init(Twi* const p_twi );
uint32_t ov_configure(Twi* const p_twi, const e_OV2640_format format);
uint32_t ov_configure_finish(Twi* const p_twi);
uint32_t ov_configure_manual(Twi* const p_twi);
uint32_t ov_store_manual(Twi* const p_twi, volatile uint32_t *p_backup_addr,
		uint32_t ul_size);
uint32_t ov_restore_manual(volatile uint32_t *p_backup_addr, uint32_t ul_size);

#ifdef __cplusplus
}
#endif

/**
 * \page sam_component_OV2640_quick_start Quick Start Guide for the OV2640
 * component.
 *
 * This is the quick start guide for the \ref OV2640_sensor_group, with
 * step-by-step instructions on how to configure and use the driver for
 * a specific use case. The code examples can be copied into e.g the main
 * application loop or any other function that will need to control the
 * OV2640 CMOS image sensor.
 *
 * \section OV2640_qs_use_cases Use cases
 * - \ref OV2640_basic
 *
 * \section OV2640_basic OV2640 basic usage
 *
 * This use case will demonstrate how to initialize the OV2640 sensor
 * module.
 *
 * \section OV2640_basic_setup Setup steps
 *
 * \subsection OV2640_basic_prereq Prerequisites
 *
 * This module requires the following driver
 * - \ref group_sam_pio_capture
 *
 * \subsection OV2640_basic_usage_code
 *
 * If the OV2640 is clocked using Peripheral Clock, the according PLL must be
 * configured to ouput the right frequency on the given PCK pin.
 * If the microcontroller has at least two PLL, another PLL can be used to
 * clock the system at different frequency.
 * Then the OV2640 can be configured according to the system requirements.
 *
 * Other external components, like SRAM/SDRAM have to be configured also with
 * correct SMC settings.
 *
 * Finally, the PIO capture mode has to be configured for acquisition.
 *
 * \endcode
 */

#endif /* OV2640_H_INCLUDED */
