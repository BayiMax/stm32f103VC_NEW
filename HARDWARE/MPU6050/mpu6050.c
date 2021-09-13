#include "main.h"
#include "mpu6050.h"
#include "delay.h"
#include "stdio.h"

#if MPU6050==1

/********/
#define MPU_IIC_SDA(n)	n?(GPIOB->BSRR=GPIO_Pin_12):\
													(GPIOB->BRR=GPIO_Pin_12)
#define MPU_IIC_SCL(n)	n?(GPIOB->BSRR=GPIO_Pin_11):\
													(GPIOB->BRR=GPIO_Pin_11)
#define MPU_READ_SDA	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
/********/
#define min(a,b)	((a<b)?a:b)
#define DMP_MAX_PACKET_LENGTH   (32)
/*驱动寄存器*/
struct gyro_reg_s {
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
    unsigned char prod_id;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char motion_thr;
    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
    unsigned char mem_r_w;
    unsigned char accel_offs;
    unsigned char i2c_mst;
    unsigned char bank_sel;
    unsigned char mem_start_addr;
    unsigned char prgm_start_h;
};
/*MPU6050信息*/
struct hw_s {
    unsigned char addr;
    unsigned short max_fifo;
    unsigned char num_reg;
    unsigned short temp_sens;
    short temp_offset;
    unsigned short bank_size;
};
/*旧信息*/
struct motion_int_cache_s {
    unsigned short gyro_fsr;
    unsigned char accel_fsr;
    unsigned short lpf;
    unsigned short sample_rate;
    unsigned char sensors_on;
    unsigned char fifo_sensors;
    unsigned char dmp_on;
};
/*芯片数据缓存*/
struct chip_cfg_s {
    /* Matches gyro_cfg >> 3 & 0x03 */
    unsigned char gyro_fsr;
    /* Matches accel_cfg >> 3 & 0x03 */
    unsigned char accel_fsr;
    /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
    unsigned char sensors;
    /* Matches config register. */
    unsigned char lpf;
    unsigned char clk_src;
    /* Sample rate, NOT rate divider. */
    unsigned short sample_rate;
    /* Matches fifo_en register. */
    unsigned char fifo_enable;
    /* Matches int enable register. */
    unsigned char int_enable;
    /* 1 if devices on auxiliary I2C bus appear on the primary. */
    unsigned char bypass_mode;
    /* 1 if half-sensitivity.
     * NOTE: This doesn't belong here, but everything else in hw_s is const,
     * and this allows us to save some precious RAM.
     */
    unsigned char accel_half;
    /* 1 if device in low-power accel-only mode. */
    unsigned char lp_accel_mode;
    /* 1 if interrupts are only triggered on motion events. */
    unsigned char int_motion_only;
    struct motion_int_cache_s cache;
    /* 1 for active low interrupts. */
    unsigned char active_low_int;
    /* 1 for latched interrupts. */
    unsigned char latched_int;
    /* 1 if DMP is enabled. */
    unsigned char dmp_on;
    /* Ensures that DMP will only be loaded once.*/
    unsigned char dmp_loaded;
    /* Sampling rate used when DMP is enabled.*/
    unsigned short dmp_sample_rate;
};
/*自检*/
struct test_s {
	unsigned long gyro_sens;
	unsigned long accel_sens;
	unsigned char reg_rate_div;
	unsigned char reg_lpf;
	unsigned char reg_gyro_fsr;
	unsigned char reg_accel_fsr;
	unsigned short wait_ms;
	unsigned char packet_thresh;
	float min_dps;
	float max_dps;
	float max_gyro_var;
	float min_g;
	float max_g;
	float max_accel_var;
};

/*陀螺仪驱动状态*/
struct gyro_state_s {
	const struct gyro_reg_s *reg;
	const struct hw_s *hw;
	struct chip_cfg_s chip_cfg;
	const struct test_s *test;
};
/*过滤器配置*/
enum lpf_e {
	INV_FILTER_256HZ_NOLPF2 = 0,
	INV_FILTER_188HZ,
	INV_FILTER_98HZ,
	INV_FILTER_42HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_2100HZ_NOLPF,
	NUM_FILTER
};
/*mpu陀螺仪量程*/
enum gyro_fsr_e {
	INV_FSR_250DPS = 0,
	INV_FSR_500DPS,
	INV_FSR_1000DPS,
	INV_FSR_2000DPS,
	NUM_GYRO_FSR
};
/*MPU加速度量程*/
enum accel_fsr_e {
	INV_FSR_2G = 0,
	INV_FSR_4G,
	INV_FSR_8G,
	INV_FSR_16G,
	NUM_ACCEL_FSR
};
/*mpu时钟源选择*/
enum clock_sel_e {
	INV_CLK_INTERNAL = 0,
	INV_CLK_PLL,
	NUM_CLK
};
/*唤醒速率*/
enum lp_accel_rate_e {
	INV_LPA_1_25HZ,
	INV_LPA_5HZ,
	INV_LPA_20HZ,
	INV_LPA_40HZ
};
#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_MPU_Read_Len        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)
/**/
const struct gyro_reg_s reg = {
0x75,  //who_am_i
0x19,  //rate_div
0x1A,  //lpf
0x0C,  //prod_id
0x6A,  //user_ctrl
0x23,  //fifo_en
0x1B,  //gyro_cfg
0x1C,  //accel_cfg
0x1F,  // motion_thr
0x20,  // motion_dur
0x72,  // fifo_count_h
0x74,  // fifo_r_w
0x43,  // raw_gyro
0x3B,  // raw_accel
0x41,  // temp
0x38,  // int_enable
0x39,  //  dmp_int_status
0x3A,  //  int_status
0x6B,  // pwr_mgmt_1
0x6C,  // pwr_mgmt_2
0x37,  // int_pin_cfg
0x6F,  // mem_r_w
0x06,  // accel_offs
0x24,  // i2c_mst
0x6D,  // bank_sel
0x6E,  // mem_start_addr
0x70   // prgm_start_h
};
/**/
const struct hw_s hw={
  0x68,	 //addr
  1024,	 //max_fifo
  118,	 //num_reg
  340,	 //temp_sens
  -521,	 //temp_offset
  256	 //bank_size
};
const struct test_s test={
32768/250,		 //gyro_sens
32768/16,		 //	accel_sens
0,				 //	reg_rate_div
1,				//	reg_lpf
0,				 //	reg_gyro_fsr
0x18,			//	reg_accel_fsr
50,				//	wait_ms
5,				//	packet_thresh
10.0f,			 //	min_dps
105.0f,			 //	max_dps
0.14f,			//	max_gyro_var
0.3f,		   //	min_g
0.95f,		   //	max_g
0.14f		   //	max_accel_var
};
static struct gyro_state_s st={
  &reg,
  &hw,
  {0},
  &test
};
/*最大数据长度*/
#define MAX_PACKET_LENGTH (12)
/*移植inv_mpu_dmp_motion_driver.h
传感器驱动*/
/*这些定义是从一般MPI *版本中的dmpDefaultMPU6050.c中复制的。
不同的DMP映像，定义可能会发生变化，在切换到新映像时,修改这些值*/
#define CFG_LP_QUAT             (2712)
#define END_ORIENT_TEMP         (1866)
#define CFG_27                  (2742)
#define CFG_20                  (2224)
#define CFG_23                  (2745)
#define CFG_FIFO_ON_EVENT       (2690)
#define END_PREDICTION_UPDATE   (1761)
#define CGNOTICE_INTR           (2620)
#define X_GRT_Y_TMP             (1358)
#define CFG_DR_INT              (1029)
#define CFG_AUTH                (1035)
#define UPDATE_PROP_ROT         (1835)
#define END_COMPARE_Y_X_TMP2    (1455)
#define SKIP_X_GRT_Y_TMP        (1359)
#define SKIP_END_COMPARE        (1435)
#define FCFG_3                  (1088)
#define FCFG_2                  (1066)
#define FCFG_1                  (1062)
#define END_COMPARE_Y_X_TMP3    (1434)
#define FCFG_7                  (1073)
#define FCFG_6                  (1106)
#define FLAT_STATE_END          (1713)
#define SWING_END_4             (1616)
#define SWING_END_2             (1565)
#define SWING_END_3             (1587)
#define SWING_END_1             (1550)
#define CFG_8                   (2718)
#define CFG_15                  (2727)
#define CFG_16                  (2746)
#define CFG_EXT_GYRO_BIAS       (1189)
#define END_COMPARE_Y_X_TMP     (1407)
#define DO_NOT_UPDATE_PROP_ROT  (1839)
#define CFG_7                   (1205)
#define FLAT_STATE_END_TEMP     (1683)
#define END_COMPARE_Y_X         (1484)
#define SKIP_SWING_END_1        (1551)
#define SKIP_SWING_END_3        (1588)
#define SKIP_SWING_END_2        (1566)
#define TILTG75_START           (1672)
#define CFG_6                   (2753)
#define TILTL75_END             (1669)
#define END_ORIENT              (1884)
#define CFG_FLICK_IN            (2573)
#define TILTL75_START           (1643)
#define CFG_MOTION_BIAS         (1208)
#define X_GRT_Y                 (1408)
#define TEMPLABEL               (2324)
#define CFG_ANDROID_ORIENT_INT  (1853)
#define CFG_GYRO_RAW_DATA       (2722)
#define X_GRT_Y_TMP2            (1379)

#define D_0_22                  (22+512)
#define D_0_24                  (24+512)

#define D_0_36                  (36)
#define D_0_52                  (52)
#define D_0_96                  (96)
#define D_0_104                 (104)
#define D_0_108                 (108)
#define D_0_163                 (163)
#define D_0_188                 (188)
#define D_0_192                 (192)
#define D_0_224                 (224)
#define D_0_228                 (228)
#define D_0_232                 (232)
#define D_0_236                 (236)

#define D_1_2                   (256 + 2)
#define D_1_4                   (256 + 4)
#define D_1_8                   (256 + 8)
#define D_1_10                  (256 + 10)
#define D_1_24                  (256 + 24)
#define D_1_28                  (256 + 28)
#define D_1_36                  (256 + 36)
#define D_1_40                  (256 + 40)
#define D_1_44                  (256 + 44)
#define D_1_72                  (256 + 72)
#define D_1_74                  (256 + 74)
#define D_1_79                  (256 + 79)
#define D_1_88                  (256 + 88)
#define D_1_90                  (256 + 90)
#define D_1_92                  (256 + 92)
#define D_1_96                  (256 + 96)
#define D_1_98                  (256 + 98)
#define D_1_106                 (256 + 106)
#define D_1_108                 (256 + 108)
#define D_1_112                 (256 + 112)
#define D_1_128                 (256 + 144)
#define D_1_152                 (256 + 12)
#define D_1_160                 (256 + 160)
#define D_1_176                 (256 + 176)
#define D_1_178                 (256 + 178)
#define D_1_218                 (256 + 218)
#define D_1_232                 (256 + 232)
#define D_1_236                 (256 + 236)
#define D_1_240                 (256 + 240)
#define D_1_244                 (256 + 244)
#define D_1_250                 (256 + 250)
#define D_1_252                 (256 + 252)
#define D_2_12                  (512 + 12)
#define D_2_96                  (512 + 96)
#define D_2_108                 (512 + 108)
#define D_2_208                 (512 + 208)
#define D_2_224                 (512 + 224)
#define D_2_236                 (512 + 236)
#define D_2_244                 (512 + 244)
#define D_2_248                 (512 + 248)
#define D_2_252                 (512 + 252)

#define CPASS_BIAS_X            (35 * 16 + 4)
#define CPASS_BIAS_Y            (35 * 16 + 8)
#define CPASS_BIAS_Z            (35 * 16 + 12)
#define CPASS_MTX_00            (36 * 16)
#define CPASS_MTX_01            (36 * 16 + 4)
#define CPASS_MTX_02            (36 * 16 + 8)
#define CPASS_MTX_10            (36 * 16 + 12)
#define CPASS_MTX_11            (37 * 16)
#define CPASS_MTX_12            (37 * 16 + 4)
#define CPASS_MTX_20            (37 * 16 + 8)
#define CPASS_MTX_21            (37 * 16 + 12)
#define CPASS_MTX_22            (43 * 16 + 12)
#define D_EXT_GYRO_BIAS_X       (61 * 16)
#define D_EXT_GYRO_BIAS_Y       (61 * 16) + 4
#define D_EXT_GYRO_BIAS_Z       (61 * 16) + 8
#define D_ACT0                  (40 * 16)
#define D_ACSX                  (40 * 16 + 4)
#define D_ACSY                  (40 * 16 + 8)
#define D_ACSZ                  (40 * 16 + 12)

#define FLICK_MSG               (45 * 16 + 4)
#define FLICK_COUNTER           (45 * 16 + 8)
#define FLICK_LOWER             (45 * 16 + 12)
#define FLICK_UPPER             (46 * 16 + 12)

#define D_AUTH_OUT              (992)
#define D_AUTH_IN               (996)
#define D_AUTH_A                (1000)
#define D_AUTH_B                (1004)

#define D_PEDSTD_BP_B           (768 + 0x1C)
#define D_PEDSTD_HP_A           (768 + 0x78)
#define D_PEDSTD_HP_B           (768 + 0x7C)
#define D_PEDSTD_BP_A4          (768 + 0x40)
#define D_PEDSTD_BP_A3          (768 + 0x44)
#define D_PEDSTD_BP_A2          (768 + 0x48)
#define D_PEDSTD_BP_A1          (768 + 0x4C)
#define D_PEDSTD_INT_THRSH      (768 + 0x68)
#define D_PEDSTD_CLIP           (768 + 0x6C)
#define D_PEDSTD_SB             (768 + 0x28)
#define D_PEDSTD_SB_TIME        (768 + 0x2C)
#define D_PEDSTD_PEAKTHRSH      (768 + 0x98)
#define D_PEDSTD_TIML           (768 + 0x2A)
#define D_PEDSTD_TIMH           (768 + 0x2E)
#define D_PEDSTD_PEAK           (768 + 0X94)
#define D_PEDSTD_STEPCTR        (768 + 0x60)
#define D_PEDSTD_TIMECTR        (964)
#define D_PEDSTD_DECI           (768 + 0xA0)

#define D_HOST_NO_MOT           (976)
#define D_ACCEL_BIAS            (660)

#define D_ORIENT_GAP            (76)

#define D_TILT0_H               (48)
#define D_TILT0_L               (50)
#define D_TILT1_H               (52)
#define D_TILT1_L               (54)
#define D_TILT2_H               (56)
#define D_TILT2_L               (58)
#define D_TILT3_H               (60)
#define D_TILT3_L               (62)

#define DMP_CODE_SIZE           (3062)

static const unsigned char dmp_memory[DMP_CODE_SIZE] = {
    /* bank # 0 */
    0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xff, 0xef, 0x00, 0x00, 0xfa, 0x80, 0x00, 0x0b, 0x12, 0x82, 0x00, 0x01,
    0x03, 0x0c, 0x30, 0xc3, 0x0e, 0x8c, 0x8c, 0xe9, 0x14, 0xd5, 0x40, 0x02, 0x13, 0x71, 0x0f, 0x8e,
    0x38, 0x83, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83, 0x25, 0x8e, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83,
    0xff, 0xff, 0xff, 0xff, 0x0f, 0xfe, 0xa9, 0xd6, 0x24, 0x00, 0x04, 0x00, 0x1a, 0x82, 0x79, 0xa1,
    0x00, 0x00, 0x00, 0x3c, 0xff, 0xff, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x38, 0x83, 0x6f, 0xa2,
    0x00, 0x3e, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xca, 0xe3, 0x09, 0x3e, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x00, 0x0c, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x6e, 0x00, 0x00, 0x06, 0x92, 0x0a, 0x16, 0xc0, 0xdf,
    0xff, 0xff, 0x02, 0x56, 0xfd, 0x8c, 0xd3, 0x77, 0xff, 0xe1, 0xc4, 0x96, 0xe0, 0xc5, 0xbe, 0xaa,
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0b, 0x2b, 0x00, 0x00, 0x16, 0x57, 0x00, 0x00, 0x03, 0x59,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0xfa, 0x00, 0x02, 0x6c, 0x1d, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0xdf, 0xeb, 0x00, 0x3e, 0xb3, 0xb6, 0x00, 0x0d, 0x22, 0x78, 0x00, 0x00, 0x2f, 0x3c,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x42, 0xb5, 0x00, 0x00, 0x39, 0xa2, 0x00, 0x00, 0xb3, 0x65,
    0xd9, 0x0e, 0x9f, 0xc9, 0x1d, 0xcf, 0x4c, 0x34, 0x30, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00,
    0x3b, 0xb6, 0x7a, 0xe8, 0x00, 0x64, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* bank # 1 */
    0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0xfa, 0x92, 0x10, 0x00, 0x22, 0x5e, 0x00, 0x0d, 0x22, 0x9f,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0xff, 0x46, 0x00, 0x00, 0x63, 0xd4, 0x00, 0x00,
    0x10, 0x00, 0x00, 0x00, 0x04, 0xd6, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x72, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x02, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x05, 0x00, 0x64, 0x00, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x32, 0xf8, 0x98, 0x00, 0x00, 0xff, 0x65, 0x00, 0x00, 0x83, 0x0f, 0x00, 0x00,
    0xff, 0x9b, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0xb2, 0x6a, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x01, 0xfb, 0x83, 0x00, 0x68, 0x00, 0x00, 0x00, 0xd9, 0xfc, 0x00, 0x7c, 0xf1, 0xff, 0x83,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x64, 0x03, 0xe8, 0x00, 0x64, 0x00, 0x28,
    0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x16, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x00, 0x00, 0x10, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf4, 0x00, 0x00, 0x10, 0x00,
    /* bank # 2 */
    0x00, 0x28, 0x00, 0x00, 0xff, 0xff, 0x45, 0x81, 0xff, 0xff, 0xfa, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x05, 0x00, 0x05, 0xba, 0xc6, 0x00, 0x47, 0x78, 0xa2,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x25, 0x4d, 0x00, 0x2f, 0x70, 0x6d, 0x00, 0x00, 0x05, 0xae, 0x00, 0x0c, 0x02, 0xd0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x64, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x0e,
    0x00, 0x00, 0x0a, 0xc7, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xff, 0xff, 0xff, 0x9c,
    0x00, 0x00, 0x0b, 0x2b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64,
    0xff, 0xe5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* bank # 3 */
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x24, 0x26, 0xd3,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x00, 0x96, 0x00, 0x3c,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0c, 0x0a, 0x4e, 0x68, 0xcd, 0xcf, 0x77, 0x09, 0x50, 0x16, 0x67, 0x59, 0xc6, 0x19, 0xce, 0x82,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xd7, 0x84, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc7, 0x93, 0x8f, 0x9d, 0x1e, 0x1b, 0x1c, 0x19,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x18, 0x85, 0x00, 0x00, 0x40, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x67, 0x7d, 0xdf, 0x7e, 0x72, 0x90, 0x2e, 0x55, 0x4c, 0xf6, 0xe6, 0x88,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /* bank # 4 */
    0xd8, 0xdc, 0xb4, 0xb8, 0xb0, 0xd8, 0xb9, 0xab, 0xf3, 0xf8, 0xfa, 0xb3, 0xb7, 0xbb, 0x8e, 0x9e,
    0xae, 0xf1, 0x32, 0xf5, 0x1b, 0xf1, 0xb4, 0xb8, 0xb0, 0x80, 0x97, 0xf1, 0xa9, 0xdf, 0xdf, 0xdf,
    0xaa, 0xdf, 0xdf, 0xdf, 0xf2, 0xaa, 0xc5, 0xcd, 0xc7, 0xa9, 0x0c, 0xc9, 0x2c, 0x97, 0xf1, 0xa9,
    0x89, 0x26, 0x46, 0x66, 0xb2, 0x89, 0x99, 0xa9, 0x2d, 0x55, 0x7d, 0xb0, 0xb0, 0x8a, 0xa8, 0x96,
    0x36, 0x56, 0x76, 0xf1, 0xba, 0xa3, 0xb4, 0xb2, 0x80, 0xc0, 0xb8, 0xa8, 0x97, 0x11, 0xb2, 0x83,
    0x98, 0xba, 0xa3, 0xf0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xb2, 0xb9, 0xb4, 0x98, 0x83, 0xf1,
    0xa3, 0x29, 0x55, 0x7d, 0xba, 0xb5, 0xb1, 0xa3, 0x83, 0x93, 0xf0, 0x00, 0x28, 0x50, 0xf5, 0xb2,
    0xb6, 0xaa, 0x83, 0x93, 0x28, 0x54, 0x7c, 0xf1, 0xb9, 0xa3, 0x82, 0x93, 0x61, 0xba, 0xa2, 0xda,
    0xde, 0xdf, 0xdb, 0x81, 0x9a, 0xb9, 0xae, 0xf5, 0x60, 0x68, 0x70, 0xf1, 0xda, 0xba, 0xa2, 0xdf,
    0xd9, 0xba, 0xa2, 0xfa, 0xb9, 0xa3, 0x82, 0x92, 0xdb, 0x31, 0xba, 0xa2, 0xd9, 0xba, 0xa2, 0xf8,
    0xdf, 0x85, 0xa4, 0xd0, 0xc1, 0xbb, 0xad, 0x83, 0xc2, 0xc5, 0xc7, 0xb8, 0xa2, 0xdf, 0xdf, 0xdf,
    0xba, 0xa0, 0xdf, 0xdf, 0xdf, 0xd8, 0xd8, 0xf1, 0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35,
    0x5d, 0xb2, 0xb6, 0xba, 0xaf, 0x8c, 0x96, 0x19, 0x8f, 0x9f, 0xa7, 0x0e, 0x16, 0x1e, 0xb4, 0x9a,
    0xb8, 0xaa, 0x87, 0x2c, 0x54, 0x7c, 0xba, 0xa4, 0xb0, 0x8a, 0xb6, 0x91, 0x32, 0x56, 0x76, 0xb2,
    0x84, 0x94, 0xa4, 0xc8, 0x08, 0xcd, 0xd8, 0xb8, 0xb4, 0xb0, 0xf1, 0x99, 0x82, 0xa8, 0x2d, 0x55,
    0x7d, 0x98, 0xa8, 0x0e, 0x16, 0x1e, 0xa2, 0x2c, 0x54, 0x7c, 0x92, 0xa4, 0xf0, 0x2c, 0x50, 0x78,
    /* bank # 5 */
    0xf1, 0x84, 0xa8, 0x98, 0xc4, 0xcd, 0xfc, 0xd8, 0x0d, 0xdb, 0xa8, 0xfc, 0x2d, 0xf3, 0xd9, 0xba,
    0xa6, 0xf8, 0xda, 0xba, 0xa6, 0xde, 0xd8, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xf3, 0xc8,
    0x41, 0xda, 0xa6, 0xc8, 0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0x82, 0xa8, 0x92, 0xf5, 0x2c, 0x54, 0x88,
    0x98, 0xf1, 0x35, 0xd9, 0xf4, 0x18, 0xd8, 0xf1, 0xa2, 0xd0, 0xf8, 0xf9, 0xa8, 0x84, 0xd9, 0xc7,
    0xdf, 0xf8, 0xf8, 0x83, 0xc5, 0xda, 0xdf, 0x69, 0xdf, 0x83, 0xc1, 0xd8, 0xf4, 0x01, 0x14, 0xf1,
    0xa8, 0x82, 0x4e, 0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x28, 0x97, 0x88, 0xf1,
    0x09, 0xf4, 0x1c, 0x1c, 0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x29,
    0xf4, 0x0d, 0xd8, 0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc2, 0x03, 0xd8, 0xde, 0xdf, 0x1a,
    0xd8, 0xf1, 0xa2, 0xfa, 0xf9, 0xa8, 0x84, 0x98, 0xd9, 0xc7, 0xdf, 0xf8, 0xf8, 0xf8, 0x83, 0xc7,
    0xda, 0xdf, 0x69, 0xdf, 0xf8, 0x83, 0xc3, 0xd8, 0xf4, 0x01, 0x14, 0xf1, 0x98, 0xa8, 0x82, 0x2e,
    0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x50, 0x97, 0x88, 0xf1, 0x09, 0xf4, 0x1c,
    0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf8, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x49, 0xf4, 0x0d, 0xd8,
    0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc4, 0x03, 0xd8, 0xde, 0xdf, 0xd8, 0xf1, 0xad, 0x88,
    0x98, 0xcc, 0xa8, 0x09, 0xf9, 0xd9, 0x82, 0x92, 0xa8, 0xf5, 0x7c, 0xf1, 0x88, 0x3a, 0xcf, 0x94,
    0x4a, 0x6e, 0x98, 0xdb, 0x69, 0x31, 0xda, 0xad, 0xf2, 0xde, 0xf9, 0xd8, 0x87, 0x95, 0xa8, 0xf2,
    0x21, 0xd1, 0xda, 0xa5, 0xf9, 0xf4, 0x17, 0xd9, 0xf1, 0xae, 0x8e, 0xd0, 0xc0, 0xc3, 0xae, 0x82,
    /* bank # 6 */
    0xc6, 0x84, 0xc3, 0xa8, 0x85, 0x95, 0xc8, 0xa5, 0x88, 0xf2, 0xc0, 0xf1, 0xf4, 0x01, 0x0e, 0xf1,
    0x8e, 0x9e, 0xa8, 0xc6, 0x3e, 0x56, 0xf5, 0x54, 0xf1, 0x88, 0x72, 0xf4, 0x01, 0x15, 0xf1, 0x98,
    0x45, 0x85, 0x6e, 0xf5, 0x8e, 0x9e, 0x04, 0x88, 0xf1, 0x42, 0x98, 0x5a, 0x8e, 0x9e, 0x06, 0x88,
    0x69, 0xf4, 0x01, 0x1c, 0xf1, 0x98, 0x1e, 0x11, 0x08, 0xd0, 0xf5, 0x04, 0xf1, 0x1e, 0x97, 0x02,
    0x02, 0x98, 0x36, 0x25, 0xdb, 0xf9, 0xd9, 0x85, 0xa5, 0xf3, 0xc1, 0xda, 0x85, 0xa5, 0xf3, 0xdf,
    0xd8, 0x85, 0x95, 0xa8, 0xf3, 0x09, 0xda, 0xa5, 0xfa, 0xd8, 0x82, 0x92, 0xa8, 0xf5, 0x78, 0xf1,
    0x88, 0x1a, 0x84, 0x9f, 0x26, 0x88, 0x98, 0x21, 0xda, 0xf4, 0x1d, 0xf3, 0xd8, 0x87, 0x9f, 0x39,
    0xd1, 0xaf, 0xd9, 0xdf, 0xdf, 0xfb, 0xf9, 0xf4, 0x0c, 0xf3, 0xd8, 0xfa, 0xd0, 0xf8, 0xda, 0xf9,
    0xf9, 0xd0, 0xdf, 0xd9, 0xf9, 0xd8, 0xf4, 0x0b, 0xd8, 0xf3, 0x87, 0x9f, 0x39, 0xd1, 0xaf, 0xd9,
    0xdf, 0xdf, 0xf4, 0x1d, 0xf3, 0xd8, 0xfa, 0xfc, 0xa8, 0x69, 0xf9, 0xf9, 0xaf, 0xd0, 0xda, 0xde,
    0xfa, 0xd9, 0xf8, 0x8f, 0x9f, 0xa8, 0xf1, 0xcc, 0xf3, 0x98, 0xdb, 0x45, 0xd9, 0xaf, 0xdf, 0xd0,
    0xf8, 0xd8, 0xf1, 0x8f, 0x9f, 0xa8, 0xca, 0xf3, 0x88, 0x09, 0xda, 0xaf, 0x8f, 0xcb, 0xf8, 0xd8,
    0xf2, 0xad, 0x97, 0x8d, 0x0c, 0xd9, 0xa5, 0xdf, 0xf9, 0xba, 0xa6, 0xf3, 0xfa, 0xf4, 0x12, 0xf2,
    0xd8, 0x95, 0x0d, 0xd1, 0xd9, 0xba, 0xa6, 0xf3, 0xfa, 0xda, 0xa5, 0xf2, 0xc1, 0xba, 0xa6, 0xf3,
    0xdf, 0xd8, 0xf1, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xca, 0xf3, 0x49, 0xda, 0xa6, 0xcb,
    0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0xd8, 0xad, 0x84, 0xf2, 0xc0, 0xdf, 0xf1, 0x8f, 0xcb, 0xc3, 0xa8,
    /* bank # 7 */
    0xb2, 0xb6, 0x86, 0x96, 0xc8, 0xc1, 0xcb, 0xc3, 0xf3, 0xb0, 0xb4, 0x88, 0x98, 0xa8, 0x21, 0xdb,
    0x71, 0x8d, 0x9d, 0x71, 0x85, 0x95, 0x21, 0xd9, 0xad, 0xf2, 0xfa, 0xd8, 0x85, 0x97, 0xa8, 0x28,
    0xd9, 0xf4, 0x08, 0xd8, 0xf2, 0x8d, 0x29, 0xda, 0xf4, 0x05, 0xd9, 0xf2, 0x85, 0xa4, 0xc2, 0xf2,
    0xd8, 0xa8, 0x8d, 0x94, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xf2, 0xd8, 0x87, 0x21, 0xd8, 0xf4, 0x0a,
    0xd8, 0xf2, 0x84, 0x98, 0xa8, 0xc8, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xd8, 0xf3, 0xa4, 0xc8, 0xbb,
    0xaf, 0xd0, 0xf2, 0xde, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xd8, 0xf1, 0xb8, 0xf6,
    0xb5, 0xb9, 0xb0, 0x8a, 0x95, 0xa3, 0xde, 0x3c, 0xa3, 0xd9, 0xf8, 0xd8, 0x5c, 0xa3, 0xd9, 0xf8,
    0xd8, 0x7c, 0xa3, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa5, 0xd9, 0xdf, 0xda, 0xfa, 0xd8, 0xb1,
    0x85, 0x30, 0xf7, 0xd9, 0xde, 0xd8, 0xf8, 0x30, 0xad, 0xda, 0xde, 0xd8, 0xf2, 0xb4, 0x8c, 0x99,
    0xa3, 0x2d, 0x55, 0x7d, 0xa0, 0x83, 0xdf, 0xdf, 0xdf, 0xb5, 0x91, 0xa0, 0xf6, 0x29, 0xd9, 0xfb,
    0xd8, 0xa0, 0xfc, 0x29, 0xd9, 0xfa, 0xd8, 0xa0, 0xd0, 0x51, 0xd9, 0xf8, 0xd8, 0xfc, 0x51, 0xd9,
    0xf9, 0xd8, 0x79, 0xd9, 0xfb, 0xd8, 0xa0, 0xd0, 0xfc, 0x79, 0xd9, 0xfa, 0xd8, 0xa1, 0xf9, 0xf9,
    0xf9, 0xf9, 0xf9, 0xa0, 0xda, 0xdf, 0xdf, 0xdf, 0xd8, 0xa1, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xac,
    0xde, 0xf8, 0xad, 0xde, 0x83, 0x93, 0xac, 0x2c, 0x54, 0x7c, 0xf1, 0xa8, 0xdf, 0xdf, 0xdf, 0xf6,
    0x9d, 0x2c, 0xda, 0xa0, 0xdf, 0xd9, 0xfa, 0xdb, 0x2d, 0xf8, 0xd8, 0xa8, 0x50, 0xda, 0xa0, 0xd0,
    0xde, 0xd9, 0xd0, 0xf8, 0xf8, 0xf8, 0xdb, 0x55, 0xf8, 0xd8, 0xa8, 0x78, 0xda, 0xa0, 0xd0, 0xdf,
    /* bank # 8 */
    0xd9, 0xd0, 0xfa, 0xf8, 0xf8, 0xf8, 0xf8, 0xdb, 0x7d, 0xf8, 0xd8, 0x9c, 0xa8, 0x8c, 0xf5, 0x30,
    0xdb, 0x38, 0xd9, 0xd0, 0xde, 0xdf, 0xa0, 0xd0, 0xde, 0xdf, 0xd8, 0xa8, 0x48, 0xdb, 0x58, 0xd9,
    0xdf, 0xd0, 0xde, 0xa0, 0xdf, 0xd0, 0xde, 0xd8, 0xa8, 0x68, 0xdb, 0x70, 0xd9, 0xdf, 0xdf, 0xa0,
    0xdf, 0xdf, 0xd8, 0xf1, 0xa8, 0x88, 0x90, 0x2c, 0x54, 0x7c, 0x98, 0xa8, 0xd0, 0x5c, 0x38, 0xd1,
    0xda, 0xf2, 0xae, 0x8c, 0xdf, 0xf9, 0xd8, 0xb0, 0x87, 0xa8, 0xc1, 0xc1, 0xb1, 0x88, 0xa8, 0xc6,
    0xf9, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8,
    0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xf7, 0x8d, 0x9d, 0xad, 0xf8, 0x18, 0xda,
    0xf2, 0xae, 0xdf, 0xd8, 0xf7, 0xad, 0xfa, 0x30, 0xd9, 0xa4, 0xde, 0xf9, 0xd8, 0xf2, 0xae, 0xde,
    0xfa, 0xf9, 0x83, 0xa7, 0xd9, 0xc3, 0xc5, 0xc7, 0xf1, 0x88, 0x9b, 0xa7, 0x7a, 0xad, 0xf7, 0xde,
    0xdf, 0xa4, 0xf8, 0x84, 0x94, 0x08, 0xa7, 0x97, 0xf3, 0x00, 0xae, 0xf2, 0x98, 0x19, 0xa4, 0x88,
    0xc6, 0xa3, 0x94, 0x88, 0xf6, 0x32, 0xdf, 0xf2, 0x83, 0x93, 0xdb, 0x09, 0xd9, 0xf2, 0xaa, 0xdf,
    0xd8, 0xd8, 0xae, 0xf8, 0xf9, 0xd1, 0xda, 0xf3, 0xa4, 0xde, 0xa7, 0xf1, 0x88, 0x9b, 0x7a, 0xd8,
    0xf3, 0x84, 0x94, 0xae, 0x19, 0xf9, 0xda, 0xaa, 0xf1, 0xdf, 0xd8, 0xa8, 0x81, 0xc0, 0xc3, 0xc5,
    0xc7, 0xa3, 0x92, 0x83, 0xf6, 0x28, 0xad, 0xde, 0xd9, 0xf8, 0xd8, 0xa3, 0x50, 0xad, 0xd9, 0xf8,
    0xd8, 0xa3, 0x78, 0xad, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa1, 0xda, 0xde, 0xc3, 0xc5, 0xc7,
    0xd8, 0xa1, 0x81, 0x94, 0xf8, 0x18, 0xf2, 0xb0, 0x89, 0xac, 0xc3, 0xc5, 0xc7, 0xf1, 0xd8, 0xb8,
    /* bank # 9 */
    0xb4, 0xb0, 0x97, 0x86, 0xa8, 0x31, 0x9b, 0x06, 0x99, 0x07, 0xab, 0x97, 0x28, 0x88, 0x9b, 0xf0,
    0x0c, 0x20, 0x14, 0x40, 0xb0, 0xb4, 0xb8, 0xf0, 0xa8, 0x8a, 0x9a, 0x28, 0x50, 0x78, 0xb7, 0x9b,
    0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xf1, 0xbb, 0xab,
    0x88, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0xb3, 0x8b, 0xb8, 0xa8, 0x04, 0x28, 0x50, 0x78, 0xf1, 0xb0,
    0x88, 0xb4, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xbb, 0xab, 0xb3, 0x8b, 0x02, 0x26, 0x46, 0x66, 0xb0,
    0xb8, 0xf0, 0x8a, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x8b, 0x29, 0x51, 0x79, 0x8a, 0x24, 0x70, 0x59,
    0x8b, 0x20, 0x58, 0x71, 0x8a, 0x44, 0x69, 0x38, 0x8b, 0x39, 0x40, 0x68, 0x8a, 0x64, 0x48, 0x31,
    0x8b, 0x30, 0x49, 0x60, 0x88, 0xf1, 0xac, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0x8c, 0xa8, 0x04, 0x28,
    0x50, 0x78, 0xf1, 0x88, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xac, 0x8c, 0x02, 0x26, 0x46, 0x66, 0xf0,
    0x89, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xa9,
    0x88, 0x09, 0x20, 0x59, 0x70, 0xab, 0x11, 0x38, 0x40, 0x69, 0xa8, 0x19, 0x31, 0x48, 0x60, 0x8c,
    0xa8, 0x3c, 0x41, 0x5c, 0x20, 0x7c, 0x00, 0xf1, 0x87, 0x98, 0x19, 0x86, 0xa8, 0x6e, 0x76, 0x7e,
    0xa9, 0x99, 0x88, 0x2d, 0x55, 0x7d, 0xd8, 0xb1, 0xb5, 0xb9, 0xa3, 0xdf, 0xdf, 0xdf, 0xae, 0xd0,
    0xdf, 0xaa, 0xd0, 0xde, 0xf2, 0xab, 0xf8, 0xf9, 0xd9, 0xb0, 0x87, 0xc4, 0xaa, 0xf1, 0xdf, 0xdf,
    0xbb, 0xaf, 0xdf, 0xdf, 0xb9, 0xd8, 0xb1, 0xf1, 0xa3, 0x97, 0x8e, 0x60, 0xdf, 0xb0, 0x84, 0xf2,
    0xc8, 0xf8, 0xf9, 0xd9, 0xde, 0xd8, 0x93, 0x85, 0xf1, 0x4a, 0xb1, 0x83, 0xa3, 0x08, 0xb5, 0x83,
    /* bank # 10 */
    0x9a, 0x08, 0x10, 0xb7, 0x9f, 0x10, 0xd8, 0xf1, 0xb0, 0xba, 0xae, 0xb0, 0x8a, 0xc2, 0xb2, 0xb6,
    0x8e, 0x9e, 0xf1, 0xfb, 0xd9, 0xf4, 0x1d, 0xd8, 0xf9, 0xd9, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad,
    0x61, 0xd9, 0xae, 0xfb, 0xd8, 0xf4, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad, 0x19, 0xd9, 0xae, 0xfb,
    0xdf, 0xd8, 0xf4, 0x16, 0xf1, 0xd8, 0xf8, 0xad, 0x8d, 0x61, 0xd9, 0xf4, 0xf4, 0xac, 0xf5, 0x9c,
    0x9c, 0x8d, 0xdf, 0x2b, 0xba, 0xb6, 0xae, 0xfa, 0xf8, 0xf4, 0x0b, 0xd8, 0xf1, 0xae, 0xd0, 0xf8,
    0xad, 0x51, 0xda, 0xae, 0xfa, 0xf8, 0xf1, 0xd8, 0xb9, 0xb1, 0xb6, 0xa3, 0x83, 0x9c, 0x08, 0xb9,
    0xb1, 0x83, 0x9a, 0xb5, 0xaa, 0xc0, 0xfd, 0x30, 0x83, 0xb7, 0x9f, 0x10, 0xb5, 0x8b, 0x93, 0xf2,
    0x02, 0x02, 0xd1, 0xab, 0xda, 0xde, 0xd8, 0xf1, 0xb0, 0x80, 0xba, 0xab, 0xc0, 0xc3, 0xb2, 0x84,
    0xc1, 0xc3, 0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9, 0xab, 0xde, 0xb0,
    0x87, 0x9c, 0xb9, 0xa3, 0xdd, 0xf1, 0xb3, 0x8b, 0x8b, 0x8b, 0x8b, 0x8b, 0xb0, 0x87, 0xa3, 0xa3,
    0xa3, 0xa3, 0xb2, 0x8b, 0xb6, 0x9b, 0xf2, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
    0xa3, 0xf1, 0xb0, 0x87, 0xb5, 0x9a, 0xa3, 0xf3, 0x9b, 0xa3, 0xa3, 0xdc, 0xba, 0xac, 0xdf, 0xb9,
    0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
    0xd8, 0xd8, 0xd8, 0xbb, 0xb3, 0xb7, 0xf1, 0xaa, 0xf9, 0xda, 0xff, 0xd9, 0x80, 0x9a, 0xaa, 0x28,
    0xb4, 0x80, 0x98, 0xa7, 0x20, 0xb7, 0x97, 0x87, 0xa8, 0x66, 0x88, 0xf0, 0x79, 0x51, 0xf1, 0x90,
    0x2c, 0x87, 0x0c, 0xa7, 0x81, 0x97, 0x62, 0x93, 0xf0, 0x71, 0x71, 0x60, 0x85, 0x94, 0x01, 0x29,
    /* bank # 11 */
    0x51, 0x79, 0x90, 0xa5, 0xf1, 0x28, 0x4c, 0x6c, 0x87, 0x0c, 0x95, 0x18, 0x85, 0x78, 0xa3, 0x83,
    0x90, 0x28, 0x4c, 0x6c, 0x88, 0x6c, 0xd8, 0xf3, 0xa2, 0x82, 0x00, 0xf2, 0x10, 0xa8, 0x92, 0x19,
    0x80, 0xa2, 0xf2, 0xd9, 0x26, 0xd8, 0xf1, 0x88, 0xa8, 0x4d, 0xd9, 0x48, 0xd8, 0x96, 0xa8, 0x39,
    0x80, 0xd9, 0x3c, 0xd8, 0x95, 0x80, 0xa8, 0x39, 0xa6, 0x86, 0x98, 0xd9, 0x2c, 0xda, 0x87, 0xa7,
    0x2c, 0xd8, 0xa8, 0x89, 0x95, 0x19, 0xa9, 0x80, 0xd9, 0x38, 0xd8, 0xa8, 0x89, 0x39, 0xa9, 0x80,
    0xda, 0x3c, 0xd8, 0xa8, 0x2e, 0xa8, 0x39, 0x90, 0xd9, 0x0c, 0xd8, 0xa8, 0x95, 0x31, 0x98, 0xd9,
    0x0c, 0xd8, 0xa8, 0x09, 0xd9, 0xff, 0xd8, 0x01, 0xda, 0xff, 0xd8, 0x95, 0x39, 0xa9, 0xda, 0x26,
    0xff, 0xd8, 0x90, 0xa8, 0x0d, 0x89, 0x99, 0xa8, 0x10, 0x80, 0x98, 0x21, 0xda, 0x2e, 0xd8, 0x89,
    0x99, 0xa8, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x86, 0x96, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8,
    0x87, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x82, 0x92, 0xf3, 0x41, 0x80, 0xf1, 0xd9, 0x2e, 0xd8,
    0xa8, 0x82, 0xf3, 0x19, 0x80, 0xf1, 0xd9, 0x2e, 0xd8, 0x82, 0xac, 0xf3, 0xc0, 0xa2, 0x80, 0x22,
    0xf1, 0xa6, 0x2e, 0xa7, 0x2e, 0xa9, 0x22, 0x98, 0xa8, 0x29, 0xda, 0xac, 0xde, 0xff, 0xd8, 0xa2,
    0xf2, 0x2a, 0xf1, 0xa9, 0x2e, 0x82, 0x92, 0xa8, 0xf2, 0x31, 0x80, 0xa6, 0x96, 0xf1, 0xd9, 0x00,
    0xac, 0x8c, 0x9c, 0x0c, 0x30, 0xac, 0xde, 0xd0, 0xde, 0xff, 0xd8, 0x8c, 0x9c, 0xac, 0xd0, 0x10,
    0xac, 0xde, 0x80, 0x92, 0xa2, 0xf2, 0x4c, 0x82, 0xa8, 0xf1, 0xca, 0xf2, 0x35, 0xf1, 0x96, 0x88,
    0xa6, 0xd9, 0x00, 0xd8, 0xf1, 0xff
};
static const unsigned short sStartAddress = 0x0400;
/*移植dmpDefaultMPU6050.c定义结束*/

#define INT_SRC_TAP             (0x01)
#define INT_SRC_ANDROID_ORIENT  (0x08)
#define DMP_FEATURE_SEND_ANY_GYRO   (DMP_FEATURE_SEND_RAW_GYRO | \
                                     DMP_FEATURE_SEND_CAL_GYRO)
#define DMP_SAMPLE_RATE     (200)
#define GYRO_SF             (46850825LL * 200 / DMP_SAMPLE_RATE)
#define QUAT_ERROR_THRESH       (1L<<24)
#define QUAT_MAG_SQ_NORMALIZED  (1L<<28)
#define QUAT_MAG_SQ_MIN         (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX         (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)

struct dmp_s {
    void (*tap_cb)(unsigned char count, unsigned char direction);
    void (*android_orient_cb)(unsigned char orientation);
    unsigned short orient;
    unsigned short feature_mask;
    unsigned short fifo_rate;
    unsigned char packet_length;
};
static struct dmp_s dmp={
  NULL,
  NULL,
  0,
  0,
  0,
  0
};
/*mup驱动所需结构体定义END*/
/***/
static void get_ms(unsigned long *time)
{

}

static void MPU_IIC_Delay(void){
	delay_us(2);
}
/**/
static void MPU_SDA_OUT(void){
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/**/
static void MPU_SDA_IN(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

}
/**/
static void MPU_IIC_Start(void){
	MPU_SDA_OUT();
	MPU_IIC_SDA(1);
	MPU_IIC_SCL(1);
	MPU_IIC_Delay();
 	MPU_IIC_SDA(0);
	MPU_IIC_Delay();
	MPU_IIC_SCL(0);//钳住I2C总线，准备发送或接收数据
}
/**/
static void MPU_IIC_Stop(void){
	MPU_SDA_OUT();
	MPU_IIC_SCL(0);
	MPU_IIC_SDA(0);
 	MPU_IIC_Delay();
	MPU_IIC_SCL(1);
	MPU_IIC_SDA(1);//发送I2C总线结束信号
	MPU_IIC_Delay();
}
/*等待应答
@retur:1:接收应答失败
			0:接收应答成功*/
static unsigned char MPU_IIC_Wait_Ack(void)
{
	unsigned char ucErrTime=0;
	MPU_SDA_IN();      //SDA设置为输入
	MPU_IIC_SDA(1);MPU_IIC_Delay();
	MPU_IIC_SCL(1);MPU_IIC_Delay();
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL(0);//时钟输出0
	return 0;
}
//产生ACK应答
static void MPU_IIC_Ack(void){
	MPU_IIC_SCL(0);
	MPU_SDA_OUT();
	MPU_IIC_SDA(0);
	MPU_IIC_Delay();
	MPU_IIC_SCL(1);
	MPU_IIC_Delay();
	MPU_IIC_SCL(0);
}
//不产生ACK应答
static void MPU_IIC_NAck(void){
	MPU_IIC_SCL(0);
	MPU_SDA_OUT();
	MPU_IIC_SDA(1);
	MPU_IIC_Delay();
	MPU_IIC_SCL(1);
	MPU_IIC_Delay();
	MPU_IIC_SCL(0);
}
/*IIC发送一个字节
返回从机有无应答
1，有应答
0，无应答*/
static void MPU_IIC_Send_Byte(unsigned char txd){
  unsigned char t;
	MPU_SDA_OUT();
	MPU_IIC_SCL(0);//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{
		MPU_IIC_SDA((txd&0x80)>>7);
		txd<<=1;
		MPU_IIC_SCL(1);
		MPU_IIC_Delay();
		MPU_IIC_SCL(0);
		MPU_IIC_Delay();
	}
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
static unsigned char MPU_IIC_Read_Byte(unsigned char ack){
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
		MPU_IIC_SCL(0);
		MPU_IIC_Delay();
		MPU_IIC_SCL(1);
		receive<<=1;
		if(MPU_READ_SDA)receive++;
		MPU_IIC_Delay();
    }
    if (!ack)
			MPU_IIC_NAck();//发送nACK
    else
			MPU_IIC_Ack(); //发送ACK
    return receive;
}
/*IIC连续写
addr:器件地址
reg:寄存器地址
len:写入长度
buf:数据区
return:0,正常
    其他,错误*/
unsigned char MPU_Write_Len(unsigned char addr,\
	unsigned char reg,unsigned char len,unsigned char *buf){
	unsigned char i;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);	//写寄存器地址
	MPU_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//发送数据
		if(MPU_IIC_Wait_Ack())		//等待ACK
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_Stop();
	return 0;
}
/*IIC连续读
addr:器件地址
reg:要读取的寄存器地址
len:要读取的长度
buf:读取到的数据存储区
return:0,正常
    其他,错误*/
unsigned char MPU_Read_Len(unsigned char addr,\
	unsigned char reg,unsigned char len,unsigned char *buf){
 	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令
	if(MPU_IIC_Wait_Ack()){	//等待应答
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);
	MPU_IIC_Wait_Ack();
	while(len){
		if(len==1)*buf=MPU_IIC_Read_Byte(0);
		else *buf=MPU_IIC_Read_Byte(1);
		len--;
		buf++;
	}
  MPU_IIC_Stop();
	return 0;
}
/*IIC写一个字节
reg:寄存器地址
data:数据
return:0,正常
    其他,错误代码*/
unsigned char MPU_Write_Byte(unsigned char reg,\
	unsigned char data){
  MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令
	if(MPU_IIC_Wait_Ack()){	//等待应答
		MPU_IIC_Stop();
		return 1;
	}
	MPU_IIC_Send_Byte(reg);	//写寄存器地址
	MPU_IIC_Wait_Ack();		//等待应答
	MPU_IIC_Send_Byte(data);//发送数据
	if(MPU_IIC_Wait_Ack()){	//等待ACK
		MPU_IIC_Stop();
		return 1;
	}
  MPU_IIC_Stop();
	return 0;
}
/*IIC读一个字节
reg:寄存器地址
返回值:读到的数据*/
unsigned char MPU_Read_Byte(unsigned char reg){
	unsigned char res;
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令
	MPU_IIC_Wait_Ack();		//等待应答
	MPU_IIC_Send_Byte(reg);	//写寄存器地址
	MPU_IIC_Wait_Ack();
	MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令
	MPU_IIC_Wait_Ack();
	res=MPU_IIC_Read_Byte(0);		//读取数据,发送nACK
	MPU_IIC_Stop();		//stop
	return res;
}
/**MPUIIC基础函数END**/
/*************************************************************************************/
/*mpu6050初始化
@return:
	1:error
	0:ok
*/
static void MPUGPIO_INit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);		//A0D		PB10

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;		
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);		//		SDA		PB12
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;		
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);		//SCL		PB11
}
unsigned char MPU_Init(void){
	unsigned char res;
	MPUGPIO_INit();
	GPIOB->BRR=GPIO_Pin_10;//AD0低 从机地址0X68
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);		//复位MPU6050
	delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);		//唤醒MPU6050
	MPU_Set_Gyro_Fsr(3);		//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);		//加速度传感器,±2g
	MPU_Set_Rate(50);		//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);		//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);		//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);		//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);		//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR){	//器件ID正确
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);		//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);		//加速度与陀螺仪都工作
		MPU_Set_Rate(50);		//设置采样率为50Hz
	}else return 1;
	return 0;
}
/*设置MPU6050陀螺仪传感器满量程范围
fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
return:0,ok
    !0,error*/
unsigned char MPU_Set_Gyro_Fsr(unsigned char fsr){
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
}
/*设置MPU6050加速度传感器满量程范围
fsr:0,±2g;1,±4g;2,±8g;3,±16g
return:0,ok
    !0,error*/
unsigned char MPU_Set_Accel_Fsr(unsigned char fsr){
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
}
/*设置MPU6050的数字低通滤波器
lpf:数字低通滤波频率(Hz)
return:0,ok
    !0,error*/
unsigned char MPU_Set_LPF(unsigned short int lpf){
	unsigned char data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器
}
/*设置MPU6050的采样率(假定Fs=1KHz)
rate:4~1000(Hz)
return:0,ok
    !0,error*/
unsigned char MPU_Set_Rate(unsigned short int  rate){
	unsigned char data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}
/*温度值
return:temp(*100)*/
short MPU_Get_Temperature(void){
	unsigned char buf[2];
	short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf);
	raw=((unsigned short int)buf[0]<<8)|buf[1];
	temp=36.53+((double)raw)/340;
	return temp*100;;
}
/*陀螺仪(原始值)
gx,gy,gz:陀螺仪x,y,z轴的原始值(带符号)
return:0,ok
    !0,errpr*/
unsigned char MPU_Get_Gyroscope(short *gx,short *gy,short *gz){
	unsigned char buf[6],res;
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0){
		*gx=((unsigned short int )buf[0]<<8)|buf[1];
		*gy=((unsigned short int )buf[2]<<8)|buf[3];
		*gz=((unsigned short int )buf[4]<<8)|buf[5];
	}
	return res;;
}
/*加速度(原始值)
gx,gy,gz:陀螺仪x,y,z轴的原始值(带符号)
return:0,ok
    !0,error*/
unsigned char MPU_Get_Accelerometer(short *ax,\
	short *ay,short *az){
  unsigned char buf[6],res;
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	printf("res:%d\r\n",res);
	if(res==0){
		*ax=((unsigned short int )buf[0]<<8)|buf[1];
		*ay=((unsigned short int )buf[2]<<8)|buf[3];
		*az=((unsigned short int )buf[4]<<8)|buf[5];
	}
	return res;;
}
/*mpu6050基础初始化END*/
/*************************************************************************************/
static int set_int_enable(unsigned char enable){
	unsigned char tmp;
	if(st.chip_cfg.dmp_on){
		if(enable){
			tmp=BIT_DMP_INT_EN;
		}
		else{
			tmp=0x00;
		}
		if(MPU_Read_Len(st.hw->addr,st.reg->int_enable,1,&tmp)){
			return -1;
		}
		st.chip_cfg.int_enable=tmp;
	}
	else{
		if(!st.chip_cfg.sensors){
			return -1;
		}
		if(enable&&st.chip_cfg.int_enable){
			return 0;
		}
		if(enable){
			tmp=BIT_DATA_RDY_EN;
		}else{
			tmp=0x00;
		}
		if(MPU_Read_Len(st.hw->addr,st.reg->int_enable,1,&tmp)){
			return -1;
		}
		st.chip_cfg.int_enable=tmp;
	}
	return 0;
}
int mpu_reg_dump(void){
	unsigned char ii;
	unsigned char data;
	for (ii = 0; ii < st.hw->num_reg; ii++) {
		if (ii == st.reg->fifo_r_w || ii == st.reg->mem_r_w)continue;
		if (MPU_Read_Len(st.hw->addr, ii, 1, &data))return -1;
		printf("%#5x: %#5x\r\n", ii, data);
	}
	return 0;
}
/*读单寄存器*/
int mpu_read_reg(unsigned char reg,unsigned char *data){
	if(reg==st.reg->fifo_r_w||reg==st.reg->mem_r_w){
		return -1;
	}
	if(reg>=st.hw->num_reg){
		return -1;
	}
	return MPU_Read_Len(st.hw->addr,reg,1,data);
}
/*mpu初始化*/
int mpu_init(void){
	unsigned char data[6],rev;
	data[0]=BIT_RESET;
	if(MPU_Write_Len(st.hw->addr,st.reg->pwr_mgmt_1,1,data)){	/*复位*/
		return -1;
	}
	delay_ms(100);
	data[0]=0x00;
	if(MPU_Write_Len(st.hw->addr,st.reg->pwr_mgmt_1,1,data)){
		return -1;
	}
	/* Check product revision. */
	if(MPU_Read_Len(st.hw->addr,st.reg->accel_offs,6,data)){
		return -1;
	}
	rev=((data[5]&0x01)<<2)|((data[3]&0x01)<<1)|(data[1]&0x01);
	if(rev){
		if (rev == 1){
			st.chip_cfg.accel_half = 1;
		}
		else if (rev == 2){
			st.chip_cfg.accel_half = 0;
		}
		else{
			printf("error: %d.\n",rev);
			return -1;
		}
	}else{
		if (MPU_Read_Len(st.hw->addr,st.reg->prod_id,1,data)){
			return -1;
		}
		rev = data[0] & 0x0F;
		if (!rev) {
			printf("error\n");
			return -1;
		}else if (rev == 4) {
			// printf("Half sensitivity part found.\n");
			st.chip_cfg.accel_half=1;
		}else
			st.chip_cfg.accel_half=0;
	}
	/*设置无效值确保不跳过IIc写*/
	st.chip_cfg.sensors = 0xFF;
	st.chip_cfg.gyro_fsr = 0xFF;
	st.chip_cfg.accel_fsr = 0xFF;
	st.chip_cfg.lpf = 0xFF;
	st.chip_cfg.sample_rate = 0xFFFF;
	st.chip_cfg.fifo_enable = 0xFF;
	st.chip_cfg.bypass_mode = 0xFF;
	/*mpu传感器设置保留*/
	st.chip_cfg.clk_src = INV_CLK_PLL;
	st.chip_cfg.active_low_int = 1;
	st.chip_cfg.latched_int = 0;
	st.chip_cfg.int_motion_only = 0;
	st.chip_cfg.lp_accel_mode = 0;
	memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache));
	st.chip_cfg.dmp_on = 0;
	st.chip_cfg.dmp_loaded = 0;
	st.chip_cfg.dmp_sample_rate = 0;
	if (mpu_set_gyro_fsr(2000)) return -1;
	if (mpu_set_accel_fsr(2)) return -1;
	if (mpu_set_lpf(42)) return -1;
	if (mpu_set_sample_rate(50)) return -1;
	if (mpu_configure_fifo(0)) return -1;
	if (mpu_set_bypass(0))return -1;
	mpu_set_sensors(0);
	return 0;
}
/*设置低功率唤醒频率*/
int mpu_lp_accel_mode(unsigned char rate){
	unsigned char tmp[2];
	if (rate > 40)return -1;
	if (!rate) {
		mpu_set_int_latched(0);
		tmp[0] = 0;
		tmp[1] = BIT_STBY_XYZG;
		if (MPU_Write_Len(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))return -1;
		st.chip_cfg.lp_accel_mode = 0;
		return 0;
	}
	/*配置中断
	硬件操作将打断进入睡眠模式
	任何读写操作都将打断睡眠模式*/
	mpu_set_int_latched(1);
	tmp[0] = BIT_LPA_CYCLE;
	if(rate == 1){
		tmp[1] = INV_LPA_1_25HZ;
		mpu_set_lpf(5);
	}
	else if (rate <= 5) {
		tmp[1] = INV_LPA_5HZ;
		mpu_set_lpf(5);
	}
	else if (rate <= 20) {
		tmp[1] = INV_LPA_20HZ;
		mpu_set_lpf(10);
	}
	else {
		tmp[1] = INV_LPA_40HZ;
		mpu_set_lpf(20);
	}
	tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG;
	if (MPU_Write_Len(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))return -1;
	st.chip_cfg.sensors = INV_XYZ_ACCEL;
	st.chip_cfg.clk_src = 0;
	st.chip_cfg.lp_accel_mode = 1;
	mpu_configure_fifo(0);
	return 0;
}
/*重置FIFO读/写指针
return:
	0:ok
	!0:error*/
int mpu_reset_fifo(void){
	unsigned char data;
	if (!(st.chip_cfg.sensors)) return -1;
	data = 0;
	if (MPU_Write_Len(st.hw->addr, st.reg->int_enable, 1, &data)) return -1;
	if (MPU_Write_Len(st.hw->addr, st.reg->fifo_en, 1, &data)) return -1;
	if (MPU_Write_Len(st.hw->addr, st.reg->user_ctrl, 1, &data)) return -1;
	if (st.chip_cfg.dmp_on) {
		data = BIT_FIFO_RST | BIT_DMP_RST;
		if (MPU_Write_Len(st.hw->addr, st.reg->user_ctrl, 1, &data)) return -1;
		delay_ms(50);
		data = BIT_DMP_EN | BIT_FIFO_EN;
		if (st.chip_cfg.sensors & INV_XYZ_COMPASS)data |= BIT_AUX_IF_EN;
		if (MPU_Write_Len(st.hw->addr, st.reg->user_ctrl, 1, &data)) return -1;
		if (st.chip_cfg.int_enable)data = BIT_DMP_INT_EN;
		else {data = 0;}
		if (MPU_Write_Len(st.hw->addr, st.reg->int_enable, 1, &data)) return -1;
		data = 0;
		if (MPU_Write_Len(st.hw->addr, st.reg->fifo_en, 1, &data)) return -1;
	}
	else {
		data = BIT_FIFO_RST;
		if (MPU_Write_Len(st.hw->addr, st.reg->user_ctrl, 1, &data)) return -1;
		if (st.chip_cfg.bypass_mode || !(st.chip_cfg.sensors & INV_XYZ_COMPASS))data = BIT_FIFO_EN;
		else data = BIT_FIFO_EN | BIT_AUX_IF_EN;
		if (MPU_Write_Len(st.hw->addr, st.reg->user_ctrl, 1, &data)) return -1;
		delay_ms(50);
		if (st.chip_cfg.int_enable) data = BIT_DATA_RDY_EN;
		else data = 0;
		if (MPU_Write_Len(st.hw->addr, st.reg->int_enable, 1, &data)) return -1;
		if (MPU_Write_Len(st.hw->addr, st.reg->fifo_en, 1, &st.chip_cfg.fifo_enable)) return -1;
	}
	return 0;
}
/*获取陀螺量程值
*fsr:量程值
return:
	0:ok
	!0:error*/
int mpu_get_gyro_fsr(unsigned short *fsr){
	switch (st.chip_cfg.gyro_fsr) {
	case INV_FSR_250DPS:
		fsr[0] = 250;
		break;
	case INV_FSR_500DPS:
		fsr[0] = 500;
		break;
	case INV_FSR_1000DPS:
		fsr[0] = 1000;
		break;
	case INV_FSR_2000DPS:
		fsr[0] = 2000;
		break;
	default:
		fsr[0] = 0;
		break;
	}
	return 0;
}
/*设置陀螺仪量程
fsr:量程值
return:
	0:ok
	!0:error*/
int mpu_set_gyro_fsr(unsigned short fsr){
	unsigned char data;
	if (!(st.chip_cfg.sensors)) return -1;
	switch (fsr) {
	case 250:
		data = INV_FSR_250DPS << 3;
		break;
	case 500:
		data = INV_FSR_500DPS << 3;
		break;
	case 1000:
		data = INV_FSR_1000DPS << 3;
		break;
	case 2000:
		data = INV_FSR_2000DPS << 3;
		break;
	default: return -1;
	}
	if (st.chip_cfg.gyro_fsr == (data >> 3)) return 0;
	if (MPU_Write_Len(st.hw->addr, st.reg->gyro_cfg, 1, &data)) return -1;
	st.chip_cfg.gyro_fsr = data >> 3;
	return 0;
}

/*获得加速度满量程
*fer:获取的加速度满量程
return:
	0:ok
	!0:error*/
int mpu_get_accel_fsr(unsigned char *fsr){
	switch (st.chip_cfg.accel_fsr) {
	case INV_FSR_2G:
		fsr[0] = 2;
		break;
	case INV_FSR_4G:
		fsr[0] = 4;
		break;
	case INV_FSR_8G:
		fsr[0] = 8;
		break;
	case INV_FSR_16G:
		fsr[0] = 16;
		break;
	default: return -1;
	}
	if (st.chip_cfg.accel_half)
		fsr[0] <<= 1;
	return 0;
}

/*设置加速度计满量程
fsr:加速度量程值
return:
	0:ok
	!0:error*/
int mpu_set_accel_fsr(unsigned char fsr){
	unsigned char data;
	if (!(st.chip_cfg.sensors)) return -1;
	switch (fsr) {
	case 2:
		data = INV_FSR_2G << 3;
		break;
	case 4:
		data = INV_FSR_4G << 3;
		break;
	case 8:
		data = INV_FSR_8G << 3;
		break;
	case 16:
		data = INV_FSR_16G << 3;
		break;
	default: return -1;
	}
	if (st.chip_cfg.accel_fsr == (data >> 3)) return 0;
	if (MPU_Write_Len(st.hw->addr, st.reg->accel_cfg, 1, &data)) return -1;
	st.chip_cfg.accel_fsr = data >> 3;
	return 0;
}

/*获取当前采样率 DLPF:采样率
*lpf:采样率
return:
	0:ok
	!0:error*/
int mpu_get_lpf(unsigned short *lpf){
	switch (st.chip_cfg.lpf) {
	case INV_FILTER_188HZ:
		lpf[0] = 188;
		break;
	case INV_FILTER_98HZ:
		lpf[0] = 98;
		break;
	case INV_FILTER_42HZ:
		lpf[0] = 42;
		break;
	case INV_FILTER_20HZ:
		lpf[0] = 20;
		break;
	case INV_FILTER_10HZ:
		lpf[0] = 10;
		break;
	case INV_FILTER_5HZ:
		lpf[0] = 5;
		break;
	case INV_FILTER_256HZ_NOLPF2:
	case INV_FILTER_2100HZ_NOLPF:
	default:
		lpf[0] = 0;
		break;
	}
	return 0;
}
/*设置数字低通滤波器值
lpf:数字低通滤波器值
return:
	0:ok
	!0:error*/
int mpu_set_lpf(unsigned short lpf){
	unsigned char data;
	if (!(st.chip_cfg.sensors)) return -1;
	if (lpf >= 188) data = INV_FILTER_188HZ;
	else if (lpf >= 98) data = INV_FILTER_98HZ;
	else if (lpf >= 42) data = INV_FILTER_42HZ;
	else if (lpf >= 20) data = INV_FILTER_20HZ;
	else if (lpf >= 10) data = INV_FILTER_10HZ;
	else data = INV_FILTER_5HZ;
	if (st.chip_cfg.lpf == data) return 0;
	if (MPU_Write_Len(st.hw->addr, st.reg->lpf, 1, &data)) return -1;
	st.chip_cfg.lpf = data;
	return 0;
}

/*获取采样率
*rate:采样率值
return:
	0:ok
	!0:erron*/
int mpu_get_sample_rate(unsigned short *rate){
	if (st.chip_cfg.dmp_on) return -1;
	else rate[0] = st.chip_cfg.sample_rate;
	return 0;
}

/*设置采样率
rate:要设置的采样率
return:
	0:ok
	!0:error*/
int mpu_set_sample_rate(unsigned short rate){
	unsigned char data;
	if (!(st.chip_cfg.sensors)) return -1;
	if (st.chip_cfg.dmp_on) return -1;
	else {
		if (st.chip_cfg.lp_accel_mode) {
			if (rate && (rate <= 40)) {
				/*保持在低功率模式*/
				mpu_lp_accel_mode(rate);
				return 0;
			}
			/*设置的频率超过上限,设置为允许值上限*/
			mpu_lp_accel_mode(0);
		}
		if (rate < 4) rate = 4;
		else if (rate > 1000) rate = 1000;
		data = 1000 / rate - 1;
		if (MPU_Write_Len(st.hw->addr, st.reg->rate_div, 1, &data)) return -1;
		st.chip_cfg.sample_rate = 1000 / (1 + data);
		/*自动设置为1/2采样率*/
		mpu_set_lpf(st.chip_cfg.sample_rate >> 1);
		return 0;
	}
}
/*获取陀螺仪灵敏度
*sens:灵敏度值:转换为dps
return:
	0:ok
	!0:error*/
int mpu_get_gyro_sens(float *sens){
	switch (st.chip_cfg.gyro_fsr) {
	case INV_FSR_250DPS:
		sens[0] = 131.f;
		break;
	case INV_FSR_500DPS:
		sens[0] = 65.5f;
		break;
	case INV_FSR_1000DPS:
		sens[0] = 32.8f;
		break;
	case INV_FSR_2000DPS:
		sens[0] = 16.4f;
		break;
	default:
		return -1;
	}
	return 0;
}

/*获取加速度仪灵敏度
*sens:获取的加速度仪灵敏度,转化为g/s
return:
	0:ok
	!0:error*/
int mpu_get_accel_sens(unsigned short *sens){
	switch (st.chip_cfg.accel_fsr) {
	case INV_FSR_2G:
		sens[0] = 16384;
		break;
	case INV_FSR_4G:
		sens[0] = 8092;
		break;
	case INV_FSR_8G:
		sens[0] = 4096;
		break;
	case INV_FSR_16G:
		sens[0] = 2048;
		break;
	default:
		return -1;
	}
	if (st.chip_cfg.accel_half) sens[0] >>= 1;
	return 0;
}

/*获取当前FIFO配置
包含以下值
* INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
* INV_XYZ_GYRO
* INV_XYZ_ACCELs
sensors FIFO中传感器的掩码
return:
	0:ok
	!0:error*/
int mpu_get_fifo_config(unsigned char *sensors){
	sensors[0] = st.chip_cfg.fifo_enable;
	return 0;
}

/*选择放入FIFO的传感器
可以是以下值
* INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
* INV_XYZ_GYRO
* INV_XYZ_ACCELs
sensors:放入的传感器掩码
return:
	0:ok
	!0:error*/
int mpu_configure_fifo(unsigned char sensors){
	unsigned char prev;
	int result = 0;
	/*数据没有进入FIFO,stop*/
	sensors &= ~INV_XYZ_COMPASS;
	if (st.chip_cfg.dmp_on) return 0;
	else {
		if (!(st.chip_cfg.sensors)) return -1;
		prev = st.chip_cfg.fifo_enable;
		st.chip_cfg.fifo_enable = sensors & st.chip_cfg.sensors;
		if (st.chip_cfg.fifo_enable != sensors) result = -1;/*没有设置到指定的传感器*/
		else result = 0;
		if (sensors || st.chip_cfg.lp_accel_mode) set_int_enable(1);
		else set_int_enable(0);
		if (sensors) {
			if (mpu_reset_fifo()) {
				st.chip_cfg.fifo_enable = prev;
				return -1;
			}
		}
	}
	return result;
}

/*获取当前电源状态
*power_on:
\n1:开
\n0:关
@return:0,ok*/
int mpu_get_power_state(unsigned char *power_on){
	if (st.chip_cfg.sensors) power_on[0] = 1;
	else power_on[0] = 0;
	return 0;
}

/*指定传感器开/关
sensors:可以是以下值
INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
INV_XYZ_GYRO
INV_XYZ_ACCEL
INV_XYZ_COMPASS
return:0,ok*/
int mpu_set_sensors(unsigned char sensors){
	unsigned char data;
	if (sensors & INV_XYZ_GYRO) data = INV_CLK_PLL;
	else if (sensors) data = 0;
	else data = BIT_SLEEP;
	if (MPU_Write_Len(st.hw->addr, st.reg->pwr_mgmt_1, 1, &data)) {
		st.chip_cfg.sensors = 0;
		return -1;
	}
	st.chip_cfg.clk_src = data & ~BIT_SLEEP;
	data = 0;
	if (!(sensors & INV_X_GYRO)) data |= BIT_STBY_XG;
	if (!(sensors & INV_Y_GYRO)) data |= BIT_STBY_YG;
	if (!(sensors & INV_Z_GYRO)) data |= BIT_STBY_ZG;
	if (!(sensors & INV_XYZ_ACCEL)) data |= BIT_STBY_XYZA;
	if (MPU_Write_Len(st.hw->addr, st.reg->pwr_mgmt_2, 1, &data)) {
		st.chip_cfg.sensors = 0;
		return -1;
	}
	if (sensors && (sensors != INV_XYZ_ACCEL))
	/*锁定中断仅用于LP加速模式*/
	mpu_set_int_latched(0);
	st.chip_cfg.sensors = sensors;
	st.chip_cfg.lp_accel_mode = 0;
	delay_ms(50);
	return 0;
}

/*获取中断状态寄存器值
*status:中断状态寄存器值
return:0,ok*/
int mpu_get_int_status(short *status){
	unsigned char tmp[2];
	if (!st.chip_cfg.sensors) return -1;
	if (MPU_Read_Len(st.hw->addr, st.reg->dmp_int_status, 2, tmp)) return -1;
	status[0] = (tmp[0] << 8) | tmp[1];
	return 0;
}

/*从FIFO中获取指定传感器的值
*sensors指定传感器
*sensors可包含以下值
INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
INV_XYZ_GYRO
INV_XYZ_ACCEL
如果FIFO中没有新值,*sensors为0
如果FIFO未启用,将返回一个!0的错误代码
*gyro 陀螺仪数据
*accel 加速度仪数据
*timestamp 时间戳.ms
*sensors  读取的传感器掩码
*more  剩余数据数
return:0,ok*/
int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
        unsigned char *sensors, unsigned char *more){
	/*最大数据包accel[6],gyro[6]*/
	unsigned char data[MAX_PACKET_LENGTH];
	unsigned char packet_size = 0;
	unsigned short fifo_count, index = 0;
	if (st.chip_cfg.dmp_on) return -1;
	sensors[0] = 0;
	if (!st.chip_cfg.sensors) return -1;
	if (!st.chip_cfg.fifo_enable) return -1;
	if (st.chip_cfg.fifo_enable & INV_X_GYRO) packet_size += 2;
	if (st.chip_cfg.fifo_enable & INV_Y_GYRO) packet_size += 2;
	if (st.chip_cfg.fifo_enable & INV_Z_GYRO) packet_size += 2;
	if (st.chip_cfg.fifo_enable & INV_XYZ_ACCEL) packet_size += 6;
	if (MPU_Read_Len(st.hw->addr, st.reg->fifo_count_h, 2, data)) return -1;
	fifo_count = (data[0] << 8) | data[1];
	if (fifo_count < packet_size) return 0;
	if (fifo_count > (st.hw->max_fifo >> 1)) {
		/*FIFO1/2满,检查溢出*/
		if (MPU_Read_Len(st.hw->addr, st.reg->int_status, 1, data)) return -1;
		if (data[0] & BIT_FIFO_OVERFLOW) {
			mpu_reset_fifo();
			return -2;
		}
	}
	get_ms((unsigned long*)timestamp);
	if (MPU_Read_Len(st.hw->addr, st.reg->fifo_r_w, packet_size, data)) return -1;
	more[0] = fifo_count / packet_size - 1;
	sensors[0] = 0;
	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_XYZ_ACCEL) {
		accel[0] = (data[index+0] << 8) | data[index+1];
		accel[1] = (data[index+2] << 8) | data[index+3];
		accel[2] = (data[index+4] << 8) | data[index+5];
		sensors[0] |= INV_XYZ_ACCEL;
		index += 6;
	}
	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_X_GYRO) {
		gyro[0] = (data[index+0] << 8) | data[index+1];
		sensors[0] |= INV_X_GYRO;
		index += 2;
	}
	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Y_GYRO) {
		gyro[1] = (data[index+0] << 8) | data[index+1];
		sensors[0] |= INV_Y_GYRO;
		index += 2;
	}
	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Z_GYRO) {
		gyro[2] = (data[index+0] << 8) | data[index+1];
		sensors[0] |= INV_Z_GYRO;
		index += 2;
	}
	return 0;
}

/*从FIFO获取未解析的数据包
length	数据长度
data	FIFO数据
more	剩余数据数
return:0,ok*/
int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
    unsigned char *more){
	unsigned char tmp[2];
	unsigned short fifo_count;
	if (!st.chip_cfg.dmp_on) {return -1;}
	if (!st.chip_cfg.sensors){return -1;}
	if (MPU_Read_Len(st.hw->addr, st.reg->fifo_count_h, 2, tmp)) {return -1;}
	fifo_count = (tmp[0] << 8) | tmp[1];
	if (fifo_count < length) {
		more[0] = 0;
		return -1;
	}
	if (fifo_count > (st.hw->max_fifo >> 1)) {
		/*FIFO1/2满,检查溢出*/
		if (MPU_Read_Len(st.hw->addr, st.reg->int_status,1,tmp)) {return -1;}
		if (tmp[0] & BIT_FIFO_OVERFLOW) {
			mpu_reset_fifo();
			printf("mpu_read_fifo_stream6：error:tmp[0]=%d",tmp[0]&BIT_FIFO_OVERFLOW);
			return -2;
		}
	}
	if (MPU_Read_Len(st.hw->addr, st.reg->fifo_r_w, length, data)) {return -1;}
	more[0] = fifo_count / length - 1;
	return 0;
}

/*设置设备为旁路模式
bypass_on:1开
return:0,ok*/
int mpu_set_bypass(unsigned char bypass_on){
	unsigned char tmp;
	if (st.chip_cfg.bypass_mode == bypass_on) return 0;
	if (bypass_on) {
		if (MPU_Read_Len(st.hw->addr, st.reg->user_ctrl, 1, &tmp)) return -1;
		tmp &= ~BIT_AUX_IF_EN;
		if (MPU_Write_Len(st.hw->addr, st.reg->user_ctrl, 1, &tmp)) return -1;
		delay_ms(3);
		tmp = BIT_BYPASS_EN;
		if (st.chip_cfg.active_low_int) tmp |= BIT_ACTL;
		if (st.chip_cfg.latched_int) tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
		if (MPU_Write_Len(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp)) return -1;
	}
	else {
		if (MPU_Read_Len(st.hw->addr, st.reg->user_ctrl, 1, &tmp)) return -1;
		if (st.chip_cfg.sensors & INV_XYZ_COMPASS) tmp |= BIT_AUX_IF_EN;
		else
		tmp &= ~BIT_AUX_IF_EN;
		if (MPU_Write_Len(st.hw->addr, st.reg->user_ctrl, 1, &tmp)) return -1;
		delay_ms(3);
		if (st.chip_cfg.active_low_int) tmp = BIT_ACTL;
		else
		tmp = 0;
		if (st.chip_cfg.latched_int) tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
		if (MPU_Write_Len(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp)) return -1;
	}
	st.chip_cfg.bypass_mode = bypass_on;
	return 0;
}

/*设置中断有效电平
active_low:1,低电平有效
0,高电平有效
return:0,ok*/
int mpu_set_int_level(unsigned char active_low){
	st.chip_cfg.active_low_int = active_low;
	return 0;
}

/*使能关闭的中断
enable:1,开;0关
return:0,ok*/
int mpu_set_int_latched(unsigned char enable){
	unsigned char tmp;
	if (st.chip_cfg.latched_int == enable) return 0;
	if (enable)
			tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
	else
			tmp = 0;
	if (st.chip_cfg.bypass_mode)
			tmp |= BIT_BYPASS_EN;
	if (st.chip_cfg.active_low_int)
			tmp |= BIT_ACTL;
	if (MPU_Write_Len(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp)) return -1;
	st.chip_cfg.latched_int = enable;
	return 0;
}
static int get_accel_prod_shift(float *st_shift){
	unsigned char tmp[4], shift_code[3], ii;
	if (MPU_Read_Len(st.hw->addr, 0x0D, 4, tmp)) return 0x07;
	shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
	shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
	shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);
	for (ii = 0; ii < 3; ii++) {
		if (!shift_code[ii]) {
			st_shift[ii] = 0.f;
			continue;
		}
		/*约等于st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)*/
		st_shift[ii] = 0.34f;
		while (--shift_code[ii])
			st_shift[ii] *= 1.034f;
	}
	return 0;
}

static int accel_self_test(long *bias_regular, long *bias_st){
	int jj, result = 0;
	float st_shift[3], st_shift_cust, st_shift_var;
	get_accel_prod_shift(st_shift);
	for(jj = 0; jj < 3; jj++) {
		st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
		if (st_shift[jj]) {
			st_shift_var = st_shift_cust / st_shift[jj] - 1.f;
			if (fabs(st_shift_var) > test.max_accel_var)
				result |= 1 << jj;
		} else if ((st_shift_cust < test.min_g) ||
			(st_shift_cust > test.max_g))
			result |= 1 << jj;
	}
	return result;
}

static int gyro_self_test(long *bias_regular, long *bias_st){
	int jj, result = 0;
	unsigned char tmp[3];
	float st_shift, st_shift_cust, st_shift_var;
	if (MPU_Read_Len(st.hw->addr, 0x0D, 3, tmp)) return 0x07;
	tmp[0] &= 0x1F;
	tmp[1] &= 0x1F;
	tmp[2] &= 0x1F;
	for (jj = 0; jj < 3; jj++) {
		st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
		if (tmp[jj]) {
			st_shift = 3275.f / test.gyro_sens;
			while (--tmp[jj])
				st_shift *= 1.046f;
			st_shift_var = st_shift_cust / st_shift - 1.f;
			if (fabs(st_shift_var) > test.max_gyro_var)
				result |= 1 << jj;
		} else if ((st_shift_cust < test.min_dps) ||
			(st_shift_cust > test.max_dps))
			result |= 1 << jj;
	}
	return result;
}
static int get_st_biases(long *gyro, long *accel, unsigned char hw_test){
	unsigned char data[MAX_PACKET_LENGTH];
	unsigned char packet_count, ii;
	unsigned short fifo_count;
	data[0] = 0x01;
	data[1] = 0;
	if (MPU_Write_Len(st.hw->addr, st.reg->pwr_mgmt_1, 2, data)) return -1;
	delay_ms(200);
	data[0] = 0;
	if (MPU_Write_Len(st.hw->addr, st.reg->int_enable, 1, data)) return -1;
	if (MPU_Write_Len(st.hw->addr, st.reg->fifo_en, 1, data)) return -1;
	if (MPU_Write_Len(st.hw->addr, st.reg->pwr_mgmt_1, 1, data)) return -1;
	if (MPU_Write_Len(st.hw->addr, st.reg->i2c_mst, 1, data)) return -1;
	if (MPU_Write_Len(st.hw->addr, st.reg->user_ctrl, 1, data)) return -1;
	data[0] = BIT_FIFO_RST | BIT_DMP_RST;
	if (MPU_Write_Len(st.hw->addr, st.reg->user_ctrl, 1, data)) return -1;
	delay_ms(15);
	data[0] = st.test->reg_lpf;
	if (MPU_Write_Len(st.hw->addr, st.reg->lpf, 1, data))
			return -1;
	data[0] = st.test->reg_rate_div;
	if (MPU_Write_Len(st.hw->addr, st.reg->rate_div, 1, data)) return -1;
	if (hw_test)
		data[0] = st.test->reg_gyro_fsr | 0xE0;
	else
		data[0] = st.test->reg_gyro_fsr;
	if (MPU_Write_Len(st.hw->addr, st.reg->gyro_cfg, 1, data)) return -1;

	if (hw_test)
		data[0] = st.test->reg_accel_fsr | 0xE0;
	else
		data[0] = test.reg_accel_fsr;
	if (MPU_Write_Len(st.hw->addr, st.reg->accel_cfg, 1, data)) return -1;
	if (hw_test)
		delay_ms(200);
	/* Fill FIFO for test.wait_ms milliseconds. */
	data[0] = BIT_FIFO_EN;
	if (MPU_Write_Len(st.hw->addr, st.reg->user_ctrl, 1, data)) return -1;
	data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
	if (MPU_Write_Len(st.hw->addr, st.reg->fifo_en, 1, data)) return -1;
	delay_ms(test.wait_ms);
	data[0] = 0;
	if (MPU_Write_Len(st.hw->addr, st.reg->fifo_en, 1, data)) return -1;
	if (MPU_Read_Len(st.hw->addr, st.reg->fifo_count_h, 2, data)) return -1;
	fifo_count = (data[0] << 8) | data[1];
	packet_count = fifo_count / MAX_PACKET_LENGTH;
	gyro[0] = gyro[1] = gyro[2] = 0;
	accel[0] = accel[1] = accel[2] = 0;
	for (ii = 0; ii < packet_count; ii++) {
		short accel_cur[3], gyro_cur[3];
		if (MPU_Read_Len(st.hw->addr, st.reg->fifo_r_w, MAX_PACKET_LENGTH, data)) return -1;
		accel_cur[0] = ((short)data[0] << 8) | data[1];
		accel_cur[1] = ((short)data[2] << 8) | data[3];
		accel_cur[2] = ((short)data[4] << 8) | data[5];
		accel[0] += (long)accel_cur[0];
		accel[1] += (long)accel_cur[1];
		accel[2] += (long)accel_cur[2];
		gyro_cur[0] = (((short)data[6] << 8) | data[7]);
		gyro_cur[1] = (((short)data[8] << 8) | data[9]);
		gyro_cur[2] = (((short)data[10] << 8) | data[11]);
		gyro[0] += (long)gyro_cur[0];
		gyro[1] += (long)gyro_cur[1];
		gyro[2] += (long)gyro_cur[2];
	}
	gyro[0] = (long)(((long long)gyro[0]<<16) / test.gyro_sens / packet_count);
	gyro[1] = (long)(((long long)gyro[1]<<16) / test.gyro_sens / packet_count);
	gyro[2] = (long)(((long long)gyro[2]<<16) / test.gyro_sens / packet_count);
	accel[0] = (long)(((long long)accel[0]<<16) / test.accel_sens /
			packet_count);
	accel[1] = (long)(((long long)accel[1]<<16) / test.accel_sens /
			packet_count);
	accel[2] = (long)(((long long)accel[2]<<16) / test.accel_sens /
			packet_count);
	/* Don't remove gravity! */
	if (accel[2] > 0L)
		accel[2] -= 65536L;
	else
		accel[2] += 65536L;
	return 0;
}

/*陀螺仪,加速度仪自检
必须在z平行重力时使用
gyro 陀螺仪偏差
accel 加速度仪偏差

return:一个掩码,表示传感器
1(1):ok
0(0):error
掩码定义:
Bit 0:   Gyro.
Bit 1:   Accel.
Bit 2:   Compass.*/
int mpu_run_self_test(long *gyro, long *accel){
	const unsigned char tries = 2;
	long gyro_st[3], accel_st[3];
	unsigned char accel_result, gyro_result;
	int ii;
	int result;
	unsigned char accel_fsr, fifo_sensors, sensors_on;
	unsigned short gyro_fsr, sample_rate, lpf;
	unsigned char dmp_was_on;
	if (st.chip_cfg.dmp_on) {
		mpu_set_dmp_state(0);
		dmp_was_on = 1;
	}
	else
		dmp_was_on = 0;
	/*获取初始设置*/
	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);
	mpu_get_lpf(&lpf);
	mpu_get_sample_rate(&sample_rate);
	sensors_on = st.chip_cfg.sensors;
	mpu_get_fifo_config(&fifo_sensors);
	
	/*旧芯片自检不同*/
	for (ii = 0; ii < tries; ii++)
		if (!get_st_biases(gyro, accel, 0))
			break;
	if (ii == tries) {
		/*IICerror*/
		result = 0;
		goto restore;
	}
	for (ii = 0; ii < tries; ii++)
		if (!get_st_biases(gyro_st, accel_st, 1))
			break;
	if (ii == tries) {
		/* Again, probably an I2C error. */
		result = 0;
		goto restore;
	}
	accel_result = accel_self_test(accel, accel_st);
	gyro_result = gyro_self_test(gyro, gyro_st);
	result = 0;
	if (!gyro_result)
		result |= 0x01;
	if (!accel_result)
		result |= 0x02;

restore:
	/* Set to invalid values to ensure no I2C writes are skipped. */
	st.chip_cfg.gyro_fsr = 0xFF;
	st.chip_cfg.accel_fsr = 0xFF;
	st.chip_cfg.lpf = 0xFF;
	st.chip_cfg.sample_rate = 0xFFFF;
	st.chip_cfg.sensors = 0xFF;
	st.chip_cfg.fifo_enable = 0xFF;
	st.chip_cfg.clk_src = INV_CLK_PLL;
	mpu_set_gyro_fsr(gyro_fsr);
	mpu_set_accel_fsr(accel_fsr);
	mpu_set_lpf(lpf);
	mpu_set_sample_rate(sample_rate);
	mpu_set_sensors(sensors_on);
	mpu_configure_fifo(fifo_sensors);
	if (dmp_was_on)
		mpu_set_dmp_state(1);
	return result;
}

/*写入DMP内存
必须在mpu唤醒时使用
mem_addr   内存位置(bank << 8 | 起始位置)
length   写入的字节数
*data    要写入的字节
return:0,ok*/
int mpu_write_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data){
	unsigned char tmp[2];
	if (!data) return -1;
	if (!st.chip_cfg.sensors) return -1;
	tmp[0] = (unsigned char)(mem_addr >> 8);
	tmp[1] = (unsigned char)(mem_addr & 0xFF);
	/*检查储存范围,防止溢出*/
	if (tmp[1] + length > st.hw->bank_size) return -1;
	if (MPU_Write_Len(st.hw->addr, st.reg->bank_sel, 2, tmp)) return -1;
	if (MPU_Write_Len(st.hw->addr, st.reg->mem_r_w, length, data)) return -1;
	return 0;
}

/*读取DMP内存
必须在mpu唤醒时使用
mem_addr  内存地址(bank << 8 | 起始地址)
length    数据长度
*data    数据
return:0,ok*/
int mpu_read_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data){
    unsigned char tmp[2];
	if (!data) return -1;
	if (!st.chip_cfg.sensors) return -1;
	tmp[0] = (unsigned char)(mem_addr >> 8);
	tmp[1] = (unsigned char)(mem_addr & 0xFF);
	/*防止溢出*/
	if (tmp[1] + length > st.hw->bank_size) return -1;
	if (MPU_Write_Len(st.hw->addr, st.reg->bank_sel, 2, tmp)) return -1;
	if (MPU_Read_Len(st.hw->addr, st.reg->mem_r_w, length, data)) return -1;
	return 0;
}

/*加载和验证DMP图像
length   DMP图像长度
firmware    DMP代号
start_addr  DMP储存起始地址
sample_rate DMP启用时采用固定采样率
return:0,ok*/
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate){
	unsigned short ii;
	unsigned short this_write;
	/*平均划分为st.hw->bank_size大小,避免地址重复*/
#define LOAD_CHUNK  (16)
	unsigned char cur[LOAD_CHUNK], tmp[2];
	if (st.chip_cfg.dmp_loaded) return -1;/* DMP只可加载一次*/
	if (!firmware) return -1;
	for (ii = 0; ii < length; ii += this_write) {
		this_write = min(LOAD_CHUNK, length - ii);
		if (mpu_write_mem(ii, this_write, (unsigned char*)&firmware[ii])) return -1;
		if (mpu_read_mem(ii, this_write, cur)) return -1;
		if (memcmp(firmware+ii, cur, this_write)) return -2;
	}
	/*设置起始地址*/
	tmp[0] = start_addr >> 8;
	tmp[1] = start_addr & 0xFF;
	if (MPU_Write_Len(st.hw->addr, st.reg->prgm_start_h, 2, tmp)) return -1;
	st.chip_cfg.dmp_loaded = 1;
	st.chip_cfg.dmp_sample_rate = sample_rate;
	return 0;
}

/*DMP开关
return:
	0:ok
	!0:error*/
int mpu_set_dmp_state(unsigned char enable)
{
	unsigned char tmp;
	if (st.chip_cfg.dmp_on == enable){return 0;}
	if (enable) {
		if (!st.chip_cfg.dmp_loaded)return -1;
		/*关闭数据准备中断*/
		set_int_enable(0);
		/*关闭旁路模式*/
		mpu_set_bypass(0);
		/*为保持恒定的采样率,FIFO数据由DMP控制*/
		mpu_set_sample_rate(st.chip_cfg.dmp_sample_rate);
		/*删除FIFO设置*/
		tmp = 0;
		MPU_Write_Len(st.hw->addr, 0x23, 1, &tmp);
		st.chip_cfg.dmp_on = 1;
		/*启用DMP中断*/
		set_int_enable(1);
		mpu_reset_fifo();
	}
	
	else {
		/*关闭DMP中断*/
		set_int_enable(0);
		/*还原FIFO设置*/
		tmp = st.chip_cfg.fifo_enable;
		MPU_Write_Len(st.hw->addr, 0x23, 1, &tmp);
		st.chip_cfg.dmp_on = 0;
		mpu_reset_fifo();
	}
	return 0;
}

/*获取DMP状态
*enabled:1,on
return:0,ok*/
int mpu_get_dmp_state(unsigned char *enabled){
	enabled[0] = st.chip_cfg.dmp_on;
	return 0;
}
/*进入LP加速运动中断模式
retur:
	0:ok
	!0:error*/
int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,\
	unsigned char lpa_freq){
	unsigned char data[3];
	if (lpa_freq){
		unsigned char thresh_hw;
		if (thresh > 8160){thresh_hw = 255;}
		else if (thresh < 32){thresh_hw = 1;}
		else {thresh_hw = thresh >> 5;}
		if (!time)time = 1;
		if (lpa_freq > 40)return -1;
		if (!st.chip_cfg.int_motion_only) {
			/*储存当前设置,之后启用*/
			if (st.chip_cfg.dmp_on) {
				mpu_set_dmp_state(0);
				st.chip_cfg.cache.dmp_on = 1;
			}
			else
				st.chip_cfg.cache.dmp_on = 0;
			mpu_get_gyro_fsr(&st.chip_cfg.cache.gyro_fsr);
			mpu_get_accel_fsr(&st.chip_cfg.cache.accel_fsr);
			mpu_get_lpf(&st.chip_cfg.cache.lpf);
			mpu_get_sample_rate(&st.chip_cfg.cache.sample_rate);
			st.chip_cfg.cache.sensors_on = st.chip_cfg.sensors;
			mpu_get_fifo_config(&st.chip_cfg.cache.fifo_sensors);
		}
		/*禁用硬件中断*/
		set_int_enable(0);
		/*最大功率加速*/
		mpu_lp_accel_mode(0);
		data[0] = INV_FILTER_256HZ_NOLPF2;
		if (MPU_Write_Len(st.hw->addr, st.reg->lpf, 1, data)) return -1;
		/*配置数字高通滤波器,这里不会修改其任何位*/

		/*配置设备运动发送中断*/
		/*启用运动中断*/
		data[0] = BIT_MOT_INT_EN;
		if (MPU_Write_Len(st.hw->addr, st.reg->int_enable, 1, data))
			goto lp_int_restore;
		/*设置运动中断参数*/
		data[0] = thresh_hw;
		data[1] = time;
		if (MPU_Write_Len(st.hw->addr, st.reg->motion_thr, 2, data))
			goto lp_int_restore;
		/*使硬件锁定当前加速样本*/
		delay_ms(5);
		data[0] = (st.chip_cfg.accel_fsr << 3) | BITS_HPF;
		if (MPU_Write_Len(st.hw->addr, st.reg->accel_cfg, 1, data))
			goto lp_int_restore;
		/*设置LP加速模式*/
		data[0] = BIT_LPA_CYCLE;
		if (lpa_freq == 1)
			data[1] = INV_LPA_1_25HZ;
		else if (lpa_freq <= 5)
			data[1] = INV_LPA_5HZ;
		else if (lpa_freq <= 20)
			data[1] = INV_LPA_20HZ;
		else
			data[1] = INV_LPA_40HZ;
		data[1] = (data[1] << 6) | BIT_STBY_XYZG;
		if (MPU_Write_Len(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
			goto lp_int_restore;
		st.chip_cfg.int_motion_only = 1;
		return 0;
	}
	else {
		int ii;
		char *cache_ptr = (char*)&st.chip_cfg.cache;
		for (ii = 0; ii < sizeof(st.chip_cfg.cache); ii++) {
			if (cache_ptr[ii] != 0)goto lp_int_restore;
		}
		return -1;
	}
lp_int_restore:
	st.chip_cfg.gyro_fsr = 0xFF;
	st.chip_cfg.accel_fsr = 0xFF;
	st.chip_cfg.lpf = 0xFF;
	st.chip_cfg.sample_rate = 0xFFFF;
	st.chip_cfg.sensors = 0xFF;
	st.chip_cfg.fifo_enable = 0xFF;
	st.chip_cfg.clk_src = INV_CLK_PLL;
	mpu_set_sensors(st.chip_cfg.cache.sensors_on);
	mpu_set_gyro_fsr(st.chip_cfg.cache.gyro_fsr);
	mpu_set_accel_fsr(st.chip_cfg.cache.accel_fsr);
	mpu_set_lpf(st.chip_cfg.cache.lpf);
	mpu_set_sample_rate(st.chip_cfg.cache.sample_rate);
	mpu_configure_fifo(st.chip_cfg.cache.fifo_sensors);
	if (st.chip_cfg.cache.dmp_on)
		mpu_set_dmp_state(1);
	st.chip_cfg.int_motion_only = 0;
	return 0;
}

/*移植inv_mpu_dmp_motion_driver.h
传感器驱动*/
/************************************************************************************************/
/*加载映像到BMP*/
static int dmp_load_motion_driver_firmware(void){
	return mpu_load_firmware(DMP_CODE_SIZE, dmp_memory, sStartAddress,DMP_SAMPLE_RATE);
}

/*将陀螺仪,加速度仪写到BMP
将矩阵转为标量
在机体框架中定位陀螺仪和加速度方向
return:0,ok*/
static int dmp_set_orientation(unsigned short orient){
	unsigned char gyro_regs[3], accel_regs[3];
	const unsigned char gyro_axes[3] = {DINA4C, DINACD, DINA6C};
	const unsigned char accel_axes[3] = {DINA0C, DINAC9, DINA2C};
	const unsigned char gyro_sign[3] = {DINA36, DINA56, DINA76};
	const unsigned char accel_sign[3] = {DINA26, DINA46, DINA66};
	gyro_regs[0] = gyro_axes[orient & 3];
	gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
	gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
	accel_regs[0] = accel_axes[orient & 3];
	accel_regs[1] = accel_axes[(orient >> 3) & 3];
	accel_regs[2] = accel_axes[(orient >> 6) & 3];
	/* Chip-to-body, axes only. */
	if (mpu_write_mem(FCFG_1, 3, gyro_regs)) return -1;
	if (mpu_write_mem(FCFG_2, 3, accel_regs)) return -1;
	memcpy(gyro_regs, gyro_sign, 3);
	memcpy(accel_regs, accel_sign, 3);
	if (orient & 4) {
		gyro_regs[0] |= 1;
		accel_regs[0] |= 1;
	}
	if (orient & 0x20) {
		gyro_regs[1] |= 1;
		accel_regs[1] |= 1;
	}
	if (orient & 0x100) {
		gyro_regs[2] |= 1;
		accel_regs[2] |= 1;
	}
	/* Chip-to-body, sign only. */
	if (mpu_write_mem(FCFG_3, 3, gyro_regs)) return -1;
	if (mpu_write_mem(FCFG_7, 3, accel_regs)) return -1;
	dmp.orient = orient;
	return 0;
}

/*将陀螺仪偏差写入DMP
陀螺仪积分是在DMP中处理的,由MPL计算的陀螺仪偏差都应被写DMP内存中,消除三轴四元数漂移
如果基于DMP的qyro校准是启用的，DMP将覆盖写入该位置的偏差
*bias:q16的陀螺仪偏差
return:0,ok*/
static int dmp_set_gyro_bias(long *bias)
{
	long gyro_bias_body[3];
	unsigned char regs[4];
	gyro_bias_body[0] = bias[dmp.orient & 3];
	if (dmp.orient & 4)
		gyro_bias_body[0] *= -1;
	gyro_bias_body[1] = bias[(dmp.orient >> 3) & 3];
	if (dmp.orient & 0x20)
		gyro_bias_body[1] *= -1;
	gyro_bias_body[2] = bias[(dmp.orient >> 6) & 3];
	if (dmp.orient & 0x100)
		gyro_bias_body[2] *= -1;
	gyro_bias_body[0] = (long)(((long long)gyro_bias_body[0] * GYRO_SF) >> 30);
	gyro_bias_body[1] = (long)(((long long)gyro_bias_body[1] * GYRO_SF) >> 30);
	gyro_bias_body[2] = (long)(((long long)gyro_bias_body[2] * GYRO_SF) >> 30);
	regs[0] = (unsigned char)((gyro_bias_body[0] >> 24) & 0xFF);
	regs[1] = (unsigned char)((gyro_bias_body[0] >> 16) & 0xFF);
	regs[2] = (unsigned char)((gyro_bias_body[0] >> 8) & 0xFF);
	regs[3] = (unsigned char)(gyro_bias_body[0] & 0xFF);
	if (mpu_write_mem(D_EXT_GYRO_BIAS_X, 4, regs)) return -1;

	regs[0] = (unsigned char)((gyro_bias_body[1] >> 24) & 0xFF);
	regs[1] = (unsigned char)((gyro_bias_body[1] >> 16) & 0xFF);
	regs[2] = (unsigned char)((gyro_bias_body[1] >> 8) & 0xFF);
	regs[3] = (unsigned char)(gyro_bias_body[1] & 0xFF);
	if (mpu_write_mem(D_EXT_GYRO_BIAS_Y, 4, regs)) return -1;

	regs[0] = (unsigned char)((gyro_bias_body[2] >> 24) & 0xFF);
	regs[1] = (unsigned char)((gyro_bias_body[2] >> 16) & 0xFF);
	regs[2] = (unsigned char)((gyro_bias_body[2] >> 8) & 0xFF);
	regs[3] = (unsigned char)(gyro_bias_body[2] & 0xFF);
	return mpu_write_mem(D_EXT_GYRO_BIAS_Z, 4, regs);
}

/*将加速度仪偏差写入DMP
将覆盖之前的偏差
*bias  q16的加速度偏差。
return:0,ok*/
static int dmp_set_accel_bias(long *bias){
	long accel_bias_body[3];
	unsigned char regs[12];
	long long accel_sf;
	unsigned short accel_sens;
	mpu_get_accel_sens(&accel_sens);
	accel_sf = (long long)accel_sens << 15;
	accel_bias_body[0] = bias[dmp.orient & 3];
	if (dmp.orient & 4)
			accel_bias_body[0] *= -1;
	accel_bias_body[1] = bias[(dmp.orient >> 3) & 3];
	if (dmp.orient & 0x20)
			accel_bias_body[1] *= -1;
	accel_bias_body[2] = bias[(dmp.orient >> 6) & 3];
	if (dmp.orient & 0x100)
			accel_bias_body[2] *= -1;

	accel_bias_body[0] = (long)(((long long)accel_bias_body[0] * accel_sf) >> 30);
	accel_bias_body[1] = (long)(((long long)accel_bias_body[1] * accel_sf) >> 30);
	accel_bias_body[2] = (long)(((long long)accel_bias_body[2] * accel_sf) >> 30);

	regs[0] = (unsigned char)((accel_bias_body[0] >> 24) & 0xFF);
	regs[1] = (unsigned char)((accel_bias_body[0] >> 16) & 0xFF);
	regs[2] = (unsigned char)((accel_bias_body[0] >> 8) & 0xFF);
	regs[3] = (unsigned char)(accel_bias_body[0] & 0xFF);
	regs[4] = (unsigned char)((accel_bias_body[1] >> 24) & 0xFF);
	regs[5] = (unsigned char)((accel_bias_body[1] >> 16) & 0xFF);
	regs[6] = (unsigned char)((accel_bias_body[1] >> 8) & 0xFF);
	regs[7] = (unsigned char)(accel_bias_body[1] & 0xFF);
	regs[8] = (unsigned char)((accel_bias_body[2] >> 24) & 0xFF);
	regs[9] = (unsigned char)((accel_bias_body[2] >> 16) & 0xFF);
	regs[10] = (unsigned char)((accel_bias_body[2] >> 8) & 0xFF);
	regs[11] = (unsigned char)(accel_bias_body[2] & 0xFF);
	return mpu_write_mem(D_ACCEL_BIAS, 12, regs);
}

/*设置DMP输出速率
只在DMP开启时可用
rate  速率
return;0,ok*/
static int dmp_set_fifo_rate(unsigned short rate){
	const unsigned char regs_end[12] = {DINAFE, DINAF2, DINAAB,
			0xc4, DINAAA, DINAF1, DINADF, DINADF, 0xBB, 0xAF, DINADF, DINADF};
	unsigned short div;
	unsigned char tmp[8];

	if (rate > DMP_SAMPLE_RATE) return -1;
	div = DMP_SAMPLE_RATE / rate - 1;
	tmp[0] = (unsigned char)((div >> 8) & 0xFF);
	tmp[1] = (unsigned char)(div & 0xFF);
	if (mpu_write_mem(D_0_22, 2, tmp)) return -1;
	if (mpu_write_mem(CFG_6, 12, (unsigned char*)regs_end)) return -1;

	dmp.fifo_rate = rate;
	return 0;
}

/*获取DMP输出速率
*rate:获取的速率
return:0,ok*/
// static int dmp_get_fifo_rate(unsigned short *rate){
// 	rate[0] = dmp.fifo_rate;
// 	return 0;
// }

/*设置指定轴的阈值
axis    加速度的XYZ为1, 2, 4
thresh  mg/ms.
return:0,ok*/
static int dmp_set_tap_thresh(unsigned char axis, unsigned short thresh)
{
	unsigned char tmp[4], accel_fsr;
	float scaled_thresh;
	unsigned short dmp_thresh, dmp_thresh_2;
	if (!(axis & TAP_XYZ) || thresh > 1600) return -1;

	scaled_thresh = (float)thresh / DMP_SAMPLE_RATE;
	mpu_get_accel_fsr(&accel_fsr);
	switch (accel_fsr) {
	case 2:
		dmp_thresh = (unsigned short)(scaled_thresh * 16384);
		/* dmp_thresh * 0.75 */
		dmp_thresh_2 = (unsigned short)(scaled_thresh * 12288);
		break;
	case 4:
		dmp_thresh = (unsigned short)(scaled_thresh * 8192);
		/* dmp_thresh * 0.75 */
		dmp_thresh_2 = (unsigned short)(scaled_thresh * 6144);
		break;
	case 8:
		dmp_thresh = (unsigned short)(scaled_thresh * 4096);
		/* dmp_thresh * 0.75 */
		dmp_thresh_2 = (unsigned short)(scaled_thresh * 3072);
		break;
	case 16:
		dmp_thresh = (unsigned short)(scaled_thresh * 2048);
		/* dmp_thresh * 0.75 */
		dmp_thresh_2 = (unsigned short)(scaled_thresh * 1536);
		break;
	default: return -1;
	}
	tmp[0] = (unsigned char)(dmp_thresh >> 8);
	tmp[1] = (unsigned char)(dmp_thresh & 0xFF);
	tmp[2] = (unsigned char)(dmp_thresh_2 >> 8);
	tmp[3] = (unsigned char)(dmp_thresh_2 & 0xFF);

	if (axis & TAP_X) {
		if (mpu_write_mem(DMP_TAP_THX, 2, tmp)) return -1;
		if (mpu_write_mem(D_1_36, 2, tmp+2)) return -1;
	}
	if (axis & TAP_Y) {
		if (mpu_write_mem(DMP_TAP_THY, 2, tmp)) return -1;
		if (mpu_write_mem(D_1_40, 2, tmp+2)) return -1;
	}
	if (axis & TAP_Z) {
		if (mpu_write_mem(DMP_TAP_THZ, 2, tmp)) return -1;
		if (mpu_write_mem(D_1_44, 2, tmp+2)) return -1;
	}
	return 0;
}

/*选择一个轴为主轴
 *  @param[in]  axis    1, 2, and 4 for XYZ, respectively.
 *  @return     0 if successful.
 */
static int dmp_set_tap_axes(unsigned char axis){
	unsigned char tmp = 0;
	if (axis & TAP_X)
		tmp |= 0x30;
	if (axis & TAP_Y)
		tmp |= 0x0C;
	if (axis & TAP_Z)
		tmp |= 0x03;
	return mpu_write_mem(D_1_72, 1, &tmp);
}

/**
 *  @brief      Set minimum number of taps needed for an interrupt.
 *  @param[in]  min_taps    Minimum consecutive taps (1-4).
 *  @return     0 if successful.
 */
static int dmp_set_tap_count(unsigned char min_taps)
{
    unsigned char tmp;

    if (min_taps < 1)
        min_taps = 1;
    else if (min_taps > 4)
        min_taps = 4;

    tmp = min_taps - 1;
    return mpu_write_mem(D_1_79, 1, &tmp);
}

/**
 *  @brief      Set length between valid taps.
 *  @param[in]  time    Milliseconds between taps.
 *  @return     0 if successful.
 */
static int dmp_set_tap_time(unsigned short time)
{
    unsigned short dmp_time;
    unsigned char tmp[2];

    dmp_time = time / (1000 / DMP_SAMPLE_RATE);
    tmp[0] = (unsigned char)(dmp_time >> 8);
    tmp[1] = (unsigned char)(dmp_time & 0xFF);
    return mpu_write_mem(DMP_TAPW_MIN, 2, tmp);
}

/**
 *  @brief      Set max time between taps to register as a multi-tap.
 *  @param[in]  time    Max milliseconds between taps.
 *  @return     0 if successful.
 */
static int dmp_set_tap_time_multi(unsigned short time)
{
    unsigned short dmp_time;
    unsigned char tmp[2];

    dmp_time = time / (1000 / DMP_SAMPLE_RATE);
    tmp[0] = (unsigned char)(dmp_time >> 8);
    tmp[1] = (unsigned char)(dmp_time & 0xFF);
    return mpu_write_mem(D_1_218, 2, tmp);
}

/**
 *  @brief      Set shake rejection threshold.
 *  If the DMP detects a gyro sample larger than @e thresh, taps are rejected.
 *  @param[in]  sf      Gyro scale factor.
 *  @param[in]  thresh  Gyro threshold in dps.
 *  @return     0 if successful.
 */
static int dmp_set_shake_reject_thresh(long sf, unsigned short thresh)
{
    unsigned char tmp[4];
    long thresh_scaled = sf / 1000 * thresh;
    tmp[0] = (unsigned char)(((long)thresh_scaled >> 24) & 0xFF);
    tmp[1] = (unsigned char)(((long)thresh_scaled >> 16) & 0xFF);
    tmp[2] = (unsigned char)(((long)thresh_scaled >> 8) & 0xFF);
    tmp[3] = (unsigned char)((long)thresh_scaled & 0xFF);
    return mpu_write_mem(D_1_92, 4, tmp);
}

/**
 *  @brief      Set shake rejection time.
 *  Sets the length of time that the gyro must be outside of the threshold set
 *  by @e gyro_set_shake_reject_thresh before taps are rejected. A mandatory
 *  60 ms is added to this parameter.
 *  @param[in]  time    Time in milliseconds.
 *  @return     0 if successful.
 */
static int dmp_set_shake_reject_time(unsigned short time)
{
    unsigned char tmp[2];

    time /= (1000 / DMP_SAMPLE_RATE);
    tmp[0] = time >> 8;
    tmp[1] = time & 0xFF;
    return mpu_write_mem(D_1_90,2,tmp);
}

/**
 *  @brief      Set shake rejection timeout.
 *  Sets the length of time after a shake rejection that the gyro must stay
 *  inside of the threshold before taps can be detected again. A mandatory
 *  60 ms is added to this parameter.
 *  @param[in]  time    Time in milliseconds.
 *  @return     0 if successful.
 */
static int dmp_set_shake_reject_timeout(unsigned short time)
{
    unsigned char tmp[2];

    time /= (1000 / DMP_SAMPLE_RATE);
    tmp[0] = time >> 8;
    tmp[1] = time & 0xFF;
    return mpu_write_mem(D_1_88,2,tmp);
}

// /**
//  *  @brief      Get current step count.
//  *  @param[out] count   Number of steps detected.
//  *  @return     0 if successful.
//  */
// static int dmp_get_pedometer_step_count(unsigned long *count){
// 	unsigned char tmp[4];
// 	if (!count) return -1;
// 	if (mpu_read_mem(D_PEDSTD_STEPCTR, 4, tmp)) return -1;
// 	count[0] = ((unsigned long)tmp[0] << 24) | ((unsigned long)tmp[1] << 16) |
// 		((unsigned long)tmp[2] << 8) | tmp[3];
// 	return 0;
// }

// /**
//  *  @brief      Overwrite current step count.
//  *  WARNING: This function writes to DMP memory and could potentially encounter
//  *  a race condition if called while the pedometer is enabled.
//  *  @param[in]  count   New step count.
//  *  @return     0 if successful.
//  */
// static int dmp_set_pedometer_step_count(unsigned long count){
// 	unsigned char tmp[4];

// 	tmp[0] = (unsigned char)((count >> 24) & 0xFF);
// 	tmp[1] = (unsigned char)((count >> 16) & 0xFF);
// 	tmp[2] = (unsigned char)((count >> 8) & 0xFF);
// 	tmp[3] = (unsigned char)(count & 0xFF);
// 	return mpu_write_mem(D_PEDSTD_STEPCTR, 4, tmp);
// }

// /**
//  *  @brief      Get duration of walking time.
//  *  @param[in]  time    Walk time in milliseconds.
//  *  @return     0 if successful.
//  */
// static int dmp_get_pedometer_walk_time(unsigned long *time){
// 	unsigned char tmp[4];
// 	if (!time) return -1;
// 	if (mpu_read_mem(D_PEDSTD_TIMECTR, 4, tmp)) return -1;
// 	time[0] = (((unsigned long)tmp[0] << 24) | ((unsigned long)tmp[1] << 16) |
// 			((unsigned long)tmp[2] << 8) | tmp[3]) * 20;
// 	return 0;
// }

// /**
//  *  @brief      Overwrite current walk time.
//  *  WARNING: This function writes to DMP memory and could potentially encounter
//  *  a race condition if called while the pedometer is enabled.
//  *  @param[in]  time    New walk time in milliseconds.
//  */
// static int dmp_set_pedometer_walk_time(unsigned long time){
// 	unsigned char tmp[4];
// 	time /= 20;
// 	tmp[0] = (unsigned char)((time >> 24) & 0xFF);
// 	tmp[1] = (unsigned char)((time >> 16) & 0xFF);
// 	tmp[2] = (unsigned char)((time >> 8) & 0xFF);
// 	tmp[3] = (unsigned char)(time & 0xFF);
// 	return mpu_write_mem(D_PEDSTD_TIMECTR, 4, tmp);
// }

/**
 *  @brief       Generate 6-axis quaternions from the DMP.
 *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *  exclusive.
 *  @param[in]   enable  1 to enable 6-axis quaternion.
 *  @return      0 if successful.
 */
static int dmp_enable_6x_lp_quat(unsigned char enable)
{
	unsigned char regs[4];
	if (enable) {
		regs[0] = DINA20;
		regs[1] = DINA28;
		regs[2] = DINA30;
		regs[3] = DINA38;
	}
	else
		memset(regs, 0xA3, 4);
	mpu_write_mem(CFG_8, 4, regs);
	return mpu_reset_fifo();
}
/**
 *  @brief      Calibrate the gyro data in the DMP.
 *  After eight seconds of no motion, the DMP will compute gyro biases and
 *  subtract them from the quaternion output. If @e dmp_enable_feature is
 *  called with @e DMP_FEATURE_SEND_CAL_GYRO, the biases will also be
 *  subtracted from the gyro output.
 *  @param[in]  enable  1 to enable gyro calibration.
 *  @return     0 if successful.
 */
static int dmp_enable_gyro_cal(unsigned char enable){
	if (enable) {
		unsigned char regs[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
		return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
	}
	else {
		unsigned char regs[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};
		return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
	}
}
/**
 *  @brief      Generate 3-axis quaternions from the DMP.
 *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *  exclusive.
 *  @param[in]  enable  1 to enable 3-axis quaternion.
 *  @return     0 if successful.
 */
static int dmp_enable_lp_quat(unsigned char enable){
	unsigned char regs[4];
	if (enable) {
		regs[0] = DINBC0;
		regs[1] = DINBC2;
		regs[2] = DINBC4;
		regs[3] = DINBC6;
	}
	else
		memset(regs, 0x8B, 4);
	mpu_write_mem(CFG_LP_QUAT, 4, regs);
	return mpu_reset_fifo();
}
/**
 *  @brief      Enable DMP features.
 *  The following \#define's are used in the input mask:
 *  \n DMP_FEATURE_TAP
 *  \n DMP_FEATURE_ANDROID_ORIENT
 *  \n DMP_FEATURE_LP_QUAT
 *  \n DMP_FEATURE_6X_LP_QUAT
 *  \n DMP_FEATURE_GYRO_CAL
 *  \n DMP_FEATURE_SEND_RAW_ACCEL
 *  \n DMP_FEATURE_SEND_RAW_GYRO
 *  \n NOTE: DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually
 *  exclusive.
 *  \n NOTE: DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also
 *  mutually exclusive.
 *  @param[in]  mask    Mask of features to enable.
 *  @return     0 if successful.
 */
static int dmp_enable_feature(unsigned short mask){
	unsigned char tmp[10];

	/* TODO: All of these settings can probably be integrated into the default
		* DMP image.
		*/
	/* Set integration scale factor. */
	tmp[0] = (unsigned char)((GYRO_SF >> 24) & 0xFF);
	tmp[1] = (unsigned char)((GYRO_SF >> 16) & 0xFF);
	tmp[2] = (unsigned char)((GYRO_SF >> 8) & 0xFF);
	tmp[3] = (unsigned char)(GYRO_SF & 0xFF);
	mpu_write_mem(D_0_104, 4, tmp);
	/* Send sensor data to the FIFO. */
	tmp[0] = 0xA3;
	if (mask & DMP_FEATURE_SEND_RAW_ACCEL) {
		tmp[1] = 0xC0;
		tmp[2] = 0xC8;
		tmp[3] = 0xC2;
	} else {
		tmp[1] = 0xA3;
		tmp[2] = 0xA3;
		tmp[3] = 0xA3;
	}
	if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
		tmp[4] = 0xC4;
		tmp[5] = 0xCC;
		tmp[6] = 0xC6;
	} else {
		tmp[4] = 0xA3;
		tmp[5] = 0xA3;
		tmp[6] = 0xA3;
	}
	tmp[7] = 0xA3;
	tmp[8] = 0xA3;
	tmp[9] = 0xA3;
	mpu_write_mem(CFG_15,10,tmp);

	/* Send gesture data to the FIFO. */
	if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
		tmp[0] = DINA20;
	else
		tmp[0] = 0xD8;
	mpu_write_mem(CFG_27,1,tmp);

	if (mask & DMP_FEATURE_GYRO_CAL)
		dmp_enable_gyro_cal(1);
	else
		dmp_enable_gyro_cal(0);

	if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
		if (mask & DMP_FEATURE_SEND_CAL_GYRO) {
			tmp[0] = 0xB2;
			tmp[1] = 0x8B;
			tmp[2] = 0xB6;
			tmp[3] = 0x9B;
		} else {
			tmp[0] = DINAC0;
			tmp[1] = DINA80;
			tmp[2] = DINAC2;
			tmp[3] = DINA90;
		}
		mpu_write_mem(CFG_GYRO_RAW_DATA, 4, tmp);
	}
	if (mask & DMP_FEATURE_TAP) {
		/* Enable tap. */
		tmp[0] = 0xF8;
		mpu_write_mem(CFG_20, 1, tmp);
		dmp_set_tap_thresh(TAP_XYZ, 250);
		dmp_set_tap_axes(TAP_XYZ);
		dmp_set_tap_count(1);
		dmp_set_tap_time(100);
		dmp_set_tap_time_multi(500);

		dmp_set_shake_reject_thresh(GYRO_SF, 200);
		dmp_set_shake_reject_time(40);
		dmp_set_shake_reject_timeout(10);
	} else {
		tmp[0] = 0xD8;
		mpu_write_mem(CFG_20, 1, tmp);
}

	if (mask & DMP_FEATURE_ANDROID_ORIENT) {
		tmp[0] = 0xD9;
	} else
		tmp[0] = 0xD8;
	mpu_write_mem(CFG_ANDROID_ORIENT_INT, 1, tmp);

	if (mask & DMP_FEATURE_LP_QUAT)
		dmp_enable_lp_quat(1);
	else
		dmp_enable_lp_quat(0);

	if (mask & DMP_FEATURE_6X_LP_QUAT)
		dmp_enable_6x_lp_quat(1);
	else
		dmp_enable_6x_lp_quat(0);

	/* Pedometer is always enabled. */
	dmp.feature_mask = mask | DMP_FEATURE_PEDOMETER;
	mpu_reset_fifo();

	dmp.packet_length = 0;
	if (mask & DMP_FEATURE_SEND_RAW_ACCEL)
		dmp.packet_length += 6;
	if (mask & DMP_FEATURE_SEND_ANY_GYRO)
		dmp.packet_length += 6;
	if (mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT))
		dmp.packet_length += 16;
	if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
		dmp.packet_length += 4;

	return 0;
}


/**
 *  @brief      Decode the four-byte gesture data and execute any callbacks.
 *  @param[in]  gesture Gesture data from DMP packet.
 *  @return     0 if successful.
 */
static int decode_gesture(unsigned char *gesture){
	unsigned char tap, android_orient;
	android_orient = gesture[3] & 0xC0;
	tap = 0x3F & gesture[3];
	if (gesture[1] & INT_SRC_TAP) {
		unsigned char direction, count;
		direction = tap >> 3;
		count = (tap % 8) + 1;
		if (dmp.tap_cb)
			dmp.tap_cb(direction, count);
	}
	if (gesture[1] & INT_SRC_ANDROID_ORIENT) {
		if (dmp.android_orient_cb)
			dmp.android_orient_cb(android_orient >> 6);
	}
	return 0;
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_WXYZ_QUAT
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] quat        3-axis quaternion data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
static int dmp_read_fifo(short *gyro, short *accel, long *quat,
    unsigned long *timestamp, short *sensors, unsigned char *more){
	unsigned char fifo_data[DMP_MAX_PACKET_LENGTH];//?
	unsigned char ii = 0;

	/* TODO: sensors[0] only changes when dmp_enable_feature is called. We can
		* cache this value and save some cycles.
		*/
	sensors[0] = 0;

	/* Get a packet. */
	if (mpu_read_fifo_stream(dmp.packet_length, fifo_data, more)){
		return -1;
	}
	/* Parse DMP packet. */
	if (dmp.feature_mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT)) {
		long quat_q14[4], quat_mag_sq;
		quat[0] = ((long)fifo_data[0] << 24) | ((long)fifo_data[1] << 16) |
			((long)fifo_data[2] << 8) | fifo_data[3];
		quat[1] = ((long)fifo_data[4] << 24) | ((long)fifo_data[5] << 16) |
			((long)fifo_data[6] << 8) | fifo_data[7];
		quat[2] = ((long)fifo_data[8] << 24) | ((long)fifo_data[9] << 16) |
			((long)fifo_data[10] << 8) | fifo_data[11];
		quat[3] = ((long)fifo_data[12] << 24) | ((long)fifo_data[13] << 16) |
			((long)fifo_data[14] << 8) | fifo_data[15];
		ii += 16;
		quat_q14[0] = quat[0] >> 16;
		quat_q14[1] = quat[1] >> 16;
		quat_q14[2] = quat[2] >> 16;
		quat_q14[3] = quat[3] >> 16;
		quat_mag_sq = quat_q14[0] * quat_q14[0] + quat_q14[1] * quat_q14[1] +
			quat_q14[2] * quat_q14[2] + quat_q14[3] * quat_q14[3];
		if ((quat_mag_sq < QUAT_MAG_SQ_MIN) ||
			(quat_mag_sq > QUAT_MAG_SQ_MAX)) {
			/* Quaternion is outside of the acceptable threshold. */
			mpu_reset_fifo();
			sensors[0] = 0;
			return -1;
		}
		sensors[0] |= INV_WXYZ_QUAT;
	}
	if (dmp.feature_mask & DMP_FEATURE_SEND_RAW_ACCEL) {
		accel[0] = ((short)fifo_data[ii+0] << 8) | fifo_data[ii+1];
		accel[1] = ((short)fifo_data[ii+2] << 8) | fifo_data[ii+3];
		accel[2] = ((short)fifo_data[ii+4] << 8) | fifo_data[ii+5];
		ii += 6;
		sensors[0] |= INV_XYZ_ACCEL;
	}

	if (dmp.feature_mask & DMP_FEATURE_SEND_ANY_GYRO) {
		gyro[0] = ((short)fifo_data[ii+0] << 8) | fifo_data[ii+1];
		gyro[1] = ((short)fifo_data[ii+2] << 8) | fifo_data[ii+3];
		gyro[2] = ((short)fifo_data[ii+4] << 8) | fifo_data[ii+5];
		ii += 6;
		sensors[0] |= INV_XYZ_GYRO;
	}

	/* Gesture data is at the end of the DMP packet. Parse it and call
		* the gesture callbacks (if registered).
		*/
	if (dmp.feature_mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
		decode_gesture(fifo_data + ii);

	get_ms(timestamp);
	return 0;
}

// /**
//  *  @brief      Register a function to be executed on a tap event.
//  *  The tap direction is represented by one of the following:
//  *  \n TAP_X_UP
//  *  \n TAP_X_DOWN
//  *  \n TAP_Y_UP
//  *  \n TAP_Y_DOWN
//  *  \n TAP_Z_UP
//  *  \n TAP_Z_DOWN
//  *  @param[in]  func    Callback function.
//  *  @return     0 if successful.
//  */
// static int dmp_register_tap_cb(void (*func)(unsigned char, unsigned char))
// {
//     dmp.tap_cb = func;
//     return 0;
// }

// /**
//  *  @brief      Register a function to be executed on a android orientation event.
//  *  @param[in]  func    Callback function.
//  *  @return     0 if successful.
//  */
// static int dmp_register_android_orient_cb(void (*func)(unsigned char))
// {
//     dmp.android_orient_cb = func;
//     return 0;
// }

//q30格式,long转float时的除数.
#define q30  1073741824.0f
//陀螺仪方向设置
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};
/*MPU6050自检
//return:0,ok
//    !0,error*/
unsigned char run_self_test(void){
	int result;
	//char test_packet[4] = {0};
	long gyro[3], accel[3];
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x3){
		/*测试通过,将数据写入DMP*/
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		return 0;
	}
	else {
		return 1;
	}
}
//陀螺仪方向控制
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx){
	unsigned short scalar;
	/*
			XYZ  010_001_000 Identity Matrix
			XZY  001_010_000
			YXZ  010_000_001
			YZX  000_010_001
			ZXY  001_000_010
			ZYX  000_001_010
	*/
	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;
	return scalar;
}
//方向转换
unsigned short inv_row_2_scale(const signed char *row){
	unsigned short b;
	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;		// error
	return b;
}
/*dmp初始化
//return:0,ok
//    !0,error*/
unsigned char mpu_dmp_init(void){
	unsigned char res=0;
	MPUGPIO_INit();
	if(mpu_init()==0){		//初始化MPU6050
		printf("mpu_init is ok\r\n");
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
		printf("%d",res);
		if(res)return 1;
		res=mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置FIFO
		if(res)return 2;
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	//设置采样率
		if(res)return 3;
		res=dmp_load_motion_driver_firmware();		//加载dmp固件
		if(res)return 4;
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
		if(res)return 5;
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//设置dmp功能
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 6;
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);		//设置DMP输出速率(最大不超过200Hz)
		if(res)return 7;
		res=run_self_test();		//自检
		if(res)return 8;
		res=mpu_set_dmp_state(1);	//使能DMP
		if(res)return 9;
	}else return 10;
	return 0;
}
/*dmp处理后数据
pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
return:0,ok
   !0,error*/
unsigned char mpu_dmp_get_data(float *pitch,float *roll,float *yaw){
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4];
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more)){
		return 1;
	}
	/*将陀螺仪加速度值写到dmp*/
	if(sensors&INV_WXYZ_QUAT){
		q0 = quat[0] / q30;		//q30格式转换为浮点数
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;
		//计算得到俯仰角/横滚角/航向角
		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	}else return 2;
	return 0;
}


/*END*/

#endif
