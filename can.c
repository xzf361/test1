

#include "can.h"
#include "tcp_send.h"
#include "save_data.h"
#include "gpio.h"

/******************************* 宏定义 *******************************/
// #define PRINT_TIMEOUT
// #define PRINT_INFO
// #define RUN_TIME

/****************************** 函数声明 ******************************/
int plc_run(void);                      // PLC运行应用程序
void downtime(int line, char *p_str);   // 宕机
void save_log(uint8 flag, int line, char *p_str);

/****************************** 全局变量 ******************************/
uint8               g_system_date = 0;                      // 日期切换工作组  0 信任A系, 1 信任B系

T_system_forcibly   g_system_forcibly = 0;                  // 切换强制位

static T_CAN_line_status    g_can_line_status[2] = {CAN_LINE_TRUST, CAN_LINE_OK};   // CAN线路状态
static uint16   g_mcu_heart_other[2] = {0xFFFF, 0xFFFF};    // 另一板卡MCU心跳(两条CAN线路)
T_mcu_status    g_mcu_status = MCU_BACKUP;                  // MCU运行状态
static uint8    g_mcu_status_other = 0;                     // 另一MCU运行状态    0 故障， 1运行
static uint8    g_mcu_status_other_a = 0;//add by 11.11 1俩MCU通信正常
T_extid         g_mcu_can_id[2] = {{0}};                    // MCU CAN ID   0 本板卡, 1 另一板卡
static uint32   g_mcu_offset_err[2] = {0};                  // MCU序号错误标志(两条CAN线路)

// uint8                   g_plc_data[2 * DIO_CNT] =
// {
// 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
// 0x1F, 0x01, 0x63, 0x00, 0x7F, 0x01, 0x1C, 0x00, 0x04, 0x00,
// 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
// 0x1F, 0x01, 0x63, 0x00, 0x7F, 0x01, 0x1C, 0x00, 0x04, 0x00
// }; // PLC数据 test
uint8           g_plc_data[2 * DIO_CNT] = {0};              // PLC数据
static uint8    g_plc_data_last[2 * DIO_CNT] = {0xFF};      // PLC数据 备份

uint8           g_plc_yx_data[2] = {0};              // PLC硬线数据
static uint32   g_counter_send = 0;
static uint32   g_counter_send_cycle = 0;
int           g_hard_id = 0;

extern uint8    g_run_flag_can_periodic;                    // 运行监测
extern uint8    g_run_flag_check_time_out;                  // 运行监测

uint8           g_mcu_status_stop = 0;     //add by 11.25  判断对方MCU是否正常工作
uint8           g_mcu_stay_status = 0;     //add by 11.25  当掉电系重新上电后不会再进入主备分支
uint8           g_mcu_id_save = 0;         //add by 11.25  保存当前分配的id

uint8           g_fake_cnt = 0;            //初始化板卡丢失的个数
uint8           g_fake_cnt_y = 0;          //运行中板卡丢失的个数
uint8           g_fake_cnt_all = 0;          //保存记录运行过程中整系断过电

int             g_reset_cnt = 0;           //B系断电重启后接收不到A系时间后继续运行

// int  jishuqi = 0;//test

/*******************************∨∨∨ DIO ∨∨∨*******************************/
static uint32       g_dio_offset_err[2][DIO_CNT] = {{0}};   // DIO序号错误标志(两条CAN线路)
static uint32       g_dio_err_cnt[2][DIO_CNT] = {{0}};      // DIO板卡丢帧次数(两条CAN线路)
//static T_dio_status g_dio_status[DIO_CNT] =
T_dio_status g_dio_status[DIO_CNT] =
{
    DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT,
    DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT,
    DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT,
    DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT, DIO_DEFAULT
};  // DIO板卡状态

static T_dio_ctrl   g_dio_ctrl[DIO_CNT] =
{
    DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT,
    DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT,
    DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT,
    DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT, DIO_CTRL_DEFAULT
};  // DIO板卡主备状态    0 备, 1 主
/*******************************∧∧∧ DIO ∧∧∧*******************************/


/*******************************∨∨∨ MVB ∨∨∨*******************************/
static T_mvb_status     g_mvb_status = MVB_DEFAULT;     // MVB板卡状态
static T_mvb_connect    g_mvb_connect = MVB_DISCONNECT; // MVB板卡连接状态
static uint32           g_mvb_err_cnt[2] = {0};         // MVB板卡错误帧次数(两条CAN线路)
static T_mvb_run        g_mvb_run = MVB_STOP;           // MVB板卡运行状态    0 停止, 1 运行
/*******************************∧∧∧ MVB ∧∧∧*******************************/


/*******************************∨∨∨ CAN ∨∨∨*******************************/
static T_can_status     g_can_status = CAN_DEFAULT;     // CAN板卡状态
static uint32           g_can_err_cnt[2] = {0};         // CAN板卡错误帧次数(两条CAN线路)
static T_can_run        g_can_run = CAN_STOP;           // CAN板卡运行状态    0 停止, 1 运行
/*******************************∧∧∧ CAN ∧∧∧*******************************/

/****************************** 函数定义 ******************************/
/*************************************************
 * @函数名称: check_can_line_status
 * @函数功能: CAN线路状态切换
 * @输入参数: 无
 * @输出参数: 无
 * @返回值  : 无
 * @其它说明: 无
 *************************************************/
void check_can_line_status(void)
{
    int result = 0;
    int card_num = 0;

    // CAN0故障
    if (CAN_LINE_ERR == g_can_line_status[0])
    {
        // 清空标志位
        for (card_num = 0; card_num < DIO_CNT; card_num++)
        {
            // 设置DIO板卡错误计数
            g_dio_err_cnt[0][card_num] = DIO_MAX_ERR_CNT;
        }
        // 设置MVB板卡错误计数
        g_mvb_err_cnt[0] = MVB_MAX_ERR_CNT;
        // 设置CAN板卡错误计数
        g_can_err_cnt[0] = CAN_MAX_ERR_CNT;
    }

    // CAN1故障
    if (CAN_LINE_ERR == g_can_line_status[1])
    {
        // 清空标志位
        for (card_num = 0; card_num < DIO_CNT; card_num++)
        {
            // 设置DIO板卡错误计数
            g_dio_err_cnt[1][card_num] = DIO_MAX_ERR_CNT;
        }
        // 设置MVB板卡错误计数
        g_mvb_err_cnt[1] = MVB_MAX_ERR_CNT;
        // 设置CAN板卡错误计数
        g_can_err_cnt[1] = CAN_MAX_ERR_CNT;
    }


    // CAN0非故障 则信任CAN0
    if (CAN_LINE_ERR != g_can_line_status[0])
    {
        g_can_line_status[0] = CAN_LINE_TRUST;
    }
    // CAN0故障
    else
    {
        // CAN0故障 CAN1非故障 则信任CAN1
        if (CAN_LINE_ERR != g_can_line_status[1])
        {
            g_can_line_status[1] = CAN_LINE_TRUST;
        }
        // CAN1故障 & CAN0故障
        else
        {
            // 清空标志位
            for (card_num = 0; card_num < DIO_CNT; card_num++)
            {
                // 设置DIO板卡错误计数
                g_dio_err_cnt[0][card_num] = 0;
                g_dio_err_cnt[1][card_num] = 0;
            }

            // 设置MVB板卡错误计数
            g_mvb_err_cnt[0] = 0;
            g_mvb_err_cnt[1] = 0;

            // 设置CAN板卡错误计数
            g_can_err_cnt[0] = 0;
            g_can_err_cnt[1] = 0;

            // 设置MCU运行状态
            g_mcu_status = MCU_BACKUP;
            DEBUG_INFO();

            {
                // 可选择恢复总线
                // 复位总线
                g_can_line_status[0] = CAN_LINE_OK;
                g_can_line_status[1] = CAN_LINE_OK;
            }
#if 0
            // 宕机
            downtime(__LINE__, "CAN_LINE all");
#endif
        }
    }
}

/*************************************************
 * @函数名称: clear_all_dio_send_flag
 * @函数功能: 清除dio发帧标志
 * @输入参数: 无
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0  成功
                  -1  失败
 * @其它说明: 无
 *************************************************/
int clear_all_dio_send_flag(T_can_buff *p_can_buff)
{
    uint16 offset = 0;          // 发帧序号
    uint8 card_num = 0;         // 板卡号

    if (NULL == p_can_buff)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    for (card_num = 0; card_num < DIO_CNT; card_num++)
    {
        for (offset = 0; offset < CAN_FRAME_CNT; offset++)
        {
            // 发送标志位置0
            p_can_buff->DIO_status[card_num].data[offset].frame_send_flag = 0;
            p_can_buff->DIO_DI[card_num].data[offset].frame_send_flag = 0;
            p_can_buff->DIO_DO[card_num].data[offset].frame_send_flag = 0;

            // 接收标志位置0
            p_can_buff->DIO_status[card_num].data[offset].frame_recv_flag = 0;
            p_can_buff->DIO_DI[card_num].data[offset].frame_recv_flag = 0;
            p_can_buff->DIO_DO[card_num].data[offset].frame_recv_flag = 0;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: clear_err_dio_send_flag
 * @函数功能: 清除dio发帧标志
 * @输入参数: 无
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0  成功
                  -1  失败
 * @其它说明: 无
 *************************************************/
int clear_err_dio_send_flag(T_can_buff *p_can_buff)
{
    uint16 offset = 0;          // 发帧序号
    uint8 card_num = 0;         // 板卡号

    if (NULL == p_can_buff)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    // 发送标志位置0
    for (card_num = 0; card_num < DIO_CNT; card_num++)
    {
        // DIO板卡状态
        if (DIO_FAKE_ERR == g_dio_status[card_num])
        {
            for (offset = 0; offset < CAN_FRAME_CNT; offset++)
            {
                // 发送标志位置0
                p_can_buff->DIO_status[card_num].data[offset].frame_send_flag = 0;
                p_can_buff->DIO_DI[card_num].data[offset].frame_send_flag = 0;
                p_can_buff->DIO_DO[card_num].data[offset].frame_send_flag = 0;

                // 接收标志位置0
                p_can_buff->DIO_status[card_num].data[offset].frame_recv_flag = 0;
                p_can_buff->DIO_DI[card_num].data[offset].frame_recv_flag = 0;
                p_can_buff->DIO_DO[card_num].data[offset].frame_recv_flag = 0;
            }
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_all_stop
 * @函数功能: 停止所有设备
 * @输入参数: line_num  调用此函数所在的的行
 * @输出参数: 无
 * @返回值  : 无
 * @其它说明: 无
 *************************************************/
void can_send_all_stop(int line_num)
{
    int     result = 0;
    int     can_line = 0;
    uint8   timeout_cnt = 0;    // 超时次数
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    extern int g_fd_can[2];                     // CAN文件描述符


    /* CAN ID */
    memset(send_frame.data, 0, 8);
    send_frame.can_id = 0x0400;
    send_frame.can_dlc = 4;
    send_frame.data[0] = 0x5A;
    send_frame.data[1] = g_mcu_can_id[0].hard_addr;
    send_frame.data[2] = (line_num >> 8) & 0xFF;
    send_frame.data[3] = line_num & 0xFF;

    for (can_line = 0, timeout_cnt = 0; can_line < 2; can_line++)
    {
        // 当前CAN线路故障
        if (CAN_LINE_ERR == g_can_line_status[can_line])
        {
            continue;
        }

        /* CAN发送 */
        while (sizeof(struct can_frame) != write(g_fd_can[can_line], &send_frame, sizeof(struct can_frame)))
        {
            timeout_cnt++;

            if (timeout_cnt >= 60)  // 6ms
            {
                // 当前CAN线路故障
                break;
            }
            usleep(100);
        }
    }
}

/*************************************************
 * @函数名称: check_plc_data_valid
 * @函数功能: PLC数据失效检测
 * @输入参数: 无
 * @输出参数: 无
 * @返回值  : 无
 * @其它说明: 无
 *************************************************/
void check_plc_data_valid(void)
{
    int result = 0;
    int card_num = 0;
    int plc_data_err_flag = 0;  // PLC数据失效标志位

    for (card_num = 0, plc_data_err_flag = 0; card_num < 10; card_num += 2)
    {
        // DIO板卡主备状态
        if ((DIO_FAKE_ERR == g_dio_status[card_num]) && (DIO_FAKE_ERR == g_dio_status[card_num + 1]))
        {
            // PLC数据无效
            plc_data_err_flag = 1;
            g_plc_data[2 * (card_num / 2)] = 0;
            g_plc_data[2 * (card_num / 2) + 1] = 0;

            // qjq test
            // print_line("g_dio_status[%2d] = %d g_dio_status[%2d] = %d\r\n", card_num, g_dio_status[card_num], card_num + 1, g_dio_status[card_num + 1]);
        }
    }

    // qjq test
    //if (0)
    if (1 == plc_data_err_flag)
    {
        // 停止所有设备
        can_send_all_stop(__LINE__);

        // 宕机
        downtime(__LINE__, "plc_data_err");
    }
}

/*************************************************
 * @函数名称: check_dio_status
 * @函数功能: DIO板卡状态切换
 * @输入参数: 无
 * @输出参数: 无
 * @返回值  : 无
 * @其它说明: 无
 *************************************************/
void check_dio_status(void)
{
    int result = 0;
    int card_num = 0;	
	
    
	// MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
		return;
    }
		

    // 自动
    if (FORCIBLY_AUTO == g_system_forcibly)
    {
		// for (card_num = 0; card_num < DIO_CNT; card_num++)
		// {
		 // printf("%3d ", g_dio_status[card_num]);
		// }
        for (card_num = 0; card_num < DIO_CNT; card_num += 2)
        {
			//OK & OK
			if ((DIO_OK == g_dio_status[card_num]) && (DIO_OK == g_dio_status[card_num + 1]))
            {				
				if (1 == g_mcu_status_other)
				{
					if (0 == g_mcu_status_stop)
					{
						//此时B系MCU为主，将DIO主切换到B系
						if ((g_mcu_can_id[0].all == 0x04) && (g_mcu_can_id[1].all == 0x03))
						{
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
						//此时A系MCU为主，将DIO主切换到A系
						else if ((g_mcu_can_id[0].all == 0x03) && (g_mcu_can_id[1].all == 0x04))
						{
							g_dio_status[card_num] = DIO_TRUST;
						}
					}
					else if (10 == g_mcu_status_stop)
					{
						if (0x03 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						else if (0x04 == g_mcu_id_save)	
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
				}
				else if (1 == g_mcu_status_other_a)	
				{
					if (0 == g_mcu_status_other)
					{
						//此时B系MCU为主，仍保持DIO为A系
						//if (0 == g_system_date)
						if ((g_mcu_can_id[0].all == 0x04) && (g_mcu_can_id[1].all == 0x03))							
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						//此时A系MCU为主，仍保持DIO为B系
						//else if (1 == g_system_date)
						else if ((g_mcu_can_id[0].all == 0x03) && (g_mcu_can_id[1].all == 0x04))	
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
				}
				else if (0 == g_mcu_status_other_a)	
				{
					if (0 == g_mcu_status_other)
					{
						//此时B系MCU为主，仍保持DIO为A系
						if (0 == g_system_date)
						//if ((g_mcu_can_id[0].all == 0x04) && (g_mcu_can_id[1].all == 0x03))							
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						//此时A系MCU为主，仍保持DIO为B系
						else if (1 == g_system_date)
						//else if ((g_mcu_can_id[0].all == 0x03) && (g_mcu_can_id[1].all == 0x04))	
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
				}
            }
			//OK & TRUST
			else if ((DIO_OK == g_dio_status[card_num]) && (DIO_TRUST == g_dio_status[card_num + 1]))
			{
				
				if (0 == g_mcu_status_other)
				{
					if ((5 > (g_fake_cnt + g_fake_cnt_y - 10)))
					{	
						//此时B系MCU为主，仍保持DIO为A系
						if (0 == g_system_date)	
						//if ((g_mcu_can_id[0].all == 0x04) && (g_mcu_can_id[1].all == 0x03))
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						//此时A系MCU为主，仍保持DIO为B系
						else if (1 == g_system_date)
						//else if ((g_mcu_can_id[0].all == 0x03) && (g_mcu_can_id[1].all == 0x04))	
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
					else if (5 <= (g_fake_cnt + g_fake_cnt_y - 10))
					{
						if (0x03 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						else if (0x04 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
						//g_fake_cnt_all = g_fake_cnt + g_fake_cnt_y - 10;
						
					}
				}
				else if (1 == g_mcu_status_other)
				{
					if (10 == g_mcu_status_stop)
					{
						if (0x03 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						else if (0x04 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
				}
							
			}
			//TRUST & OK
			else if ((DIO_TRUST == g_dio_status[card_num]) && (DIO_OK == g_dio_status[card_num + 1]))
			{
				if (0 == g_mcu_status_other)
				{
					// debug_line("g_fake_cnt = %2d\r\n", g_fake_cnt);
					//debug_line("g_fake_cnt_all = %2d\r\n", (g_fake_cnt + g_fake_cnt_y));
					if ((5 > (g_fake_cnt + g_fake_cnt_y - 10)))
					{                                                                    
						//此时B系MCU为主，仍保持DIO为A系
						if (0 == g_system_date)
						//if ((g_mcu_can_id[0].all == 0x04) && (g_mcu_can_id[1].all == 0x03))	
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						//此时A系MCU为主，仍保持DIO为B系
						else if (1 == g_system_date)
						//else if ((g_mcu_can_id[0].all == 0x03) && (g_mcu_can_id[1].all == 0x04))
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
					else if (5 <= (g_fake_cnt + g_fake_cnt_y - 10))
					{
						//DEBUG_INFO();
						if (0x03 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						else if (0x04 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
						//g_fake_cnt_all = g_fake_cnt + g_fake_cnt_y - 10;
						//printf("g_fake_cnt_all = %d\n ", g_fake_cnt_all);
					}
				}
				else if (1 == g_mcu_status_other)
				{
					if (10 == g_mcu_status_stop)
					{
						if (0x03 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						else if (0x04 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
				}
			}
			//OK & ERR
            else if ((DIO_OK == g_dio_status[card_num]) && (DIO_FAKE_ERR == g_dio_status[card_num + 1]))
            {
				// 信任A板卡
                g_dio_status[card_num] = DIO_TRUST;
            }
			//ERR & OK
            else if ((DIO_FAKE_ERR == g_dio_status[card_num]) && (DIO_OK == g_dio_status[card_num + 1]))
            {
				// 信任B板卡
                g_dio_status[card_num + 1] = DIO_TRUST;
            }
            // 不会出现此种情况，以防万一
            else if ((DIO_TRUST == g_dio_status[card_num]) && (DIO_TRUST == g_dio_status[card_num + 1]))
            {
				// 信任A板卡
                g_dio_status[card_num] = DIO_TRUST;
                g_dio_status[card_num + 1] = DIO_OK;
            }
        }
    }
    // 强制A路
    else if (FORCIBLY_A == g_system_forcibly)
    {
        for (card_num = 0; card_num < DIO_CNT; card_num += 2)
        {
            // OK & OK
            if ((DIO_OK == g_dio_status[card_num]) && (DIO_OK == g_dio_status[card_num + 1]))
            {
                // 信任A板卡
                g_dio_status[card_num] = DIO_TRUST;
                g_dio_status[card_num + 1] = DIO_OK;
            }
            // OK & ERR
            else if ((DIO_OK == g_dio_status[card_num]) && (DIO_FAKE_ERR == g_dio_status[card_num + 1]))
            {
                // 信任A板卡
                g_dio_status[card_num] = DIO_TRUST;
            }
            // OK & TRUST
            else if ((DIO_OK == g_dio_status[card_num]) && (DIO_TRUST == g_dio_status[card_num + 1]))
            {
                // 原信任为B板卡，现切换到A板卡为信任，B板卡为ok
                // g_dio_status[card_num] = DIO_TRUST;
                // g_dio_status[card_num + 1] = DIO_OK;
				
				if (1 == g_mcu_status_other)
				{                                                                    
					//正常运行
					if (0 == g_mcu_status_stop)
					{
						g_dio_status[card_num] = DIO_TRUST;
						g_dio_status[card_num + 1] = DIO_OK;
					}
					//整系重新上电
					else if (10 == g_mcu_status_stop)
					{
						g_dio_status[card_num] = DIO_OK;
						g_dio_status[card_num + 1] = DIO_TRUST;
					}
						
				}
				else if (0 == g_mcu_status_other)
				{
					if (5 >= (g_fake_cnt + g_fake_cnt_y))
					{
						g_dio_status[card_num] = DIO_TRUST;
						g_dio_status[card_num + 1] = DIO_OK;
					}
					else if (5 < (g_fake_cnt + g_fake_cnt_y))
					{
						if (0x03 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						else if (0x04 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
				}
            }
            // ERR & OK
            else if ((DIO_FAKE_ERR == g_dio_status[card_num]) && (DIO_OK == g_dio_status[card_num + 1]))
            {
                // 信任B板卡:A系的该板卡故障，由B系的互为一组的板卡替换
                g_dio_status[card_num + 1] = DIO_TRUST;
            }
            // ERR & ERR
            else if ((DIO_TRUE_ERR == g_dio_status[card_num]) && (DIO_TRUE_ERR == g_dio_status[card_num + 1]))
            {
                // 宕机？
                // 停止所有设备
                can_send_all_stop(__LINE__);

                // 宕机
                downtime(__LINE__, "num");
            }
            // ERR & TRUST
            else if ((DIO_FAKE_ERR == g_dio_status[card_num]) && (DIO_TRUST == g_dio_status[card_num + 1]))
            {
                // 信任B板卡:A系的该板卡故障，由B系的互为一组的板卡替换
                g_dio_status[card_num + 1] = DIO_TRUST;
            }
            // ERR & DEF
            else if ((DIO_FAKE_ERR == g_dio_status[card_num]) && (DIO_DEFAULT == g_dio_status[card_num + 1]))
            {
                // 信任B板卡:A系的该板卡故障，由B系的互为一组的板卡替换
                g_dio_status[card_num + 1] = DIO_TRUST;
            }
            // TRUST & DEF
            else if ((DIO_TRUST == g_dio_status[card_num]) && (DIO_DEFAULT == g_dio_status[card_num + 1]))
            {
                // 信任B板卡:A系的该板卡故障，由B系的互为一组的板卡替换
                g_dio_status[card_num] = DIO_TRUST;
                g_dio_status[card_num + 1] = DIO_OK;
            }
            // TRUST & OK
            else if ((DIO_TRUST == g_dio_status[card_num]) && (DIO_OK == g_dio_status[card_num + 1]))
            {
                // g_dio_status[card_num] = DIO_TRUST;
                // g_dio_status[card_num + 1] = DIO_OK;
				
				if (1 == g_mcu_status_other)
				{                                                                    
					//正常运行
					if (0 == g_mcu_status_stop)
					{
						g_dio_status[card_num] = DIO_TRUST;
						g_dio_status[card_num + 1] = DIO_OK;
					}
					//断电后重新上电
					else if (10 == g_mcu_status_stop)
					{
						g_dio_status[card_num] = DIO_TRUST;
						g_dio_status[card_num + 1] = DIO_OK;
					}
						
				}
				else if (0 == g_mcu_status_other)
				{
					//拔MCU板卡
					if (5 >= (g_fake_cnt + g_fake_cnt_y))
					{
						g_dio_status[card_num] = DIO_TRUST;
						g_dio_status[card_num + 1] = DIO_OK;
					}
					//整系断电
					else if (5 < (g_fake_cnt + g_fake_cnt_y))
					{
						if (0x03 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						else if (0x04 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
				}
            }
            // TRUST & ERR
            else if ((DIO_TRUST == g_dio_status[card_num]) && (DIO_FAKE_ERR == g_dio_status[card_num + 1]))
            {
                // 信任A板卡
                g_dio_status[card_num] = DIO_TRUST;

            }
            // TRUST & TRUST
            else if ((DIO_TRUST == g_dio_status[card_num]) && (DIO_TRUST == g_dio_status[card_num + 1]))
            {
                // 信任A板卡
                g_dio_status[card_num] = DIO_TRUST;
                g_dio_status[card_num + 1] = DIO_OK;
            }
        }
    }
    // 强制B路
    else if (FORCIBLY_B == g_system_forcibly)
    {
        for (card_num = 0; card_num < DIO_CNT; card_num += 2)
        {
            // OK & OK
            if ((DIO_OK == g_dio_status[card_num]) && (DIO_OK == g_dio_status[card_num + 1]))
            {
				// 信任B板卡
                g_dio_status[card_num + 1] = DIO_TRUST;
            }
            // OK & ERR
            else if ((DIO_OK == g_dio_status[card_num]) && (DIO_FAKE_ERR == g_dio_status[card_num + 1]))
            {
                // 信任A板卡
                g_dio_status[card_num] = DIO_TRUST;
            }
            // OK & TRUST
            else if ((DIO_OK == g_dio_status[card_num]) && (DIO_TRUST == g_dio_status[card_num + 1]))
            {
				if (1 == g_mcu_status_other)
				{                                                                    
					//正常运行
					if (0 == g_mcu_status_stop)
					{
						g_dio_status[card_num] = DIO_OK;
						g_dio_status[card_num + 1] = DIO_TRUST;
					}
					//断电后重新上电
					else if (10 == g_mcu_status_stop)
					{
						g_dio_status[card_num] = DIO_OK;
						g_dio_status[card_num + 1] = DIO_TRUST;
					}
						
				}
				else if (0 == g_mcu_status_other)
				{
					//拔MCU板卡
					if (5 >= (g_fake_cnt + g_fake_cnt_y))
					{
						g_dio_status[card_num] = DIO_OK;
						g_dio_status[card_num + 1] = DIO_TRUST;
					}
					//整系断电
					else if (5 < (g_fake_cnt + g_fake_cnt_y))
					{
						if (0x03 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						else if (0x04 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
				}
            }
            // ERR & OK
            else if ((DIO_FAKE_ERR == g_dio_status[card_num]) && (DIO_OK == g_dio_status[card_num + 1]))
            {
                // 信任B板卡:A系的该板卡故障，由B系的互为一组的板卡替换
                g_dio_status[card_num + 1] = DIO_TRUST;
            }
            // ERR & ERR
            else if ((DIO_TRUE_ERR == g_dio_status[card_num]) && (DIO_TRUE_ERR == g_dio_status[card_num + 1]))
            {
                // 宕机？

                // 停止所有设备
                can_send_all_stop(__LINE__);

                // 宕机
                downtime(__LINE__, "num");
            }
            // ERR & TRUST
            else if ((DIO_FAKE_ERR == g_dio_status[card_num]) && (DIO_TRUST == g_dio_status[card_num + 1]))
            {
            }
            // ERR & DEF
            else if ((DIO_FAKE_ERR == g_dio_status[card_num]) && (DIO_DEFAULT == g_dio_status[card_num + 1]))
            {
                // 信任B板卡:A系的该板卡故障，由B系的互为一组的板卡替换
                g_dio_status[card_num + 1] = DIO_TRUST;
            }
            // TRUST & DEF
            else if ((DIO_TRUST == g_dio_status[card_num]) && (DIO_DEFAULT == g_dio_status[card_num + 1]))
            {
				// 信任B板卡:A系的该板卡故障，由B系的互为一组的板卡替换
                g_dio_status[card_num] = DIO_OK;
                g_dio_status[card_num + 1] = DIO_TRUST;
            }
            // TRUST & OK
            else if ((DIO_TRUST == g_dio_status[card_num]) && (DIO_OK == g_dio_status[card_num + 1]))
            {
                // 信任B板卡:A系的该板卡故障，由B系的互为一组的板卡替换
                // g_dio_status[card_num] = DIO_OK;
                // g_dio_status[card_num + 1] = DIO_TRUST;
				if (1 == g_mcu_status_other)
				{                                                                    
					if (0 == g_mcu_status_stop)
					{
						g_dio_status[card_num] = DIO_OK;
						g_dio_status[card_num + 1] = DIO_TRUST;
					}
					else if (10 == g_mcu_status_stop)
					{
						g_dio_status[card_num] = DIO_TRUST;
						g_dio_status[card_num + 1] = DIO_OK;
					}
						
				}
				else if (0 == g_mcu_status_other)
				{
					if (5 >= (g_fake_cnt + g_fake_cnt_y))
					{
						g_dio_status[card_num] = DIO_OK;
						g_dio_status[card_num + 1] = DIO_TRUST;
					}
					else if (5 < (g_fake_cnt + g_fake_cnt_y))
					{
						if (0x03 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_TRUST;
							g_dio_status[card_num + 1] = DIO_OK;
						}
						else if (0x04 == g_mcu_id_save)
						{
							g_dio_status[card_num] = DIO_OK;
							g_dio_status[card_num + 1] = DIO_TRUST;
						}
					}
				}
            }
            // TRUST & ERR
            else if ((DIO_TRUST == g_dio_status[card_num]) && (DIO_FAKE_ERR == g_dio_status[card_num + 1]))
            {
                // 信任A板卡
                g_dio_status[card_num] = DIO_TRUST;

            }
            // TRUST & TRUST
            else if ((DIO_TRUST == g_dio_status[card_num]) && (DIO_TRUST == g_dio_status[card_num + 1]))
            {
                // 信任A板卡
                g_dio_status[card_num] = DIO_OK;
                g_dio_status[card_num + 1] = DIO_TRUST;
            }
        }
    }

    // PLC数据失效检测
    check_plc_data_valid();
}

/*************************************************
 * @函数名称: calc_crc8
 * @函数功能: CRC8校验
 * @输入参数: *p_data   输入数据
              cnt       字节数
 * @输出参数: 无
 * @返回值  : uint8
 * @其它说明: 无
 *************************************************/
// P(x) = x^8 + x^5 + x^4 + 1 = 100110001
#define POLYNOMIAL_8    0x131
#define POLYNOMIAL_16   0x8005
uint8 calc_crc8(uint8 *p_data, uint8 cnt)
{
    uint8 bit = 0;      // bit mask
    uint8 crc = 0xFF;   // calculated checksum
    uint8 i = 0;        // byte counter

    if (NULL == p_data)
    {
        DEBUG_INFO();
        return 0xFF;
    }


    // calculates 8-Bit checksum with given polynomial
    for (i = 0; i < cnt; i++)
    {
        crc ^= (p_data[i]);
        for (bit = 8; bit > 0; bit--)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ POLYNOMIAL_8;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/*************************************************
 * @函数名称: calc_crc16_8
 * @函数功能: CRC16校验，校验值再CRC8校验
 * @输入参数: *p_data   输入数据
              cnt       字节数
 * @输出参数: 无
 * @返回值  : uint8
 * @其它说明: 无
 *************************************************/
uint8 calc_crc16_8(uint8 *p_data, uint8 cnt)
{
    uint8 bit = 0;          // bit mask
    uint16 crc = 0xFFFF;    // calculated checksum
    uint8 i = 0;            // byte counter
    uint8 data[2] = {0};

    if (NULL == p_data)
    {
        DEBUG_INFO();
        return 0xFF;
    }


    for (i = 0; i < cnt; i++)
    {
        crc ^= ((p_data[i]) << 8);
        for (bit = 8; bit > 0; bit--)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ POLYNOMIAL_16;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    data[0] = crc & 0xFF;
    data[1] = crc >> 8 & 0xFF;

    return calc_crc8(data, 2);
}

/*************************************************
 * @函数名称: set_fd_nonblock
 * @函数功能: 设置fd非阻塞模式
 * @输入参数: sock_fd   文件描述符
 * @输出参数: 无
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int set_fd_nonblock(int sock_fd)
{
    int result = 0;
    int flag = 0;

    if (sock_fd < 0)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    flag = fcntl(sock_fd, F_GETFL, 0);
    if (-1 == flag)
    {
        perror("fcntl F_GETFL fail");
        save_log(2, __LINE__, NULL);
        return -1;
    }
    result = fcntl(sock_fd, F_SETFL, flag | O_NONBLOCK);
    if (-1 == result)
    {
        perror("fcntl F_SETFL fail");
        save_log(2, __LINE__, NULL);
        return -1;
    }

    return 0;
}

/*
struct sockaddr
{
    unsigned short  sa_family;      // address family, AF_xxx
    char            sa_data[14];    // 14 bytes of protocol address
};

struct ifreq
{
#define IFHWADDRLEN    6
    union
    {
        char    ifrn_name[IFNAMSIZ];        // if name, e.g. "en0"
    } ifr_ifrn;

    union
    {
        struct  sockaddr ifru_addr;
        struct  sockaddr ifru_dstaddr;
        struct  sockaddr ifru_broadaddr;
        struct  sockaddr ifru_netmask;
        struct  sockaddr ifru_hwaddr;
        short   ifru_flags;
        int     ifru_ivalue;
        int     ifru_mtu;
        struct  ifmap ifru_map;
        char    ifru_slave[IFNAMSIZ];       // Just fits the size
        char    ifru_newname[IFNAMSIZ];
        void    __user      *ifru_data;
        struct  if_settings ifru_settings;
    } ifr_ifru;
};

struct sockaddr_can
{
    sa_family_t can_family;
    int         can_ifindex;
    union
    {
        // transport protocol class address information (e.g. ISOTP)
        struct
        {
            canid_t rx_id, tx_id;
        } tp;

        // reserved for future CAN protocols address information
    } can_addr;
};
*/
/*************************************************
 * @函数名称: can_bus_init
 * @函数功能: CAN总线初始化
 * @输入参数: 无
 * @输出参数: p_can_fd 文件描述符
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_bus_init(int *p_can_fd)
{
    int result = 0;
    int can_fd = 0;
    struct ifreq ifr = {{{0}}, {{0, {0}}}};
    struct sockaddr_can can_addr = {0, 0, {{0, 0}}};

    if (NULL == p_can_fd)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    /* CAN0 */
    system("ip link set can0 down");
    system("ip link set can0 up type can bitrate 1000000");

    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (-1 == can_fd)
    {
        perror("can0 socket error");
        save_log(2, __LINE__, NULL);
        return -1;
    }
    strcpy((char *)(ifr.ifr_name), "can0");
    result = ioctl(can_fd, SIOCGIFINDEX, &ifr);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    // printf("can0 can_ifindex = 0x%02X\r\n", ifr.ifr_ifindex);
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = ifr.ifr_ifindex;
    result = bind(can_fd, (struct sockaddr *)&can_addr, sizeof(can_addr));
    if (-1 == result)
    {
        perror("can0 bind err!");
        save_log(2, __LINE__, NULL);
        return -1;
    }
    result = set_fd_nonblock(can_fd);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    p_can_fd[0] = can_fd;


    /* CAN1 */
    system("ip link set can1 down");
    system("ip link set can1 up type can bitrate 1000000");

    can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (-1 == can_fd)
    {
        perror("can1 socket error");
        save_log(2, __LINE__, NULL);
        return -1;
    }
    strcpy((char *)(ifr.ifr_name), "can1");
    result = ioctl(can_fd, SIOCGIFINDEX, &ifr);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    // printf("can1 can_ifindex = 0x%02X\r\n", ifr.ifr_ifindex);
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = ifr.ifr_ifindex;
    result = bind(can_fd, (struct sockaddr *)&can_addr, sizeof(can_addr));
    if (-1 == result)
    {
        perror("can1 bind err!");
        save_log(2, __LINE__, NULL);
        return -1;
    }
    result = set_fd_nonblock(can_fd);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    p_can_fd[1] = can_fd;

    return 0;
}

/*************************************************
 * @函数名称: set_system_time
 * @函数功能: 设置系统时间
 * @输入参数: can_frame 帧数据
 * @输出参数: 无
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int set_system_time(struct can_frame can_frame)
{
    int result = 0;
    time_t rawtime = 0;
    struct timeval tv = {0, 0};
    struct tm mvb_time = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, NULL};
	
	struct tm *ptm;
	int y,m,d,h,n,s;
	
	rawtime = time(NULL);
	ptm = localtime(&rawtime);
	
	y = ptm->tm_year + 1900;
	m = ptm-> tm_mon + 1; 
	d = ptm-> tm_mday;
	h = ptm-> tm_hour;
	n = ptm-> tm_min;
	s = ptm-> tm_sec;
    // 获取收帧时间戳
    // result = gettimeofday(&tv, NULL);
    // if (-1 == result)
    // {
        // DEBUG_INFO();
        // save_log(2, __LINE__, NULL);
        // return -1;
    // }
    //tv.tv_sec -= 28800; // UTC time
	//printf("aaaaaaaaaa---time = %d,%d,%d,%d,%d,%d\n",y,m,d,h,n,s);
	
    mvb_time.tm_year = can_frame.data[1] + 100;
    mvb_time.tm_mon = can_frame.data[2] - 1;
    mvb_time.tm_mday = can_frame.data[3];
    mvb_time.tm_hour = can_frame.data[4];
    mvb_time.tm_min = can_frame.data[5];
    mvb_time.tm_sec = can_frame.data[6];
	// printf("bbbbbbbbbbb---time = %d,%d,%d,%d,%d,%d\n",mvb_time.tm_year,mvb_time.tm_mon,mvb_time.tm_mday,mvb_time.tm_hour,mvb_time.tm_min,mvb_time.tm_sec);
    //time = mktime(&mvb_time);
	
    // if (-1 == time)
    // {
        // DEBUG_INFO();
        // save_log(2, __LINE__, NULL);
        // return -1;
    // }
	
    // 超3秒则更新时间
	if ((0 != mvb_time.tm_sec) && (0 != s))
	{
		if ((3 <= abs(s - mvb_time.tm_sec)) || (mvb_time.tm_min != n))	
		{
			printf("local_sec2222222222222222222222 = %d\r\n", s);
			printf("change_sec22222222222222222222222222 = %d\r\n", mvb_time.tm_sec);
			tv.tv_sec = mktime(&mvb_time);       
			tv.tv_usec = 0;
			result = settimeofday(&tv, NULL);
			if (-1 == result)
			{
				DEBUG_INFO();
				save_log(2, __LINE__, NULL);
				return -1;
			}
			system("hwclock -w");
		}
	}

    return 0;
}

/*************************************************
 * @函数名称: set_mcu_system_time
 * @函数功能: 设置系统时间
 * @输入参数: can_frame 帧数据
 * @输出参数: 无
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int set_mcu_system_time(struct can_frame can_frame)
{
    int result = 0;
    time_t time = 0;
    struct timeval tv = {0, 0};
    struct tm mcu_time = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, NULL};


    mcu_time.tm_year = can_frame.data[0] + 100;
    mcu_time.tm_mon = can_frame.data[1] - 1;
    mcu_time.tm_mday = can_frame.data[2];
    mcu_time.tm_hour = can_frame.data[3];
    mcu_time.tm_min = can_frame.data[4];
    mcu_time.tm_sec = can_frame.data[5];
		
	time = mktime(&mcu_time);
	
    if (-1 == time)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

	if (0x03 == can_frame.data[6])
	{
		//printf("local_sec = %d\r\n", tv.tv_sec);
		tv.tv_sec = time;
		//printf("change_sec = %d\r\n", tv.tv_sec);
		tv.tv_usec = 0;
		result = settimeofday(&tv, NULL);
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		system("hwclock -w");
		return 0;
	}


    //return 0;
}

/*
struct timeval
{
    long int tv_sec;    // 秒数
    long int tv_usec;   // 微秒数
};
*/
/*************************************************
 * @函数名称: save_frame
 * @函数功能: 保存数据
 * @输入参数: can_frame 帧数据
              flag      0 发送帧   1 接收帧   2 错误帧
 * @输出参数: 无
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int save_frame(struct can_frame can_frame, uint8 flag)
{
    return 0;

    int result = 0;
    struct timeval tv = {0, 0};
    char buf[64] = {0x7E, 0x7F, 0x7E};

    if (flag > 2)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // 获取发帧时间戳
    result = gettimeofday(&tv, NULL);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    tv.tv_sec -= 28800; // UTC time
    tv.tv_usec /= 1000; // us转为ms

    if (0 == flag)
    {
        buf[3] = 0x00;
    }
    else if (1 == flag)
    {
        buf[3] = 0xAA;
    }
    else if (2 == flag)
    {
        buf[3] = 0xFF;
    }

    memcpy(buf + 4, &tv.tv_sec, 4);
    memcpy(buf + 4 + 4, &tv.tv_usec, 2);
    memcpy(buf + 4 + 4 + 2, &can_frame.can_id, 4);
    memcpy(buf + 4 + 4 + 2 + 4, &can_frame.can_dlc, 1);
    memcpy(buf + 4 + 4 + 2 + 4 + 1, &can_frame.data, 8);
    result = save_data(buf, 4 + 4 + 2 + sizeof(struct can_frame) - 3);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    return 0;
}

/*************************************************
 * @函数名称: can_recv_mcu
 * @函数功能: CAN MCU通讯帧处理
 * @输入参数: recv_frame    CAN接收帧
              can_line      CAN线路
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_recv_mcu(struct can_frame recv_frame, T_can_buff *p_can_buff, uint8 can_line)
{
    int     result = 0;
    static uint8 offset_last[2] = {0};  // 上次收帧位置(两条CAN线路)


	if ((NULL == p_can_buff) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if (recv_frame.data[0] != ((offset_last[can_line] + 1) % 256))
    {
        // MCU序号错误++
        g_mcu_offset_err[can_line]++;
        if (g_mcu_offset_err[can_line] > 9)
        {
            // 停止所有设备
            can_send_all_stop(__LINE__);

            // 宕机
            downtime(__LINE__, "num");
        }
    }
    offset_last[can_line] = recv_frame.data[0];

    // 另一块板卡ID
    if (g_mcu_can_id[1].all == recv_frame.can_id)
    {
        // 帧长度
        if (8 == recv_frame.can_dlc)
        {
            if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
            {
                // 处理心跳
                g_mcu_heart_other[can_line] = recv_frame.data[1] + ((recv_frame.data[2] << 8) & 0xFF);

                // 当前为信任CAN线路
                if (CAN_LINE_TRUST == g_can_line_status[can_line])
                {
                    g_mcu_status_other = 1;
					g_mcu_status_other_a = 1; //add by 11.11

                    // 主备模式
                    if (MCU_MASTER == recv_frame.data[3])   // 另一块板卡为主
                    {
                        // MCU运行状态
                        if (MCU_BACKUP != g_mcu_status)
                        {
                            if (FORCIBLY_AUTO == g_system_forcibly)
                            {
                                //debug_line("MCU_BACKUP! recv_mcu\r\n");
                                // 当前板卡转为备板
                                g_mcu_status = MCU_BACKUP;

                                // 发送标志位置0
                                result = clear_all_dio_send_flag(p_can_buff);
                                if (-1 == result)
                                {
                                    DEBUG_INFO();
                                    save_log(2, __LINE__, NULL);
                                    return -1;
                                }
                            }
                        }
                        if (recv_frame.data[4] < DIO_CNT / 2)
                        {
                            // 备板卡存储主板卡DI信息
                            g_plc_data[2 * recv_frame.data[4]] =  recv_frame.data[5];       // DI数据
                            g_plc_data[2 * recv_frame.data[4] + 1] =  recv_frame.data[6];   // DI数据
                        }
                        else
                        {
                            DEBUG_INFO();
                            // 故障
                            // 停止所有设备
                            can_send_all_stop(__LINE__);

                            // 宕机
                            downtime(__LINE__, "MCU other");
                        }
                    }
                    else if (MCU_BACKUP == recv_frame.data[3])  // 另一块板卡为备
                    {
						// MCU运行状态
                        if (MCU_MASTER != g_mcu_status)
                        {
                            if (FORCIBLY_AUTO == g_system_forcibly)
                            {
                                g_mcu_status = MCU_MASTER;
                            }
                        }

                        // 保存文件
                        result = save_frame(recv_frame, 1);
                        if (-1 == result)
                        {
                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
					//禁止重新上电系争取主资格
					else if (2 == recv_frame.data[3])
					{
						g_mcu_status = MCU_BACKUP;
						g_mcu_stay_status = 3;
					}
                }
            }
            else
            {
                // 帧校验错误
                debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                // 停止所有设备
                can_send_all_stop(__LINE__);

                // 宕机
                downtime(__LINE__, "crc");
            }
        }
        else
        {
            // 帧长度错误
            DEBUG_INFO();

            // 停止所有设备
            can_send_all_stop(__LINE__);

            // 宕机
            downtime(__LINE__, "frame len");
        }
    }
    else
    {
        // MCU帧ID错误
        debug_line("recv id=0x%04X,dlc=%d\r\n", recv_frame.can_id, recv_frame.can_dlc);
        DEBUG_INFO();
        // return -111;
    }

    return 0;
}

/*************************************************
 * @函数名称: can_recv_mcu_time
 * @函数功能: CAN MCU通讯帧处理
 * @输入参数: recv_frame    CAN接收帧
              can_line      CAN线路
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_recv_mcu_time(struct can_frame recv_frame, T_can_buff *p_can_buff, uint8 can_line)
{
    int     result = 0;
	T_extid can_id = {0};   // CAN 接收ExtID

	if ((NULL == p_can_buff) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
	can_id.all = recv_frame.can_id;
	
	// 帧类型
	if (0x01 == can_id.frame_type)  // 以下均是回复帧
    {
		// 帧长度
		if (7 == recv_frame.can_dlc)
		{			
			// 当前为信任CAN线路
			//if (CAN_LINE_TRUST == g_can_line_status[can_line])
			{
				// 设置系统时间
				result = set_mcu_system_time(recv_frame);
				if (-1 == result)
				{
					DEBUG_INFO();
					//return -1;
				}
				else if (0 == result)
				{
					//DEBUG_INFO();
					return 0;
				}
			}
		}
		else
		{
			// 帧长度错误
			DEBUG_INFO();
			return -1;

			// 停止所有设备
			//can_send_all_stop(__LINE__);

			// 宕机
			//downtime(__LINE__, "frame len");
		}
    }
    else
    {
        // MCU帧ID错误
        //debug_line("recv id=0x%04X,dlc=%d\r\n", recv_frame.can_id, recv_frame.can_dlc);
        DEBUG_INFO();
        // return -1;
		g_reset_cnt++;
		
    }

    return -1;
}

/*************************************************
 * @函数名称: can_recv_dio
 * @函数功能: CAN DIO通讯帧处理
 * @输入参数: recv_frame    CAN接收帧
              can_line      CAN线路
              recv_tv       时间结构体
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_recv_dio(struct can_frame recv_frame, T_can_buff *p_can_buff, uint8 can_line, struct timeval recv_tv)
{
    int     result = 0;
    uint16  offset = 0;     // 当前收帧位置
    int8    card_num = 0;   // DIO板卡号
    int     time_after = 0; // 时间间隔
    T_extid can_id = {0};   // CAN 接收ExtID
    static uint16 offset_status_last[2][DIO_CNT] = {{0}};   // 上次收状态帧位置(两条CAN线路)
    static uint16 offset_DI_last[2][DIO_CNT] = {{0}};       // 上次收DI帧位置
    static uint16 offset_DO_last[2][DIO_CNT] = {{0}};       // 上次收DO帧位置
    static uint8 dio_status_last[DIO_CNT][3] =
    {
        {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF,}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, \
        {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF,}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, \
        {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF,}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, \
        {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF,}, {0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF},
    };
    static uint8 dio_di_last[DIO_CNT][2] =
    {
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, \
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, \
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, \
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, \
    };
    static uint8 dio_do_last[DIO_CNT][2] =
    {
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, \
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, \
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, \
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}
    };
    char str[64] = {0};

    if ((NULL == p_can_buff) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    can_id.all = recv_frame.can_id;
    offset = recv_frame.data[0];    // 当前收帧位置
    if (can_id.hard_addr < 0x20)
    {
        card_num = can_id.hard_addr - 5;
    }
    else if (can_id.hard_addr > 0x20)
    {
        card_num = can_id.hard_addr - 0x20 + 5;
    }

    // 通讯方向
    if (((0x03 == g_mcu_can_id[0].all) && (0x01 == can_id.direction)) || \
            ((0x04 == g_mcu_can_id[0].all) && (0x03 == can_id.direction)))
    {
        // 帧类型
        switch (can_id.frame_type)  // 以下均是回复帧
        {			
			case 2: // 状态请求帧 该帧包含启停，控制和状态
            {
                // 板卡编号
                if ((0 <= card_num) && (card_num < DIO_CNT))
                {
                    // 帧长度
                    if (6 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 已经收到DIO回复帧
                                if (1 == p_can_buff->DIO_status[card_num].data[offset].frame_recv_flag)
                                {
                                    // 设备地址重复 或 单板卡重复帧
                                    // 停止所有设备
                                    can_send_all_stop(__LINE__);

                                    // 宕机
                                    sprintf(str, "can%d card %2d frame repeat", can_line, card_num);
                                    downtime(__LINE__, str);
                                    // downtime(__LINE__, "frame repeat");
                                }
                                // 收到DIO回复帧
                                p_can_buff->DIO_status[card_num].data[offset].frame_recv_flag = 1;

                                // 时间间隔
                                time_after = (recv_tv.tv_sec - p_can_buff->DIO_status[card_num].data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->DIO_status[card_num].data[offset].send_tv.tv_usec);
                                // 帧超时
                                //if (time_after < FRAME_TIMEOUT)
								if (1)	
                                {
                                    // DIO状态帧序号需要保证每次加1
                                    if (offset != ((offset_status_last[can_line][card_num] + 1) % 256))
                                    {
                                        // DIO序号错误++
                                        g_dio_offset_err[can_line][card_num]++;
                                        if (g_dio_offset_err[can_line][card_num] > 18)
                                        {
                                            // 停止所有设备
                                            can_send_all_stop(__LINE__);

                                            // 宕机
                                            downtime(__LINE__, "num");
                                        }
                                    }
                                    else
                                    {
                                        recv_frame.data[1];                 // 硬件主版本号
                                        recv_frame.data[2] & 0xFF;          // 硬件次版本号
                                        recv_frame.data[3];                 // 软件主版本号
                                        (recv_frame.data[2] >> 4) & 0xFF;   // 软件次版本号

                                        g_dio_ctrl[card_num] = recv_frame.data[4];   // DIO板卡主备状态
                                        if (recv_frame.data[4] != p_can_buff->DIO_status[card_num].data[offset].send_data[3])
                                        {
                                            // DIO板卡故障
                                            g_dio_status[card_num] = DIO_FAKE_ERR;  //接收帧与发送帧不匹配判断为真故障

                                            // DIO板卡状态切换
                                            check_dio_status();

                                            //保存故障
											sprintf(str, "ST:card %2d offset %d", card_num, offset);
											save_log(1, __LINE__, str);


                                            g_dio_ctrl[card_num] = 2;
                                        }
                                        else
                                        {
                                            // 设置DIO板卡错误计数
                                            g_dio_err_cnt[can_line][card_num] = 0;
                                            //假err可以设置为ok   真err就不进行任何状态操作
                                            if ((DIO_FAKE_ERR == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
                                            {
                                                // DIO板卡正常
                                                g_dio_status[card_num] = DIO_OK;

                                                // DIO板卡状态切换
                                                check_dio_status();

                                                debug_line("card_num %2d OK!\r\n", card_num);
                                            }
                                        }
                                    }
                                    offset_status_last[can_line][card_num] = offset;
                                }
                                else
                                {
                                    // 收帧超时
                                    debug_line("stime s=%ds ms=%dms, rtime s=%ds ms=%dms\r\n", p_can_buff->DIO_status[card_num].data[offset].send_tv.tv_sec, p_can_buff->DIO_status[card_num].data[offset].send_tv.tv_usec, recv_tv.tv_sec, recv_tv.tv_usec);
                                    debug_line("offset = 0x%02X, card_num = %2d, frame_type = %d, overtime = %dms\r\n", offset, card_num, can_id.frame_type, time_after);

                                    // qjq 超时处理
                                    // ...
                                    DEBUG_INFO();
                                    // return -111;
                                }

                                // 数据变化保存
                                result = memcmp(&dio_status_last[card_num][0], &recv_frame.data[1], recv_frame.can_dlc - 2);
                                if (0 != result)
                                {
                                    memcpy(&dio_status_last[card_num][0], &recv_frame.data[1], recv_frame.can_dlc - 2);

                                    // 保存文件
                                    result = save_frame(recv_frame, 1);
                                    if (-1 == result)
                                    {
                                        DEBUG_INFO();
                                        save_log(2, __LINE__, NULL);
                                        return -1;
                                    }
                                }
                            }
                            else
                            {
                                // 设置DIO板卡错误计数
                                g_dio_err_cnt[can_line][card_num] = 0;
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();

                            // 停止所有设备
                            can_send_all_stop(__LINE__);

                            // 宕机
                            downtime(__LINE__, "crc");
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();

                        // 停止所有设备
                        can_send_all_stop(__LINE__);

                        // 宕机
                        downtime(__LINE__, "frame len");
                    }
                }
                else
                {
                    // 硬件地址错误
                    debug_line("recv id=0x%04X,dlc=%d\r\n", recv_frame.can_id, recv_frame.can_dlc);
                    // return -111;
                }


                break;
            }

            case 3: // 数据请求帧
            {

                // 板卡编号
                if ((0 <= card_num) && (card_num < DIO_CNT))
                {
                    // 帧长度
                    if (4 == recv_frame.can_dlc)
                    {
						if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 已经收到DIO回复帧
                                if (1 == p_can_buff->DIO_DI[card_num].data[offset].frame_recv_flag)
                                {
                                    // 设备地址重复 或 单板卡重复帧
                                    // 停止所有设备
                                    can_send_all_stop(__LINE__);

                                    // 宕机
                                    sprintf(str, "can%d card %2d frame repeat", can_line, card_num);
                                    downtime(__LINE__, str);
                                    // downtime(__LINE__, "frame repeat");
                                }
                                // 收到DIO回复帧
                                p_can_buff->DIO_DI[card_num].data[offset].frame_recv_flag = 1;								
								
                                // 时间间隔
                                time_after = (recv_tv.tv_sec - p_can_buff->DIO_DI[card_num].data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->DIO_DI[card_num].data[offset].send_tv.tv_usec);
                                // 帧超时
                                //if (time_after < FRAME_TIMEOUT)
								if (1)	
                                {
                                    // DIO数据请求帧序号需要保证每次加1
                                    if (offset != ((offset_DI_last[can_line][card_num] + 1) % 256))
                                    {
                                        // DIO序号错误++
                                        g_dio_offset_err[can_line][card_num]++;
                                        if (g_dio_offset_err[can_line][card_num] > 18)
                                        {
                                            // 停止所有设备
                                            can_send_all_stop(__LINE__);

                                            // 宕机
                                            downtime(__LINE__, "num");
                                        }
                                    }
                                    else
                                    {
                                        // 设置DIO板卡错误计数
                                        g_dio_err_cnt[can_line][card_num] = 0;

                                        g_plc_data[2 * (card_num / 2)] = recv_frame.data[1];
                                        g_plc_data[2 * (card_num / 2) + 1] = recv_frame.data[2];

                                        //debug_line("card_num = 0x%02X, 0x%02X, 0x%02X\r\n", card_num, g_plc_data[2 * (card_num / 2)], g_plc_data[2 * (card_num / 2) + 1]);  // qjq test
                                    }
                                    offset_DI_last[can_line][card_num] = offset;

                                }
                                else
                                {
                                    // 收帧超时
                                    debug_line("stime s=%ds ms=%dms, rtime s=%ds ms=%dms\r\n", p_can_buff->DIO_DI[card_num].data[offset].send_tv.tv_sec, p_can_buff->DIO_DI[card_num].data[offset].send_tv.tv_usec, recv_tv.tv_sec, recv_tv.tv_usec);
                                    debug_line("offset = 0x%02X, card_num = %2d, frame_type = %d, overtime = %dms\r\n", offset, card_num, can_id.frame_type, time_after);

                                    // qjq 超时处理
                                    // ...

                                    DEBUG_INFO();
                                    // return -111;
                                }

                                // 数据变化保存
                                result = memcmp(&dio_di_last[card_num][0], &recv_frame.data[1], recv_frame.can_dlc - 2);
                                if (0 != result)
                                {
                                    memcpy(&dio_di_last[card_num][0], &recv_frame.data[1], recv_frame.can_dlc - 2);

                                    // 保存文件
                                    result = save_frame(recv_frame, 1);
                                    if (-1 == result)
                                    {
                                        DEBUG_INFO();
                                        save_log(2, __LINE__, NULL);
                                        return -1;
                                    }
                                }
                            }
                            else
                            {
                                // 设置DIO板卡错误计数
                                g_dio_err_cnt[can_line][card_num] = 0;
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();

                            // 停止所有设备
                            can_send_all_stop(__LINE__);

                            // 宕机
                            downtime(__LINE__, "crc");
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();

                        // 停止所有设备
                        can_send_all_stop(__LINE__);

                        // 宕机
                        downtime(__LINE__, "frame len");
                    }
                }
                else
                {
                    // 硬件地址错误
                    debug_line("recv id=0x%04X,dlc=%d\r\n", recv_frame.can_id, recv_frame.can_dlc);
                    // return -111;
                }

                break;
            }

            case 4: // 数据下发帧
            {

                // 板卡编号
                if ((0 <= card_num) && (card_num < DIO_CNT))
                {
                    // 帧长度
                    if (4 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 已经收到DIO回复帧
                                if (1 == p_can_buff->DIO_DO[card_num].data[offset].frame_recv_flag)
                                {
                                    // 设备地址重复 或 单板卡重复帧
                                    // 停止所有设备
                                    can_send_all_stop(__LINE__);

                                    // 宕机
                                    sprintf(str, "can%d card %2d offset 0x%02X frame repeat", can_line, card_num, offset);
                                    downtime(__LINE__, str);
                                    // downtime(__LINE__, "frame repeat");
                                }
                                // 收到DIO回复帧
                                p_can_buff->DIO_DO[card_num].data[offset].frame_recv_flag = 1;

                                // 回复帧与发送帧相同
                                result = memcmp(recv_frame.data, p_can_buff->DIO_DO[card_num].data[offset].send_data, recv_frame.can_dlc);
                                if (0 == result)    // 帧数据正确
                                {
                                    // 时间间隔
                                    time_after = (recv_tv.tv_sec - p_can_buff->DIO_DO[card_num].data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->DIO_DO[card_num].data[offset].send_tv.tv_usec);
                                    // 帧超时
                                    //if (time_after < FRAME_TIMEOUT)
									if (1)
                                    {
                                        // DIO数据输出帧序号需要保证每次加1
                                        if (offset != ((offset_DO_last[can_line][card_num] + 1) % 256))
                                        {
                                            // DIO序号错误++
                                            g_dio_offset_err[can_line][card_num]++;
                                            if (g_dio_offset_err[can_line][card_num] > 18)
                                            {
                                                // 停止所有设备
                                                can_send_all_stop(__LINE__);

                                                // 宕机
                                                downtime(__LINE__, "num");
                                            }
                                        }
                                        else
                                        {
                                            // 设置DIO板卡错误计数
                                            g_dio_err_cnt[can_line][card_num] = 0;
                                        }
                                        offset_DO_last[can_line][card_num] = offset;
                                    }
                                    else
                                    {
                                        // 收帧超时
                                        debug_line("stime s=%ds ms=%dms, rtime s=%ds ms=%dms\r\n", p_can_buff->DIO_DO[card_num].data[offset].send_tv.tv_sec, p_can_buff->DIO_DO[card_num].data[offset].send_tv.tv_usec, recv_tv.tv_sec, recv_tv.tv_usec);
                                        debug_line("offset = 0x%02X, card_num = %2d, frame_type = %d, overtime = %dms\r\n", offset, card_num, can_id.frame_type, time_after);

                                        // qjq 超时处理
                                        // ...

                                        DEBUG_INFO();
                                        // return -111;
                                    }

                                    // 数据变化保存
                                    result = memcmp(&dio_do_last[card_num][0], &recv_frame.data[1], recv_frame.can_dlc - 2);
                                    if (0 != result)
                                    {
                                        memcpy(&dio_do_last[card_num][0], &recv_frame.data[1], recv_frame.can_dlc - 2);

                                        // 保存文件
                                        result = save_frame(recv_frame, 1);
                                        if (-1 == result)
                                        {
                                            DEBUG_INFO();
                                            save_log(2, __LINE__, NULL);
                                            return -1;
                                        }
                                    }
                                }
                                else
                                {
                                    // 帧数据错误
                                    DEBUG_INFO();

                                    // 停止所有设备
                                    can_send_all_stop(__LINE__);

                                    // 宕机
                                    downtime(__LINE__, "data");
                                }
                            }
                            else
                            {
                                // 设置DIO板卡错误计数
                                g_dio_err_cnt[can_line][card_num] = 0;
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();

                            // 停止所有设备
                            can_send_all_stop(__LINE__);

                            // 宕机
                            downtime(__LINE__, "crc");
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();

                        // 停止所有设备
                        can_send_all_stop(__LINE__);

                        // 宕机
                        downtime(__LINE__, "frame len");
                    }
                }
                else
                {
                    // 硬件地址错误
                    debug_line("recv id=0x%04X,dlc=%d\r\n", recv_frame.can_id, recv_frame.can_dlc);
                    // return -111;
                }

                break;
            }

            default:
            {
                // 帧类型错误
                DEBUG_INFO();
                // return -111;
                break;
            }
        }
    }
    else
    {
        // 通讯方向错误
        // debug_line("recv id=0x%04X,dlc=%d\r\n", recv_frame.can_id, recv_frame.can_dlc);
        // return -111;
    }

#ifdef PRINT_TIMEOUT
    // qjq test
    static int time_dio_after_max = 0;

    if (time_after > time_dio_after_max)
    {
        time_dio_after_max = time_after;
        print_line("time_dio_after_max=%dms\r\n\r\n", time_dio_after_max);
    }
#endif

    return 0;
}

/*************************************************
 * @函数名称: can_recv_mvb
 * @函数功能: CAN MVB通讯帧处理
 * @输入参数: recv_frame    CAN接收帧
              can_line      CAN线路
              recv_tv       时间结构体
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_recv_mvb(struct can_frame recv_frame, T_can_buff *p_can_buff, uint8 can_line, struct timeval recv_tv)
{
    int     result = 0;
    uint16  offset = 0;     // 当前收帧位置
    int     time_after = 0; // 时间间隔
    T_extid can_id = {0};   // CAN 接收ExtID
    static uint8 mvb_status_last[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static uint8 dio_last[5][4] =
    {
        {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}, \
        {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}
    };
    static uint8 mvb_heart_last[2] = {0xFF, 0xFF};
    static uint8 mvb_data_last[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	static uint8 mvb_yinxian_last[1] = {0xFF};
	char str[64] = {0};

    if ((NULL == p_can_buff) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    offset = recv_frame.data[0];    // 当前收帧位置

    can_id.all = recv_frame.can_id;
    // 通讯方向
    if (((0x03 == g_mcu_can_id[0].all) && (0x01 == can_id.direction)) || \
            ((0x04 == g_mcu_can_id[0].all) && (0x03 == can_id.direction)))
    {
        // 硬件地址
        if (0x01 == can_id.hard_addr)    // MVB板卡地址
        {
            // 帧类型
            switch (can_id.frame_type)  // 以下均是回复帧
            {
                case 0: // 起停帧
                {
                    // 设置MVB板卡错误计数
                    g_mvb_err_cnt[can_line] = 0;

                    // 帧长度
                    if (3 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到MVB回复帧
                                p_can_buff->MVB_run.data[offset].frame_recv_flag = 1;

                                // 回复帧与发送帧相同
                                result = memcmp(recv_frame.data, p_can_buff->MVB_run.data[offset].send_data, recv_frame.can_dlc);
                                if (0 == result)    // 帧数据正确
                                {
                                    // 时间间隔
                                    time_after = (recv_tv.tv_sec - p_can_buff->MVB_run.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->MVB_run.data[offset].send_tv.tv_usec);
                                    // 帧超时
                                    if (time_after < FRAME_TIMEOUT)
                                    {
                                        // 保存MVB板卡起停状态
                                        g_mvb_run = recv_frame.data[1]; // 0 停止, 1 启动
                                    }
                                    else
                                    {
                                        //保存故障
										sprintf(str, "MVBQT:offset %d overtime %dms timeout",offset, time_after);
										save_log(1, __LINE__, str);
                                    }

                                    // 保存文件
                                    result = save_frame(recv_frame, 1);
                                    if (-1 == result)
                                    {
                                        DEBUG_INFO();
                                        save_log(2, __LINE__, NULL);
                                        return -1;
                                    }
                                }
                                else
                                {
                                    // 帧数据错误
                                    DEBUG_INFO();
                                    save_log(2, __LINE__, NULL);
                                    return -1;
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }

                    break;
                }

                case 2: // 状态请求帧
                {
                    // 设置MVB板卡错误计数
                    g_mvb_err_cnt[can_line] = 0;

                    // 帧长度
                    if (8 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到MVB回复帧
                                p_can_buff->MVB_status.data[offset].frame_recv_flag = 1;

                                // 时间间隔
                                time_after = (recv_tv.tv_sec - p_can_buff->MVB_status.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->MVB_status.data[offset].send_tv.tv_usec);
                                // 帧超时
                                if (time_after < FRAME_TIMEOUT)
                                {
                                    // qjq
                                    recv_frame.data[1]; // 硬件主版本号
                                    recv_frame.data[2]; // 硬件次版本号
                                    recv_frame.data[3]; // 软件主版本号
                                    recv_frame.data[4]; // 软件次版本号
																		
                                    if (1 == (recv_frame.data[5] & 0x01))    // 当前板卡故障
                                    {
                                        // MVB板卡错误
                                        g_mvb_status = MVB_ERR;

                                        // MVB板卡连接状态
                                        g_mvb_connect = 0;  // 0 断开, 1 连接
                                    }
                                    else
                                    {
                                        // MVB板卡正常
                                        g_mvb_status = MVB_OK;

                                        // MVB板卡连接状态
                                        g_mvb_connect = (recv_frame.data[5] >> 1) & 0x01;   // 0 断开, 1 连接
										//debug_line("g_mvb_connect = %x\r\n", g_mvb_connect);
                                    }
									//接收到的值为1表示为头车，接收到的值为6表示为尾车
									g_plc_yx_data[1] = recv_frame.data[6];
									//debug_line("g_plc_yx_data[1] = %x\r\n", g_plc_yx_data[1]);
                                }
                                else
                                {
                                    //保存故障
									sprintf(str, "MVBST:offset %d overtime %dms timeout",offset, time_after);
									save_log(1, __LINE__, str);
                                }

                                // 数据变化保存
                                result = memcmp(&mvb_status_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);
                                if (0 != result)
                                {
                                    memcpy(&mvb_status_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);

                                    // 保存文件
                                    result = save_frame(recv_frame, 1);
                                    if (-1 == result)
                                    {
                                        DEBUG_INFO();
                                        save_log(2, __LINE__, NULL);
                                        return -1;
                                    }
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }

                    break;
                }
#if 0				
				case 3: // MVB硬线取消通信
                {
                    // 设置MVB板卡错误计数
                    g_mvb_err_cnt[can_line] = 0;

                    // 帧长度
                    if (3 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到MVB回复帧
                                p_can_buff->MVB_yinxian.data[offset].frame_recv_flag = 1;

								// 时间间隔
								time_after = (recv_tv.tv_sec - p_can_buff->MVB_yinxian.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->MVB_yinxian.data[offset].send_tv.tv_usec);
								// 帧超时
								if (time_after < FRAME_TIMEOUT)
								{
									g_plc_yx_data[0] = recv_frame.data[1];
									//debug_line("g_plc_yx_data = %x\r\n", g_plc_yx_data[0]);
								}
								else
								{
									// 收帧超时
									debug_line("overtime = %dms\r\n", time_after);

									// qjq 超时处理
									// ...

									DEBUG_INFO();
									// return -111;
								}

								// 数据变化保存
								result = memcmp(&mvb_yinxian_last[0], &recv_frame.data[3], recv_frame.can_dlc - 2 - 2);
								if (0 != result)
								{
									memcpy(&mvb_yinxian_last[0], &recv_frame.data[3], recv_frame.can_dlc - 2 - 2);

									// 保存文件
									result = save_frame(recv_frame, 1);
									if (-1 == result)
									{
										DEBUG_INFO();
										save_log(2, __LINE__, NULL);
										return -1;
									}
								}
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }


                    break;
                }
#endif
                case 4: // 数据下发帧
                {
                    // 设置MVB板卡错误计数
                    g_mvb_err_cnt[can_line] = 0;

                    // 帧长度
                    if (7 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到MVB回复帧
                                p_can_buff->MVB_DIO.data[offset].frame_recv_flag = 1;

                                // 回复帧与发送帧相同
                                result = memcmp(recv_frame.data, p_can_buff->MVB_DIO.data[offset].send_data, recv_frame.can_dlc);
                                if (0 == result)    // 帧数据正确
                                {
                                    // 时间间隔
                                    time_after = (recv_tv.tv_sec - p_can_buff->MVB_DIO.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->MVB_DIO.data[offset].send_tv.tv_usec);
                                    // 帧超时
                                    if (time_after < FRAME_TIMEOUT)
                                    {
										// MVB板卡正常
                                        g_mvb_status = MVB_OK;
                                    }
                                    else
                                    {
                                        //保存故障
										sprintf(str, "MVBIO:offset %d overtime %dms timeout",offset, time_after);
										save_log(1, __LINE__, str);
                                    }

                                    // 数据变化保存
                                    result = memcmp(&dio_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);
                                    if (0 != result)
                                    {
                                        memcpy(&dio_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);

                                        // 保存文件
                                        result = save_frame(recv_frame, 1);
                                        if (-1 == result)
                                        {
                                            DEBUG_INFO();
                                            save_log(2, __LINE__, NULL);
                                            return -1;
                                        }
                                    }
                                }
                                else
                                {
                                    // 帧数据错误
                                    DEBUG_INFO();
                                    save_log(2, __LINE__, NULL);
                                    return -1;
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }

                    break;
                }

                case 5: // heart
                {
                    // 设置MVB板卡错误计数
                    g_mvb_err_cnt[can_line] = 0;

                    // 帧长度
                    if (6 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到MVB回复帧
                                p_can_buff->MVB_heart.data[offset].frame_recv_flag = 1;

                                // 回复帧与发送帧相同
                                result = memcmp(recv_frame.data, p_can_buff->MVB_heart.data[offset].send_data, recv_frame.can_dlc);
                                if (0 == result)    // 帧数据正确
                                {
                                    // 时间间隔
                                    time_after = (recv_tv.tv_sec - p_can_buff->MVB_heart.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->MVB_heart.data[offset].send_tv.tv_usec);
                                    // 帧超时
                                    if (time_after < FRAME_TIMEOUT)
                                    {
										// MVB板卡正常
                                        g_mvb_status = MVB_OK;
                                    }
                                    else
                                    {
                                        //保存故障
										sprintf(str, "MVBHT:offset %d overtime %dms timeout",offset, time_after);
										save_log(1, __LINE__, str); 
                                    }

                                    // 数据变化保存
                                    result = memcmp(&mvb_heart_last[0], &recv_frame.data[3], recv_frame.can_dlc - 2 - 2);
                                    if (0 != result)
                                    {
                                        memcpy(&mvb_heart_last[0], &recv_frame.data[3], recv_frame.can_dlc - 2 - 2);

                                        // 保存文件
                                        result = save_frame(recv_frame, 1);
                                        if (-1 == result)
                                        {
                                            DEBUG_INFO();
                                            save_log(2, __LINE__, NULL);
                                            return -1;
                                        }
                                    }
                                }
                                else
                                {
                                    // 帧数据错误
                                    DEBUG_INFO();
                                    save_log(2, __LINE__, NULL);
                                    return -1;
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }


                    break;
                }

                case 6: // data
                {
                    // 设置MVB板卡错误计数
                    g_mvb_err_cnt[can_line] = 0;

                    // 帧长度
                    if (6 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到MVB回复帧
                                p_can_buff->MVB_data.data[offset].frame_recv_flag = 1;

                                // 回复帧与发送帧相同
                                result = memcmp(recv_frame.data, p_can_buff->MVB_data.data[offset].send_data, recv_frame.can_dlc);
                                if (0 == result)    // 帧数据正确
                                {
                                    // 时间间隔
                                    time_after = (recv_tv.tv_sec - p_can_buff->MVB_data.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->MVB_data.data[offset].send_tv.tv_usec);
                                    // 帧超时
                                    if (time_after < FRAME_TIMEOUT)
                                    {
										// MVB板卡正常
                                        g_mvb_status = MVB_OK;
                                    }
                                    else
                                    {
                                        //保存故障
										sprintf(str, "MVBDT:offset %d overtime %dms timeout",offset, time_after);
										save_log(1, __LINE__, str); 
                                    }

                                    // 数据变化保存
                                    result = memcmp(&mvb_data_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);
                                    if (0 != result)
                                    {
                                        memcpy(&mvb_data_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);

                                        // 保存文件
                                        result = save_frame(recv_frame, 1);
                                        if (-1 == result)
                                        {
                                            DEBUG_INFO();
                                            save_log(2, __LINE__, NULL);
                                            return -1;
                                        }
                                    }
                                }
                                else
                                {
                                    // 帧数据错误
                                    DEBUG_INFO();
                                    save_log(2, __LINE__, NULL);
                                    return -1;
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }

                    break;
                }

                //case 7: // 时间同步帧
				case 3:
                {
                    // 设置MVB板卡错误计数
                    g_mvb_err_cnt[can_line] = 0;

                    // 帧长度
                    if (8 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到MVB回复帧
                                p_can_buff->MVB_time.data[offset].frame_recv_flag = 1;

                                // 时间间隔
                                time_after = (recv_tv.tv_sec - p_can_buff->MVB_time.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->MVB_time.data[offset].send_tv.tv_usec);
                                // 帧超时
                                if (time_after < FRAME_TIMEOUT)
                                {
									// MVB板卡正常
                                    g_mvb_status = MVB_OK;
									// 设置系统时间
                                    result = set_system_time(recv_frame);
                                    if (-1 == result)
                                    {
                                        DEBUG_INFO();
                                        save_log(2, __LINE__, NULL);
                                        return -1;
                                    }
                                }
                                else
                                {
                                    //保存故障
									sprintf(str, "MVBTM:offset %d overtime %dms timeout",offset, time_after);
									save_log(1, __LINE__, str);
                                }

                                // 保存文件
                                result = save_frame(recv_frame, 1);
                                if (-1 == result)
                                {
                                    DEBUG_INFO();
                                    save_log(2, __LINE__, NULL);
                                    return -1;
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }

                    break;
                }
				
				
                default:
                {
                    // 帧类型错误
                    DEBUG_INFO();
                    // return -111;
                    break;
                }
            }
        }
        else
        {
            // 硬件地址错误
            debug_line("recv id=0x%04X,dlc=%d\r\n", recv_frame.can_id, recv_frame.can_dlc);
            DEBUG_INFO();
            // return -111;
        }
    }
    else
    {
        // 通讯方向错误
        //debug_line("recv id=0x%04X,dlc=%d\r\n", recv_frame.can_id, recv_frame.can_dlc);
        //DEBUG_INFO();
        // return -111;
    }

#ifdef PRINT_TIMEOUT
    // qjq test
    static int time_mvb_after_max = 0;

    if (time_after > time_mvb_after_max)
    {
        time_mvb_after_max = time_after;
        print_line("time_mvb_after_max=%dms\r\n\r\n", time_mvb_after_max);
    }
#endif

    return 0;
}

/*************************************************
 * @函数名称: can_recv_can
 * @函数功能: CAN CAN通讯帧处理
 * @输入参数: recv_frame    CAN接收帧
              can_line      CAN线路
              recv_tv       时间结构体
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_recv_can(struct can_frame recv_frame, T_can_buff *p_can_buff, uint8 can_line, struct timeval recv_tv)
{
    int     result = 0;
    uint16  offset = 0;     // 当前收帧位置
    int     time_after = 0; // 时间间隔
    T_extid can_id = {0};   // CAN 接收ExtID
    static uint8 can_status_last[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static uint8 dio_last[5][4] =
    {
        {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}, \
        {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}
    };
    static uint8 can_heart_last[2] = {0xFF, 0xFF};
    static uint8 can_data_last[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if ((NULL == p_can_buff) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    offset = recv_frame.data[0];    // 当前收帧位置

    can_id.all = recv_frame.can_id;
    // 通讯方向
    if (((0x03 == g_mcu_can_id[0].all) && (0x01 == can_id.direction)) || \
            ((0x04 == g_mcu_can_id[0].all) && (0x03 == can_id.direction)))
    {
        // 硬件地址
        if (0x01 == can_id.hard_addr)    // CAN板卡地址
        {
            // 帧类型
            switch (can_id.frame_type)  // 以下均是回复帧
            {
                case 0: // 起停帧
                {
                    // 设置CAN板卡错误计数
                    g_can_err_cnt[can_line] = 0;

                    // 帧长度
                    if (3 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到CAN回复帧
                                p_can_buff->CAN_run.data[offset].frame_recv_flag = 1;

                                // 回复帧与发送帧相同
                                result = memcmp(recv_frame.data, p_can_buff->CAN_run.data[offset].send_data, recv_frame.can_dlc);
                                if (0 == result)    // 帧数据正确
                                {
                                    // 时间间隔
                                    time_after = (recv_tv.tv_sec - p_can_buff->CAN_run.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->CAN_run.data[offset].send_tv.tv_usec);
                                    // 帧超时
                                    if (time_after < FRAME_TIMEOUT)
                                    {
                                        // 保存CAN板卡起停状态
                                        g_can_run = recv_frame.data[1]; // 0 停止, 1 启动
                                    }
                                    else
                                    {
                                        // 收帧超时
                                        debug_line("overtime = %dms\r\n", time_after);

                                        // qjq 超时处理
                                        // ...

                                        DEBUG_INFO();
                                        // return -111;
                                    }

                                    // 保存文件
                                    result = save_frame(recv_frame, 1);
                                    if (-1 == result)
                                    {
                                        DEBUG_INFO();
                                        save_log(2, __LINE__, NULL);
                                        return -1;
                                    }
                                }
                                else
                                {
                                    // 帧数据错误
                                    DEBUG_INFO();
                                    save_log(2, __LINE__, NULL);
                                    return -1;
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }

                    break;
                }

                case 2: // 状态请求帧
                {
                    // 设置CAN板卡错误计数
                    g_can_err_cnt[can_line] = 0;

                    // 帧长度
                    if (7 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到CAN回复帧
                                p_can_buff->CAN_status.data[offset].frame_recv_flag = 1;

                                // 时间间隔
                                time_after = (recv_tv.tv_sec - p_can_buff->CAN_status.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->CAN_status.data[offset].send_tv.tv_usec);
                                // 帧超时
                                if (time_after < FRAME_TIMEOUT)
                                {
                                    // qjq
                                    recv_frame.data[1]; // 硬件主版本号
                                    recv_frame.data[2]; // 硬件次版本号
                                    recv_frame.data[3]; // 软件主版本号
                                    recv_frame.data[4]; // 软件次版本号

                                    if (1 == (recv_frame.data[5] & 0x01))    // 当前板卡故障
                                    {
                                        // CAN板卡错误
                                        g_can_status = CAN_ERR;
                                    }
                                    else
                                    {
                                        // CAN板卡正常
                                        g_can_status = CAN_OK;
                                    }
                                }
                                else
                                {
                                    // 收帧超时
                                    debug_line("overtime = %dms\r\n", time_after);

                                    // qjq 超时处理
                                    // ...

                                    DEBUG_INFO();
                                    // return -111;
                                }

                                // 数据变化保存
                                result = memcmp(&can_status_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);
                                if (0 != result)
                                {
                                    memcpy(&can_status_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);

                                    // 保存文件
                                    result = save_frame(recv_frame, 1);
                                    if (-1 == result)
                                    {
                                        DEBUG_INFO();
                                        save_log(2, __LINE__, NULL);
                                        return -1;
                                    }
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }

                    break;
                }

                case 4: // 数据下发帧
                {
                    // 设置CAN板卡错误计数
                    g_can_err_cnt[can_line] = 0;

                    // 帧长度
                    if (6 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到CAN回复帧
                                p_can_buff->CAN_DIO.data[offset].frame_recv_flag = 1;

                                // 回复帧与发送帧相同
                                result = memcmp(recv_frame.data, p_can_buff->CAN_DIO.data[offset].send_data, recv_frame.can_dlc);
                                if (0 == result)    // 帧数据正确
                                {
                                    // 时间间隔
                                    time_after = (recv_tv.tv_sec - p_can_buff->CAN_DIO.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->CAN_DIO.data[offset].send_tv.tv_usec);
                                    // 帧超时
                                    if (time_after < FRAME_TIMEOUT)
                                    {
                                    }
                                    else
                                    {
                                        // 收帧超时
                                        debug_line("overtime = %dms\r\n", time_after);

                                        // qjq 超时处理
                                        // ...

                                        DEBUG_INFO();
                                        // return -111;
                                    }

                                    // 数据变化保存
                                    result = memcmp(&dio_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);
                                    if (0 != result)
                                    {
                                        memcpy(&dio_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);

                                        // 保存文件
                                        result = save_frame(recv_frame, 1);
                                        if (-1 == result)
                                        {
                                            DEBUG_INFO();
                                            save_log(2, __LINE__, NULL);
                                            return -1;
                                        }
                                    }
                                }
                                else
                                {
                                    // 帧数据错误
                                    DEBUG_INFO();
                                    save_log(2, __LINE__, NULL);
                                    return -1;
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }

                    break;
                }

                case 5: // heart
                {
                    // 设置CAN板卡错误计数
                    g_can_err_cnt[can_line] = 0;

                    // 帧长度
                    if (6 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到CAN回复帧
                                p_can_buff->CAN_heart.data[offset].frame_recv_flag = 1;

                                // 回复帧与发送帧相同
                                result = memcmp(recv_frame.data, p_can_buff->CAN_heart.data[offset].send_data, recv_frame.can_dlc);
                                if (0 == result)    // 帧数据正确
                                {
                                    // 时间间隔
                                    time_after = (recv_tv.tv_sec - p_can_buff->CAN_heart.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->CAN_heart.data[offset].send_tv.tv_usec);
                                    // 帧超时
                                    if (time_after < FRAME_TIMEOUT)
                                    {
                                    }
                                    else
                                    {
                                        // 收帧超时
                                        debug_line("overtime = %dms\r\n", time_after);

                                        // qjq 超时处理
                                        // ...

                                        DEBUG_INFO();
                                        // return -111;
                                    }

                                    // 数据变化保存
                                    result = memcmp(&can_heart_last[0], &recv_frame.data[3], recv_frame.can_dlc - 2 - 2);
                                    if (0 != result)
                                    {
                                        memcpy(&can_heart_last[0], &recv_frame.data[3], recv_frame.can_dlc - 2 - 2);

                                        // 保存文件
                                        result = save_frame(recv_frame, 1);
                                        if (-1 == result)
                                        {
                                            DEBUG_INFO();
                                            save_log(2, __LINE__, NULL);
                                            return -1;
                                        }
                                    }
                                }
                                else
                                {
                                    // 帧数据错误
                                    DEBUG_INFO();
                                    save_log(2, __LINE__, NULL);
                                    return -1;
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }


                    break;
                }

                case 6: // data
                {
                    // 设置CAN板卡错误计数
                    g_can_err_cnt[can_line] = 0;

                    // 帧长度
                    if (8 == recv_frame.can_dlc)
                    {
                        if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                        {
                            // CAN线路为信任路
                            if (CAN_LINE_TRUST == g_can_line_status[can_line])
                            {
                                // 收到CAN回复帧
                                p_can_buff->CAN_data.data[offset].frame_recv_flag = 1;

                                // 回复帧与发送帧相同
                                result = memcmp(recv_frame.data, p_can_buff->CAN_data.data[offset].send_data, recv_frame.can_dlc);
                                if (0 == result)    // 帧数据正确
                                {
                                    // 时间间隔
                                    time_after = (recv_tv.tv_sec - p_can_buff->CAN_data.data[offset].send_tv.tv_sec) * 1000 + (recv_tv.tv_usec - p_can_buff->CAN_data.data[offset].send_tv.tv_usec);
                                    // 帧超时
                                    if (time_after < FRAME_TIMEOUT)
                                    {
                                    }
                                    else
                                    {
                                        // 收帧超时
                                        debug_line("overtime = %dms\r\n", time_after);

                                        // qjq 超时处理
                                        // ...

                                        DEBUG_INFO();
                                        // return -111;
                                    }

                                    // 数据变化保存
                                    result = memcmp(&can_data_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);
                                    if (0 != result)
                                    {
                                        memcpy(&can_data_last[0], &recv_frame.data[1], recv_frame.can_dlc - 2);

                                        // 保存文件
                                        result = save_frame(recv_frame, 1);
                                        if (-1 == result)
                                        {
                                            DEBUG_INFO();
                                            save_log(2, __LINE__, NULL);
                                            return -1;
                                        }
                                    }
                                }
                                else
                                {
                                    // 帧数据错误
                                    DEBUG_INFO();
                                    save_log(2, __LINE__, NULL);
                                    return -1;
                                }
                            }
                        }
                        else
                        {
                            // 帧校验错误
                            debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                    }
                    else
                    {
                        // 帧长度错误
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }

                    break;
                }



                default:
                {
                    // 帧类型错误
                    DEBUG_INFO();
                    // return -111;
                    break;
                }
            }
        }
        else
        {
            // 硬件地址错误
            debug_line("recv id=0x%04X,dlc=%d\r\n", recv_frame.can_id, recv_frame.can_dlc);
            DEBUG_INFO();
            // return -111;
        }
    }
    else
    {
        // 通讯方向错误
        //debug_line("recv id=0x%04X,dlc=%d\r\n", recv_frame.can_id, recv_frame.can_dlc);
        //DEBUG_INFO();
        // return -111;
    }

#ifdef PRINT_TIMEOUT
    // qjq test
    static int time_can_after_max = 0;

    if (time_after > time_can_after_max)
    {
        time_can_after_max = time_after;
        print_line("time_can_after_max=%dms\r\n\r\n", time_can_after_max);
    }
#endif

    return 0;
}

/*************************************************
 * @函数名称: can_recv_frame
 * @函数功能: CAN收到帧处理
 * @输入参数: recv_frame    CAN接收帧
              can_line      CAN线路
              recv_tv       时间结构体
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_recv_frame(struct can_frame recv_frame, T_can_buff *p_can_buff, uint8 can_line, struct timeval recv_tv)
{
    int result = 0;
    T_extid can_id = {0};   // CAN 接收ExtID

    if ((NULL == p_can_buff) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    can_id.all = recv_frame.can_id;
    // 通讯类型
    switch (can_id.hard_addr)
    {
        // MCU
        case 0x03:
        case 0x04:
        {
			result = can_recv_mcu(recv_frame, p_can_buff, can_line);
            if (-1 == result)
            {
                // 错误处理
                DEBUG_INFO();
                save_log(2, __LINE__, NULL);
                return -1;
            }

            break;
        }

        // 接收20块DIO数据
        case 0x05:
        case 0x06:
        case 0x07:
        case 0x08:
        case 0x09:
        case 0x0A:
        case 0x0B:
        case 0x0C:
        case 0x0D:
        case 0x0E:
        case 0x25:
        case 0x26:
        case 0x27:
        case 0x28:
        case 0x29:
        case 0x2A:
        case 0x2B:
        case 0x2C:
        case 0x2D:
        case 0x2E:
        {
            /* MCU主设备 */
            //if (MCU_MASTER == g_mcu_status)
            {
                result = can_recv_dio(recv_frame, p_can_buff, can_line, recv_tv);
                if (-1 == result)
                {
                    // 错误处理
                    DEBUG_INFO();
                    save_log(2, __LINE__, NULL);
                    return -1;
                }
            }

            break;
        }

        // MVB
        case 0x01:
        {
            /* MCU主设备 */
            //if (MCU_MASTER == g_mcu_status)
            {
                result = can_recv_mvb(recv_frame, p_can_buff, can_line, recv_tv);
                if (-1 == result)
                {
                    // 错误处理
                    DEBUG_INFO();
                    save_log(2, __LINE__, NULL);
                    return -1;
                }
            }

            break;
        }

        // CAN
        case 0x02:
        {
            /* MCU主设备 */
            //if (MCU_MASTER == g_mcu_status)
            {
                result = can_recv_can(recv_frame, p_can_buff, can_line, recv_tv);
                if (-1 == result)
                {
                    // 错误处理
                    DEBUG_INFO();
                    save_log(2, __LINE__, NULL);
                    return -1;
                }
            }

            break;
        }

        case 0x00:
        {
            if ((0x0400 == can_id.all) && (0x5A == recv_frame.data[0]) && (4 == recv_frame.can_dlc))
            {
                /* LCU停止帧 */
                // 宕机
                char str[64];
                sprintf(str, "can%d recv card [0x%02X] line [%04d]", can_line, recv_frame.data[1], ((uint16)(recv_frame.data[2]) << 8) | (recv_frame.data[3]));
                downtime(__LINE__, str);
            }

            //接收手动强制切换帧
            else if (0x0100 == can_id.all)
            {
				/* LCU强制切换帧 */
                //帧长度
                if (1 == recv_frame.can_dlc)
                {
                    //if (recv_frame.data[recv_frame.can_dlc - 1] == calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1))   // 帧校验正确
                    if (0 == g_mcu_stay_status)
					{
						if (0 == g_mcu_status_stop)
						{
							if (1 == g_mcu_status_other)	
							{
								//当前为信任CAN线路
								if (CAN_LINE_TRUST == g_can_line_status[can_line])
								{
									//主备模式
									g_system_forcibly = recv_frame.data[0];   // 强制位
									 
									//自动   00 96
									if (FORCIBLY_AUTO == g_system_forcibly)
									{
										//do nothing
										/***add by 1025***/
										if (1 == g_system_date)
										{
											if (0x03 == g_mcu_can_id[0].all)
											{
												g_mcu_status = MCU_BACKUP;
											}
											else
											{
												g_mcu_status = MCU_MASTER;
											}
										}
										else if (0 == g_system_date)
										{
											if (0x03 == g_mcu_can_id[0].all)
											{
												g_mcu_status = MCU_MASTER;
											}
											else
											{
												g_mcu_status = MCU_BACKUP;
											}
										}
										/***add by 1025***/

										//debug_line("forcibly = 0x%02X\r\n", g_system_forcibly);
									}
									//强制A  5A D7
									else if (FORCIBLY_A == g_system_forcibly)
									{
										//debug_line("forcibly = 0x%02X\r\n", g_system_forcibly);

										if (0x03 == g_mcu_can_id[0].all)
										{
											g_mcu_status = MCU_MASTER;
										}
										else
										{
											g_mcu_status = MCU_BACKUP;
										}
									}
									//强制B  A5 6C
									else if (FORCIBLY_B == g_system_forcibly)
									{
										//debug_line("forcibly = 0x%02X\r\n", g_system_forcibly);

										if (0x03 == g_mcu_can_id[0].all)
										{
											g_mcu_status = MCU_BACKUP;
										}
										else
										{
											g_mcu_status = MCU_MASTER;
										}
									}
									//错误指令
									else
									{
										debug_line("forcibly = 0x%02X\r\n", g_system_forcibly);

										g_system_forcibly = FORCIBLY_AUTO;
									}
								}
							}
							else if (0 == g_mcu_status_other)
							{
								if (MCU_MASTER != g_mcu_status)
								{
									g_mcu_status = MCU_MASTER;
								}
							}
						}
						//保存存活系永远为主
						else if (10 == g_mcu_status_stop)
						{
							g_mcu_status = MCU_MASTER;
							//g_system_forcibly = FORCIBLY_AUTO;
							g_system_forcibly = recv_frame.data[0];
							// if (MCU_MASTER != g_mcu_status)
							// {
								// g_mcu_status = MCU_MASTER;
							// }
							//debug_line("g_mcu_status_other=0x%04X\r\n", g_mcu_status_other);
							//帧校验错误
							//debug_line("crc_err=0x%02X, crc=0x%02X\r\n", recv_frame.data[recv_frame.can_dlc - 1], calc_crc16_8(recv_frame.data, recv_frame.can_dlc - 1));

							//停止所有设备
							//can_send_all_stop(__LINE__);

							//宕机
							//downtime(__LINE__, "crc");
						}
					}
					//禁止重新上电系对其DIO状态进行切换
					else if (3 == g_mcu_stay_status)
					{
						g_mcu_status = MCU_BACKUP;
						g_system_forcibly = 0x01;
					}
                }
            }

            break;
        }

        default:
        {
            // 板卡地址错误
            debug_line("can%d recv id=0x%04X,dlc=%d,data=0x%02X %02X %02X %02X %02X %02X %02X %02X\r\n", can_line, recv_frame.can_id, recv_frame.can_dlc, \
                       recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);
            DEBUG_INFO();
            break;
        }
    }

    return 0;
}

/*
#define CAN_EFF_FLAG 0x80000000U // 扩展帧的标识
#define CAN_RTR_FLAG 0x40000000U // 远程帧的标识
#define CAN_ERR_FLAG 0x20000000U // 错误帧的标识，用于错误检查
*/
/*************************************************
 * @函数名称: can_recv
 * @函数功能: CAN收帧任务
 * @输入参数: p_can_fd      文件描述符
              can_line      CAN线路
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_recv(int can_fd, T_can_buff *p_can_buff, uint8 can_line)
{
    int     result = 0;
    struct timeval recv_tv = {0, 0};
    struct can_frame recv_frame = {0, 0, {0}};  // CAN收到的数据

    if ((can_fd < 0) || (NULL == p_can_buff) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // 数据变化立即刷新plc数据
    result = memcmp(g_plc_data_last, g_plc_data, 2 * DIO_CNT);
    if (0 != result)
    {
        // PLC运行应用程序
        result = plc_run();
        if (-1 == result)
        {
            downtime(__LINE__, "plc_run");
        }
        memcpy(g_plc_data_last, g_plc_data, 2 * DIO_CNT);
    }

    // 当前CAN线路故障
    if (CAN_LINE_ERR == g_can_line_status[can_line])
    {
        return 0;
    }

    // 获取收帧时间戳
    result = gettimeofday(&recv_tv, NULL);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    recv_tv.tv_sec -= 28800; // UTC time
    recv_tv.tv_usec /= 1000; // us转为ms

    /* CAN接收 */
    while (1)
    {
        memset(&recv_frame, 0, sizeof(struct can_frame));
        result = read(can_fd, &recv_frame, sizeof(struct can_frame));
        if (-1 == result)   // CAN总线非阻塞无数据直接返回
        {
            // perror("can read none!");
            // DEBUG_INFO();
            break;
        }
        else if (0 == result)   // 当前无数据
        {
            // DEBUG_INFO();
            break;
        }
        else    // 接收到数据
        {
            // 帧初步筛选
            if (CAN_EFF_FLAG != recv_frame.can_id)  // 收到正确标准帧
            {
                // 当前为非故障CAN线路
                // CAN收到帧处理
                result = can_recv_frame(recv_frame, p_can_buff, can_line, recv_tv);
                if (-1 == result)
                {
                    DEBUG_INFO();
                    result = save_frame(recv_frame, 2);
                    if (-1 == result)
                    {
                        DEBUG_INFO();
                        save_log(2, __LINE__, NULL);
                        return -1;
                    }

                    save_log(2, __LINE__, NULL);
                    return -1;
                }
            }
            else
            {
                if (recv_frame.can_id & CAN_ERR_FLAG)   // 硬件收帧错误
                {
                    // 当前CAN线路故障
                    // qjq 收到错误帧是否应该判断线路错误?
                    g_can_line_status[can_line] = CAN_LINE_ERR;
                    // CAN线路状态切换
                    check_can_line_status();

                    debug_line("\e[31mhardware: CAN%d recv frame err!\e[0m\r\n",  can_line);
                }
                else
                {
                    DEBUG_INFO();
                }
            }
        }
    }


    return 0;
}

/*************************************************
 * @函数名称: can_recv_mcu_RTCtime
 * @函数功能: CAN收帧任务
 * @输入参数: p_can_fd      文件描述符
              can_line      CAN线路
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_recv_mcu_RTCtime(int can_fd, T_can_buff *p_can_buff, uint8 can_line)
{
    int     result = 0;
    struct timeval recv_tv = {0, 0};
    struct can_frame recv_frame = {0, 0, {0}};  // CAN收到的数据
	
	int recv_cnt = 0;

    if ((can_fd < 0) || (NULL == p_can_buff) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    /* CAN接收 */
    while (1)
    {
        memset(&recv_frame, 0, sizeof(struct can_frame));
        result = read(can_fd, &recv_frame, sizeof(struct can_frame));
        if (-1 == result)   // CAN总线非阻塞无数据直接返回
        {
           //DEBUG_INFO();
		   recv_cnt++;
		   usleep(500000);
		   if (recv_cnt > 10)
		   {
			   return -1;
		   }
		   
		   
        }
        else if (0 == result)   // 当前无数据
        {
            //DEBUG_INFO();
			if (6 == g_reset_cnt)
			{
				g_reset_cnt = 0;
				return 0;
			}
        }
        else    // 接收到数据
        {
            //DEBUG_INFO();
			// 帧初步筛选
            if (CAN_EFF_FLAG != recv_frame.can_id)  // 收到正确标准帧
            {
				DEBUG_INFO();
				// 当前为非故障CAN线路
                // CAN收到帧处理
                result = can_recv_mcu_time(recv_frame, p_can_buff, can_line);
                if (-1 == result)
                {
					if (6 == g_reset_cnt)
					{
						//DEBUG_INFO();
						g_reset_cnt = 0;
						return 0;
					}
                }
				else if (0 == result)
				{
					//DEBUG_INFO();
					return 0;
					//break;
				}
            }
            else
            {
				DEBUG_INFO();
				return -1;                
            }
        }
    }
    return 0;
}

/*************************************************
 * @函数名称: can_send_frame
 * @函数功能: CAN发送帧 (先接收帧)
 * @输入参数: send_frame    CAN发送帧
              can_fd        文件描述符
              can_line      CAN线路
 * @输出参数: p_can_buff    CAN帧接收缓冲区
              p_can_data    保存CAN帧数据
 * @返回值  : int  0 成功
                  -1 失败
                   1 CAN故障未发送
 * @其它说明: 无
 *************************************************/
int can_send_frame(struct can_frame send_frame, int can_fd, T_can_buff *p_can_buff, T_can_data *p_can_data, uint8 can_line)
{
    int     result = 0;
    uint8   timeout_cnt = 0;    // 超时次数
    struct timeval tv = {0, 0};
	char str[64] = {0};

    if ((can_fd < 0) || (NULL == p_can_buff) || (NULL == p_can_data) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    /* CAN接收 */
    result = can_recv(can_fd, p_can_buff, can_line);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // 当前CAN线路故障
    if (CAN_LINE_ERR == g_can_line_status[can_line])
    {
        return 1;
    }

    while (sizeof(struct can_frame) != write(can_fd, &send_frame, sizeof(struct can_frame)))
    {
        timeout_cnt++;
#ifdef PRINT_TIMEOUT
        if (timeout_cnt >= 20)
        {
            print_line("timeout_cnt=%d\r\n", timeout_cnt);
        }
#endif

        if (timeout_cnt >= 60)  // 6ms
        {
            //保存故障
			sprintf(str, "canid 0x%04X can %d CAN_LINE_ERR", send_frame.can_id, can_line);
			save_log(1, __LINE__, str);

            // 当前CAN线路故障
            g_can_line_status[can_line] = CAN_LINE_ERR;
            // CAN线路状态切换
            check_can_line_status();

            return 1;
            // break;
        }
        usleep(100);
    }
    //判断当前CAN线路是否为信任线路
    if (CAN_LINE_TRUST == g_can_line_status[can_line])
    {
        // 获取发帧时间戳
        result = gettimeofday(&tv, NULL);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
        tv.tv_sec -= 28800; // UTC time
        tv.tv_usec /= 1000; // us转为ms

        // 保存发送的帧
        memcpy(p_can_data->send_data, send_frame.data, 8);
        // CAN发帧时间
        p_can_data->send_tv = tv;
        // CAN帧已发
        p_can_data->frame_send_flag = 1;
        // 发送时清空收帧标志
        p_can_data->frame_recv_flag = 0;
    }

#ifdef PRINT_TIMEOUT
    // 当前为信任CAN线路
    if (CAN_LINE_TRUST == g_can_line_status[can_line])
    {
        static int timeout_cnt_max = 0;

        if (timeout_cnt > timeout_cnt_max)
        {
            timeout_cnt_max = timeout_cnt;
            print_line("timeout_cnt_max=%d\r\n\r\n", timeout_cnt_max);
        }
    }
#endif

    return 0;
}

/*************************************************
 * @函数名称: can_send_frame_time
 * @函数功能: CAN发送帧 (先接收帧)
 * @输入参数: send_frame    CAN发送帧
              can_fd        文件描述符
              can_line      CAN线路
 * @输出参数: p_can_buff    CAN帧接收缓冲区
              p_can_data    保存CAN帧数据
 * @返回值  : int  0 成功
                  -1 失败
                   1 CAN故障未发送
 * @其它说明: 无
 *************************************************/
int can_send_frame_time(struct can_frame send_frame, int can_fd, uint8 can_line)
{
    int     result = 0;
    uint8   timeout_cnt = 0;    // 超时次数
    struct timeval tv = {0, 0};
	char str[64] = {0};

    if ((can_fd < 0) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    // 当前CAN线路故障
    if (CAN_LINE_ERR == g_can_line_status[can_line])
    {
        return 1;
    }

    while (sizeof(struct can_frame) != write(can_fd, &send_frame, sizeof(struct can_frame)))
    {
        timeout_cnt++;

        if (timeout_cnt >= 60)  // 6ms
        {
            //保存故障
			sprintf(str, "canid 0x%04X can %d CAN_LINE_ERR", send_frame.can_id, can_line);
			save_log(1, __LINE__, str);

            // 当前CAN线路故障
            g_can_line_status[can_line] = CAN_LINE_ERR;
            // CAN线路状态切换
            check_can_line_status();

            return 1;
            // break;
        }
        usleep(100);
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_frame_err_state
 * @函数功能: CAN发送帧 (先接收帧) 不保存发送的帧
 * @输入参数: send_frame    CAN发送帧
              can_fd        文件描述符
              can_line      CAN线路
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
                   1 CAN故障未发送
 * @其它说明: 无
 *************************************************/
int can_send_frame_err_state(struct can_frame send_frame, int can_fd, T_can_buff *p_can_buff, T_can_data *p_can_data, uint8 can_line)
{
    int     result = 0;
    uint8   timeout_cnt = 0;    // 超时次数
    struct timeval tv = {0, 0};
	char str[64] = {0};

    if ((can_fd < 0) || (NULL == p_can_buff) || (can_line >= 2))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    /* CAN接收 */
    result = can_recv(can_fd, p_can_buff, can_line);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // 当前CAN线路故障
    if (CAN_LINE_ERR == g_can_line_status[can_line])
    {
        return 1;
    }

    while (sizeof(struct can_frame) != write(can_fd, &send_frame, sizeof(struct can_frame)))
    {
        timeout_cnt++;
#ifdef PRINT_TIMEOUT
        if (timeout_cnt >= 20)
        {
            print_line("timeout_cnt=%d\r\n", timeout_cnt);
        }
#endif

        if (timeout_cnt >= 60)  // 6ms
        {			
			//保存故障
			sprintf(str, "canid 0x%04X can %d CAN_LINE_ERR", send_frame.can_id, can_line);
			save_log(1, __LINE__, str);

            // 当前CAN线路故障
            g_can_line_status[can_line] = CAN_LINE_ERR;
            // CAN线路状态切换
            check_can_line_status();

            return 1;
            // break;
        }
        usleep(100);
    }
	
	if (CAN_LINE_TRUST == g_can_line_status[can_line])  //add by 3.8
    {

        // 保存发送的帧
        memcpy(p_can_data->send_data, send_frame.data, 8);
        // CAN帧已发
        p_can_data->frame_send_flag = 1;
        // 发送时清空收帧标志
        p_can_data->frame_recv_flag = 0;
    }

#ifdef PRINT_TIMEOUT
    // 当前为信任CAN线路
    if (CAN_LINE_TRUST == g_can_line_status[can_line])
    {
        static int timeout_cnt_max = 0;

        if (timeout_cnt > timeout_cnt_max)
        {
            timeout_cnt_max = timeout_cnt;
            print_line("timeout_cnt_max=%d\r\n\r\n", timeout_cnt_max);
        }
    }
#endif

    return 0;
}

/*************************************************
 * @函数名称: can_send_mcu_heart
 * @函数功能: 发送MCU心跳
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
              mcu_heart     MCU心跳
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_mcu_heart(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff, uint16 mcu_heart)
{
    static uint8 card_group = 0;                // 板卡编组 分时发送DI数据

    int result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    /* CAN发送 */
    can_id.all = g_mcu_can_id[0].all;

    // debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 8;
    send_frame.data[0] = offset;
    send_frame.data[1] = mcu_heart & 0xFF;        //取高八位
    send_frame.data[2] = (mcu_heart >> 8) & 0xFF; //取低八位
    //send_frame.data[3] = g_mcu_status;
	
	//当运行正常时正常发送主备状态，当主系掉电又上电后，
	//原存活系给又上电系发送2告知对方不能再进入主备切换分支只能为备
	if (0 == g_mcu_status_stop)
	{
		send_frame.data[3] = g_mcu_status;   //add by 11.25
	}
	else if (10 == g_mcu_status_stop)
	{
		send_frame.data[3] = 2;
		g_mcu_id_save = get_mcu_Id();
		
	}

    // 分时发送DI数据，循环依次向各板卡发送数据
    send_frame.data[4] = card_group;    // IO板号
    send_frame.data[5] = g_plc_data[2 * card_group];        // DI数据
    send_frame.data[6] = g_plc_data[2 * card_group + 1];    // DI数据
    card_group++;
    if (card_group >= DIO_CNT / 2)
    {
        card_group = 0;
    }
    send_frame.data[7] = calc_crc16_8(send_frame.data, 7);  // 计算crc校验

    //CAN1总线发送数据
    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->MCU_status.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    //CAN2总线发送数据
    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->MCU_status.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    // 保存文件
    result = save_frame(send_frame, 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_mcu_time
 * @函数功能: 发送MCU的RTC时间
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
              time     MCU获取当前RTC时间
			  hard_id   本地硬件地址
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_mcu_time(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff, int year, int mon, int day, int hour, int min, int sec, uint8 hard_id)
{
    static uint8 card_group = 0;                // 板卡编组 分时发送DI数据

    int result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


	/* CAN发送 */
    can_id.all = 0;

    can_id.frame_type = 0x01;           // 帧类型 
    can_id.hard_addr = g_hard_id;       // 本板卡地址
    //CAN发送
    //can_id.all = g_hard_id;

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 7;
    send_frame.data[0] = year - 100;
    send_frame.data[1] = 1 + mon;        
    send_frame.data[2] = day; 
	send_frame.data[3] = hour; 
	send_frame.data[4] = min;
	send_frame.data[5] = sec; 	
	
    send_frame.data[6] = hard_id;  // 本地硬件地址
	

    //CAN1总线发送数据
    result = can_send_frame_time(send_frame, p_can_fd[0], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    //CAN2总线发送数据
    result = can_send_frame_time(send_frame, p_can_fd[1],  1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_dio_status
 * @函数功能: 发送状态请求
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
              card_num      DIO板卡号
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_dio_status(int *p_can_fd, uint16 offset, uint8 card_num, T_can_buff **p_can_buff)
{
    static uint8 heart[DIO_CNT] = {0};
    int     result = 0;
    int     i = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 save_frame_flag[DIO_CNT] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
                                             0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
                                            };

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (card_num >= DIO_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    // 读取DIO板卡状态
    //if ((DIO_ERR == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
    //{
    //    return 0;
    //}

    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x02;           // 帧类型 状态请求帧
    if (card_num <= 9)
    {
        can_id.hard_addr = card_num + 5;     // 0-9 DIO
    }
    else if (card_num > 9)
    {
        can_id.hard_addr = card_num + 0x20 - 5;     // 10-19 DIO
    }

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 5;
    send_frame.data[0] = offset;
    if (0 == heart[card_num])
    {
        heart[card_num] = 10;
    }
    if (0x03 == g_mcu_can_id[0].all)
    {
        send_frame.data[1] = heart[card_num];
        send_frame.data[2] = 0;
    }
    else if (0x04 == g_mcu_can_id[0].all)
    {
        send_frame.data[1] = 0;
        send_frame.data[2] = heart[card_num];
    }
    heart[card_num]++;
    if (DIO_TRUST == g_dio_status[card_num])
    {
        send_frame.data[3] = 1;     // 主备   0 备, 1 主
    }
    // else if (DIO_ERR == g_dio_status[card_num])
    // {
    // send_frame.data[3] = 2;     // 主备   0 备, 1 主, 2 Down
    // }
    else
    {
        send_frame.data[3] = 0;     // 主备   0 备, 1 主
    }
    send_frame.data[4] = calc_crc16_8(send_frame.data, 4);  // 计算crc校验
	//DIO状态正常
	if ((DIO_OK == g_dio_status[card_num]) || (DIO_TRUST == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
	{
		result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->DIO_status[card_num].data[offset], 0);
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		else if (0 == result)
		{
			// DIO板卡CAN0错误计数++
			g_dio_err_cnt[0][card_num]++;
		}

		result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->DIO_status[card_num].data[offset], 1);
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		else if (0 == result)
		{
			// DIO板卡CAN1错误计数++
			g_dio_err_cnt[1][card_num]++;
		}
	}
	//DIO板卡异常
	//else if ((DIO_ERR == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
	else if (DIO_FAKE_ERR == g_dio_status[card_num])
	{
		if((card_num >= 0) && (card_num < 10))  //add by 3.8
		{
			result = can_send_frame_err_state(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->DIO_status[card_num].data[offset], 0);
			if (-1 == result)
			{
				DEBUG_INFO();
				save_log(2, __LINE__, NULL);
				return -1;
			}
			else if (0 == result)
			{
				// DIO板卡CAN0错误计数++
				g_dio_err_cnt[0][card_num]++;
			}

			result = can_send_frame_err_state(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->DIO_status[card_num].data[offset], 1);
			if (-1 == result)
			{
				DEBUG_INFO();
				save_log(2, __LINE__, NULL);
				return -1;
			}
			else if (0 == result)
			{
				// DIO板卡CAN1错误计数++
				g_dio_err_cnt[1][card_num]++;
			}
		}
	}

    // 数据变化保存
    if (save_frame_flag[card_num])
    {
        save_frame_flag[card_num] = 0;

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_dio_di
 * @函数功能: 发送DI数据请求
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
              card_num      DIO板卡号
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_dio_di(int *p_can_fd, uint16 offset, uint8 card_num, T_can_buff **p_can_buff)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 save_frame_flag[DIO_CNT] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
                                             0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
                                            };

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (card_num >= DIO_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // DIO板卡状态
    if (DIO_TRUST != g_dio_status[card_num])
    {
        return 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x03;           // 帧类型 数据请求帧
    if (card_num <= 9)
    {
        can_id.hard_addr = card_num + 5;     // 0-9 DIO
    }
    else if (card_num > 9)
    {
        can_id.hard_addr = card_num + 0x20 - 5;     // 10-19 DIO
    }

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 2;
    send_frame.data[0] = offset;
    send_frame.data[1] = calc_crc16_8(send_frame.data, 1);  // 计算crc校验

	//DIO状态正常
	if ((DIO_OK == g_dio_status[card_num]) || (DIO_TRUST == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
	{
		result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->DIO_DI[card_num].data[offset], 0);			 
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		else if (0 == result)
		{
			// DIO板卡CAN0错误计数++
			g_dio_err_cnt[0][card_num]++;
		}

		result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->DIO_DI[card_num].data[offset], 1);
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		else if (0 == result)
		{
			// DIO板卡CAN1错误计数++
			g_dio_err_cnt[1][card_num]++;
		}
	}
	//DIO板卡异常
	//else if ((DIO_ERR == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
	else if (DIO_FAKE_ERR == g_dio_status[card_num])
	{
		if((card_num >= 0) && (card_num < 10))   //add by 3.8
		{
			result = can_send_frame_err_state(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->DIO_DI[card_num].data[offset], 0);
			if (-1 == result)
			{
				DEBUG_INFO();
				save_log(2, __LINE__, NULL);
				return -1;
			}
			else if (0 == result)
			{
				// DIO板卡CAN0错误计数++
				g_dio_err_cnt[0][card_num]++;
			}

			result = can_send_frame_err_state(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->DIO_DI[card_num].data[offset], 1);
			if (-1 == result)
			{
				DEBUG_INFO();
				save_log(2, __LINE__, NULL);
				return -1;
			}
			else if (0 == result)
			{
				// DIO板卡CAN1错误计数++
				g_dio_err_cnt[1][card_num]++;
			}
		}
	}

    // 数据变化保存
    if (save_frame_flag[card_num])
    {
        save_frame_flag[card_num] = 0;

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_dio_qz
 * @函数功能: 发送DI数据请求
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
              card_num      DIO板卡号
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_dio_qz(int *p_can_fd, uint16 offset, uint8 card_num, T_can_buff **p_can_buff)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 save_frame_flag[DIO_CNT] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
                                             0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
                                            };

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (card_num >= DIO_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // DIO板卡状态
    // if (DIO_TRUST != g_dio_status[card_num])
    // {
        // return 0;
    // }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x01;           // 帧类型 数据请求帧
    if (card_num <= 9)
    {
        can_id.hard_addr = card_num + 5;     // 0-9 DIO
    }
    else if (card_num > 9)
    {
        can_id.hard_addr = card_num + 0x20 - 5;     // 10-19 DIO
    }

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 2;
    send_frame.data[0] = offset;
    send_frame.data[1] = calc_crc16_8(send_frame.data, 1);  // 计算crc校验

	//DIO状态正常
	if ((DIO_OK == g_dio_status[card_num]) || (DIO_TRUST == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
	{
		result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->DIO_QZ[card_num].data[offset], 0);			 
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		else if (0 == result)
		{
			// DIO板卡CAN0错误计数++
			g_dio_err_cnt[0][card_num]++;
		}

		result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->DIO_QZ[card_num].data[offset], 1);
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		else if (0 == result)
		{
			// DIO板卡CAN1错误计数++
			g_dio_err_cnt[1][card_num]++;
		}
	}
#if 0
	//DIO板卡异常
	//else if ((DIO_ERR == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
	else if (DIO_FAKE_ERR == g_dio_status[card_num])
	{
		result = can_send_frame_err_state(send_frame, p_can_fd[0], p_can_buff[0], 0);
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		else if (0 == result)
		{
			// DIO板卡CAN0错误计数++
			g_dio_err_cnt[0][card_num]++;
		}

		result = can_send_frame_err_state(send_frame, p_can_fd[1], p_can_buff[1], 1);
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		else if (0 == result)
		{
			// DIO板卡CAN1错误计数++
			g_dio_err_cnt[1][card_num]++;
		}
	}
#endif
    // 数据变化保存
    if (save_frame_flag[card_num])
    {
        save_frame_flag[card_num] = 0;

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_dio_do
 * @函数功能: 发送DO数据
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
              card_num      DIO板卡号
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_dio_do(int *p_can_fd, uint16 offset, uint8 card_num, T_can_buff **p_can_buff)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 do_data_last[DIO_CNT][2] =
    {
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, \
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, \
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, \
        {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}
    };

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (card_num >= DIO_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // DIO板卡状态
    // if (DIO_TRUST != g_dio_status[card_num]) // 单驱
    if ((DIO_TRUST != g_dio_status[card_num]) && (DIO_OK != g_dio_status[card_num]))    // 并驱
    {
        return 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x04;           // 帧类型 数据下发帧
    if (card_num <= 9)
    {
        can_id.hard_addr = card_num + 5;     // 0-9 DIO
    }
    else if (card_num > 9)
    {
        can_id.hard_addr = card_num + 0x20 - 5;     // 10-19 DIO
    }

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 4;
    send_frame.data[0] = offset;
    send_frame.data[1] = g_plc_data[2 * (card_num / 2) + DIO_CNT];
    send_frame.data[2] = g_plc_data[2 * (card_num / 2) + DIO_CNT + 1];
    send_frame.data[3] = calc_crc16_8(send_frame.data, 3);  // 计算crc校验

	//DIO状态正常
	if ((DIO_OK == g_dio_status[card_num]) || (DIO_TRUST == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
	{
		result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->DIO_DO[card_num].data[offset], 0);
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		else if (0 == result)
		{
			// DIO板卡CAN0错误计数++
			g_dio_err_cnt[0][card_num]++;
		}

		result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->DIO_DO[card_num].data[offset], 1);
		if (-1 == result)
		{
			DEBUG_INFO();
			save_log(2, __LINE__, NULL);
			return -1;
		}
		else if (0 == result)
		{
			// DIO板卡CAN1错误计数++
			g_dio_err_cnt[1][card_num]++;
		}
	}
	//DIO板卡异常
	//else if ((DIO_ERR == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
	else if (DIO_FAKE_ERR == g_dio_status[card_num])
	{
		if((card_num >= 0) && (card_num < 10))
		{
			result = can_send_frame_err_state(send_frame, p_can_fd[0], &p_can_buff[0]->DIO_DO[card_num].data[offset],p_can_buff[0], 0);
			if (-1 == result)
			{
				DEBUG_INFO();
				save_log(2, __LINE__, NULL);
				return -1;
			}
			else if (0 == result)
			{
				// DIO板卡CAN0错误计数++
				g_dio_err_cnt[0][card_num]++;
			}

			result = can_send_frame_err_state(send_frame, p_can_fd[1], &p_can_buff[1]->DIO_DO[card_num].data[offset],p_can_buff[1], 1);
			if (-1 == result)
			{
				DEBUG_INFO();
				save_log(2, __LINE__, NULL);
				return -1;
			}
			else if (0 == result)
			{
				// DIO板卡CAN1错误计数++
				g_dio_err_cnt[1][card_num]++;
			}
		}
	}

    // 数据变化保存
    result = memcmp(&do_data_last[card_num][0], &send_frame.data[1], send_frame.can_dlc - 2);
    if (0 != result)
    {
        memcpy(&do_data_last[card_num][0], &send_frame.data[1], send_frame.can_dlc - 2);

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_mvb_status
 * @函数功能: 发送状态请求
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_mvb_status(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 save_frame_flag = 0xFF;

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // MVB板卡状态
    // if (MVB_ERR == g_mvb_status)
    // {
        // return 0;
    // }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x02;           // 帧类型 状态请求帧
    can_id.hard_addr = 0x01;             // MVB板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 2;
    send_frame.data[0] = offset;
    send_frame.data[1] = calc_crc16_8(send_frame.data, 1);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->MVB_status.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN0错误计数++
        g_mvb_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->MVB_status.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN1错误计数++
        g_mvb_err_cnt[1]++;
    }


    // 数据变化保存
    if (save_frame_flag)
    {
        save_frame_flag = 0;

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_mvb_run
 * @函数功能: 发送起停帧
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_mvb_run(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // MVB板卡状态
    if (MVB_DEFAULT == g_mvb_status)
    {
        return 0;
    }

    // MVB板卡状态 & MVB起停状态
    if (((MVB_OK == g_mvb_status) && (MVB_RUN == g_mvb_run)) || ((MVB_ERR == g_mvb_status) && (MVB_STOP == g_mvb_run)))
    {
        return 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x00;           // 帧类型 起停帧
    can_id.hard_addr = 0x01;             // MVB板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 3;
    send_frame.data[0] = offset;
    send_frame.data[1] = !g_mvb_run;                // 起停   0 停止, 1 启动
    send_frame.data[2] = calc_crc16_8(send_frame.data, 2);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->MVB_run.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN0错误计数++
        g_mvb_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->MVB_run.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN1错误计数++
        g_mvb_err_cnt[1]++;
    }


    // 保存文件
    result = save_frame(send_frame, 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_mvb_dio
 * @函数功能: 发送MVB DIO数据
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
              card_num      DIO板卡号
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_mvb_dio(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff)
{
    static uint8 card_group = 0;                // 板卡编组 分时发送DI数据

    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 dio_data_last[5][4] =
    {
        {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}, \
        {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}
    };

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // MVB板卡状态
    // if (MVB_ERR == g_mvb_status)
    // {
        // return 0;
    // }

    // MVB板卡起停状态
    // if (MVB_RUN != g_mvb_run) //
        // return 0;
    // }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x04;           // 帧类型 数据下发帧   
    can_id.hard_addr = 0x01;             // MVB板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 7;
    send_frame.data[0] = offset;

    //分时发送DI数据
    send_frame.data[1] = card_group;
    send_frame.data[2] = g_plc_data[2 * card_group];
    send_frame.data[3] = g_plc_data[2 * card_group + 1];
    send_frame.data[4] = g_plc_data[2 * card_group + DIO_CNT];
    send_frame.data[5] = g_plc_data[2 * card_group + DIO_CNT + 1];

    card_group++;
    if (card_group >= DIO_CNT / 2)
    {
        card_group = 0;
    }
    send_frame.data[6] = calc_crc16_8(send_frame.data, 6);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->MVB_DIO.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN0错误计数++
        g_mvb_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->MVB_DIO.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN1错误计数++
        g_mvb_err_cnt[1]++;
    }


    // 数据变化保存
    result = memcmp(&dio_data_last[card_group][0], &send_frame.data[1], send_frame.can_dlc - 2);
    if (0 != result)
    {
        memcpy(&dio_data_last[card_group][0], &send_frame.data[1], send_frame.can_dlc - 2);

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_mvb_heart
 * @函数功能: 发送MVB 心跳
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
              mvb_heart     MVB心跳
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_mvb_heart(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff, uint16 mvb_heart)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 save_frame_flag = 0xFF;

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // MVB板卡状态
    // if (MVB_ERR == g_mvb_status)
    // {
        // return 0;
    // }

    // MVB板卡起停状态
    // if (MVB_RUN != g_mvb_run)
    // {
        // return 0;
    // }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x05;           // 帧类型 数据下发帧
    can_id.hard_addr = 0x01;             // MVB板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 6;
    send_frame.data[0] = offset;
    send_frame.data[1] = mvb_heart & 0xFF;
    send_frame.data[2] = (mvb_heart >> 8) & 0xFF;
    send_frame.data[3] = MCU_SOFT_VER_MIN;
    send_frame.data[4] = MCU_SOFT_VER_MAJ;
    send_frame.data[5] = calc_crc16_8(send_frame.data, 5);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->MVB_heart.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN0错误计数++
        g_mvb_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->MVB_heart.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN1错误计数++
        g_mvb_err_cnt[1]++;
    }


    // 数据变化保存
    if (save_frame_flag)
    {
        save_frame_flag = 0;

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_mvb_data
 * @函数功能: 发送MVB 数据
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_mvb_data(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 mvb_data_last[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // MVB板卡状态
    // if (MVB_ERR == g_mvb_status)
    // {
        // return 0;
    // }

    // MVB板卡起停状态
    // if (MVB_RUN != g_mvb_run)
    // {
        // return 0;
    // }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x06;           // 帧类型 数据下发帧
    can_id.hard_addr = 0x01;             // MVB板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 6;
    send_frame.data[0] = offset;

    //DIO1-8
    ((DIO_FAKE_ERR == g_dio_status[0]) || (DIO_TRUE_ERR == g_dio_status[0]))  ? SET_BIT(send_frame.data[1], 0) : CLR_BIT(send_frame.data[1], 0);  // DIO1  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[1]) || (DIO_TRUE_ERR == g_dio_status[1]))  ? SET_BIT(send_frame.data[1], 1) : CLR_BIT(send_frame.data[1], 1);  // DIO2  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[2]) || (DIO_TRUE_ERR == g_dio_status[2]))  ? SET_BIT(send_frame.data[1], 2) : CLR_BIT(send_frame.data[1], 2);  // DIO3  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[3]) || (DIO_TRUE_ERR == g_dio_status[3]))  ? SET_BIT(send_frame.data[1], 3) : CLR_BIT(send_frame.data[1], 3);  // DIO4  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[4]) || (DIO_TRUE_ERR == g_dio_status[4]))  ? SET_BIT(send_frame.data[1], 4) : CLR_BIT(send_frame.data[1], 4);  // DIO5  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[5]) || (DIO_TRUE_ERR == g_dio_status[5]))  ? SET_BIT(send_frame.data[1], 5) : CLR_BIT(send_frame.data[1], 5);  // DIO6  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[6]) || (DIO_TRUE_ERR == g_dio_status[6]))  ? SET_BIT(send_frame.data[1], 6) : CLR_BIT(send_frame.data[1], 6);  // DIO7  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[7]) || (DIO_TRUE_ERR == g_dio_status[7]))  ? SET_BIT(send_frame.data[1], 7) : CLR_BIT(send_frame.data[1], 7);  // DIO8  0 正常, 1 故障
    //DIO9-10
    ((DIO_FAKE_ERR == g_dio_status[8]) || (DIO_TRUE_ERR == g_dio_status[8]))  ? SET_BIT(send_frame.data[2], 0) : CLR_BIT(send_frame.data[2], 0);  // DIO9 B故障  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[9]) || (DIO_TRUE_ERR == g_dio_status[9]))  ? SET_BIT(send_frame.data[2], 1) : CLR_BIT(send_frame.data[2], 1);  // DIO10 B故障  0 正常, 1 故障
    //DIO11-18
    ((DIO_FAKE_ERR == g_dio_status[10]) || (DIO_TRUE_ERR == g_dio_status[10]))  ? SET_BIT(send_frame.data[3], 0) : CLR_BIT(send_frame.data[3], 0);  // DIO11  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[11]) || (DIO_TRUE_ERR == g_dio_status[11]))  ? SET_BIT(send_frame.data[3], 1) : CLR_BIT(send_frame.data[3], 1);  // DIO12  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[12]) || (DIO_TRUE_ERR == g_dio_status[12]))  ? SET_BIT(send_frame.data[3], 2) : CLR_BIT(send_frame.data[3], 2);  // DIO13  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[13]) || (DIO_TRUE_ERR == g_dio_status[13]))  ? SET_BIT(send_frame.data[3], 3) : CLR_BIT(send_frame.data[3], 3);  // DIO14  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[14]) || (DIO_TRUE_ERR == g_dio_status[14]))  ? SET_BIT(send_frame.data[3], 4) : CLR_BIT(send_frame.data[3], 4);  // DIO15  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[15]) || (DIO_TRUE_ERR == g_dio_status[15]))  ? SET_BIT(send_frame.data[3], 5) : CLR_BIT(send_frame.data[3], 5);  // DIO16  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[16]) || (DIO_TRUE_ERR == g_dio_status[16]))  ? SET_BIT(send_frame.data[3], 6) : CLR_BIT(send_frame.data[3], 6);  // DIO17  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[17]) || (DIO_TRUE_ERR == g_dio_status[17]))  ? SET_BIT(send_frame.data[3], 7) : CLR_BIT(send_frame.data[3], 7);  // DIO18  0 正常, 1 故障
    //DIO19-20
    ((DIO_FAKE_ERR == g_dio_status[18]) || (DIO_TRUE_ERR == g_dio_status[18]))  ? SET_BIT(send_frame.data[4], 0) : CLR_BIT(send_frame.data[4], 0);  // DIO19  0 正常, 1 故障
    ((DIO_FAKE_ERR == g_dio_status[19]) || (DIO_TRUE_ERR == g_dio_status[19]))  ? SET_BIT(send_frame.data[4], 1) : CLR_BIT(send_frame.data[4], 1);  // DIO20  0 正常, 1 故障

    //DIO_FAULT_ERR == g_dio_status[3]  ? SET_BIT(send_frame.data[4], 7) : CLR_BIT(send_frame.data[4], 7);  // DIO1 B故障  0 正常, 1 故障
    //主设备才会发此帧，发送此帧必为主设备
    // 0x03 == g_mcu_can_id[0].all          ? CLR_BIT(send_frame.data[1], 0) : SET_BIT(send_frame.data[1], 0); // MCU主备    0 A主, 1 B主
    // CAN_LINE_ERR == g_can_line_status[0] ? SET_BIT(send_frame.data[1], 1) : CLR_BIT(send_frame.data[1], 1); // CAN1故障   0 正常, 1 故障
    // CAN_LINE_ERR == g_can_line_status[1] ? SET_BIT(send_frame.data[1], 2) : CLR_BIT(send_frame.data[1], 2); // CAN1故障   0 正常, 1 故障

    // DIO_CTRL_MASTER == g_dio_ctrl[0]  ? SET_BIT(send_frame.data[2], 0) : CLR_BIT(send_frame.data[2], 0);    // DIO0 A主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[1]  ? SET_BIT(send_frame.data[2], 1) : CLR_BIT(send_frame.data[2], 1);    // DIO0 B主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[2]  ? SET_BIT(send_frame.data[2], 2) : CLR_BIT(send_frame.data[2], 2);    // DIO1 A主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[3]  ? SET_BIT(send_frame.data[2], 3) : CLR_BIT(send_frame.data[2], 3);    // DIO1 B主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[4]  ? SET_BIT(send_frame.data[2], 4) : CLR_BIT(send_frame.data[2], 4);    // DIO2 A主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[5]  ? SET_BIT(send_frame.data[2], 5) : CLR_BIT(send_frame.data[2], 5);    // DIO2 B主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[6]  ? SET_BIT(send_frame.data[2], 6) : CLR_BIT(send_frame.data[2], 6);    // DIO3 A主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[7]  ? SET_BIT(send_frame.data[2], 7) : CLR_BIT(send_frame.data[2], 7);    // DIO3 B主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[8]  ? SET_BIT(send_frame.data[3], 0) : CLR_BIT(send_frame.data[3], 0);    // DIO4 A主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[9]  ? SET_BIT(send_frame.data[3], 1) : CLR_BIT(send_frame.data[3], 1);    // DIO4 B主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[10] ? SET_BIT(send_frame.data[3], 2) : CLR_BIT(send_frame.data[3], 2);    // DIO5 A主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[11] ? SET_BIT(send_frame.data[3], 3) : CLR_BIT(send_frame.data[3], 3);    // DIO5 B主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[12] ? SET_BIT(send_frame.data[3], 4) : CLR_BIT(send_frame.data[3], 4);    // DIO6 A主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[13] ? SET_BIT(send_frame.data[3], 5) : CLR_BIT(send_frame.data[3], 5);    // DIO1 B主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[14] ? SET_BIT(send_frame.data[3], 6) : CLR_BIT(send_frame.data[3], 6);    // DIO7 A主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[15] ? SET_BIT(send_frame.data[3], 7) : CLR_BIT(send_frame.data[3], 7);    // DIO7 B主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[16] ? SET_BIT(send_frame.data[4], 0) : CLR_BIT(send_frame.data[4], 0);    // DIO8 A主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[17] ? SET_BIT(send_frame.data[4], 1) : CLR_BIT(send_frame.data[4], 1);    // DIO8 B主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[18] ? SET_BIT(send_frame.data[4], 2) : CLR_BIT(send_frame.data[4], 2);    // DIO9 A主备  0 备, 1 主
    // DIO_CTRL_MASTER == g_dio_ctrl[19] ? SET_BIT(send_frame.data[4], 3) : CLR_BIT(send_frame.data[4], 3);    // DIO9 B主备  0 备, 1 主
    //根据客户不同需求判断某位是否置位  故障显示
    // DIO_EP_ERR == g_dio_status[0]  ? SET_BIT(send_frame.data[4], 4) : CLR_BIT(send_frame.data[4], 4);  // DIO0 A故障  0 正常, 1 故障
    //DIO_ERR == g_dio_status[0]  ? SET_BIT(send_frame.data[4], 4) : CLR_BIT(send_frame.data[4], 4);  // DIO0 A故障  0 正常, 1 故障
    // DIO_LP_ERR == g_dio_status[1]  ? SET_BIT(send_frame.data[4], 5) : CLR_BIT(send_frame.data[4], 5);  // DIO0 B故障  0 正常, 1 故障
    // DIO_TRUE_ERR == g_dio_status[2]  ? SET_BIT(send_frame.data[4], 6) : CLR_BIT(send_frame.data[4], 6);  // DIO1 A故障  0 正常, 1 故障
    // DIO_FAULT_ERR == g_dio_status[3]  ? SET_BIT(send_frame.data[4], 7) : CLR_BIT(send_frame.data[4], 7);  // DIO1 B故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[2]  ? SET_BIT(send_frame.data[4], 6) : CLR_BIT(send_frame.data[4], 6);  // DIO1 A故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[3]  ? SET_BIT(send_frame.data[4], 7) : CLR_BIT(send_frame.data[4], 7);  // DIO1 B故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[4]  ?SET_BIT(send_frame.data[5], 0) : CLR_BIT(send_frame.data[5], 0);  // DIO2 A故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[5]  ? SET_BIT(send_frame.data[5], 1) : CLR_BIT(send_frame.data[5], 1);  // DIO2 B故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[6]  ? SET_BIT(send_frame.data[5], 2) : CLR_BIT(send_frame.data[5], 2);  // DIO3 A故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[7]  ? SET_BIT(send_frame.data[5], 3) : CLR_BIT(send_frame.data[5], 3);  // DIO3 B故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[8]  ? SET_BIT(send_frame.data[5], 4) : CLR_BIT(send_frame.data[5], 4);  // DIO4 A故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[9]  ? SET_BIT(send_frame.data[5], 5) : CLR_BIT(send_frame.data[5], 5);  // DIO4 B故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[10]  ? SET_BIT(send_frame.data[5], 6) : CLR_BIT(send_frame.data[5], 6);  // DIO5 A故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[11]  ? SET_BIT(send_frame.data[5], 7) : CLR_BIT(send_frame.data[5], 7);  // DIO5 B故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[12]  ? SET_BIT(send_frame.data[6], 0) : CLR_BIT(send_frame.data[6], 0);  // DIO6 A故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[13]  ? SET_BIT(send_frame.data[6], 1) : CLR_BIT(send_frame.data[6], 1);  // DIO1 B故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[14]  ? SET_BIT(send_frame.data[6], 2) : CLR_BIT(send_frame.data[6], 2);  // DIO7 A故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[15]  ? SET_BIT(send_frame.data[6], 3) : CLR_BIT(send_frame.data[6], 3);  // DIO7 B故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[16]  ? SET_BIT(send_frame.data[6], 4) : CLR_BIT(send_frame.data[6], 4);  // DIO8 A故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[17]  ? SET_BIT(send_frame.data[6], 5) : CLR_BIT(send_frame.data[6], 5);  // DIO8 B故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[18]  ? SET_BIT(send_frame.data[6], 6) : CLR_BIT(send_frame.data[6], 6);  // DIO9 A故障  0 正常, 1 故障
    // DIO_ERR == g_dio_status[19]  ? SET_BIT(send_frame.data[6], 7) : CLR_BIT(send_frame.data[6], 7);  // DIO9 B故障  0 正常, 1 故障
    send_frame.data[5] = calc_crc16_8(send_frame.data, 5);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->MVB_data.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN0错误计数++
        g_mvb_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->MVB_data.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN1错误计数++
        g_mvb_err_cnt[1]++;
    }


    // 数据变化保存
    result = memcmp(&mvb_data_last[0], &send_frame.data[1], send_frame.can_dlc - 2);
    if (0 != result)
    {
        memcpy(&mvb_data_last[0], &send_frame.data[1], send_frame.can_dlc - 2);

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_mvb_time
 * @函数功能: 发送时间同步帧
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_mvb_time(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff)
{
    //static int cnt = 500 / RUN_CYCLE;         // MCU上电默认同步时间
	int cnt_time = 25;
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 save_frame_flag = 0xFF;

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

	if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    // if (MCU_MASTER != g_mcu_status)
    // {
        // return 0;
    // }
#if 0
    // MVB板卡状态
    if (MVB_ERR == g_mvb_status)
    {
        return 0;
    }


	// MVB板卡连接状态
	if (MVB_DISCONNECT == g_mvb_connect)
	{
		//DEBUG_INFO();
		//cnt = (500 / RUN_CYCLE); // MVB重新连接上时，立即同步时间
		cnt_time = 10;
		return 0;
	}
#endif
	
	if ((0 == g_system_date) && (0x03 == g_hard_id)) //单数日，A板为主
	{
		// MVB板卡连接状态
		if (MVB_DISCONNECT == g_mvb_connect)
		{
			//DEBUG_INFO();
			//cnt = (500 / RUN_CYCLE); // MVB重新连接上时，立即同步时间
			cnt_time = 25;
			return 0;
		}
	}
	else if ((0 == g_system_date) && (0x04 == g_hard_id))
	{
	}
	
	if ((1 == g_system_date) && (0x04 == g_hard_id)) //双数日，B板为主
	{
		// MVB板卡连接状态
		if (MVB_DISCONNECT == g_mvb_connect)
		{
			//DEBUG_INFO();
			//cnt = (500 / RUN_CYCLE); // MVB重新连接上时，立即同步时间
			cnt_time = 25;
			return 0;
		}
	}
	else if ((1 == g_system_date) && (0x03 == g_hard_id))
	{
	}

    // 60s 发送一次
    cnt_time++;
    //if (cnt < (500 / RUN_CYCLE))
	if (cnt_time < 25)	
    {
		//DEBUG_INFO();
    	return 1;
    }
    else
    {
        cnt_time = 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x03;           // 帧类型 时间同步帧
    can_id.hard_addr = 0x01;             // MVB板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 2;
    send_frame.data[0] = offset;
    send_frame.data[1] = calc_crc16_8(send_frame.data, 1);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->MVB_time.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN0错误计数++
        g_mvb_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->MVB_time.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN1错误计数++
        g_mvb_err_cnt[1]++;
    }


    // 数据变化保存
    if (save_frame_flag)
    {
        save_frame_flag = 0;

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_mvb_yinxian
 * @函数功能: 发送MVB 硬线数据
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_mvb_yinxian(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 save_frame_flag = 0xFF;

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // MVB板卡状态
    if (MVB_ERR == g_mvb_status)
    {
        return 0;
    }

    // MVB板卡起停状态
    if (MVB_RUN != g_mvb_run)
    {
        return 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x03;           // 帧类型 数据下发帧
    can_id.hard_addr = 0x01;             // MVB板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 2;
    send_frame.data[0] = offset;
    send_frame.data[1] = calc_crc16_8(send_frame.data, 1);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->MVB_yinxian.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN0错误计数++
        g_mvb_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->MVB_yinxian.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // MVB板卡CAN1错误计数++
        g_mvb_err_cnt[1]++;
    }


    // 数据变化保存
    if (save_frame_flag)
    {
        save_frame_flag = 0;

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}


/*************************************************
 * @函数名称: can_send_can_status
 * @函数功能: 发送状态请求
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_can_status(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 save_frame_flag = 0xFF;

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // CAN板卡状态
    if (CAN_ERR == g_can_status)
    {
        return 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x02;           // 帧类型 状态请求帧
    can_id.hard_addr = 0x02;             // CAN板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 2;
    send_frame.data[0] = offset;
    send_frame.data[1] = calc_crc16_8(send_frame.data, 1);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->CAN_status.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // CAN板卡CAN0错误计数++
        g_can_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->CAN_status.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // CAN板卡CAN1错误计数++
        g_can_err_cnt[1]++;
    }


    // 数据变化保存
    if (save_frame_flag)
    {
        save_frame_flag = 0;

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_can_run
 * @函数功能: 发送起停帧
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_can_run(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // CAN板卡状态
    if (CAN_DEFAULT == g_can_status)
    {
        return 0;
    }

    // CAN板卡状态 & CAN起停状态
    if (((CAN_OK == g_can_status) && (CAN_RUN == g_can_run)) || ((CAN_ERR == g_can_status) && (CAN_STOP == g_can_run)))
    {
        return 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x00;           // 帧类型 起停帧
    can_id.hard_addr = 0x02;             // CAN板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 3;
    send_frame.data[0] = offset;
    send_frame.data[1] = !g_can_run;                // 起停   0 停止, 1 启动
    send_frame.data[2] = calc_crc16_8(send_frame.data, 2);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->CAN_run.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // CAN板卡CAN0错误计数++
        g_can_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->CAN_run.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // CAN板卡CAN1错误计数++
        g_can_err_cnt[1]++;
    }


    // 保存文件
    result = save_frame(send_frame, 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_can_dio
 * @函数功能: 发送CAN DIO数据
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_can_dio(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff)
{
    static uint8 card_group = 0;                // 板卡编组 分时发送DI数据

    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 dio_data_last[5][4] =
    {
        {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}, \
        {0xFF, 0xFF, 0xFF, 0xFF}, {0xFF, 0xFF, 0xFF, 0xFF}
    };

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // CAN板卡状态
    if (CAN_ERR == g_can_status)
    {
        return 0;
    }

    // CAN板卡起停状态
    if (CAN_RUN != g_can_run)
    {
        return 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x04;           // 帧类型 数据下发帧
    can_id.hard_addr = 0x02;             // CAN板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 7;
    send_frame.data[0] = offset;

    // 分时发送DI数据
    send_frame.data[1] = card_group;
    send_frame.data[2] = g_plc_data[2 * card_group];
    send_frame.data[3] = g_plc_data[2 * card_group + 1];
    send_frame.data[4] = g_plc_data[2 * card_group + DIO_CNT];
    send_frame.data[5] = g_plc_data[2 * card_group + DIO_CNT + 1];

    card_group++;
    if (card_group >= DIO_CNT / 2)
    {
        card_group = 0;
    }
    send_frame.data[6] = calc_crc16_8(send_frame.data, 6);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->CAN_DIO.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // CAN板卡CAN0错误计数++
        g_can_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->CAN_DIO.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // CAN板卡CAN1错误计数++
        g_can_err_cnt[1]++;
    }


    // 数据变化保存
    result = memcmp(&dio_data_last[card_group][0], &send_frame.data[1], send_frame.can_dlc - 2);
    if (0 != result)
    {
        memcpy(&dio_data_last[card_group][0], &send_frame.data[1], send_frame.can_dlc - 2);

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_can_heart
 * @函数功能: 发送CAN 心跳
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
              can_heart     CAN心跳
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_can_heart(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff, uint16 can_heart)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 save_frame_flag = 0xFF;

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // CAN板卡状态
    if (CAN_ERR == g_can_status)
    {
        return 0;
    }

    // CAN板卡起停状态
    if (CAN_RUN != g_can_run)
    {
        return 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x05;           // 帧类型 数据下发帧
    can_id.hard_addr = 0x02;             // CAN板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 6;
    send_frame.data[0] = offset;
    send_frame.data[1] = can_heart & 0xFF;
    send_frame.data[2] = (can_heart >> 8) & 0xFF;
    send_frame.data[3] = MCU_SOFT_VER_MIN;
    send_frame.data[4] = MCU_SOFT_VER_MAJ;
    send_frame.data[5] = calc_crc16_8(send_frame.data, 5);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->CAN_heart.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // CAN板卡CAN0错误计数++
        g_can_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->CAN_heart.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // CAN板卡CAN1错误计数++
        g_can_err_cnt[1]++;
    }


    // 数据变化保存
    if (save_frame_flag)
    {
        save_frame_flag = 0;

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*************************************************
 * @函数名称: can_send_can_data
 * @函数功能: 发送CAN 数据
 * @输入参数: p_can_fd      文件描述符
              offset        当前发帧位置
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_send_can_data(int *p_can_fd, uint16 offset, T_can_buff **p_can_buff)
{
    int     result = 0;
    T_extid can_id = {0};                       // CAN 发送ExtID
    struct can_frame send_frame = {0, 0, {0}};  // CAN发送的数据
    static uint8 can_data_last[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    if ((NULL == p_can_fd) || (offset >= CAN_FRAME_CNT) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return 0;
    }

    // CAN板卡状态
    if (CAN_ERR == g_can_status)
    {
        return 0;
    }

    // CAN板卡起停状态
    if (CAN_RUN != g_can_run)
    {
        return 0;
    }


    /* CAN发送 */
    can_id.all = 0;
    //主板卡发送方向设置为0x00，接收方向设置为0x01，备板卡发送方向设置为0x10，接收方向设置为0x11
    if ((0x03 == g_mcu_can_id[0].all) && 0x04 == g_mcu_can_id[1].all)
    {
        can_id.direction = 0x00;
    }
    else if ((0x04 == g_mcu_can_id[0].all) && (0x03 == g_mcu_can_id[1].all))
    {
        can_id.direction = 0x02;
    }

    can_id.frame_type = 0x06;           // 帧类型 数据下发帧
    can_id.hard_addr = 0x02;             // CAN板卡地址

    /// debug_line("can_id=0x%04X\r\n", can_id.all);

    memset(send_frame.data, 0, 8);
    send_frame.can_id = can_id.all;
    send_frame.can_dlc = 8;
    send_frame.data[0] = offset;
    // 主设备才会发此帧，发送此帧必为主设备
    0x03 == g_mcu_can_id[0].all          ? CLR_BIT(send_frame.data[1], 0) : SET_BIT(send_frame.data[1], 0); // MCU主备    0 A主, 1 B主
    CAN_LINE_ERR == g_can_line_status[0] ? SET_BIT(send_frame.data[1], 1) : CLR_BIT(send_frame.data[1], 1); // CAN1故障   0 正常, 1 故障
    CAN_LINE_ERR == g_can_line_status[1] ? SET_BIT(send_frame.data[1], 2) : CLR_BIT(send_frame.data[1], 2); // CAN1故障   0 正常, 1 故障

    DIO_CTRL_MASTER == g_dio_ctrl[0]  ? SET_BIT(send_frame.data[2], 0) : CLR_BIT(send_frame.data[2], 0);    // DIO0 A主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[1]  ? SET_BIT(send_frame.data[2], 1) : CLR_BIT(send_frame.data[2], 1);    // DIO0 B主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[2]  ? SET_BIT(send_frame.data[2], 2) : CLR_BIT(send_frame.data[2], 2);    // DIO1 A主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[3]  ? SET_BIT(send_frame.data[2], 3) : CLR_BIT(send_frame.data[2], 3);    // DIO1 B主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[4]  ? SET_BIT(send_frame.data[2], 4) : CLR_BIT(send_frame.data[2], 4);    // DIO2 A主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[5]  ? SET_BIT(send_frame.data[2], 5) : CLR_BIT(send_frame.data[2], 5);    // DIO2 B主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[6]  ? SET_BIT(send_frame.data[2], 6) : CLR_BIT(send_frame.data[2], 6);    // DIO3 A主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[7]  ? SET_BIT(send_frame.data[2], 7) : CLR_BIT(send_frame.data[2], 7);    // DIO3 B主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[8]  ? SET_BIT(send_frame.data[3], 0) : CLR_BIT(send_frame.data[3], 0);    // DIO4 A主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[9]  ? SET_BIT(send_frame.data[3], 1) : CLR_BIT(send_frame.data[3], 1);    // DIO4 B主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[10] ? SET_BIT(send_frame.data[3], 2) : CLR_BIT(send_frame.data[3], 2);    // DIO5 A主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[11] ? SET_BIT(send_frame.data[3], 3) : CLR_BIT(send_frame.data[3], 3);    // DIO5 B主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[12] ? SET_BIT(send_frame.data[3], 4) : CLR_BIT(send_frame.data[3], 4);    // DIO6 A主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[13] ? SET_BIT(send_frame.data[3], 5) : CLR_BIT(send_frame.data[3], 5);    // DIO1 B主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[14] ? SET_BIT(send_frame.data[3], 6) : CLR_BIT(send_frame.data[3], 6);    // DIO7 A主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[15] ? SET_BIT(send_frame.data[3], 7) : CLR_BIT(send_frame.data[3], 7);    // DIO7 B主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[16] ? SET_BIT(send_frame.data[4], 0) : CLR_BIT(send_frame.data[4], 0);    // DIO8 A主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[17] ? SET_BIT(send_frame.data[4], 1) : CLR_BIT(send_frame.data[4], 1);    // DIO8 B主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[18] ? SET_BIT(send_frame.data[4], 2) : CLR_BIT(send_frame.data[4], 2);    // DIO9 A主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[19] ? SET_BIT(send_frame.data[4], 3) : CLR_BIT(send_frame.data[4], 3);    // DIO9 B主备  0 备, 1 主
    //根据客户不同需求判断某位是否置位
    //((DIO_ERR == g_dio_status[0]) && (DIO_LP_ERR == g_dio_status[0])) ? SET_BIT(send_frame.data[4], 4) : CLR_BIT(send_frame.data[4], 4); // DIO0 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[0]  ? SET_BIT(send_frame.data[4], 4) : CLR_BIT(send_frame.data[4], 4);  // DIO0 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[1]  ? SET_BIT(send_frame.data[4], 5) : CLR_BIT(send_frame.data[4], 5);  // DIO0 B故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[2]  ? SET_BIT(send_frame.data[4], 6) : CLR_BIT(send_frame.data[4], 6);  // DIO1 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[3]  ? SET_BIT(send_frame.data[4], 7) : CLR_BIT(send_frame.data[4], 7);  // DIO1 B故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[4]  ? SET_BIT(send_frame.data[5], 0) : CLR_BIT(send_frame.data[5], 0);  // DIO2 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[5]  ? SET_BIT(send_frame.data[5], 1) : CLR_BIT(send_frame.data[5], 1);  // DIO2 B故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[6]  ? SET_BIT(send_frame.data[5], 2) : CLR_BIT(send_frame.data[5], 2);  // DIO3 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[7]  ? SET_BIT(send_frame.data[5], 3) : CLR_BIT(send_frame.data[5], 3);  // DIO3 B故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[8]  ? SET_BIT(send_frame.data[5], 4) : CLR_BIT(send_frame.data[5], 4);  // DIO4 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[9]  ? SET_BIT(send_frame.data[5], 5) : CLR_BIT(send_frame.data[5], 5);  // DIO4 B故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[10] ? SET_BIT(send_frame.data[5], 6) : CLR_BIT(send_frame.data[5], 6);  // DIO5 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[11] ? SET_BIT(send_frame.data[5], 7) : CLR_BIT(send_frame.data[5], 7);  // DIO5 B故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[12] ? SET_BIT(send_frame.data[6], 0) : CLR_BIT(send_frame.data[6], 0);  // DIO6 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[13] ? SET_BIT(send_frame.data[6], 1) : CLR_BIT(send_frame.data[6], 1);  // DIO1 B故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[14] ? SET_BIT(send_frame.data[6], 2) : CLR_BIT(send_frame.data[6], 2);  // DIO7 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[15] ? SET_BIT(send_frame.data[6], 3) : CLR_BIT(send_frame.data[6], 3);  // DIO7 B故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[16] ? SET_BIT(send_frame.data[6], 4) : CLR_BIT(send_frame.data[6], 4);  // DIO8 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[17] ? SET_BIT(send_frame.data[6], 5) : CLR_BIT(send_frame.data[6], 5);  // DIO8 B故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[18] ? SET_BIT(send_frame.data[6], 6) : CLR_BIT(send_frame.data[6], 6);  // DIO9 A故障  0 正常, 1 故障
    DIO_FAKE_ERR == g_dio_status[19] ? SET_BIT(send_frame.data[6], 7) : CLR_BIT(send_frame.data[6], 7);  // DIO9 B故障  0 正常, 1 故障
    send_frame.data[7] = calc_crc16_8(send_frame.data, 7);  // 计算crc校验


    result = can_send_frame(send_frame, p_can_fd[0], p_can_buff[0], &p_can_buff[0]->CAN_data.data[offset], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // CAN板卡CAN0错误计数++
        g_can_err_cnt[0]++;
    }

    result = can_send_frame(send_frame, p_can_fd[1], p_can_buff[1], &p_can_buff[1]->CAN_data.data[offset], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    else if (0 == result)
    {
        // CAN板卡CAN1错误计数++
        g_can_err_cnt[1]++;
    }


    // 数据变化保存
    result = memcmp(&can_data_last[0], &send_frame.data[1], send_frame.can_dlc - 2);
    if (0 != result)
    {
        memcpy(&can_data_last[0], &send_frame.data[1], send_frame.can_dlc - 2);

        // 保存文件
        result = save_frame(send_frame, 0);
        if (-1 == result)
        {
            DEBUG_INFO();
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    return 0;
}

/*
struct can_frame
{
    canid_t can_id;     // CAN 标识符
    __u8    can_dlc;    // 数据场的长度
    __u8    data[8];    // 数据
};
*/
/*************************************************
 * @函数名称: can_periodic
 * @函数功能: CAN周期任务
 * @输入参数: p_can_fd      文件描述符
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_periodic(int *p_can_fd, T_can_buff **p_can_buff)
{
    static uint16   offset = 0;                                 // 当前发帧位置
    static uint16   mcu_heart_self = 0;                         // 本板卡MCU心跳
    static uint16   mcu_heart_other_bkp[2] = {0xFFFF, 0xFFFF};  // 另一块MCU心跳备份(两条CAN线路)
    static uint32   mcu_heart_other_err[2] = {0};               // 另一块MCU心跳超时计数(两条CAN线路)

    static int      can_line_err_cnt_mcu[2] = {0};              // MCU CAN错误计数
    static int      can_line_err_cnt_dio[2][DIO_CNT] = {{0}};   // DIO CAN错误计数
    static int      can_line_err_cnt_mvb[2] = {0};              // MVB CAN错误计数
    static int      can_line_err_cnt_can[2] = {0};              // CAN CAN错误计数

    int result = 0;
    int card_num = 0;
	char str[64] = {0};

    if ((NULL == p_can_fd) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    /******************************∨∨∨ Flag ∨∨∨*******************************/
    /* 发帧位置处理 */
    offset++;
    if (offset >= CAN_FRAME_CNT)
    {
        offset = 0;
    }


    /* 本板卡MCU心跳处理 */
    mcu_heart_self++;

    /* 另一块MCU心跳处理(两条CAN线路) */
    if (mcu_heart_other_bkp[0] == g_mcu_heart_other[0])
    {
        mcu_heart_other_err[0]++;
    }
    else
    {
        mcu_heart_other_err[0] = 0;
    }
    if (mcu_heart_other_bkp[1] == g_mcu_heart_other[1])
    {
        mcu_heart_other_err[1]++;
    }
    else
    {
        mcu_heart_other_err[1] = 0;
    }
    // 备份另一块MCU心跳(两条CAN线路)
    mcu_heart_other_bkp[0] = g_mcu_heart_other[0];
    mcu_heart_other_bkp[1] = g_mcu_heart_other[1];

    if (mcu_heart_other_err[0] < MCU_MAX_ERR_CNT)
    {
        can_line_err_cnt_mcu[0] = 0;
    }
    if (mcu_heart_other_err[1] < MCU_MAX_ERR_CNT)
    {
        can_line_err_cnt_mcu[1] = 0;
    }
    if ((mcu_heart_other_err[0] >= MCU_MAX_ERR_CNT) && (mcu_heart_other_err[1] >= MCU_MAX_ERR_CNT))  // nS未收到心跳
    {
        mcu_heart_other_err[0] = MCU_MAX_ERR_CNT;
        mcu_heart_other_err[1] = MCU_MAX_ERR_CNT;
		
		// CAN通讯正常
        if ((CAN_LINE_TRUST == g_can_line_status[0]) || (CAN_LINE_TRUST == g_can_line_status[1]))
        {
            g_mcu_status_other = 0;
			
			if ((g_mcu_heart_other[0] != 0xFFFF) && (g_mcu_heart_other[1] != 0xFFFF))
			{
				g_mcu_status_stop = 10; //add by 11.25  在正常运行过程中对方系MCU掉电
			}
            // MCU运行状态
            if (MCU_MASTER != g_mcu_status)
            {
                debug_line("MCU_MASTER! mcu_heart\r\n");
                // 当前板卡设为主设备
                g_mcu_status = MCU_MASTER;
            }
        }
    }
    // else if ((mcu_heart_other_err[0] >= MCU_MAX_ERR_CNT) && (0 == mcu_heart_other_err[1]))
    else if ((mcu_heart_other_err[0] >= MCU_MAX_ERR_CNT) && (mcu_heart_other_err[1] <= 2))
    {
        mcu_heart_other_err[0] = MCU_MAX_ERR_CNT;

        // 读取CAN0线路状态
        if (CAN_LINE_ERR != g_can_line_status[0])
        {
            can_line_err_cnt_mcu[0]++;
            if (can_line_err_cnt_mcu[0] >= 3)
            {
                //debug_line("CAN_LINE 0 ERR! MCU\r\n");
				sprintf(str, "CAN_LINE 0 ERR! MCU");
				save_log(1, __LINE__, str);

                // 当前CAN线路故障
                g_can_line_status[0] = CAN_LINE_ERR;
                // CAN线路状态切换
                check_can_line_status();
            }
        }
    }
    // else if ((0 == mcu_heart_other_err[0]) && (mcu_heart_other_err[1] >= MCU_MAX_ERR_CNT))
    else if ((mcu_heart_other_err[0] <= 2) && (mcu_heart_other_err[1] >= MCU_MAX_ERR_CNT))
    {
        mcu_heart_other_err[1] = MCU_MAX_ERR_CNT;

        // 读取CAN1线路状态
        if (CAN_LINE_ERR != g_can_line_status[1])
        {
            can_line_err_cnt_mcu[1]++;
            if (can_line_err_cnt_mcu[1] >= 3)
            {
                //debug_line("CAN_LINE 1 ERR! MCU\r\n");
				sprintf(str, "CAN_LINE 1 ERR! MCU");
				save_log(1, __LINE__, str);

                // 当前CAN线路故障
                g_can_line_status[1] = CAN_LINE_ERR;
                // CAN线路状态切换
                check_can_line_status();
            }
        }
    }

    for (card_num = 0; card_num < DIO_CNT; card_num += 2)
    {
        // 初始化
        if (((DIO_DEFAULT == g_dio_status[card_num]) && (DIO_OK == g_dio_status[card_num + 1])) ||
                ((DIO_DEFAULT == g_dio_status[card_num]) && (DIO_DEFAULT == g_dio_status[card_num + 1])) ||
                ((DIO_OK == g_dio_status[card_num]) && (DIO_DEFAULT == g_dio_status[card_num + 1])))
        {
            uint32 dio_max_err_cnt = DIO_MAX_ERR_CNT * 1;  // 每周期发帧数
            int i = 0;
            for (i = 0; i < 2; i++, card_num++)
            {
                if (g_dio_err_cnt[0][card_num] < dio_max_err_cnt)
                {
                    can_line_err_cnt_dio[0][card_num] = 0;
                }
                if (g_dio_err_cnt[1][card_num] < dio_max_err_cnt)
                {
                    can_line_err_cnt_dio[1][card_num] = 0;
                }

                // DIO板卡错误计数
                if ((g_dio_err_cnt[0][card_num] >= dio_max_err_cnt) && (g_dio_err_cnt[1][card_num] >= dio_max_err_cnt))
                {
                    // 设置DIO板卡错误计数
                    g_dio_err_cnt[0][card_num] = dio_max_err_cnt;
                    g_dio_err_cnt[1][card_num] = dio_max_err_cnt;

					
                    // 读取DIO板卡状态
                    if (DIO_FAKE_ERR != g_dio_status[card_num])
                    {
                        //debug_line("card_num %2d ERR!\r\n", card_num);
						sprintf(str, "CAN_LINE 12 ERR! card_num %2d", card_num);
						save_log(1, __LINE__, str);

                        // DIO板卡错误
                        g_dio_status[card_num] = DIO_FAKE_ERR;   //空板卡
						g_fake_cnt++;
                        // DIO板卡主备状态
                        g_dio_ctrl[card_num] = DIO_CTRL_BACKUP;

                        // 发送标志位置0
                        result = clear_err_dio_send_flag(p_can_buff[0]);
                        if (-1 == result)
                        {
                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                        result = clear_err_dio_send_flag(p_can_buff[1]);
                        if (-1 == result)
                        {
                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }

                        // DIO板卡状态切换
                        check_dio_status();
                    }
                }
                else if ((g_dio_err_cnt[0][card_num] >= dio_max_err_cnt) && (g_dio_err_cnt[1][card_num] <= 6))
                {
                    // 设置DIO板卡错误计数
                    g_dio_err_cnt[0][card_num] = dio_max_err_cnt;

                    // 读取CAN0线路状态
                    if (CAN_LINE_ERR != g_can_line_status[0])
                    {
                        can_line_err_cnt_dio[0][card_num]++;
                        if (can_line_err_cnt_dio[0][card_num] >= 3)
                        {
                            //debug_line("CAN_LINE 0 ERR! card_num %2d\r\n", card_num);
							sprintf(str, "CAN_LINE 0 ERR! card_num %2d", card_num);
							save_log(1, __LINE__, str);

                            // 当前CAN线路故障
                            g_can_line_status[0] = CAN_LINE_ERR;
                            // CAN线路状态切换
                            check_can_line_status();
                        }
                    }
                }
                else if ((g_dio_err_cnt[0][card_num] <= 6) && (g_dio_err_cnt[1][card_num] >= dio_max_err_cnt))
                {
                    // 设置DIO板卡错误计数
                    g_dio_err_cnt[1][card_num] = dio_max_err_cnt;

                    // 读取CAN1线路状态
                    if (CAN_LINE_ERR != g_can_line_status[1])
                    {
                        can_line_err_cnt_dio[1][card_num]++;
                        if (can_line_err_cnt_dio[1][card_num] >= 3)
                        {
                            //debug_line("CAN_LINE 1 ERR! card_num %2d\r\n", card_num);
							sprintf(str, "CAN_LINE 1 ERR! card_num %2d", card_num);
							save_log(1, __LINE__, str);

                            // 当前CAN线路故障
                            g_can_line_status[1] = CAN_LINE_ERR;
                            // CAN线路状态切换
                            check_can_line_status();
                        }
                    }
                }
            }
            card_num -= 2;
        }
        // 运行
        else if ((DIO_TRUST == g_dio_status[card_num]) || (DIO_TRUST == g_dio_status[card_num + 1]))
        {
            uint32 dio_max_err_cnt = DIO_MAX_ERR_CNT * 1;  // 每周期发帧数
            int i = 0;
            for (i = 0; i < 2; i++, card_num++)
            {
                // 主
                if (DIO_TRUST == g_dio_status[card_num])
                {
                    dio_max_err_cnt = DIO_MAX_ERR_CNT * 3;
                }
                // 备
                else if (DIO_TRUST != g_dio_status[card_num])
                {
                    dio_max_err_cnt = DIO_MAX_ERR_CNT * 2;
                }

                if (g_dio_err_cnt[0][card_num] < dio_max_err_cnt)
                {
                    can_line_err_cnt_dio[0][card_num] = 0;
                }
                if (g_dio_err_cnt[1][card_num] < dio_max_err_cnt)
                {
                    can_line_err_cnt_dio[1][card_num] = 0;
                }

                // DIO板卡错误计数
                if ((g_dio_err_cnt[0][card_num] >= dio_max_err_cnt) && (g_dio_err_cnt[1][card_num] >= dio_max_err_cnt))
                {
                    // 设置DIO板卡错误计数
                    g_dio_err_cnt[0][card_num] = dio_max_err_cnt;
                    g_dio_err_cnt[1][card_num] = dio_max_err_cnt;

                    // 读取DIO板卡状态
                    if (DIO_FAKE_ERR != g_dio_status[card_num])
                    {
                        //debug_line("card_num %2d ERR!\r\n", card_num);
						sprintf(str, "CAN_LINE 12 ERR! card_num %2d", card_num);
						save_log(1, __LINE__, str);
						g_fake_cnt_y++;
                        // DIO板卡错误
                        g_dio_status[card_num] = DIO_FAKE_ERR;   //运行中丢失板卡
                        // DIO板卡主备状态
                        g_dio_ctrl[card_num] = DIO_CTRL_BACKUP;

                        // 发送标志位置0
                        result = clear_err_dio_send_flag(p_can_buff[0]);
                        if (-1 == result)
                        {
                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
                        result = clear_err_dio_send_flag(p_can_buff[1]);
                        if (-1 == result)
                        {
                            DEBUG_INFO();
                            save_log(2, __LINE__, NULL);
                            return -1;
                        }
						
                        // DIO板卡状态切换
                        check_dio_status();
                    }
                }
                else if ((g_dio_err_cnt[0][card_num] >= dio_max_err_cnt) && (g_dio_err_cnt[1][card_num] <= 6))
                {
                    // 设置DIO板卡错误计数
                    g_dio_err_cnt[0][card_num] = dio_max_err_cnt;

                    // 读取CAN0线路状态
                    if (CAN_LINE_ERR != g_can_line_status[0])
                    {
                        can_line_err_cnt_dio[0][card_num]++;
                        if (can_line_err_cnt_dio[0][card_num] >= 3)
                        {
                            //debug_line("CAN_LINE 0 ERR! card_num %2d\r\n", card_num);
							sprintf(str, "CAN_LINE 0 ERR! card_num %2d", card_num);
							save_log(1, __LINE__, str);

                            // 当前CAN线路故障
                            g_can_line_status[0] = CAN_LINE_ERR;
                            // CAN线路状态切换
                            check_can_line_status();
                        }
                    }
                }
                else if ((g_dio_err_cnt[0][card_num] <= 6) && (g_dio_err_cnt[1][card_num] >= dio_max_err_cnt))
                {
                    // 设置DIO板卡错误计数
                    g_dio_err_cnt[1][card_num] = dio_max_err_cnt;

                    // 读取CAN1线路状态
                    if (CAN_LINE_ERR != g_can_line_status[1])
                    {
                        can_line_err_cnt_dio[1][card_num]++;
                        if (can_line_err_cnt_dio[1][card_num] >= 3)
                        {
                            //debug_line("CAN_LINE 1 ERR! card_num %2d\r\n", card_num);
							sprintf(str, "CAN_LINE 1 ERR! card_num %2d", card_num);
							save_log(1, __LINE__, str);

                            // 当前CAN线路故障
                            g_can_line_status[1] = CAN_LINE_ERR;
                            // CAN线路状态切换
                            check_can_line_status();
                        }
                    }
                }
            }
            card_num -= 2;
        }
    }


    if (g_mvb_err_cnt[0] < MVB_MAX_ERR_CNT)
    {
        can_line_err_cnt_mvb[0] = 0;
    }
    if (g_mvb_err_cnt[1] < MVB_MAX_ERR_CNT)
    {
        can_line_err_cnt_mvb[1] = 0;
    }
    /* MVB板卡状态处理 */
    if ((g_mvb_err_cnt[0] >= MVB_MAX_ERR_CNT) && (g_mvb_err_cnt[1] >= MVB_MAX_ERR_CNT))
    {
        // 设置MVB板卡错误计数
        g_mvb_err_cnt[0] = MVB_MAX_ERR_CNT;
        g_mvb_err_cnt[1] = MVB_MAX_ERR_CNT;

        // 读取MVB板卡状态
        if (MVB_ERR != g_mvb_status)
        {
            //debug_line("MVB ERR!\r\n");
			sprintf(str, "CNT1:%d CNT2: %d MVB", g_mvb_err_cnt[0], g_mvb_err_cnt[1]);
			save_log(1, __LINE__, str);

            // MVB板卡错误
            g_mvb_status = MVB_ERR;

            // MVB起停状态
            g_mvb_run = MVB_STOP;
        }
    }
    // else if ((g_mvb_err_cnt[0] >= MVB_MAX_ERR_CNT) && (0 == g_mvb_err_cnt[1]))
    else if ((g_mvb_err_cnt[0] >= MVB_MAX_ERR_CNT) && (g_mvb_err_cnt[1] <= 10))
    {
        // 设置MVB板卡错误计数
        g_mvb_err_cnt[0] = MVB_MAX_ERR_CNT;

        // 读取CAN0线路状态
        if (CAN_LINE_ERR != g_can_line_status[0])
        {
            can_line_err_cnt_mvb[0]++;
            if (can_line_err_cnt_mvb[0] >= 3)
            {
                //debug_line("CAN_LINE 0 ERR! MVB\r\n");
				sprintf(str, "CNT1:%d CNT2: %d MVB", g_mvb_err_cnt[0], g_mvb_err_cnt[1]);
				save_log(1, __LINE__, str);

                // 当前CAN线路故障
                g_can_line_status[0] = CAN_LINE_ERR;
                // CAN线路状态切换
                check_can_line_status();
            }
        }
    }
    // else if ((0 == g_mvb_err_cnt[0]) && (g_mvb_err_cnt[1] >= MVB_MAX_ERR_CNT))
    else if ((g_mvb_err_cnt[0] <= 10) && (g_mvb_err_cnt[1] >= MVB_MAX_ERR_CNT))
    {
        // 设置MVB板卡错误计数
        g_mvb_err_cnt[1] = MVB_MAX_ERR_CNT;

        // 读取CAN1线路状态
        if (CAN_LINE_ERR != g_can_line_status[1])
        {
            can_line_err_cnt_mvb[1]++;
            if (can_line_err_cnt_mvb[1] >= 3)
            {
                //debug_line("CAN_LINE 1 ERR! MVB\r\n");
				
				sprintf(str, "CNT1:%d CNT2: %d MVB", g_mvb_err_cnt[0], g_mvb_err_cnt[1]);
				save_log(1, __LINE__, str);

                // 当前CAN线路故障
                g_can_line_status[1] = CAN_LINE_ERR;
                // CAN线路状态切换
                check_can_line_status();
            }
        }
    }


    if (g_can_err_cnt[0] < CAN_MAX_ERR_CNT)
    {
        can_line_err_cnt_can[0] = 0;
    }
    if (g_can_err_cnt[1] < CAN_MAX_ERR_CNT)
    {
        can_line_err_cnt_can[1] = 0;
    }
    /* CAN板卡状态处理 */
    if ((g_can_err_cnt[0] >= CAN_MAX_ERR_CNT) && (g_can_err_cnt[1] >= CAN_MAX_ERR_CNT))
    {
        // 设置CAN板卡错误计数
        g_can_err_cnt[0] = CAN_MAX_ERR_CNT;
        g_can_err_cnt[1] = CAN_MAX_ERR_CNT;

        // 读取CAN板卡状态
        if (CAN_ERR != g_can_status)
        {
            debug_line("CAN ERR!\r\n");

            // CAN板卡错误
            g_can_status = CAN_ERR;

            // CAN起停状态
            g_can_run = CAN_STOP;
        }
    }
    // else if ((g_can_err_cnt[0] >= CAN_MAX_ERR_CNT) && (0 == g_can_err_cnt[1]))
    else if ((g_can_err_cnt[0] >= CAN_MAX_ERR_CNT) && (g_can_err_cnt[1] <= 10))
    {
        // 设置CAN板卡错误计数
        g_can_err_cnt[0] = CAN_MAX_ERR_CNT;

        // 读取CAN0线路状态
        if (CAN_LINE_ERR != g_can_line_status[0])
        {
            can_line_err_cnt_can[0]++;
            if (can_line_err_cnt_can[0] >= 3)
            {
                debug_line("CAN_LINE 0 ERR! CAN\r\n");

                // 当前CAN线路故障
                g_can_line_status[0] = CAN_LINE_ERR;
                // CAN线路状态切换
                check_can_line_status();
            }
        }
    }
    // else if ((0 == g_can_err_cnt[0]) && (g_can_err_cnt[1] >= CAN_MAX_ERR_CNT))
    else if ((g_can_err_cnt[0] <= 10) && (g_can_err_cnt[1] >= CAN_MAX_ERR_CNT))
    {
        // 设置CAN板卡错误计数
        g_can_err_cnt[1] = CAN_MAX_ERR_CNT;

        // 读取CAN1线路状态
        if (CAN_LINE_ERR != g_can_line_status[1])
        {
            can_line_err_cnt_can[1]++;
            if (can_line_err_cnt_can[1] >= 3)
            {
                debug_line("CAN_LINE 1 ERR! CAN\r\n");

                // 当前CAN线路故障
                g_can_line_status[1] = CAN_LINE_ERR;
                // CAN线路状态切换
                check_can_line_status();
            }
        }
    }


#ifdef PRINT_INFO
    print_line("offset = 0x%02X\r\n", offset);
    print_line("g_mcu_status = %d\r\n", g_mcu_status);
    printf("can line   %3d %3d\r\n", g_can_line_status[0], g_can_line_status[1]);
    printf("dio status ");
    for (card_num = 0; card_num < DIO_CNT; card_num++)
    {
        printf("%3d ", g_dio_status[card_num]);
    }
    printf("\r\n");
    printf("dio ctrl   ");
    for (card_num = 0; card_num < DIO_CNT; card_num++)
    {
        printf("%3d ", g_dio_ctrl[card_num]);
    }
    printf("\r\n");
    printf("dio cnt 0  ");
    for (card_num = 0; card_num < DIO_CNT; card_num++)
    {
        printf("%3d ", g_dio_err_cnt[0][card_num]);
    }
    printf("\r\n");
    printf("dio cnt 1  ");
    for (card_num = 0; card_num < DIO_CNT; card_num++)
    {
        printf("%3d ", g_dio_err_cnt[1][card_num]);
    }
    printf("\r\n");


    printf("mvb cnt 0  %3d, mvb cnt 1  %3d\r\n", g_mvb_err_cnt[0], g_mvb_err_cnt[1]);
    printf("can cnt 0  %3d, can cnt 1  %3d\r\n", g_can_err_cnt[0], g_can_err_cnt[1]);
    printf("mvb status %3d, can status %3d\r\n", g_mvb_status, g_can_status);
#endif

    /******************************∧∧∧ Flag ∧∧∧*******************************/


    /****************************∨∨∨ MCU heart ∨∨∨****************************/
    result = can_send_mcu_heart(p_can_fd, offset, p_can_buff, mcu_heart_self);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    /****************************∧∧∧ MCU heart ∧∧∧****************************/


    /* MCU主设备进行CAN周期发送任务 */
    /******************************∨∨∨ DIO ∨∨∨********************************/
    for (card_num = 0; card_num < DIO_CNT; card_num++)
    {
        /* DIO状态请求帧 */
        result = can_send_dio_status(p_can_fd, offset, card_num, p_can_buff);
        if (-1 == result)
        {
            DEBUG_INFO();
            debug_line("card_num = %2d\r\n", card_num);
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }

    for (card_num = 0; card_num < DIO_CNT; card_num++)
    {
        /* DI数据请求帧 */
        result = can_send_dio_di(p_can_fd, offset, card_num, p_can_buff);
        if (-1 == result)
        {
            DEBUG_INFO();
            debug_line("card_num = %2d\r\n", card_num);
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }
	
	g_counter_send ++;
	if (g_counter_send >= 160)
	{
		//g_counter_send_cycle ++;
		//if (30 == g_counter_send_cycle) //600ms发送一次
		//{
			for (card_num = 2; card_num < 4; card_num++)
			{
			   /* DI数据请求帧 */
				result = can_send_dio_qz(p_can_fd, offset, card_num, p_can_buff);
				if (-1 == result)
				{
					DEBUG_INFO();
					debug_line("card_num = %2d\r\n", card_num);
					save_log(2, __LINE__, NULL);
					return -1;
				}
			}
			//g_counter_send_cycle = 0;
		//}		
		g_counter_send = 162;
	}
	

    /******************************∧∧∧ DIO ∧∧∧********************************/


    /******************************∨∨∨ MVB ∨∨∨********************************/
    /* MVB状态请求帧 */
    //printf("step in can_send_mvb_status\n");
    result = can_send_mvb_status(p_can_fd, offset, p_can_buff);
    if (-1 == result)
    {
        DEBUG_INFO();
        debug_line("card_num = %2d\r\n", card_num);
        save_log(2, __LINE__, NULL);
        return -1;
    }

    /* MVB起停帧 */
    result = can_send_mvb_run(p_can_fd, offset, p_can_buff);
    if (-1 == result)
    {
        DEBUG_INFO();
        debug_line("card_num = %2d\r\n", card_num);
        save_log(2, __LINE__, NULL);
        return -1;
    }

    // for (card_num = 0; card_num < DIO_CNT / 2; card_num++)
    // {
    // /* MVB DIO数据下发帧 */
    // result = can_send_mvb_dio(p_can_fd, offset, card_num, p_can_buff);
    // if (-1 == result)
    // {
    // DEBUG_INFO();
    // debug_line("card_num = %2d\r\n", card_num);
    // save_log(2, __LINE__, NULL);
    // return -1;
    // }
    // }

    /* MVB DIO数据下发帧 */
    result = can_send_mvb_dio(p_can_fd, offset, p_can_buff);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    /* MVB 心跳帧 */
    result = can_send_mvb_heart(p_can_fd, offset, p_can_buff, mcu_heart_self);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    // /* MVB 数据下发帧 */
    result = can_send_mvb_data(p_can_fd, offset, p_can_buff);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    /* MVB 时间同步帧 */
    result = can_send_mvb_time(p_can_fd, offset, p_can_buff);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
#if 0	
	/* MVB 硬线数据帧 */
    result = can_send_mvb_yinxian(p_can_fd, offset, p_can_buff);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
#endif
    /******************************∧∧∧ MVB ∧∧∧********************************/


    /******************************∨∨∨ CAN ∨∨∨********************************/
    /* CAN状态请求帧 */
    // result = can_send_can_status(p_can_fd, offset, p_can_buff);
    // if (-1 == result)
    // {
    // DEBUG_INFO();
    // debug_line("card_num = %2d\r\n", card_num);
    // save_log(2, __LINE__, NULL);
    // return -1;
    // }

    /* CAN起停帧 */
    // result = can_send_can_run(p_can_fd, offset, p_can_buff);
    // if (-1 == result)
    // {
    // DEBUG_INFO();
    // debug_line("card_num = %2d\r\n", card_num);
    // save_log(2, __LINE__, NULL);
    // return -1;
    // }

    // for (card_num = 0; card_num < DIO_CNT / 2; card_num++)
    // {
    // /* CAN DIO数据下发帧 */
    // result = can_send_can_dio(p_can_fd, offset, card_num, p_can_buff);
    // if (-1 == result)
    // {
    // DEBUG_INFO();
    // debug_line("card_num = %2d\r\n", card_num);
    // save_log(2, __LINE__, NULL);
    // return -1;
    // }
    // }

    /* CAN DIO数据下发帧 */
    // result = can_send_can_dio(p_can_fd, offset, p_can_buff);
    // if (-1 == result)
    // {
    // DEBUG_INFO();
    // save_log(2, __LINE__, NULL);
    // return -1;
    // }

    /* CAN 心跳帧 */
    // result = can_send_can_heart(p_can_fd, offset, p_can_buff, mcu_heart_self);
    // if (-1 == result)
    // {
    // DEBUG_INFO();
    // save_log(2, __LINE__, NULL);
    // return -1;
    // }

    /* CAN 数据下发帧 */
    // result = can_send_can_data(p_can_fd, offset, p_can_buff);
    // if (-1 == result)
    // {
    // DEBUG_INFO();
    // save_log(2, __LINE__, NULL);
    // return -1;
    // }
    /******************************∧∧∧ CAN ∧∧∧********************************/


    /*****************************∨∨∨ DIO DO ∨∨∨******************************/
    for (card_num = 0; card_num < DIO_CNT; card_num++)
    {
        /* DO数据下发帧 */
        result = can_send_dio_do(p_can_fd, offset, card_num, p_can_buff);
        if (-1 == result)
        {
            DEBUG_INFO();
            debug_line("card_num = %2d\r\n", card_num);
            save_log(2, __LINE__, NULL);
            return -1;
        }
    }
    /*****************************∧∧∧ DIO DO ∧∧∧******************************/

    // 运行监测
    g_run_flag_can_periodic = 0;

    return 0;
}

/*************************************************
 * @函数名称: check_offset_err
 * @函数功能: 检测序号是否错误
 * @输入参数: 无
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : 无
 * @其它说明: 无
 *************************************************/
void check_offset_err(void)
{
    uint8 card_num = 0;         // 板卡号
    uint8 can_line = 0;         // can线路
    static uint32 mcu_offset_err_cnt[2] = {0};              // MCU序号错误计时(两条CAN线路)
    static uint32 dio_offset_err_cnt[2][DIO_CNT] = {{0}};   // DIO序号错误计时(两条CAN线路)

    for (can_line = 0; can_line < 2; can_line++)
    {
        if (g_mcu_offset_err[can_line])
        {
            mcu_offset_err_cnt[can_line]++;
            if (mcu_offset_err_cnt[can_line] > SECOND_CNT)
            {
                g_mcu_offset_err[can_line] = 0;
                mcu_offset_err_cnt[can_line] = 0;
            }
        }
    }

    // MCU运行状态
    if (MCU_MASTER != g_mcu_status)
    {
        return;
    }

    for (can_line = 0; can_line < 2; can_line++)
    {
        for (card_num = 0; card_num < DIO_CNT; card_num++)
        {
            if (g_dio_offset_err[can_line][card_num])
            {
                dio_offset_err_cnt[can_line][card_num]++;
                if (dio_offset_err_cnt[can_line][card_num] > SECOND_CNT)
                {
                    g_dio_offset_err[can_line][card_num] = 0;
                    dio_offset_err_cnt[can_line][card_num] = 0;
                }
            }
        }
    }
}

/*************************************************
 * @函数名称: check_time_out
 * @函数功能: 检测回复帧是否超时
 * @输入参数: 无
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0  成功
                  -1  失败
 * @其它说明: 无
 *************************************************/
int check_time_out(T_can_buff **p_can_buff)
{
    int result = 0;
    int time_after = 0;         // 时间间隔
    uint16 offset = 0;          // 发帧序号
    uint8 card_num = 0;         // 板卡号
    uint8 can_line = 0;         // can线路
    struct timeval tv = {0, 0}; // 时间结构体
	char str[64] = {0};
    
	// jishuqi++;
	if (NULL == p_can_buff)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    if ((NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    // MCU运行状态
    // if (MCU_MASTER != g_mcu_status)
    // {
        //发送标志位置0
        // result = clear_all_dio_send_flag(p_can_buff[0]);
        // if (-1 == result)
        // {
            // DEBUG_INFO();
            // save_log(2, __LINE__, NULL);
            // return -1;
        // }
        // result = clear_all_dio_send_flag(p_can_buff[1]);
        // if (-1 == result)
        // {
            // DEBUG_INFO();
            // save_log(2, __LINE__, NULL);
            // return -1;
        // }

        //运行监测
        // g_run_flag_check_time_out = 0;
        // return 0;
    // }

    // 获取当前时间戳
    result = gettimeofday(&tv, NULL);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }
    tv.tv_sec -= 28800; // UTC time
    tv.tv_usec /= 1000; // us转为ms

    for (can_line = 0; can_line < 2; can_line++)
    {
        // CAN线路状态
        if (CAN_LINE_ERR == g_can_line_status[can_line])
        {
            continue;
        }

        for (card_num = 0; card_num < DIO_CNT; card_num++)
		//for (card_num = 0; card_num < 10; card_num++)
        {
            // DIO板卡状态
            // if ((DIO_ERR == g_dio_status[card_num]) || (DIO_DEFAULT == g_dio_status[card_num]))
            // {
                //发送标志位置0
                // p_can_buff[0]->DIO_status[card_num].data[offset].frame_send_flag = 0;
                // p_can_buff[0]->DIO_DI[card_num].data[offset].frame_send_flag = 0;
                // p_can_buff[0]->DIO_DO[card_num].data[offset].frame_send_flag = 0;

                //接收标志位置0
                // p_can_buff[0]->DIO_status[card_num].data[offset].frame_recv_flag = 0;
                // p_can_buff[0]->DIO_DI[card_num].data[offset].frame_recv_flag = 0;
                // p_can_buff[0]->DIO_DO[card_num].data[offset].frame_recv_flag = 0;

                // p_can_buff[1]->DIO_status[card_num].data[offset].frame_send_flag = 0;
                // p_can_buff[1]->DIO_DI[card_num].data[offset].frame_send_flag = 0;
                // p_can_buff[1]->DIO_DO[card_num].data[offset].frame_send_flag = 0;

                //接收标志位置0
                // p_can_buff[1]->DIO_status[card_num].data[offset].frame_recv_flag = 0;
                // p_can_buff[1]->DIO_DI[card_num].data[offset].frame_recv_flag = 0;
                // p_can_buff[1]->DIO_DO[card_num].data[offset].frame_recv_flag = 0;

                // continue;
            // }

            for (offset = 0; offset < CAN_FRAME_CNT; offset++)
            {
                // 帧已发送
                if (1 == p_can_buff[can_line]->DIO_status[card_num].data[offset].frame_send_flag)
                {
					// if(0x03 == g_hard_id)
					// {
						// if(jishuqi > 2000)
						// {
							// downtime(__LINE__, "CAN_LINE all");
						// }
					// }
                    // 已经收到帧
                    if (1 == p_can_buff[can_line]->DIO_status[card_num].data[offset].frame_recv_flag)
                    {
                        // 发送标志位置0
                        p_can_buff[can_line]->DIO_status[card_num].data[offset].frame_send_flag = 0;
                    }
                    // 未收到帧
                    else
                    {
                        time_after = (tv.tv_sec - p_can_buff[can_line]->DIO_status[card_num].data[offset].send_tv.tv_sec) * 1000 + (tv.tv_usec - p_can_buff[can_line]->DIO_status[card_num].data[offset].send_tv.tv_usec);
                        if (time_after > FRAME_TIMEOUT)
                        {
                            // debug_line("DIO_status can_line %d card_num %2d offset 0x%02X time_out %dms!\r\n", can_line, card_num, offset, time_after);
							//保存超时日志
							sprintf(str, "can %d offset %d card %d STtime_out", can_line, offset, card_num);
							save_log(1, __LINE__, str);

                            // DIO板卡错误
                            g_dio_status[card_num] = DIO_FAKE_ERR;   //超时为真故障
                            // DIO板卡主备状态
                            g_dio_ctrl[card_num] = DIO_CTRL_BACKUP;

                            // 发送标志位置0
                            result = clear_err_dio_send_flag(p_can_buff[0]);
                            if (-1 == result)
                            {
                                DEBUG_INFO();
                                save_log(2, __LINE__, NULL);
                                return -1;
                            }
                            result = clear_err_dio_send_flag(p_can_buff[1]);
                            if (-1 == result)
                            {
                                DEBUG_INFO();
                                save_log(2, __LINE__, NULL);
                                return -1;
                            }

                            // DIO板卡状态切换
                            check_dio_status();

                            break;
                        }
                    }
                }

                // 帧已发送
                if (1 == p_can_buff[can_line]->DIO_DI[card_num].data[offset].frame_send_flag)
                {
                    // 已经收到帧
                    if (1 == p_can_buff[can_line]->DIO_DI[card_num].data[offset].frame_recv_flag)
                    {
                        // 发送标志位置0
                        p_can_buff[can_line]->DIO_DI[card_num].data[offset].frame_send_flag = 0;
                    }
                    // 未收到帧
                    else
                    {
                        time_after = (tv.tv_sec - p_can_buff[can_line]->DIO_DI[card_num].data[offset].send_tv.tv_sec) * 1000 + (tv.tv_usec - p_can_buff[can_line]->DIO_DI[card_num].data[offset].send_tv.tv_usec);
                        if (time_after > FRAME_TIMEOUT)
                        {
                            // debug_line("DIO_DI     can_line %d card_num %2d offset 0x%02X time_out %dms!\r\n", can_line, card_num, offset, time_after);
							//保存超时日志
							sprintf(str, "can %d offset %d card %d DItime_out", can_line, offset, card_num);
							save_log(1, __LINE__, str);

                            // DIO板卡错误
                            g_dio_status[card_num] = DIO_FAKE_ERR;  //超时为真故障
                            // DIO板卡主备状态
                            g_dio_ctrl[card_num] = DIO_CTRL_BACKUP;

                            // 发送标志位置0
                            result = clear_err_dio_send_flag(p_can_buff[0]);
                            if (-1 == result)
                            {
                                DEBUG_INFO();
                                save_log(2, __LINE__, NULL);
                                return -1;
                            }
                            result = clear_err_dio_send_flag(p_can_buff[1]);
                            if (-1 == result)
                            {
                                DEBUG_INFO();
                                save_log(2, __LINE__, NULL);
                                return -1;
                            }

							// DIO板卡状态切换
                            check_dio_status();

                            break;
                        }
                    }
                }

                // 帧已发送
                if (1 == p_can_buff[can_line]->DIO_DO[card_num].data[offset].frame_send_flag)
                {
                    // 已经收到帧
                    if (1 == p_can_buff[can_line]->DIO_DO[card_num].data[offset].frame_recv_flag)
                    {
                        // 发送标志位置0
                        p_can_buff[can_line]->DIO_DO[card_num].data[offset].frame_send_flag = 0;
                    }
                    // 未收到帧
                    else
                    {
                        time_after = (tv.tv_sec - p_can_buff[can_line]->DIO_DO[card_num].data[offset].send_tv.tv_sec) * 1000 + (tv.tv_usec - p_can_buff[can_line]->DIO_DO[card_num].data[offset].send_tv.tv_usec);
                        if (time_after > FRAME_TIMEOUT)
                        {
                            // debug_line("DIO_DO     can_line %d card_num %2d offset 0x%02X time_out %dms!\r\n", can_line, card_num, offset, time_after);
							//保存超时日志
							sprintf(str, "can %d offset %d card %d DOtime_out", can_line, offset, card_num);
							save_log(1, __LINE__, str);

                            // DIO板卡错误
                            g_dio_status[card_num] = DIO_FAKE_ERR;  //超时为真故障
                            // DIO板卡主备状态
                            g_dio_ctrl[card_num] = DIO_CTRL_BACKUP;

                            // 发送标志位置0
                            result = clear_err_dio_send_flag(p_can_buff[0]);
                            if (-1 == result)
                            {
                                DEBUG_INFO();
                                save_log(2, __LINE__, NULL);
                                return -1;
                            }
                            result = clear_err_dio_send_flag(p_can_buff[1]);
                            if (-1 == result)
                            {
                                DEBUG_INFO();
                                save_log(2, __LINE__, NULL);
                                return -1;
                            }

							// DIO板卡状态切换
                            check_dio_status();

                            break;
                        }
                    }
                }
            }
        }
    }


    // 运行监测
    g_run_flag_check_time_out = 0;

    return 0;
}

/*************************************************
 * @函数名称: set_tcp_data
 * @函数功能: 打包TCP数据
 * @输入参数: 无
 * @输出参数: p_buff    tcp数据
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int set_tcp_data(uint8 *p_buff)
{
    int result = 0;

    if (NULL == p_buff)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


    // 打包plc数据
    memcpy(p_buff, g_plc_data, DIO_CNT * 2);

    // DIO主备  0 备, 1 主
    DIO_CTRL_MASTER == g_dio_ctrl[0]  ? SET_BIT(p_buff[DIO_CNT * 2], 0)     : CLR_BIT(p_buff[DIO_CNT * 2], 0);      // DIO0 A主备
    DIO_CTRL_MASTER == g_dio_ctrl[1]  ? SET_BIT(p_buff[DIO_CNT * 2], 1)     : CLR_BIT(p_buff[DIO_CNT * 2], 1);      // DIO0 B主备
    DIO_CTRL_MASTER == g_dio_ctrl[2]  ? SET_BIT(p_buff[DIO_CNT * 2], 2)     : CLR_BIT(p_buff[DIO_CNT * 2], 2);      // DIO1 A主备
    DIO_CTRL_MASTER == g_dio_ctrl[3]  ? SET_BIT(p_buff[DIO_CNT * 2], 3)     : CLR_BIT(p_buff[DIO_CNT * 2], 3);      // DIO1 B主备
    DIO_CTRL_MASTER == g_dio_ctrl[4]  ? SET_BIT(p_buff[DIO_CNT * 2], 4)     : CLR_BIT(p_buff[DIO_CNT * 2], 4);      // DIO2 A主备
    DIO_CTRL_MASTER == g_dio_ctrl[5]  ? SET_BIT(p_buff[DIO_CNT * 2], 5)     : CLR_BIT(p_buff[DIO_CNT * 2], 5);      // DIO2 B主备
    DIO_CTRL_MASTER == g_dio_ctrl[6]  ? SET_BIT(p_buff[DIO_CNT * 2], 6)     : CLR_BIT(p_buff[DIO_CNT * 2], 6);      // DIO3 A主备
    DIO_CTRL_MASTER == g_dio_ctrl[7]  ? SET_BIT(p_buff[DIO_CNT * 2], 7)     : CLR_BIT(p_buff[DIO_CNT * 2], 7);      // DIO3 B主备
    DIO_CTRL_MASTER == g_dio_ctrl[8]  ? SET_BIT(p_buff[DIO_CNT * 2 + 1], 0) : CLR_BIT(p_buff[DIO_CNT * 2 + 1], 0);  // DIO4 A主备
    DIO_CTRL_MASTER == g_dio_ctrl[9]  ? SET_BIT(p_buff[DIO_CNT * 2 + 1], 1) : CLR_BIT(p_buff[DIO_CNT * 2 + 1], 1);  // DIO4 B主备
    DIO_CTRL_MASTER == g_dio_ctrl[10] ? SET_BIT(p_buff[DIO_CNT * 2 + 1], 2) : CLR_BIT(p_buff[DIO_CNT * 2 + 1], 2);  // DIO5 A主备
    DIO_CTRL_MASTER == g_dio_ctrl[11] ? SET_BIT(p_buff[DIO_CNT * 2 + 1], 3) : CLR_BIT(p_buff[DIO_CNT * 2 + 1], 3);  // DIO5 B主备
    DIO_CTRL_MASTER == g_dio_ctrl[12] ? SET_BIT(p_buff[DIO_CNT * 2 + 1], 4) : CLR_BIT(p_buff[DIO_CNT * 2 + 1], 4);  // DIO6 A主备
    DIO_CTRL_MASTER == g_dio_ctrl[13] ? SET_BIT(p_buff[DIO_CNT * 2 + 1], 5) : CLR_BIT(p_buff[DIO_CNT * 2 + 1], 5);  // DIO6 B主备
    DIO_CTRL_MASTER == g_dio_ctrl[14] ? SET_BIT(p_buff[DIO_CNT * 2 + 1], 6) : CLR_BIT(p_buff[DIO_CNT * 2 + 1], 6);  // DIO7 A主备
    DIO_CTRL_MASTER == g_dio_ctrl[15] ? SET_BIT(p_buff[DIO_CNT * 2 + 1], 7) : CLR_BIT(p_buff[DIO_CNT * 2 + 1], 7);  // DIO7 B主备
    DIO_CTRL_MASTER == g_dio_ctrl[16] ? SET_BIT(p_buff[DIO_CNT * 2 + 2], 0) : CLR_BIT(p_buff[DIO_CNT * 2 + 2], 0);  // DIO8 A主备
    DIO_CTRL_MASTER == g_dio_ctrl[17] ? SET_BIT(p_buff[DIO_CNT * 2 + 2], 1) : CLR_BIT(p_buff[DIO_CNT * 2 + 2], 1);  // DIO8 B主备
    DIO_CTRL_MASTER == g_dio_ctrl[18] ? SET_BIT(p_buff[DIO_CNT * 2 + 2], 2) : CLR_BIT(p_buff[DIO_CNT * 2 + 2], 2);  // DIO9 A主备
    DIO_CTRL_MASTER == g_dio_ctrl[19] ? SET_BIT(p_buff[DIO_CNT * 2 + 2], 3) : CLR_BIT(p_buff[DIO_CNT * 2 + 2], 3);  // DIO9 B主备

    return 0;
}

/*************************************************
 * @函数名称: can_task
 * @函数功能: can收发任务
 * @输入参数: p_can_fd      文件描述符
 * @输出参数: p_can_buff    CAN帧接收缓冲区
 * @返回值  : int  0 成功
                  -1 失败
 * @其它说明: 无
 *************************************************/
int can_task(int *p_can_fd, T_can_buff **p_can_buff)
{
    int result = 0;
    int sem_value = 0;
    uint8 tcp_buff[TCP_BUFF_SIZE - 2 - 4 - 2] = {0};

#ifdef RUN_TIME
    struct timeval tv = {0, 0};
    struct timeval tv_tmp = {0, 0};
#endif

    if ((NULL == p_can_fd) || (NULL == p_can_buff))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    if ((p_can_fd[0] < 0) || (p_can_fd[1] < 0) || (NULL == p_can_buff[0]) || (NULL == p_can_buff[1]))
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }


#ifdef RUN_TIME
    gettimeofday(&tv, NULL);
#endif


    /* PLC运行应用程序 */
    result = plc_run();
    if (-1 == result)
    {
        downtime(__LINE__, "plc_run");
    }

    /* CAN接收 */
    result = can_recv(p_can_fd[0], p_can_buff[0], 0);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    /* CAN接收 */
    result = can_recv(p_can_fd[1], p_can_buff[1], 1);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    // CAN线路状态切换
    check_can_line_status();

    // DIO板卡状态切换
    check_dio_status();

    /* 序号判断 */
    check_offset_err();

    /* 超时判断 */
    result = check_time_out(p_can_buff);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    /* CAN周期任务 */
    result = can_periodic(p_can_fd, p_can_buff);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

#ifdef RUN_TIME
    gettimeofday(&tv_tmp, NULL);
    if (11 < (tv_tmp.tv_sec - tv.tv_sec) * 1000 + (tv_tmp.tv_usec - tv.tv_usec) / 1000)
    {
        system("date \"+%H:%M:%S\"");
        print_line("can_periodic run time = %dms\r\n", (tv_tmp.tv_sec - tv.tv_sec) * 1000 + (tv_tmp.tv_usec - tv.tv_usec) / 1000);
    }
#endif

    /*  TCP任务 */
    memset(tcp_buff, 0, sizeof(tcp_buff));

    // 打包TCP数据
    result = set_tcp_data(tcp_buff);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

    // TCP发送数据
    result = tcp_send(tcp_buff);
    if (-1 == result)
    {
        DEBUG_INFO();
        save_log(2, __LINE__, NULL);
        return -1;
    }

#ifdef RUN_TIME
    gettimeofday(&tv, NULL);
    if (5 < (tv.tv_sec - tv_tmp.tv_sec) * 1000 + (tv.tv_usec - tv_tmp.tv_usec) / 1000)
    {
        system("date \"+%H:%M:%S\"");
        print_line("tcp_send run time = %dms\r\n", (tv.tv_sec - tv_tmp.tv_sec) * 1000 + (tv.tv_usec - tv_tmp.tv_usec) / 1000);
    }
#endif

    return 0;
}
