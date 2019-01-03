#ifndef _TMS570LS1224_H_
#define _TMS570LS1224_H_

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"


//注意：定义的引脚是目标板上的引脚名

#define     JTAG_PIN_RCC        RCC_APB2Periph_GPIOE
#define     JTAG_PIN_PORT        GPIOE
#define     TARG_TMS_PIN         GPIO_Pin_11     
#define     TARG_TCK_PIN         GPIO_Pin_12     
#define     TARG_TDI_PIN         GPIO_Pin_13     
#define     TARG_TDO_PIN         GPIO_Pin_14     
#define     TARG_RST_PIN         GPIO_Pin_15     

#define     JTAG_EN_RCC         RCC_APB2Periph_GPIOE
#define     JTAG_EN_PORT        GPIOE
#define     TARG_JTAG_EN_PIN    GPIO_Pin_4      //10

#define     TMS_PIN_H()        GPIO_SetBits(JTAG_PIN_PORT,TARG_TMS_PIN)    
#define     TMS_PIN_L()        GPIO_ResetBits(JTAG_PIN_PORT,TARG_TMS_PIN)    

#define     TCK_PIN_H()        GPIO_SetBits(JTAG_PIN_PORT,TARG_TCK_PIN)    
#define     TCK_PIN_L()        GPIO_ResetBits(JTAG_PIN_PORT,TARG_TCK_PIN)   

#define     TDI_PIN_H()        GPIO_SetBits(JTAG_PIN_PORT,TARG_TDI_PIN)    
#define     TDI_PIN_L()        GPIO_ResetBits(JTAG_PIN_PORT,TARG_TDI_PIN)  

#define     RST_PIN_H()        GPIO_SetBits(JTAG_PIN_PORT,TARG_RST_PIN)    
#define     RST_PIN_L()        GPIO_ResetBits(JTAG_PIN_PORT,TARG_RST_PIN) 

#define     POWEREN()           GPIO_SetBits(GPIOC,GPIO_Pin_8)
#define     POWERDISEN()        GPIO_ResetBits(GPIOC,GPIO_Pin_8)
#define     POWER4V5()          GPIO_SetBits(GPIOC,GPIO_Pin_9)
#define     POWER3V3()          GPIO_ResetBits(GPIOC,GPIO_Pin_9)

#define     Read_TargetTDOPin()  GPIO_ReadInputDataBit(JTAG_PIN_PORT,TARG_TDO_PIN)

#define		Select_ScanState()	do{\
									TCK_PIN_L();\
									TMS_PIN_H();\
									SYS_DelayUs(1);\
									TCK_PIN_H();\
									SYS_DelayUs(1);\
								}while(0)

#define		Capture_State()		do{\
									TCK_PIN_L();\
									TMS_PIN_L();\
									SYS_DelayUs(1);\
									TCK_PIN_H();\
									SYS_DelayUs(1);\
								}while(0)

#define		Exit_State()	do{\
									TCK_PIN_L();\
									TMS_PIN_H();\
									SYS_DelayUs(1);\
									TCK_PIN_H();\
									SYS_DelayUs(300);\
								}while(0)

#define		Pause_State(tick)		do{\
									TCK_PIN_L();\
									TMS_PIN_L();\
									SYS_DelayUs(1);\
									TCK_PIN_H();\
									SYS_DelayUs(tick);\
								}while(0)

#define		Update_State()		do{\
									TCK_PIN_L();\
									TMS_PIN_H();\
									TDI_PIN_H();\
									SYS_DelayUs(1);\
									TCK_PIN_H();\
									SYS_DelayUs(1);\
								}while(0)

#define		MS	1
#define		US	0
//在复位状态下再次进入Run_TestIDLE状态，tock：保持状态的时间,unit:单位(ms/us)
#define		Run_TestIDLE_NoReset(tock,unit)	do{\
												TCK_PIN_L();\
												TMS_PIN_L();\
												SYS_DelayUs(1);\
												TCK_PIN_H();\
												switch(unit)\
												{\
													case MS:\
														SYS_DelayMs(tock); break;\
													case US:\
														SYS_DelayUs(tock); break;\
													default:\
														break;\
												}\
											}while(0)

#define Select_DRScan()		Select_ScanState()
#define Select_IRScan()		Select_ScanState()
#define Capture_DR()		Capture_State()
#define Capture_IR()		Capture_State()
#define Exit1_DR()			Exit_State()
#define Exit1_IR()			Exit_State()
#define Exit2_IR()			Exit_State()
#define Exit2_DR()			Exit_State()
#define Update_DR()			Update_State()
#define Update_IR()			Update_State()
#define Pause_IR(tk)		Pause_State(tk)
#define Pause_DR(tk)		Pause_State(tk)


#define		READ				0x00	//TAP指令读写标志在最高位
#define		WRITE				0x01
#define		ACK					0x01

        /* JTAG-ICEPick TAP寄存器地址 */
#define     SYS_CNTL				0x01
#define     SDTAP0_ADDR				(0x02<<4)
#define		CONFIG_SYSCNTL_WORD		0x021000

		/*ICEPick TAP Instruction 6bit*/
#define     ROUTER              0x02
#define     CONNECT             0x07
#define     BYPASS				0x3F
#define     IDCODE              0x04
#define		CONNCET_FLAG		0x09
#define		WRITE_CONNECT_FLAG	((WRITE << 7) | CONNCET_FLAG)

		//DAP Instruction 4bit
#define     DPACC           0x0A
#define     APACC           0x0B
#define     ABORT           0x08
#define		DAP_IDCODE		0x0e
#define     DAP_BYPASS      0x0F
#define		DAP_CONFIGWORD	0x002008 
#define     DAP_DATA_ACK    0x02	//35位数据的ack
#define     DAP_CMD_ACK     0x011	//10位命令的ack



		/* DAP Register Command*/
#define		READ_DAP_CMD			((READ << 31) | ((SDTAP0_ADDR & 0x7f) << 24))
#define		CONFIG_DAP_CMD			((WRITE << 31) | ((SDTAP0_ADDR & 0x7f) << 24) | DAP_CONFIGWORD)
#define		CONFIG_DAP_CMD_2		((WRITE << 31) | ((SDTAP0_ADDR & 0x7f) << 24) | DAP_CONFIGWORD+0x100)
#define		SELET_DPACC				((BYPASS << 4) | DPACC)     
#define		SELET_APACC				((BYPASS << 4) | APACC)  

        /* JTAG-DP Register Address */
#define     RESERVED            0x00        //0x00
#define     CTRL_STAT_REG       0x01        //0x04
#define     SELECT_AP_REG       0x02        //0x08
#define     RD_BUFF             0x03        //0x0C
#define		DAP_POWER_FLAG		0xF8000000


#define		DAP_READ			0x01		//DAP指令读写标志在数据最低位
#define		DAP_WRITE			0x00


		/* JTAG APB-AP寄存器 */
#define		CSW_REG			    0x00		//0x00
#define		TAR_REG			    0x01		//0x04
#define		DRW_REG		        0x03		//0x0C
#define		IDR					0x03		//0xFC

#define		CFG			    	0x01		//0x04
#define     DBGBASE             0x02        //0x08
#define		IDR			        0x03		//0x0C

        /* choose AP */
#define     AHB_AP              0x00
#define     APB_AP              0x01
#define     JTAG_AP             0x02

        /* choose Bank */
#define     Bank0               0x00
#define     Bank1               0x01
#define     BankF               0x0F


#define     SUCCESS             0x00
        /* ERROR */
#define     DAP_DISCONNECTED		0x70
#define		CONFIG_SYSCTL_ERR		0x71
#define		CONFIG_SDTAP0_ERR		0x72
#define		OPERATION_ERR			0x73
#define		DAP_NACK				0x74
#define		DAP_NO_POWER			0x75	//DAP电源没开，设置CTRL_STAT寄存器开电源

void wait(uint32_t us);
void JTAGTestPin_Init(void);
void Target_External_Debug(void);
void JTAG_StatusInit(void);
uint32_t Read_JTAG_IDCODE(void);
uint32_t Read_DeviceID(void);
uint8_t ICEPick_Connect(uint32_t *ConnRegister);
uint8_t Read_JTAG_Register(uint32_t *addr, uint32_t *RdOut, uint8_t regnum);
uint8_t Write_JTAG_Register(uint32_t *addr, uint32_t *WriteIn, uint8_t regnum);

uint8_t test_read(uint32_t *count);

#endif
