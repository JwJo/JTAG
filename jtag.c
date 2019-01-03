
#include <cgpro_chip/TMS570/tms570ls1224.h>
#include <cgpro_chip/bdm_chip_operation_if.h>
#include "bdm.h"
#include "Board_Vcc.h"
#include  <os.h>
#include  <cpu.h>
#include "hal_usart.h"   
#include <cgpro_chip/eeprom_chip/eeprom_chip_operation_comm.h>


/**
 * @brief  引脚初始化
 * @details 	
 * @author   	
 * @data    	20181016
 */
void JTAGTestPin_Init(void)
{
    RCC_APB2PeriphClockCmd(JTAG_PIN_RCC,ENABLE);
    GPIO_InitTypeDef GPIOStructureInit;
    GPIOStructureInit.GPIO_Pin = TARG_TMS_PIN;
    GPIOStructureInit.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIOStructureInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(JTAG_PIN_PORT, &GPIOStructureInit);
    
    GPIOStructureInit.GPIO_Pin = TARG_TCK_PIN;
    GPIOStructureInit.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIOStructureInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(JTAG_PIN_PORT, &GPIOStructureInit);
        
    GPIOStructureInit.GPIO_Pin = TARG_TDI_PIN;           
    GPIOStructureInit.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIOStructureInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(JTAG_PIN_PORT, &GPIOStructureInit);
        
    GPIOStructureInit.GPIO_Pin = TARG_TDO_PIN;
    GPIOStructureInit.GPIO_Mode = GPIO_Mode_IPD;      //下拉输入
    GPIO_Init(JTAG_PIN_PORT, &GPIOStructureInit);
    
    GPIOStructureInit.GPIO_Pin = TARG_RST_PIN;
    GPIOStructureInit.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIOStructureInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(JTAG_PIN_PORT, &GPIOStructureInit);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    GPIOStructureInit.GPIO_Pin = GPIO_Pin_8;
    GPIOStructureInit.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIOStructureInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIOStructureInit);
    GPIOStructureInit.GPIO_Pin = GPIO_Pin_9;
    GPIOStructureInit.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIOStructureInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIOStructureInit);
    
    POWEREN();
    POWER3V3();
    SYS_DelayMs(400);
    TMS_PIN_L();
    TDI_PIN_L();
    RST_PIN_L();
}


/**
 * @brief  芯片复位,进入Test-Logic-Reset状态
 * @details 	
 * @author   	
 * @data    	20181016
 */
void Chip_Reset(void)
{
    RST_PIN_L();
    SYS_DelayMs(100);
    RST_PIN_H();
}

/**
 * @brief  进入Test-Logic-Reset状态
 * @details 	
 * @author   	
 * @data    	20181016
 */
void Test_Logic_Reset(void)
{
    TMS_PIN_H();
    for(int i = 0; i < 6; i++)
    {
        TCK_PIN_H();
        SYS_DelayMs(1);
        TCK_PIN_L();
        SYS_DelayMs(1);
    }
}

/* 首次通过芯片复位进入Run-TestIDLE状态 
 * tick:保持该状态的时间
 */
void First_EnterRun_TestIDLE(uint32_t tick)
{
    Chip_Reset();	//芯片复位
    SYS_DelayMs(1);
    TCK_PIN_H();	//状态机处于Run_TestIDLE状态
    SYS_DelayMs(tick);
}

//发送32位以下的数据/指令
//request：发送的指令/数据，response：目标芯片反回的值，lenth：指令/数据的位长度
//此函数执行后状态机处于Exit_xR状态
//反回0
uint32_t Shitf_Data_32bit(uint32_t *request, uint32_t *response, uint32_t lenth)
{
	int i;
    uint32_t *p = request;
	uint32_t buf = 0;
	uint32_t cnt = 0;
	uint32_t ack = 0;
	//进入Shitf_xR状态
	TCK_PIN_L();
	TMS_PIN_L();
	SYS_DelayUs(1);
	TCK_PIN_H();
	SYS_DelayUs(1);

	//发送和接收
	for (i = 0; i < lenth; i++)
	{
		TCK_PIN_L();
		if (i == (lenth - 1))
			TMS_PIN_H();
		if (((*p >> i) & 0x01) == 1)
			TDI_PIN_H();
		else
			TDI_PIN_L();
		//SYS_DelayUs(1);
		TCK_PIN_H();
		SYS_DelayUs(1);
		if (Read_TargetTDOPin() == 1)
			buf |= 0x01 << i;
		else
			buf &= ~(0x01 << i);
	}
    *response = buf;
	SYS_DelayUs(500);	//进入Exit_xR状态，持续时间
	return 0;
}

//JTAG-DP寄存器是35位
#define JTAG_DAP_REGISTER_LENTH		35

//发送接收36位数据/指令
//request：发送的指令
//response：接收的数据
//此函数执行后状态机处于Exit_xR状态
//反回应答位
uint32_t Shift_Data_36bit(uint32_t *request, uint32_t *response)
{
	int i;
	uint32_t *p = request;
    uint32_t buf = 0;
	uint32_t cnt = 0;
	uint32_t ack = 0;
	uint32_t total_lenth = JTAG_DAP_REGISTER_LENTH + 1;		//加一位校验位,指令总长度
	uint32_t len = 0;
	//进入Shitf_xR状态
	TCK_PIN_L();
	TMS_PIN_L();
	SYS_DelayUs(1);
	TCK_PIN_H();
	SYS_DelayUs(1);

	len = 3;	//传进来的数组第一个元素为地址 + 读写位(3位)，第二个元素为32位数据，先发第一个元素
LOOP:
//发送和接收
	for (i = 0; i < len; i++)
	{
		TCK_PIN_L();
		if (((*p >> i) & 0x01) == 1)
			TDI_PIN_H();
		else
			TDI_PIN_L();
		//SYS_DelayUs(1);
		TCK_PIN_H();
		SYS_DelayUs(1);
		if (Read_TargetTDOPin() == 1)
			buf |= 0x01 << i;
		else
			buf &= ~(0x01 << i);
	}
	if(len < 32)	//发送后32位数据/指令
	{
		ack = buf;
		len = 32;	//发送第二个元素32位数据 
		p++;
		goto LOOP;
	}
    *response = buf;
	//最后一个校验位
	TCK_PIN_L();
    TMS_PIN_H();
	TDI_PIN_L();
	TCK_PIN_H();
	SYS_DelayUs(500);	//进入Exit_xR状态，持续时间
	return ack;
}

uint32_t Shift_IR(uint32_t *request, uint32_t *response, uint32_t lenth)
{
	uint32_t ret = 0;
	ret = Shitf_Data_32bit(request, response, lenth);
	return ret;
}

uint32_t Shift_DR(uint32_t *request, uint32_t *response, uint32_t lenth)
{
	uint32_t ret = 0;
	if (lenth > 32)
	{
		ret = Shift_Data_36bit(request, response);
	}
	else
		ret = Shitf_Data_32bit(request, response, lenth);
	return ret;
}

//写指令寄存器,cmd:指令,response:响应值，len：指令长度
//tick：写完指令之后再进入Run_TestIDLE状态时保持的时间,uint:单位
uint32_t Set_IR_Register(uint32_t *cmd, uint32_t *response, uint32_t len, uint32_t tick,uint32_t unit)
{
	uint32_t ret = 0;
	Select_DRScan();
	Select_IRScan();
	Capture_IR();
	ret = Shift_IR(cmd, response, len);
	Update_IR();
	Run_TestIDLE_NoReset(tick, unit);
	return ret;
}

//写数据寄存器,cmd:数据,response:响应值，len：数据长度
//tick：写完数据之后再进入Run_TestIDLE状态时保持的时间,uint:单位
uint32_t Set_DR_Register(uint32_t *cmd, uint32_t *response, uint32_t len, uint32_t tick, uint32_t unit)
{
	uint32_t ret = 0;
	Select_DRScan();
	Capture_DR();
	ret = Shift_DR(cmd, response, len);
	Update_DR();
	Run_TestIDLE_NoReset(tick, unit);
	return ret;
}


void Read_IDCODE(uint32_t *id)
{
	uint32_t cmd = IDCODE;
	Set_IR_Register(&cmd, id, 6, 1,MS);
	cmd = READ;
	Set_DR_Register(&cmd, id, 32, 1, MS);
}

uint32_t Read_DAP_IDCODE(uint32_t *id)
{
	uint32_t cmd = (BYPASS << 4) | DAP_IDCODE;
	Set_IR_Register(&cmd, id, 10, 1, MS);
	if (*id != 0x011)	//没应答
		return -1;
	cmd = 0xffffffff;
	Set_DR_Register(&cmd, id, 32, 1, MS);
	return 0;
}

//配置CONNECT寄存器
//反回 -1 ：失败，0：成功
uint32_t Conncet_to_DAP(uint32_t *response)
{
	uint32_t cmd = CONNECT;
	Set_IR_Register(&cmd, response, 6, 3, MS);
	cmd = READ;
	Set_DR_Register(&cmd, response, 8, 3, MS);
	if (*response != CONNCET_FLAG)
	{
		cmd = WRITE_CONNECT_FLAG;	//配置CONNCET寄存器
		Set_DR_Register(&cmd, response, 8, 3, MS);
	}
	cmd = READ;
	Set_DR_Register(&cmd, response, 8, 3, MS);
	if (*response != CONNCET_FLAG)
		return -1;
	return 0;
}

//执行两次Set_DR_Register函数
//cmd:命令
//ret:读到的值
uint32_t Twice_Operation(uint32_t *cmd, uint32_t *ret, uint32_t len, uint32_t tick,uint32_t uint)
{
	uint32_t ack = 0;
	Set_DR_Register(cmd, ret, len, tick, uint);
	ack = Set_DR_Register(cmd, ret, len, tick, uint);
	return ack;
}

//配置SDTAP0寄存器，反回-1失败，反回0成功; response:SDTAP0寄存器的值
uint32_t Config_SDTAP0(uint32_t *response)
{

	uint32_t cmd = 0;
	uint32_t ret = 0;

	cmd = ROUTER;
	/*Set_IR_Register(&cmd,&ret,6,500,US);
	cmd = READ_DAP_CMD;
	Set_DR_Register(&cmd, &ret, 32, 500, US);
	Set_DR_Register(&cmd, &ret, 32, 500, US);
	cmd = CONFIG_DAP_CMD_2;
	Set_DR_Register(&cmd, &ret, 32, 500, US);
	Set_DR_Register(&cmd, &ret, 32, 500, US);
	cmd = READ_DAP_CMD;
	Set_DR_Register(&cmd, &ret, 32, 500, US);
	Set_DR_Register(&cmd, &ret, 32, 500, US);
	cmd = BYPASS;
	Set_IR_Register(&cmd, &ret, 6, 1500, US);
	for (int i = 0; i < 6; i++)
	{
		TCK_PIN_L();
		SYS_DelayUs(1);
		TCK_PIN_H();
		SYS_DelayUs(1);
	}
	Run_TestIDLE_NoReset(3, MS); */
	Select_DRScan();
	Select_IRScan();
	Capture_IR();
	Shift_IR(&cmd, &ret, 6);
	Pause_IR(300);
	Exit2_IR();
	Update_IR();
	Select_DRScan();
	Capture_DR();
	Exit1_DR();
	Pause_DR(300);
	Exit2_DR();
	cmd = READ_DAP_CMD;	//读SDTAP0寄存器
	Shift_DR(&cmd, response, 32);
	Pause_DR(300);
	Exit2_DR();
	Update_DR();
	Select_DRScan();
	Capture_DR();
	Shift_DR(&cmd, response, 32);
	Pause_DR(300);
	Exit2_DR();
	cmd = CONFIG_DAP_CMD;	//配置SDTAP0寄存器
	Shift_DR(&cmd, response, 32);
	Pause_DR(300);
	Exit2_DR();

	Update_DR();
	Select_DRScan();
	Capture_DR();
	Shift_DR(&cmd, response, 32);
	Pause_DR(300);
	Exit2_DR();

	cmd = READ_DAP_CMD;	//读SDTAP0寄存器
	Shift_DR(&cmd, response, 32);
	Pause_DR(300);
	Exit2_DR();
	Update_DR();
	Select_DRScan();
	Capture_DR();
	Shift_DR(&cmd, response, 32);
	Pause_DR(300);
	Exit2_DR();

	cmd = CONFIG_DAP_CMD_2;
	Shift_DR(&cmd, response, 32);
	Pause_DR(300);
	Exit2_DR();
	Update_DR();
	Select_DRScan();
	Capture_DR();

	cmd = 0x00;
	Shift_DR(&cmd, response, 32);
	Pause_DR(300);
	Exit2_DR();
	Update_DR();
	Select_DRScan();
	Select_IRScan();
	Capture_IR();
	cmd = 0x3F;
	Shift_IR(&cmd,&ret,6);
	Update_IR();
	Run_TestIDLE_NoReset(2,US);
	for (int i = 0; i < 6; i++)
	{
		TCK_PIN_L();
		SYS_DelayUs(1);
		TCK_PIN_H();
		SYS_DelayUs(1);
	}
	Run_TestIDLE_NoReset(3, MS);
    return 0;
}

uint32_t Config_SYSCTL(uint32_t *response)
{
	uint32_t cmd = ROUTER;
	Set_IR_Register(&cmd, response, 6, 1, MS);
	if (*response != ACK)
		return -1;
	cmd = (READ << 31) | (SYS_CNTL << 24);
	Twice_Operation(&cmd, response, 32, 1, MS);
	cmd = (WRITE << 31) | (SYS_CNTL << 24) | CONFIG_SYSCNTL_WORD;	//bit12 置1
	Set_DR_Register(&cmd, response, 32, 1, MS);
	cmd = (READ << 31) | (SYS_CNTL << 24);
	Twice_Operation(&cmd, response, 32, 2, MS);
	return 0;
}

//检查CONNECT寄存器状态
uint32_t Check_CONNECT_Status(uint32_t *stat)
{
	uint32_t cmd = 0;
	cmd = 0x07;
	Set_IR_Register(&cmd, stat, 6, 1000, US);
	cmd = 0x00;
	Set_IR_Register(&cmd, stat, 6, 1000, US);
    Set_IR_Register(&cmd, stat, 6, 1000, US);
	return 0;
}

////////////////////////////////DAP Opration/////////////////////////////////////////////
//读/写JTAG-DAP(35位)寄存器，包括:JTAG-DP,AHB-AP,APB-AP
//oper:操作(读/写),reg:寄存器，data:32位数据，reponse：该寄存器的值
//函数会操作两次Set_DR_Register()函数
//反回值：应答，非应答
uint32_t Operation_JTAG_DAP_Register_Twice(uint32_t oper, uint32_t reg, uint32_t data, uint32_t *response)
{
	uint32_t buf[2] = { 0 };
	uint32_t ack = 0;
	buf[0] = (reg << 1) | oper;
	buf[1] = data;
	ack = Twice_Operation(buf, response, 36, 1, MS);	//操作35位寄存器就要发36位，暂时不知道为什么要发第36位
	return ack;
}

//操作一次Set_DR_Register函数
uint32_t Operation_JTAG_DAP_Register_Once(uint32_t oper, uint32_t reg, uint32_t data, uint32_t *response)
{
	uint32_t buf[2] = { 0 };
	uint32_t ack = 0;
	buf[0] = (reg << 1) | oper;
	buf[1] = data;
	ack = Set_DR_Register(buf, response, 36, 1, MS);	//操作35位寄存器就要发36位，暂时不知道为什么要发第36位
	return ack;
}

//读/写JTAG-DP的CTRL/STAT寄存器
//oper:操作(读/写),data:32位数据，reponse：该寄存器的值
//反回值：应答，非应答
uint32_t Set_CTRL_STAT_Register(uint32_t oper, uint32_t data, uint32_t *response)
{
	uint32_t ack = 0;
	ack = Operation_JTAG_DAP_Register_Twice(oper, CTRL_STAT_REG, data, response);
	return ack;
}

//选择AP，xxx_AP：选择的AP, Bank_x:选择的Bank
//配置JTAG-DP的Select寄存器
//反回ack
uint32_t Set_Select_AP_Register(uint32_t xxx_AP, uint32_t Bank_x)
{
	uint32_t response = 0;
	uint32_t ack = 0;
	uint32_t Select_Register = 0;
	Select_Register = (xxx_AP << 24) | (Bank_x << 4);
	ack = Operation_JTAG_DAP_Register_Twice(DAP_WRITE, SELECT_AP_REG, Select_Register, &response);
	//Operation_JTAG_DAP_Register(DAP_READ, SELECT_AP_REG, 0, &response);
	return ack;
}
/*
uint32_t Read_APB_ID()
{
    if (Set_Select_AP_Register(APB_AP, BankF) != DAP_DATA_ACK)
		return OPERATION_ERR;
    if (Operation_JTAG_DAP_Register(DAP_READ, IDR, 0, &ret) != DAP_DATA_ACK)
		return -1;
}
*/
uint32_t Set_CSW_Register()
{
	uint32_t ret = 0;
	uint32_t ack = 0;
	ack = Operation_JTAG_DAP_Register_Twice(DAP_WRITE, CSW_REG, 0x43000012, &ret);
	return ack;
}

//初始化DAP，步骤至配置AHB-AP的CSW寄存器
uint32_t JTAG_DAP_Init()
{
    uint32_t ret = 0;
	uint32_t cmd = 0;
    
	First_EnterRun_TestIDLE(30);
	if (Conncet_to_DAP(&ret) != 0)
		return DAP_DISCONNECTED;
	if (Config_SYSCTL(&ret) != 0)
		return CONFIG_SYSCTL_ERR;
	//ret = 0;
	Check_CONNECT_Status(&ret);
    if(ret != CONNCET_FLAG)
    {
        if (Conncet_to_DAP(&ret) != 0)
			return DAP_DISCONNECTED;
    }
	if(Config_SDTAP0(&ret)!=0)
        return CONFIG_SDTAP0_ERR;

	//Read_IDCODE(&ret);
    if(Read_DAP_IDCODE(&ret)!=0)
        return -1;

	cmd = SELET_DPACC;
	Set_IR_Register(&cmd, &ret, 10, 1, MS);
	if (ret != DAP_CMD_ACK)	//未响应
		return -1;
    
    if (Operation_JTAG_DAP_Register_Twice(DAP_WRITE, CTRL_STAT_REG,0x50000000,&ret) != DAP_DATA_ACK) //配置CTRL_STAT寄存器，开启DAP电源
		return -1;
    
	if (Set_CTRL_STAT_Register(DAP_READ,0x00,&ret) != DAP_DATA_ACK)		//设置CTRL_STAT寄存器
		return OPERATION_ERR;
	if (Set_Select_AP_Register(AHB_AP, Bank0) != DAP_DATA_ACK)	//设置Select寄存器选择AHB-AP BANK0
		return OPERATION_ERR;

	cmd = SELET_APACC;
	Set_IR_Register(&cmd, &ret, 10, 1, MS);	//选择APACC
	if (ret != DAP_CMD_ACK)	//未响应
		return -1;

	if (Set_CSW_Register() != DAP_DATA_ACK)	//设置CSW寄存器
		return -1;
    if (Operation_JTAG_DAP_Register_Twice(DAP_READ, CSW_REG, 0, &ret) != DAP_DATA_ACK)
		return -1;
    return SUCCESS;
}

//读取TAR寄存器值
uint32_t Read_TAR_Register(uint32_t *response)
{
	uint32_t cmd = SELET_DPACC;
	Set_IR_Register(&cmd, response, 10, 1, MS);	//选择DPACC
	if (*response != DAP_CMD_ACK)	//未响应
		return DAP_NACK;
	if (Operation_JTAG_DAP_Register_Twice(DAP_READ, CTRL_STAT_REG, 0, response) != DAP_DATA_ACK)	//读取CTRL_STAT寄存器检测DAP调试电源状态
		return DAP_NACK;
	if (*response != DAP_POWER_FLAG)
		return DAP_NO_POWER;					//DAP电源未开启
	cmd = SELET_APACC;
	Set_IR_Register(&cmd, response, 10, 1, MS);	//选择APACC
	if (*response != DAP_CMD_ACK)	//未响应
		return DAP_NACK;
	if (Operation_JTAG_DAP_Register_Twice(DAP_READ, TAR_REG, 0, response) != DAP_DATA_ACK)	//读取TAR寄存器值
		return DAP_NACK;
	return SUCCESS;
}

uint32_t readout[1024] = { 0 };
uint32_t Read_Byte(uint32_t addr, uint8_t* buf, uint32_t length)
{
	uint32_t ret = 0;
    uint32_t tmp[2] = { 0 };
	tmp[0] = (DRW_REG << 1) | DAP_READ;
	tmp[1] = 0;
	
	if (Operation_JTAG_DAP_Register_Twice(DAP_WRITE, TAR_REG, 0, &ret) != DAP_DATA_ACK)	//设置地址
		return -1;
	if (Operation_JTAG_DAP_Register_Twice(DAP_READ, TAR_REG, 0, &ret) != DAP_DATA_ACK)	//读取地址
		return -1;
	for (int i = 0; i < 1024; i++)
	{
		if (Set_DR_Register(tmp, &readout[i], 36, 500, US)!=DAP_DATA_ACK)	//读数据，执行一次读取4个字节
			return -1;
	}
	memcpy(buf, tmp, length);
	return 1;
}


bool chip_tms570_Init(uint8_t* buffer)
{
    JTAGTestPin_Init();
    hc595_gpio_init();
	if(JTAG_DAP_Init()!=SUCCESS)
        return 0;
    return 1;
}

uint32_t chip_tms570_read(uint32_t addr,uint8_t* buf,uint32_t length,uint32_t alllength)
{
    uint32_t ret = 0;
	uint32_t cmd = 0;
    if (Operation_JTAG_DAP_Register_Twice(DAP_WRITE, TAR_REG, 0x08000000, &ret) != DAP_DATA_ACK)	//设置地址
		return -1;
	if (Operation_JTAG_DAP_Register_Twice(DAP_WRITE, DRW_REG, 0x11111111, &ret) != DAP_DATA_ACK)	
		return -1;
	if (Operation_JTAG_DAP_Register_Twice(DAP_WRITE, TAR_REG, 0, &ret) != DAP_DATA_ACK)	//读取TAR寄存器值
		return DAP_NACK;
	if (Operation_JTAG_DAP_Register_Twice(DAP_READ, DRW_REG, 0, &ret) != DAP_DATA_ACK)	//读取TAR寄存器值
		return DAP_NACK;
	if (Operation_JTAG_DAP_Register_Twice(DAP_WRITE, TAR_REG, 0x08000000, &ret) != DAP_DATA_ACK)	//读取TAR寄存器值
		return DAP_NACK;
	if (Operation_JTAG_DAP_Register_Twice(DAP_READ, DRW_REG, 0, &ret) != DAP_DATA_ACK)	//读取TAR寄存器值
		return DAP_NACK;
	//Read_Byte(addr, buf, 4096);
	return length;
}

__root static BDM_Chip_OperationIf_t TC1793_Operation = {
		.model			    = "TC1793XXXXXXXXXX",
		.mask      		    = "XXXXX",
		.sn_number 		    = "XXXXXXXX",
        // .powerup         = chip_avr_PowerOn,
        //.poweroff           = chip_tc1793_PowerOff,
		.init               = chip_tms570_Init,
		//.write      	    = chip_tc1793_write,
        .read               = chip_tms570_read,
        //.erase_sector       = chip_tc1793_Erase,
        //.write_fuselock     = chip_tc1793_Verify,
};

bool TC1793_Init()
{
	return  BDM_Chip_OperationIf_Regsiter  (&TC1793_Operation );
}

















