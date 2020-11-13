
 
#include "MAIN.h"
#include "rte.h"
#include  "aeb.h"


void task2(void);
void task2Normal_v_s(void);
void task2PowerUpTest_v_s(void);
void task2PowerDownTest_v_s(void);
void task2PowerDown_v_s(void);
void task2Fault_v_s(void);





/*!************************************************************************************
*	@fn	        void task2(void)
*	@brief	          任务2函数
*	@author     Matt Zhang
*	@param[in]	void
*	@return	    void
*	@note       在CCU60.c中CCU60_viNodeI3()函数中调用
*	@Data       2019-3-12
****************************************************************************************/
void task2(void)
{

	switch(SystemMode_u8_Sig)
	{
		case PowerUpTestMode:
			task2PowerUpTest_v_s();
		break;

		case NormalMode:
			task2Normal_v_s();
		break;

		case FaultMode:
			task2Fault_v_s();
		break;

		case PowerDownTestMode:
			task2PowerDownTest_v_s();
		break;

		case PowerDownMode:
			task2PowerDown_v_s();
		break;

		default:
		break;
	}
}


/*!************************************************************************************
*	@fn	        void task2PowerUpTest_v_s(void)
*	@brief	         任务2上电测试函数
*	@author     Matt Zhang
*	@param[in]	void
*	@return	    void
*	@note       在Task2.c的task2()函数中调用
*	@Data       2019-3-12
****************************************************************************************/
void task2PowerUpTest_v_s(void)
{

}


/*!************************************************************************************
*	@fn	        void task2Normal_v_s(void)
*	@brief	         任务2正常模式函数
*	@author     Matt Zhang
*	@param[in]	void
*	@return	    void
*	@note       在Task2.c的task2()函数中调用；另外AEB算法放在此函数中
*	@Data       2019-3-12
****************************************************************************************/
INT32U testvalue11=0;
INT32U testvalue12=0;
INT32U testvalue13=0;
void task2Normal_v_s(void)
{
	/*--------------------制动灯控制------------------*/

	if(0)
	{
		IO_vSetPin(IO_P6_2); /* open brake lamp */
	}
	else
	{
		IO_vResetPin(IO_P6_2); /* close brake lamp */
	}
}


/*!************************************************************************************
*	@fn	        void task2PowerDownTest_v_s(void)
*	@brief	         任务2下电测试模式函数
*	@author     Matt Zhang
*	@param[in]	void
*	@return	    void
*	@note       在Task2.c的task2()函数中调用
*	@Data       2019-3-12
****************************************************************************************/
void task2PowerDownTest_v_s(void)
{

}


/*!************************************************************************************
*	@fn	        void task2PowerDown_v_s(void)
*	@brief	         任务2下电模式函数
*	@author     Matt Zhang
*	@param[in]	void
*	@return	    void
*	@note       在Task2.c的task2()函数中调用
*	@Data       2019-3-12
****************************************************************************************/
void task2PowerDown_v_s(void)
{

}


/*!************************************************************************************
*	@fn	        void task2Fault_v_s(void)
*	@brief	         任务2故障模式函数
*	@author     Matt Zhang
*	@param[in]	void
*	@return	    void
*	@note       在Task2.c的task2()函数中调用
*	@Data       2019-3-12
****************************************************************************************/
void task2Fault_v_s(void)
{

}

