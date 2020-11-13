/***********************************************************************
*   Copyright (C) JWD Automotive Co., Ltd.				               * 
*	All Rights Reserved.            					               *
*   Department : iCAS SW      									       *
*   AUTHOR	   :            										   *
************************************************************************
* Object        : 
* Module        : energy_recovery.c
* Instance      : 
* Description   : Brake energy recovery
*-----------------------------------------------------------------------
* Version: v0.1
* Date   : Dec 27,2019 
* Author : Gao zehngzhong
***********************************************************************/
/*-History--------------------------------------------------------------
* Version    Date           Name            Changes and comments
------------------------------------------------------------------------
* 0.1	     Dec 27,2019    Gao Zhengzhong  Initial version
*=====================================================================*/

#include <stdlib.h>
#include <string.h>

#include "XC23xxBREGS.h"

#include "booster.h"
#include "energy_recovery.h"

#define ENERGY_RECOVERY_GLOBALS

ENERGY_RECOVERY_STAT INT16U MCU_MotToqLmt; 
ENERGY_RECOVERY_STAT INT16U MCU_MotActuToq;

/**** Definition of variables ****/
ENERGY_RECOVERY_STAT T_ENERGY_RECOVERY tEnergyRecovery = {0u};

/**** Declaration of functions ****/
#if (ENABLE == FUNC_BRAKE_ENERGY_RECOVERY)
ENERGY_RECOVERY_STAT void EnergyRecovery_McuMotorMaxiumTorque(void);
ENERGY_RECOVERY_STAT void EnergyRecovery_McuMotorActualTorque(void);
ENERGY_RECOVERY_STAT void EnergyRecovery_McuMotorObjTorqueRateOfChange(void);
#endif

/***********************************************************************
*  Name        : EnergyRecovery_Init
*  Description : After power on, initiliaze brake energy recovery alogrithm variable
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void EnergyRecovery_Init(void)
{
	memset(&tEnergyRecovery, 0, sizeof(tEnergyRecovery));
	return;
}

/***********************************************************************
*  Name        : AdcApp_DeInit
*  Description : During running, need initilize brake energy recovery alogrithm variable
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void EnergyRecovery_DeInit(void)
{
	memset(&tEnergyRecovery, 0, sizeof(tEnergyRecovery));
	return;
}

/***********************************************************************
*  Name        : EnergyRecovery_Algorithm
*  Description : Brake energy recovery control algorithm
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void EnergyRecovery_Algorithm(void)
{
#if (ENABLE == FUNC_BRAKE_ENERGY_RECOVERY)
	EnergyRecovery_McuMotorMaxiumTorque();
	EnergyRecovery_McuMotorActualTorque();
	EnergyRecovery_McuMotorObjTorqueRateOfChange();
#else
	EnergyRecovery_DeInit();
#endif
	return;
}


#if (ENABLE == FUNC_BRAKE_ENERGY_RECOVERY)
/***********************************************************************
*  Name        : EnergyRecovery_McuMotorMaxiumTorque
*  Description : Get vehicle mcu motor maxium brake torque
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ENERGY_RECOVERY_STAT void EnergyRecovery_McuMotorMaxiumTorque(void)
{
	/* Vehicle mcu motor maxium brake torque limiter processing. */
	if (MCU_MotToqLmt < MCU_MOTOR_TORQUE_MAX_LIMIT)
	{
		tEnergyRecovery.mcuMotorMaxiumTorque = MCU_MOTOR_TORQUE_DEFAULT - MCU_MOTOR_TORQUE_MAX_LIMIT;
	}
	else
	{
		/* More than default value, indicate vehicle is accelerate, not have brake torque,
		 * less than default value, indicate vehicle is decelerate, have brake torque. */
		if (MCU_MotToqLmt > MCU_MOTOR_TORQUE_DEFAULT)
		{
			tEnergyRecovery.mcuMotorMaxiumTorque = 0u;
		}
		else
		{
			tEnergyRecovery.mcuMotorMaxiumTorque = MCU_MOTOR_TORQUE_DEFAULT - MCU_MotToqLmt;
		}
	}

	return;
}
#endif


#if (ENABLE == FUNC_BRAKE_ENERGY_RECOVERY)
/***********************************************************************
*  Name        : EnergyRecovery_McuMotorActualTorque
*  Description : Get vehicle mcu motor actual apply brake torque
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ENERGY_RECOVERY_STAT void EnergyRecovery_McuMotorActualTorque(void)
{
	/* Vehicle mcu motor acutal brake torque limiter processing. */
	if (MCU_MotActuToq < MCU_MOTOR_TORQUE_MAX_LIMIT)
	{
		tEnergyRecovery.mcuMotorActualTorque = MCU_MOTOR_TORQUE_DEFAULT - MCU_MOTOR_TORQUE_MAX_LIMIT;
	}
	else
	{
		/* More than default value, indicate vehicle is accelerate, not have brake torque,
		 * less than default value, indicate vehicle is decelerate, have brake torque. */
		if (MCU_MotActuToq > MCU_MOTOR_TORQUE_DEFAULT)
		{
			tEnergyRecovery.mcuMotorActualTorque = 0u;
		}
		else
		{
			tEnergyRecovery.mcuMotorActualTorque = MCU_MOTOR_TORQUE_DEFAULT - MCU_MotActuToq;
		}
	}	
	
	return;
}
#endif


#if (ENABLE == FUNC_BRAKE_ENERGY_RECOVERY)
/***********************************************************************
*  Name        : EnergyRecovery_McuMotorObjTorqueRateOfChange
*  Description : Request mcu motor object torque rate of change
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ENERGY_RECOVERY_STAT void EnergyRecovery_McuMotorObjTorqueRateOfChange(void)
{
	INT8U i;
	ENERGY_RECOVERY_STAT INT16U objTorque[7] = {0u};

	/* Calculate the change rate of braking torque value of the requested motor. */
	tEnergyRecovery.mcuMotorObjTorqueRateOfChange = 
		  7 * abs(tEnergyRecovery.mcuMotorObjTorque - objTorque[0])
		+ 5 * abs(objTorque[6] - objTorque[1])
		+ 3 * abs(objTorque[5] - objTorque[2])
		+     abs(objTorque[4] - objTorque[3]);

	for (i=0; i<sizeof(objTorque)-1; i++)
	{
		objTorque[i] = objTorque[i+1];
	}

	objTorque[6] = tEnergyRecovery.mcuMotorObjTorque;
	return;
}
#endif



/***********************************************************************
*  Name        : Booster_BrakeTorqueReduction
*  Description : Judge brake torque reduction value.
*  Parameter   : torque, coef
*  Returns     : tempTorque
***********************************************************************/
ENERGY_RECOVERY_STAT INT16U EnergyRecovery_BrakeTorqueReduction(INT16U torque, INT8U coef)
{
	INT16U tempTorque;

	if (torque > MCU_MOTOR_TORQUE_DEFAULT)
	{
		torque = MCU_MOTOR_TORQUE_DEFAULT;
	}
	else
	{
		if (torque < MCU_MOTOR_TORQUE_MAX_LIMIT)
		{
			torque = MCU_MOTOR_TORQUE_MAX_LIMIT;
		}
	}

	tempTorque = MCU_MOTOR_TORQUE_DEFAULT - torque;

	if (coef > 1)
	{
		coef = 1;
	}
	else
	{
		if (coef < 0)
		{
			coef = 0;
		}
	}
	
	tempTorque = MCU_MOTOR_TORQUE_DEFAULT - tempTorque * coef;
	return tempTorque;
}


#if (ENABLE == FUNC_BRAKE_ENERGY_RECOVERY)
/***********************************************************************
*  Name        : EnergyRecovery_CalculateFrictionObjTorque
*  Description : Calculate friction brake object torque.
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ENERGY_RECOVERY_EXT INT16U EnergyRecovery_CalculateFrictionObjTorque(void)
{
	const T_BOOSTER *ptr_booster = Booster_AlogrithmData();
	INT16U frictionBrakeObjTorque;
	ENERGY_RECOVERY_STAT INT16U frictionBrakeObjTorqueIncrement;
	ENERGY_RECOVERY_STAT INT8U hybridBrakeRelaseCnt = 0;
	ENERGY_RECOVERY_STAT INT16S mcuMotorObjTorqueRateOfChangeLast = 0;

	switch (ptr_booster->brakeStatus)
	{
		case STATUS_INIT_BRAKE:
			frictionBrakeObjTorque = 0u;
			frictionBrakeObjTorqueIncrement = 0;
			tEnergyRecovery.mcuMotorObjTorque = MCU_MOTOR_TORQUE_DEFAULT;			
			break;

		case STATUS_FRICTION_BRAKE:
		case STATUS_TRANSIENT:
			/* Judge VCU motor torque facutal value cannot apply brake force requirement. 
			 * Not request VCU motor torque agian. */
			if (ptr_booster->totalTorqueRequest < tEnergyRecovery.mcuMotorActualTorque)
			{
				tEnergyRecovery.mcuMotorObjTorque = MCU_MOTOR_TORQUE_DEFAULT;
				frictionBrakeObjTorque = tEnergyRecovery.mcuMotorActualTorque - ptr_booster->totalTorqueRequest;
			}
			else
			{
				frictionBrakeObjTorque = 0u;
				tEnergyRecovery.mcuMotorObjTorque = MCU_MOTOR_TORQUE_DEFAULT;
			}		
			break;

		case STATUS_HYBRID_BRAKE:
			/* Judge VCU motor torque capacity value cannot apply brake force requirement. */
			if (ptr_booster->totalTorqueRequest < tEnergyRecovery.mcuMotorMaxiumTorque)
			{
				tEnergyRecovery.mcuMotorObjTorque = tEnergyRecovery.mcuMotorMaxiumTorque;
				tEnergyRecovery.brakeMotorObjTorque = tEnergyRecovery.mcuMotorMaxiumTorque - ptr_booster->totalTorqueRequest;
			}
			else
			{
				tEnergyRecovery.brakeMotorObjTorque = 0u;
				tEnergyRecovery.mcuMotorObjTorque = ptr_booster->totalTorqueRequest;
			}

			frictionBrakeObjTorque = tEnergyRecovery.brakeMotorObjTorque;	
			break;

		case STATUS_RELASE_HYBRID_BRAKE:
			/* Energy recovery brake torque reduction */
			tEnergyRecovery.mcuMotorObjTorque = EnergyRecovery_BrakeTorqueReduction(ptr_booster->totalTorqueRequest, ptr_booster->coef);
			if (tEnergyRecovery.mcuMotorObjTorque < tEnergyRecovery.mcuMotorMaxiumTorque)
			{
				tEnergyRecovery.mcuMotorObjTorque = tEnergyRecovery.mcuMotorMaxiumTorque;
			}

			frictionBrakeObjTorque = tEnergyRecovery.mcuMotorActualTorque - ptr_booster->totalTorqueRequest;
			
			if(tEnergyRecovery.mcuMotorObjTorque > tEnergyRecovery.mcuMotorActualTorque)
			{
				frictionBrakeObjTorque = frictionBrakeObjTorque + (tEnergyRecovery.mcuMotorObjTorque - tEnergyRecovery.mcuMotorActualTorque) >> 1;
			}
			
			if((tEnergyRecovery.mcuMotorObjTorqueRateOfChange > 300) 
				&& (mcuMotorObjTorqueRateOfChangeLast <= 300)
				&& (0 == hybridBrakeRelaseCnt))
			{
				hybridBrakeRelaseCnt = 80;
				frictionBrakeObjTorqueIncrement = (MCU_MOTOR_TORQUE_DEFAULT - tEnergyRecovery.mcuMotorActualTorque) >> 1;
				
			}
			else if(hybridBrakeRelaseCnt > 0)
			{
				if(hybridBrakeRelaseCnt < 20)
				{
					frictionBrakeObjTorqueIncrement = frictionBrakeObjTorqueIncrement * hybridBrakeRelaseCnt / 20;
				}
				else if(hybridBrakeRelaseCnt > 60)
				{
					frictionBrakeObjTorqueIncrement = frictionBrakeObjTorqueIncrement * (80 - hybridBrakeRelaseCnt) / 20;
				}
				
				frictionBrakeObjTorque = frictionBrakeObjTorque + frictionBrakeObjTorqueIncrement;
				hybridBrakeRelaseCnt--;
			}
			
			mcuMotorObjTorqueRateOfChangeLast = tEnergyRecovery.mcuMotorObjTorqueRateOfChange;
			break;

		default:
			frictionBrakeObjTorque = 0u;	
			tEnergyRecovery.mcuMotorObjTorque = MCU_MOTOR_TORQUE_DEFAULT;			
			break;
	}

	/* VCU motor brake torque requirement value limiting range. */
	if (tEnergyRecovery.mcuMotorObjTorque < MCU_MOTOR_TORQUE_MAX_LIMIT)
	{
		tEnergyRecovery.mcuMotorObjTorque = MCU_MOTOR_TORQUE_MAX_LIMIT;
	}
	else
	{
		if (tEnergyRecovery.mcuMotorObjTorque > MCU_MOTOR_TORQUE_DEFAULT)
		{
			tEnergyRecovery.mcuMotorObjTorque = MCU_MOTOR_TORQUE_DEFAULT;
		}
	}

	return frictionBrakeObjTorque;
}
#endif

/***********************************************************************
*  Name        : EnergyRecovery_AlogrithmData
*  Description : Brake energy recovery alogrithm data
*  Parameter   : None
*  Returns     : Brake energy recovery alogrithm data structure access address
***********************************************************************/
const T_ENERGY_RECOVERY* EnergyRecovery_AlogrithmData(void)
{
	return (&tEnergyRecovery);
}

/* _END_OF_ENERGY_RECOVERY_ */