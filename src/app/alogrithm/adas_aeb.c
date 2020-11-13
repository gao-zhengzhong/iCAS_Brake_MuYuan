//****************************************************************************
// @Module        Project Settings
// @Filename      adas_aeb.c
// @Project       P300.dav
//----------------------------------------------------------------------------
// @Controller    Infineon XC2365B-40F80
//
// @Compiler      Keil
//
// @Codegenerator 2.0
//
// @Description   This file contains ADAS AEB function.
//
//----------------------------------------------------------------------------
// @Date          2020-04-16 15:47:42
//
//****************************************************************************


/**** Project Includes ****/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "adas_aeb.h"
#include "can_com.h"


/**** Definition of macros ****/
#define ADAS_AEB_GLOBALS


/**** Definition of variables ****/
ADAS_AEB_STAT T_ADAS_AEB_STATUS t_AebInfo = {0u};
ADAS_AEB_STAT T_ADAS_AEB_OBJ_INFO t_AebObjInfo = {0};


/**** Declaration of functions ****/
ADAS_AEB_STAT void FCW_StateMashine(void);
ADAS_AEB_STAT void AEB_StateMashine(void);
ADAS_AEB_STAT void AEB_ModeControl(void);
ADAS_AEB_STAT void AEB_RiskAssessment(void);
ADAS_AEB_STAT void AEB_DangerObjectIdentification(void);
ADAS_AEB_STAT void AEB_TimeToCollisionAssessment(void);
ADAS_AEB_STAT void AEB_OptimalBrakeDecelerationAssessment(void);


/***********************************************************************
*  Name        : AEB_Init
*  Description : None
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void AEB_Init(void)
{
	memset(&t_AebInfo, 0, sizeof(t_AebInfo));
	memset(&t_AebObjInfo, 0, sizeof(t_AebObjInfo));	
}


/***********************************************************************
*  Name        : AEB_DeInit
*  Description : None
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void AEB_DeInit(void) 
{
	memset(&t_AebInfo, 0, sizeof(t_AebInfo));
	memset(&t_AebObjInfo, 0, sizeof(t_AebObjInfo));	
}


/***********************************************************************
*  Name        : AEB_TaskEntry
*  Description : None
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void AEB_TaskEntry(void) 
{
	AEB_ModeControl();
	AEB_DangerObjectIdentification();

	AEB_RiskAssessment();
	AEB_TimeToCollisionAssessment();	

	AEB_StateMashine();
	FCW_StateMashine();
} 


FP32 Dis_b;
FP32 Dis_pb;
FP32 Dis_w;
FP32 Dis_w_exit;

FP32 Dis_b_exit;
FP32 TTC_w = AEB_TTC_WARNING_VALUE;
FP32 TTC_fb = AEB_TTC_FULL_BRAKE_VALUE;
FP32 TTC_pb = AEB_TTC_PARTIAL_BRAKE_VALUE;
FP32 TTC_w_exit = AEB_TTC_EXIT_WARN_VALUE;
FP32 TTC_b_exit = AEB_TTC_EXIT_BRAKE_VALUE;
FP32 ParF_AEB_R_Active_Vhost_entry = AEB_FCW_ENABLE_VELOCITY_LOWER;
FP32 ParF_AEB_R_Active_Vrel_entry = 0.0f;
FP32 ParF_AEB_R_Active_Vrel_exit_w = 0.0f;
FP32 ParF_AEB_R_Active_Vrel_exit_b = 0.0f;
/***********************************************************************
*  Name        : AEB_StateMashine
*  Description : None
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ADAS_AEB_STAT void AEB_StateMashine(void) 
{
	switch (t_AebInfo.AebStatus)
	{
		case AEB_STATUS_OFF:
			AEB_DeInit();
			t_AebInfo.AebStatus = AEB_STATUS_STANDBY;
			break;

		case AEB_STATUS_STANDBY:
			/* 1: AEB_STATUS_STANDBY -> AEB_STATUS_WARNING */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )			
			if ((t_AebInfo.ObjDistLong <= Dis_w) && (t_AebInfo.ObjDistLong > Dis_pb)
			//&& (t_AebInfo.Velocity_host > ParF_AEB_R_Active_Vhost_entry)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (~t_AebInfo.Brake_Pedal_state) && (~t_AebInfo.Standstill_state))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_w) && (t_AebInfo.TimeToCollision > TTC_pb) 
			//&& (t_AebInfo.Velocity_host > ParF_AEB_R_Active_Vhost_entry)
		    && (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
		    && (~t_AebInfo.Brake_Pedal_state) && (~t_AebInfo.Standstill_state))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_WARNING;
			}

			/* 2: AEB_STATUS_STANDBY -> AEB_STATUS_PARTIAL_BRAKE */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_pb) && (t_AebInfo.ObjDistLong > Dis_b) 
			//&& (t_AebInfo.Velocity_host > ParF_AEB_R_Active_Vhost_entry)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (~t_AebInfo.Brake_Pedal_state) && (~t_AebInfo.Standstill_state))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_pb) && (t_AebInfo.TimeToCollision > TTC_fb)
			//&& (t_AebInfo.Velocity_host > ParF_AEB_R_Active_Vhost_entry)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (~t_AebInfo.Brake_Pedal_state) && (~t_AebInfo.Standstill_state))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_PARTIAL_BRAKE;
			}

			/* 3: AEB_STATUS_STANDBY -> AEB_STATUS_ASSIST_BRAKE */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_w) && (t_AebInfo.ObjDistLong > Dis_pb)
			//&& (t_AebInfo.Velocity_host > ParF_AEB_R_Active_Vhost_entry)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (t_AebInfo.Brake_Pedal_state && (~t_AebInfo.Standstill_state)))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_w) && (t_AebInfo.TimeToCollision > TTC_pb) 
			//&& (t_AebInfo.Velocity_host > ParF_AEB_R_Active_Vhost_entry)
		    && (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (t_AebInfo.Brake_Pedal_state && (~t_AebInfo.Standstill_state)))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_ASSIST_BRAKE;
			}

			/* 4: AEB_STATUS_STANDBY -> AEB_SATTUS_FULL_BRAKE */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_b) && (~t_AebInfo.Standstill_state)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			//&& (t_AebInfo.Velocity_host > ParF_AEB_R_Active_Vhost_entry))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_fb) && (~t_AebInfo.Standstill_state)
			//&& (t_AebInfo.Velocity_host > ParF_AEB_R_Active_Vhost_entry)			
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry))
#endif
			{
				t_AebInfo.AebStatus = AEB_SATTUS_FULL_BRAKE;
			}

			/* When vehicle stop, AEB must be resume OFF. */
			if (fabs(t_AebInfo.Velocity_host) < 1e-6)
			{
				t_AebInfo.AebStatus = AEB_STATUS_OFF;
			}
			
			break;

		case AEB_STATUS_WARNING:
			/* 1: AEB_STATUS_WARNING -> AEB_STATUS_STANDBY */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if (t_AebInfo.Standstill_state || ((t_AebInfo.ObjDistLong > Dis_w_exit)
			&& (t_AebInfo.ObjVrelLong > ParF_AEB_R_Active_Vrel_exit_w)))
#else
			if (t_AebInfo.Standstill_state || ((t_AebInfo.TimeToCollision > TTC_w_exit)
			&& (t_AebInfo.ObjVrelLong > ParF_AEB_R_Active_Vrel_exit_w)))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_STANDBY;	
			}

			/* 2: AEB_STATUS_WARNING -> AEB_STATUS_PARTIAL_BRAKE */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_pb) && (t_AebInfo.ObjDistLong > Dis_b) 
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry) 
			&& (~t_AebInfo.Brake_Pedal_state))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_pb) && (t_AebInfo.TimeToCollision > TTC_fb) 
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry) 
			&& (~t_AebInfo.Brake_Pedal_state))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_PARTIAL_BRAKE;	
			}

			/* 3: AEB_STATUS_WARNING -> AEB_SATTUS_FULL_BRAKE */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_b)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_fb)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry))
#endif
			{
				t_AebInfo.AebStatus = AEB_SATTUS_FULL_BRAKE;	
			}

			/* 4: AEB_STATUS_WARNING -> AEB_STATUS_ASSIST_BRAKE */	
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_w) && (t_AebInfo.ObjDistLong > Dis_b) 
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry) 
			&& t_AebInfo.Brake_Pedal_state)
#else
			if ((t_AebInfo.TimeToCollision <= TTC_w) && (t_AebInfo.TimeToCollision > TTC_fb) 
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry) 
			&& t_AebInfo.Brake_Pedal_state)
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_ASSIST_BRAKE;	
			}

			/* When vehicle stop, AEB must be resume OFF. */
			if (fabs(t_AebInfo.Velocity_host) < 1e-6)
			{
				t_AebInfo.AebStatus = AEB_STATUS_OFF;
			}
			
			break;

		case AEB_STATUS_PARTIAL_BRAKE:
			/* 1: AEB_STATUS_PARTIAL_BRAKE -> AEB_STATUS_STANDBY */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if (t_AebInfo.Standstill_state || ((t_AebInfo.ObjDistLong > Dis_w_exit)
			&& (t_AebInfo.ObjVrelLong > ParF_AEB_R_Active_Vrel_exit_w)))
#else
			if (t_AebInfo.Standstill_state || ((t_AebInfo.TimeToCollision > TTC_w_exit)
			&& (t_AebInfo.ObjVrelLong > ParF_AEB_R_Active_Vrel_exit_w)))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_STANDBY;
			}

			/* 2: AEB_STATUS_PARTIAL_BRAKE -> AEB_STATUS_ASSIST_BRAKE */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_pb) && (t_AebInfo.ObjDistLong > Dis_b)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (~t_AebInfo.Brake_Pedal_state))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_pb) && (t_AebInfo.TimeToCollision > TTC_fb)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (~t_AebInfo.Brake_Pedal_state))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_ASSIST_BRAKE;
			}

			/* 3: AEB_STATUS_PARTIAL_BRAKE -> AEB_SATTUS_FULL_BRAKE */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_b) 
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_fb) 
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry))
#endif
			{
				t_AebInfo.AebStatus = AEB_SATTUS_FULL_BRAKE;
			}

			/* When vehicle stop, AEB must be resume OFF. */
			if (fabs(t_AebInfo.Velocity_host) < 1e-6)
			{
				t_AebInfo.AebStatus = AEB_STATUS_OFF;
			}
			
			break;

		case AEB_STATUS_ASSIST_BRAKE:
			/* 1: AEB_STATUS_ASSIST_BRAKE -> AEB_STATUS_STANDBY */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )	
			if (t_AebInfo.Standstill_state || ((t_AebInfo.ObjDistLong > Dis_w_exit)
			&& (t_AebInfo.ObjVrelLong > ParF_AEB_R_Active_Vrel_exit_w)))
#else
			if (t_AebInfo.Standstill_state || ((t_AebInfo.TimeToCollision > TTC_w_exit)
			&& (t_AebInfo.ObjVrelLong > ParF_AEB_R_Active_Vrel_exit_w)))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_STANDBY;
			}

			/* 2: AEB_STATUS_ASSIST_BRAKE -> AEB_SATTUS_FULL_BRAKE */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_b) 
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_fb) 
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry))
#endif
			{
				t_AebInfo.AebStatus = AEB_SATTUS_FULL_BRAKE;
			}

			/* 3: AEB_STATUS_ASSIST_BRAKE -> AEB_STATUS_PARTIAL_BRAKE */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_pb) && (t_AebInfo.ObjDistLong > Dis_b)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (~t_AebInfo.Brake_Pedal_state))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_pb) && (t_AebInfo.TimeToCollision > TTC_fb)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (~t_AebInfo.Brake_Pedal_state))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_PARTIAL_BRAKE;
			}

			/* 4: AEB_STATUS_ASSIST_BRAKE -> AEB_STATUS_WARNING */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_w) && (t_AebInfo.ObjDistLong > Dis_pb)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (~t_AebInfo.Brake_Pedal_state))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_w) && (t_AebInfo.TimeToCollision > TTC_pb)
			&& (t_AebInfo.ObjVrelLong < ParF_AEB_R_Active_Vrel_entry)
			&& (~t_AebInfo.Brake_Pedal_state))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_WARNING;
			}

			/* When vehicle stop, AEB must be resume OFF. */
			if (fabs(t_AebInfo.Velocity_host) < 1e-6)
			{
				t_AebInfo.AebStatus = AEB_STATUS_OFF;
			}
			
			break;			

		case AEB_SATTUS_FULL_BRAKE:
			/* AEB_SATTUS_FULL_BRAKE -> AEB_STATUS_STANDBY */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.Standstill_state && (t_AebInfo.ObjDistLong >= Dis_b_exit))
			|| ((t_AebInfo.ObjDistLong > Dis_w_exit) 
			&& (t_AebInfo.ObjVrelLong > ParF_AEB_R_Active_Vrel_exit_b)))
#else
			if ((t_AebInfo.Standstill_state && (t_AebInfo.TimeToCollision >= TTC_b_exit))
			|| ((t_AebInfo.TimeToCollision > TTC_w_exit) 
			&& (t_AebInfo.ObjVrelLong > ParF_AEB_R_Active_Vrel_exit_b)))
#endif
			{
				t_AebInfo.AebStatus = AEB_STATUS_STANDBY;
			}

			/* When vehicle stop, AEB must be resume OFF. */
			if (fabs(t_AebInfo.Velocity_host) < 1e-6)
			{
				t_AebInfo.AebStatus = AEB_STATUS_OFF;
			}
				
			break;

		default:
			break;
	}
}


FP32 ParF_FCW_R_Active_Vhost_entry = AEB_FCW_ENABLE_VELOCITY_LOWER;
FP32 ParF_FCW_R_TTC_Warning_upper = AEB_TTC_WARNING_VALUE;
FP32 ParF_FCW_R_Active_Vrel_entry = 0.0f;
FP32 ParF_FCW_R_Active_Vrel_exit_w = 0.0f;
/***********************************************************************
*  Name        : FCW_StateMashine
*  Description : None
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ADAS_AEB_STAT void FCW_StateMashine(void)
{
	switch (t_AebInfo.FcwStatus)
	{
		case FCW_STATUS_STANDBY:
			/* 1: FCW_STATUS_STANDBY -> FCW_STATUS_WARNING_1 */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_w) && (t_AebInfo.ObjDistLong > Dis_b)
			//&& (t_AebInfo.Velocity_host > ParF_FCW_R_Active_Vhost_entry)
			&& (t_AebInfo.ObjVrelLong < ParF_FCW_R_Active_Vrel_entry)
			&& (~t_AebInfo.Standstill_state)
			&& (t_AebInfo.TimeToCollision <= ParF_FCW_R_TTC_Warning_upper))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_w) && (t_AebInfo.TimeToCollision > TTC_fb)
			//&& (t_AebInfo.Velocity_host > ParF_FCW_R_Active_Vhost_entry)
			&& (t_AebInfo.ObjVrelLong < ParF_FCW_R_Active_Vrel_entry)
			&& (~t_AebInfo.Standstill_state)
			&& (t_AebInfo.TimeToCollision <= ParF_FCW_R_TTC_Warning_upper))
#endif
			{
				t_AebInfo.FcwStatus = FCW_STATUS_WARNING_1;
			}

			/* 2: FCW_STATUS_STANDBY -> FCW_STATUS_WARNING_2 */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_b)
			//&& (t_AebInfo.Velocity_host > ParF_FCW_R_Active_Vhost_entry)
			&& (t_AebInfo.ObjVrelLong < ParF_FCW_R_Active_Vrel_entry)
			&& (~t_AebInfo.Standstill_state)
			&& (t_AebInfo.TimeToCollision <= ParF_FCW_R_TTC_Warning_upper))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_fb)
			//&& (t_AebInfo.Velocity_host > ParF_FCW_R_Active_Vhost_entry)
			&& (t_AebInfo.ObjVrelLong < ParF_FCW_R_Active_Vrel_entry)
			&& (~t_AebInfo.Standstill_state)
			&& (t_AebInfo.TimeToCollision <= ParF_FCW_R_TTC_Warning_upper))
#endif
			{
				t_AebInfo.FcwStatus = FCW_STATUS_WARNING_2;
			}

			/* When vehicle stop, FCW must be resume OFF. */
			if (fabs(t_AebInfo.Velocity_host) < 1e-6)
			{
				t_AebInfo.FcwStatus = AEB_STATUS_OFF;
			}

			break;

		case FCW_STATUS_WARNING_1:
			/* 1: FCW_STATUS_WARNING_1 -> FCW_STATUS_STANDBY */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong > AEB_TARGET_SAFETY_DISTANCE) || t_AebInfo.Standstill_state 
			|| ((t_AebInfo.ObjDistLong > Dis_w_exit) && (t_AebInfo.ObjVrelLong > ParF_FCW_R_Active_Vrel_exit_w)))
#else
			if ((t_AebInfo.TimeToCollision > AEB_TTC_DEFAULT_VALUE) || t_AebInfo.Standstill_state 
			|| ((t_AebInfo.TimeToCollision > TTC_w_exit) && (t_AebInfo.ObjVrelLong > ParF_FCW_R_Active_Vrel_exit_w)))
#endif
			{
				t_AebInfo.FcwStatus = FCW_STATUS_STANDBY;
			}

			/* 2: FCW_STATUS_WARNING_1 -> FCW_STATUS_WARNING_2 */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )
			if ((t_AebInfo.ObjDistLong <= Dis_b) && (~t_AebInfo.Standstill_state) 
			//&& (t_AebInfo.Velocity_host > ParF_FCW_R_Active_Vhost_entry)
			&& (t_AebInfo.ObjVrelLong < ParF_FCW_R_Active_Vrel_entry))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_fb) && (~t_AebInfo.Standstill_state) 
			//&& (t_AebInfo.Velocity_host > ParF_FCW_R_Active_Vhost_entry)					
			&& (t_AebInfo.ObjVrelLong < ParF_FCW_R_Active_Vrel_entry))
#endif
			{
				t_AebInfo.FcwStatus = FCW_STATUS_WARNING_2;
			}

			/* When vehicle stop, FCW must be resume OFF. */
			if (fabs(t_AebInfo.Velocity_host) < 1e-6)
			{
				t_AebInfo.FcwStatus = AEB_STATUS_OFF;
			}
			
			break;

		case FCW_STATUS_WARNING_2:
			/* 1: FCW_STATUS_WARNING_2 -> FCW_STATUS_STANDBY */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong > AEB_TARGET_SAFETY_DISTANCE) || t_AebInfo.Standstill_state 
			|| ((t_AebInfo.ObjDistLong > Dis_w_exit) && (t_AebInfo.ObjVrelLong > ParF_FCW_R_Active_Vrel_exit_w)))
#else
			if ((t_AebInfo.TimeToCollision > AEB_TTC_DEFAULT_VALUE) || t_AebInfo.Standstill_state 
			|| ((t_AebInfo.TimeToCollision > TTC_w_exit) && (t_AebInfo.ObjVrelLong > ParF_FCW_R_Active_Vrel_exit_w)))
#endif
			{
				t_AebInfo.FcwStatus = FCW_STATUS_STANDBY;	
			}

			/* 2: FCW_STATUS_WARNING_2 -> FCW_STATUS_WARNING_1 */
#if ( 1 == ADAS_AEB_DISTANCE_ALOGRITHM )				
			if ((t_AebInfo.ObjDistLong <= Dis_w) && (t_AebInfo.ObjDistLong > Dis_b_exit)
			//&& (t_AebInfo.Velocity_host > ParF_FCW_R_Active_Vhost_entry) 
			&& (~t_AebInfo.Standstill_state))
#else
			if ((t_AebInfo.TimeToCollision <= TTC_w) && (t_AebInfo.TimeToCollision > TTC_b_exit)
			//&& (t_AebInfo.Velocity_host > ParF_FCW_R_Active_Vhost_entry) 
			&& (~t_AebInfo.Standstill_state))
#endif
			{
				t_AebInfo.FcwStatus = FCW_STATUS_WARNING_1;
			}

			/* When vehicle stop, FCW must be resume OFF. */
			if (fabs(t_AebInfo.Velocity_host) < 1e-6)
			{
				t_AebInfo.FcwStatus = AEB_STATUS_OFF;
			}
			
			break;

		default:
			break;
	}
}


/***********************************************************************
*  Name        : AEB_ModeControl
*  Description : None
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ADAS_AEB_STAT void AEB_ModeControl(void)
{
	/* AEB/FCW enable condition:
	 * 1) Velocity > 5km/h and < 80km/h
	 * 2) Gear D position
	 * 3) Steering wheel angle is small
	 * 4) Turn light steering lamp invalid */
	if (((t_AebInfo.Velocity_host < AEB_FCW_ENABLE_VELOCITY_LOWER) || (t_AebInfo.Velocity_host > AEB_FCW_ENABLE_VELOCITY_UPPER))
	|| (0) // TODO: gear signal not D position
	|| (0) // TODO: excessive steering wheel angle
	|| (0) // TODO: accelerator pedeal valid
	|| (0)) // TODO: turn light steering lamp valid
	{
		t_AebInfo.Standstill_state = FALSE;
	}
	else
	{
		t_AebInfo.Standstill_state = FALSE;
	}
}


FP32 Calc_LongDistance(INT16U Distance)
{
	FP32 retValue;
	retValue = (FP32)(Distance * 0.2f - 500.0f); 
	return retValue;
}


FP32 Calc_LatDistance(INT16U Distance)
{
	FP32 retValue;
	retValue = (FP32)(Distance * 0.2f - 204.6f); 
	return retValue;
}


FP32 Calc_LongVelocity(INT16U Velocity)
{
	FP32 retValue;
	retValue = (FP32)(Velocity * 0.25f - 128.0f); 
	return retValue;	
}


FP32 Calc_LatVelocity(INT16U Velocity)
{
	FP32 retValue;
	retValue = (FP32)(Velocity * 0.25f - 64.0f); 
	return retValue;	
}


FP32 Calc_LongAccelerate(INT16U Accelerate)
{
	FP32 retValue;
	retValue = (FP32)(Accelerate * 0.01f - 10.0f); 
	return retValue;	
}


FP32 Calc_LatAccelerate(INT16U Accelerate)
{
	FP32 retValue;
	retValue = (FP32)(Accelerate * 0.01f - 2.5f); 
	return retValue;	
}

FP32 Calc_HostVelocity(INT16U Velocity)
{
	FP32 retValue;
	retValue = (FP32)(Velocity * 0.01f); 
	return retValue;	
}

/***********************************************************************
*  Name        : AEB_DangerObjectIdentification
*  Description : None
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ADAS_AEB_STAT void AEB_DangerObjectIdentification(void) 
{
	INT8U i, tempDangerObjIndex = 0;
	FP32 tempObjDistLong = 0;
	
	const T_CAN_COM *ptr = CanCom_ApplCanComDataAccess();

	/* Before identification, taget longtitude distance initialize safety distance. */
	tempObjDistLong = AEB_TARGET_SAFETY_DISTANCE;
	
	/* Screening the most danderous target information. */	
	for (i = 0u; i < RADAR_DETECT_OBJECT_NUM; i++)
	{
		if ((tempObjDistLong > Calc_LongDistance(ptr->RadarData.ObjGeneral[i].Object_DistLong)) && (0.0f != Calc_LongDistance(ptr->RadarData.ObjGeneral[i].Object_DistLong)))
		{
			if ((Calc_LatDistance(ptr->RadarData.ObjGeneral[i].Object_DistLat) < 1.04f) && (Calc_LatDistance(ptr->RadarData.ObjGeneral[i].Object_DistLat) > -1.04f))
			{
				tempDangerObjIndex = i;
				tempObjDistLong = Calc_LongDistance(ptr->RadarData.ObjGeneral[i].Object_DistLong);				
			}
		}
	}

	/* Danger object general info */
	t_AebObjInfo.Object_ID = ptr->RadarData.ObjGeneral[tempDangerObjIndex].Object_ID;
	t_AebObjInfo.Object_DistLong = ptr->RadarData.ObjGeneral[tempDangerObjIndex].Object_DistLong;
	t_AebObjInfo.Object_DistLat = ptr->RadarData.ObjGeneral[tempDangerObjIndex].Object_DistLat;
	t_AebObjInfo.Object_VrelLong = ptr->RadarData.ObjGeneral[tempDangerObjIndex].Object_VrelLong;
	t_AebObjInfo.Object_DynProp = ptr->RadarData.ObjGeneral[tempDangerObjIndex].Object_DynProp;
	t_AebObjInfo.Object_VreLat = ptr->RadarData.ObjGeneral[tempDangerObjIndex].Object_VreLat;
	t_AebObjInfo.Object_RCS = ptr->RadarData.ObjGeneral[tempDangerObjIndex].Object_RCS;

	/* Object important info */
	t_AebObjInfo.Object_DistLong_rms = ptr->RadarData.ObjQuality[tempDangerObjIndex].Object_DistLong_rms;
	t_AebObjInfo.Object_VrelLong_rms = ptr->RadarData.ObjQuality[tempDangerObjIndex].Object_VrelLong_rms;
	t_AebObjInfo.Object_DistLat_rms = ptr->RadarData.ObjQuality[tempDangerObjIndex].Object_DistLat_rms;
	t_AebObjInfo.Object_VrelLat_rms = ptr->RadarData.ObjQuality[tempDangerObjIndex].Object_VrelLat_rms;
	t_AebObjInfo.Object_ArelLat_rms = ptr->RadarData.ObjQuality[tempDangerObjIndex].Object_ArelLat_rms;
	t_AebObjInfo.Object_ArelLong_rms = ptr->RadarData.ObjQuality[tempDangerObjIndex].Object_ArelLong_rms;
	t_AebObjInfo.Object_Orientation_rms = ptr->RadarData.ObjQuality[tempDangerObjIndex].Object_Orientation_rms;
	t_AebObjInfo.Object_MeasState = ptr->RadarData.ObjQuality[tempDangerObjIndex].Object_MeasState;
	t_AebObjInfo.Object_ProbOfExist = ptr->RadarData.ObjQuality[tempDangerObjIndex].Object_ProbOfExist;

	/* Object expand info */
	t_AebObjInfo.Object_ArelLong = ptr->RadarData.ObjExtended[tempDangerObjIndex].Object_ArelLong;	
	t_AebObjInfo.Object_Class = ptr->RadarData.ObjExtended[tempDangerObjIndex].Object_Class;	
	t_AebObjInfo.Object_ArelLat = ptr->RadarData.ObjExtended[tempDangerObjIndex].Object_ArelLat;
	t_AebObjInfo.Object_OrientationAngel = ptr->RadarData.ObjExtended[tempDangerObjIndex].Object_OrientationAngel;
	t_AebObjInfo.Object_Length = ptr->RadarData.ObjExtended[tempDangerObjIndex].Object_Length;
	t_AebObjInfo.Object_Width = ptr->RadarData.ObjExtended[tempDangerObjIndex].Object_Width;

	/* Object collision detect warning info */
	t_AebObjInfo.Object_CollDetRegionBitfield = ptr->RadarData.ObjWarning[tempDangerObjIndex].Object_CollDetRegionBitfield;

#if 1
	t_AebInfo.ObjDistLong = Calc_LongDistance(t_AebObjInfo.Object_DistLong);  // uint:0.2m/bit
	t_AebInfo.ObjDistLat = Calc_LatDistance(t_AebObjInfo.Object_DistLat);     // uint:0.2m/bit

	t_AebInfo.ObjVrelLong = Calc_LongVelocity(t_AebObjInfo.Object_VrelLong);  // uint:0.25m/s/bit	
	t_AebInfo.ObjVrelLat = Calc_LatVelocity(t_AebObjInfo.Object_VreLat);      // uint:0.25m/s/bit
		  
	t_AebInfo.ObjArelLong = Calc_LongAccelerate(t_AebObjInfo.Object_ArelLong); // uint:0.001m/s2/bit
	t_AebInfo.ObjArelLat = Calc_LatAccelerate(t_AebObjInfo.Object_ArelLat);    // uint:0.001m/s2/bit	

	t_AebInfo.Velocity_host = Calc_HostVelocity(ptr->VehicleData.VehicleSpeedABS) / 3.6f; // uint:m/s

	t_AebInfo.DisplayObjVrelLong = 3.6f * t_AebInfo.ObjVrelLong;     // km/h
	t_AebInfo.DisplayObjVrelLat = 3.6f * t_AebInfo.ObjVrelLat;       // km/h
	t_AebInfo.DisplayHostVelocity = Calc_HostVelocity(ptr->VehicleData.VehicleSpeedABS); // km/h 
#endif
} 


FP32 min_fun(FP32 a, FP32 b)
{
	FP32 min;
	return min = a < b ? a : b;
}


FP32 max_fun(FP32 a, FP32 b)
{
	FP32 max;
	return max = a > b ? a : b;
}


/***********************************************************************
*  Name        : AEB_RiskAssessment
*  Description : None
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ADAS_AEB_STAT void AEB_RiskAssessment(void) 
{
	static FP32 Dis_part_1, Dis_part_2, Dis_part_3;

	/* Dis_part_1: Warning distance, driver distance within driver response time. */
	Dis_part_1 = max_fun((t_AebInfo.ObjVrelLong * AEB_WARNING_DRIVER_RESPONSE_DELAY_1), (t_AebInfo.Velocity_host * AEB_WARNING_DRIVER_RESPONSE_DELAY_2));

	/* Dis_part_2: Braking distance, including driving distance and full braking distance within
	 * the pressure build-up time of braking pressure. */
	Dis_part_2 = (t_AebInfo.ObjVrelLong * AEB_BRAKE_PRESSURE_BUILDUP_TIME / 2.0f) 
				+ pow(t_AebInfo.ObjVrelLong, 2.0f) / (2.0f * AEB_FULL_BRAKE_MAX_DECELERATION);

	/* Dis_part_3: Prefabricated saftey distance, the safety distance reserved between the car and
	 * the front car after braking can be checked by the car speed or relative speed. */
	Dis_part_3 = AEB_SAFETY_DISTANCE_STOPPING;

	t_AebInfo.SafeDistLong = Dis_part_1 + Dis_part_2 + Dis_part_3;
}


/***********************************************************************
*  Name        : AEB_TimeToCollisionAssessment
*  Description : None
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ADAS_AEB_STAT void AEB_TimeToCollisionAssessment(void)
{
	FP32 tempTTC;
#if ( 0 == ADAS_AEB_TTC_SIMPLE_ALOGRITHM )
	FP32 A_term, B_term, C_term, D_term;
	FP32 TTC1, TTC2;

	A_term = t_AebInfo.ObjArelLong / 2.0f;
	B_term = t_AebInfo.ObjVrelLong;
	C_term = t_AebInfo.ObjDistLong;
	D_term = pow(B_term, 2) - 4.0f * B_term * C_term;

	if ((fabs(A_term) < 1e-6) && (fabs(B_term) < 1e-6))
	{
		tempTTC = AEB_TTC_DEFAULT_VALUE;
	}
	else
	{
		if ((fabs(A_term) < 1e-6) && (fabs(B_term) > 1e-6))
		{
			tempTTC = abs(C_term / B_term);
		}
		else
		{
			if (D_term < 0.0f)
			{
				tempTTC = AEB_TTC_DEFAULT_VALUE;		
			}
			else
			{
				if (fabs(D_term) < 1e-6)
				{
					tempTTC = B_term / (2.0f * A_term);
				}
				else
				{
					TTC1 = (B_term + sqrt(D_term)) / (2.0f * A_term);
					TTC2 = (B_term - sqrt(D_term)) / (2.0f * A_term);
					tempTTC = min_fun(TTC1, TTC2);
				}		
			}
		}
	}
#else
	if (t_AebInfo.ObjDistLong > 120.0f)
	{
		t_AebInfo.TimeToCollision = AEB_TTC_DEFAULT_VALUE; 
	}
	else
	{
		if (fabs(t_AebInfo.ObjVrelLong) < 1e-6)
		{
			t_AebInfo.TimeToCollision = AEB_TTC_DEFAULT_VALUE; 
		}
		else
		{
			tempTTC = t_AebInfo.ObjDistLong / t_AebInfo.ObjVrelLong;
			t_AebInfo.TimeToCollision = tempTTC >= 0.0f ? tempTTC : (-tempTTC);
		}		
	}
#endif
}


FP32 Ka = 1.0f;
/***********************************************************************
*  Name        : AEB_OptimalBrakeDecelerationAssessment
*  Description : None
*  Parameter   : None
*  Returns     : None
***********************************************************************/
ADAS_AEB_STAT void AEB_OptimalBrakeDecelerationAssessment(void)
{
	FP32 Deceleration_optim;
	
	Deceleration_optim = Ka * pow(t_AebInfo.ObjVrelLong, 2.0f) / (2.0f * (t_AebInfo.ObjDistLong - AEB_SAFETY_DISTANCE_STOPPING));

	if (Deceleration_optim > t_AebInfo.Deceleration_optim)
	{
		t_AebInfo.Deceleration_optim = Deceleration_optim;
	}
}


// End Of ADAS_AEB

