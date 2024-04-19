USING System;
USING System.Math;
USING Libs.Math;
USING Libs.PID_Incr;
USING Libs.Timer;
USING Libs.ExpFilt;

PROGRAM CoolingRegulation
    VAR
        (**SR mode selector*)
        SR_AutoMode          : SR;
//        (**TON cooling delay*)
//        TON_Cooling          : TON_s;
        (**Setpoint total offset*)
        SetP_Offset          : REAL;
        (**Cooling PID regulator*)
        PID_CoolReg          : PID_Incr;
        (**Cooling regulation output filter*)
        CoolReg_Filt         : ExpFilt;
        (**PID Cool Manual Value*)
        ManModeVal : REAL;
        (**Freeze time after DHW charge timer*)
        TOF_RPSBlock       : TOF_s;
    END_VAR
    
    IF RestoreDefault THEN
        RETURN;
    END_IF;
    
//        TON_Cooling( //Cooling mode
//            IN := ((OutDoorTempFilter > CoolModeTemp) AND SystemCfg_AllowCool),
//            PT := OpModeSwitchDelay*60
//            );
        SR_AutoMode(
                s1 := ((OutDoorTempFilter > CoolModeTemp) AND SystemCfg_AllowCool),
                R := OutDoorTempFilter < CoolModeTemp - CoolModeTempHyst
                );
        OpModeAllowCool := SR_AutoMode.Q1;    
    
   //Determine total setpoint offset
        SetP_Offset := Offset_EBCH;
    //Calculate current supply temperature setpoint
        SupplyTempCalcCH := LIMIT(SupplyTempCoolMin, ((CoolCurve_K[Sel_CoolCurve_SetP]*OutDoorTemp) + CoolCurve_M[Sel_CoolCurve_SetP]) + SetP_Offset, 30);
    
    
    {REGION OU cooling PID REGULATOR}
    //Timer to go back and freeze regulator/compressor rps after DHW charge -> test the timer and adjust
    TOF_RPSBlock(
                IN := OpState = OpStage_Status#DHW OR OpState = OpStage_Status#DEFROST,
                PT := 120
            );
    
    CoolReg_Temp := Ain_Data.Fct.ClimSupplyTemp;    
    IF OU_CompOn THEN
        IF OU_HP_Status = 2 OR OU_HP_Status = 8 OR DHW_InitRPS = 0 THEN
            ManModeVal := ScalerLimited(OU_Unit.CurrSpd_RPS, OU_Unit.MinRPS, OU_CompMaxRps, 0.0, 100.0); 
        ELSE
            ManModeVal := ScalerLimited(DHW_InitRPS, OU_Unit.MinRPS, OU_CompMaxRps, 0.0, 100.0);    
        END_IF;
    ELSE
        ManModeVal := 50.0;
    END_IF;
    
    PID_CoolReg(        
        RegVar_PID_Incr       := CoolReg_Temp,
        SetP_PID_Incr         := SupplyTempCalcCH,
        DeadBand_PID_Incr     := 0.0,
        Kp_PID_Incr           := TO_REAL(HeatRegPGain),
        Ti_PID_Incr           := TO_UINT(HeatRegIGain),
        Td_PID_Incr           := TO_UINT(HeatRegDGain),
        LowLim_PID_Incr       := 0.0,
        HighLim_PID_Incr      := 100.0,
        Period_PID_Incr       := 100,
        CoolHeatMode_PID_Incr := ENUM_COOL_HEAT#Cool,
        RevMode_PID_Incr      := ENUM_REV_MODE#Direct,
        ManModeVal_PID_Incr   := ManModeVal,
        En_PrePos_PID_Incr    := FALSE,
        En_ManMode_PID_Incr   := NOT OU_CompOn OR OU_HP_Status = 2 OR OU_HP_Status = 8 OR TOF_RPSBlock.Q,
        En_PID_Incr           := OpState = OpStage_Status#COOLING// AND SR_Energybalance.Q1
    );
 

    
    //Pass filtered request to outdoor unit
    CoolReg_Filt(
        In    := PID_CoolReg.Req_PID_Incr,
        Tau   := 50,
        En_FB := PID_CoolReg.En_PID_Incr,
        Out   => PID_CoolReq
    );
    {ENDREGION}
    
END_PROGRAM