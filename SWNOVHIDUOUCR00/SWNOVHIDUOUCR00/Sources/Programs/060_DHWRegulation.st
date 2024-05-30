USING System;
USING System.Math;
USING Libs.Math;
USING Libs.PID_Incr;
USING Libs.Timer;
USING Libs.ExpFilt;


VAR_GLOBAL
    DhwPumpMin1 : REAL;
    DhwPumpMax1 : REAL;
    DHWPumpEstimate : REAL;

    (**DHW charging period timer*)
    TON_DHWChrg       : TON_s;
    (**DHW ondelay step 2 timer*)
    TON_ChangeState      : TON_s;
    (**Extra hotwater timer*)
    TOF_ExtHotWatr     : TOF_s;  
    (**Extra hotwater pulse*)
    R_Edge_tmp          : system.timer.R_Edge;
    (**PreSet dhw controller*)
    R_Edge_Dhw          : system.timer.R_Edge;    
    (**Charge regulation PID*)
    DHWChrgRegPID : PID_Incr;
       
    temp : real;
    
    PumpSpdDHW              : REAL;
    (**DHW charge pump state *)
    DHWPumpState              : UINT := 0;
    
    DhwChrgPumpSetP         : REAL;
    
    DhwChrgPumpPGain        : REAL := 1;
    DhwChrgPumpIGain        : UINT := 50;
    DhwChrgPumpDGain        : UINT := 0;
            
        (**Circulation pump 1 timer*)
        TON_CircPump1     : TON_s;
   
END_VAR

PROGRAM DHWRegulation
    VAR
        (**Allow DHW*)
        AllowDHW          : BOOL;
        (**DHW top fill procent*)
        DHWTopProc        : REAL;
        (**DHW middle fill procent*)
        DHWMiddleProc     : REAL;
        (**DHW bottom fill procent*)
        DHWBottomProc     : REAL;
        (**Set/Reset DHW need*)
        SR_DHWNeed        : RS;        
//        (**Charge regulation PID*)
//        DHWChrgRegPID : PID_Incr;
        (**Temperature regulation PID*)
        DHWTempRegPID : PID_Incr;
        (**DHW block from smart grid*)
        SG_DHWBlock         : BOOL;
        (**Actual Pump start setpoint*)
        ActPmpStrtSetP    : REAL;
        (**Actual Pump stop setpoint*)
        ActPmpStopSetP    : REAL;
        (**Actual Pump start temperature*)
        ActPmpStrtTemp    : REAL;
        (**Actual Pump stop temperature*)
        ActPmpStopTemp    : REAL;
        (**Index variable*)
        i                 : INT;
//        (**Heating regulation output filter*)
//        DHWReg_Filt         : ExpFilt;
        (**Circulation pump 1 low speed timer*)
        TON_GP1_LowSpd      : TON_s;
        (**Timer to increase dhw pump to increase temperature*)    
        TON_DHWPumpTimer        : TON_s;
        (**PrePosition regulator*)
        PrePosDhwChrgReg    : BOOL := FALSE;
    END_VAR
    
    (*All the Programs are skipped while the default valeus are being installed*)
    IF RestoreDefault THEN
        RETURN;
    END_IF;
    
    SG_DHWBlock := SG_Status = SGStatus#BLOCKING_MODE
                    OR NOT(SystemCfg_AllowDHW);
    
    {REGION Extra Hot Water}
    (*Creates pulse on rising edge of ExtHotwatrEnbl**)
    R_Edge_tmp(
        In := ExtHotWatrEnbl,
        Reset := R_Edge_tmp.Out
        );
    (*Off delay to turn of after set time**)
    TOF_ExtHotWatr(
        IN := R_Edge_tmp.Out,
        PT := ExtHotWatrTime * 60
    );
    (**Disable ExtHotWatr when timer is done*)
    IF ExtHotWatrEnbl AND NOT TOF_ExtHotWatr.Q THEN
        ExtHotWatrEnbl := FALSE;
    END_IF;
   

    (*The extra hotwter enabled from input signal with timer or from SmartGrid [1:1]**)
    En_ExtHotWatr := ExtHotWatrEnbl OR SG_Status = SGStatus#OVERCAPACITY_MODE;
    
    {ENDREGION}
    
    {REGION START/STOP TEMPERATURES}
    //Select DHW Start/stop temperatures
    IF(DhwMode = DHWMode#ECONOMY)THEN
        DHWStartTemp := DhwEcoStartTemp;
        DHWStopTemp := DhwEcoStopTemp;
    ELSIF(DhwMode = DHWMode#COMFORT) OR (SG_Status = SGStatus#LOW_PRICE_MODE) THEN
        DHWStartTemp := DhwComfStartTemp;
        DHWStopTemp := DhwComfStopTemp;
    ELSE
        DHWStartTemp := DhwNormStartTemp;
        DHWStopTemp := DhwNormStopTemp;
    END_IF;
    {ENDREGION}
    
    {REGION DHWProc calculation}
        DHWTopProc := Limit(0,((Ain_Data.Fct.DHWTopTemp - DhwRefTop) / (DhwRefTopN - DhwRefTop))*100,200);
        DHWMiddleProc := Limit(0,((Ain_Data.Fct.DHWMiddleTemp - DhwRefMiddle) / (DhwRefMiddleN - DhwRefMiddle))*100,200);
        DHWBottomProc := Limit(0,((Ain_Data.Fct.DHWBottomTemp - DhwRefBottom) / (DhwRefBottomN - DhwRefBottom))*100,200);
        
        IF DHWTopProc = 0 THEN
            DHWProc := 0.0;
        ELSE
            DHWProc := ((DHWTopProc + DHWMiddleProc + DHWBottomProc) / 3); 
        END_IF;
    {ENDREGION}
    
    {REGION ENABLE CHARGE}
    (**Add logic to block DHW here*)
    AllowDHW := NOT(SG_DHWBlock) AND NOT(ImmersionHeatAllow) AND (MaxTankTempOk) AND NOT (DHWCompBlock) AND NOT DHWFrcSwitch;  //Allow DHW as long as there is no blocking from smart grid -> DHW blocked if immersion heater activated by energy balance
        
    //Determine need of DHW
    SR_DHWNeed(                                                                                          (**DHW START CONDITIONS*)
        S := ((Ain_Data.Fct.DHWMiddleTemp < DHWStartTemp                                                   //Low middle temp.
                    OR (Ain_Data.Fct.DHWMiddleTemp<ExtHotWatrStopTemp-ExtHotWatrDiff AND En_ExtHotWatr))   //Low middle temp.(Extra hot water)
                    AND AllowDHW AND NOT TON_CircPump1.Q)                                                  //DHW is allowed
                                                                         
                    OR ((TON_CircPump1.Q AND Ain_Data.Fct.DHWMiddleTemp < DHWStartTemp)                               //Pump active and low top temp.
                    AND UnitOn AND AllowDHW), 
                    //OR DHWProc < DHWStrtProc                                                           (**DHW STOP CONDITIONS*)
        R1  := (Ain_Data.Fct.DHWBottomTemp >= DHWStopTemp AND NOT(En_ExtHotWatr)                           //Bottom temp. reached
                    OR (Ain_Data.Fct.DHWBottomTemp>=ExtHotWatrStopTemp AND En_ExtHotWatr)                  //Bottom temp. reached(Extra hot water)
                    OR NOT(AllowDHW) OR SG_DHWBlock                                                        //DHW blocked
                    OR UnitOff)                                                          
                    //OR DHWProc >= DHWStrtProc
    );
    DHWNeed := SR_DHWNeed.Q1;
    
    {REGION DHW Charge Regulation}
    IF DHWChrgMode = 0 THEN                                                                     //Auto mode enabled
        IF OpState = OpStage_Status#DHW AND OU_CompOn THEN   
            TON_ChangeState(
                IN := OpState = OpStage_Status#DHW AND OU_CompOn,
                PT := 20
                );
            TON_DHWPumpTimer(
                IN := NOT TON_DHWPumpTimer.Q AND TON_ChangeState.Q,
                PT := 1
            );  
            IF DHWPumpState = 0 THEN                                                             //Initiation for 60 seconds the speed down pump 1% per seconds until OU_CondOut > max(45, Ain_Data.Fct.DHWMiddleTemp)
                PumpSpdDHW := 10;
                DHWPumpstate := 1;
                DHWChrgPumpSetPoint := PumpSpdDHW;
            ELSIF DHWPumpState = 1 AND TON_DHWPumpTimer.Q THEN 
                
                DHWChrgPumpSetPoint := PumpSpdDHW;
                PumpSpdDHW := Max(PumpSpdDHW - 1,10);
                
                IF  OU_CondOutTemp > max(45, Ain_Data.Fct.DHWMiddleTemp) THEN                          // Change mode
                    DHWPumpState := 2;
                    PrePosDhwChrgReg := TRUE;
                END_IF;
            ELSIF DHWPumpState = 2 THEN //AND TON_DHWPumpTimer.Q AND PumpspdDHW > 10 THEN              //Fast mode to find usable dhw -> Max of 45 or middle temperature sensor.                //PumpSpdDHW := PumpSpdDHW - 1; 
                DhwChrgPumpSetP := max(45, Ain_Data.Fct.DHWMiddleTemp)-Ain_Data.Fct.DHWBottomTemp;     //Delta T between Tank temp middle and bottom -> use this sensor to get a more stable regulator the using condensor in/out
                PrePosDhwChrgReg := FALSE;
                IF OU_CondOutTemp > max(45, Ain_Data.Fct.DHWMiddleTemp) AND Ain_Data.Fct.DHWBottomTemp > (OU_CondOutTemp - DHWChargeDeltaT) THEN
                    DHWPumpState := 3;
                 
                END_IF;
                
            ELSIF DHWPumpState = 3 OR DHWPumpState = 4 THEN  //Delta T control

                IF DHWPumpState = 3 THEN
                    DhwChrgPumpSetP := DHWChargeDeltaT; //10;
                ELSE
                    DhwChrgPumpSetP := DHWChargeMaxEnergy; //5;
                END_IF;

                  
                IF OU_CondOutTemp + 2 > DHWStopTemp AND OU_CondInTemp > (OU_CondOutTemp - DHWChargeDeltaT) THEN                                         //Change mood to maximize energy in tank
                    DHWPumpState := 4;
                END_IF;
            END_IF;
          
        ELSE  
            DHWPumpState := 0;
        END_IF;
        DHWChrgRegPID(
            RegVar_PID_Incr       := OU_CondOutTemp - OU_CondInTemp,  //replaced OU_CondInTem
            SetP_PID_Incr         := DhwChrgPumpSetP,
            DeadBand_PID_Incr     := 0.0,
            Kp_PID_Incr           := TO_REAL(DhwChrgPumpPGain),
            Ti_PID_Incr           := TO_UINT(DhwChrgPumpIGain),
            Td_PID_Incr           := TO_UINT(DhwChrgPumpDGain),
            LowLim_PID_Incr       := TO_REAL(10),//DhwPumpMin1,//
            HighLim_PID_Incr      := TO_REAL(100),//DhwPumpMax1,//
            Period_PID_Incr       := 100,
            CoolHeatMode_PID_Incr := ENUM_COOL_HEAT#Cool,
            RevMode_PID_Incr      := ENUM_REV_MODE#Direct,
            ManModeVal_PID_Incr   := 0,
            En_PrePos_PID_Incr    := PrePosDhwChrgReg,
            En_ManMode_PID_Incr   := 0,
            En_PID_Incr           := TO_BOOL(DHWPumpState = 2 OR DHWPumpState = 3 OR DHWPumpState = 4)
        );

        
        IF DHWPumpState = 2 OR DHWPumpState = 3 OR DHWPumpState = 4 THEN
            DHWChrgPumpSetPoint := DHWChrgRegPID.Req_PID_Incr;
        END_IF;
    ELSE
        DHWChrgPumpSetPoint := PumpSpdDhwFix;                                                   //Fixed speed mode
    END_IF;
    
    {ENDREGION}             
    
    
    //Calculation of waterflow
    
    
    //Circulation pump enable from flow
    TON_CircPump1(
        IN := WaterFlow_DHW > 1.5 AND Uniton, 
        PT := TO_UINT(DhwFlowSwitchOnDel)
    );
    
    
    {ENDREGION}
    
    {REGION CHARGE MANAGEMENT}
      
   
        //Boost compressor towards end of DHW charge
        IF En_ExtHotWatr THEN
            DHWBoost := (To_Real(TON_DHWTimer.ET) > To_Real(DhwTimerStop) * 60 * DhwBoostPercExtHotWatr * 0.01);
        ELSE
            DHWBoost := (TO_REAL(TON_DHWTimer.ET) > To_REAL(DhwTimerStop) * 60 * DhwBoostPerc * 0.01);
        END_IF;
    {ENDREGION}
    
    {REGION DHW PUMP REGULATION}
    
    //Parameter selection for setpoint calculation
    IF(DhwMode = DHWMode#ECONOMY)THEN
        ActPmpStrtSetP := DhwSetPointStrtEco;
        ActPmpStopSetP := DhwSetPointStpEco;
        ActPmpStrtTemp := DhwBT3StrtEco;
        ActPmpStopTemp := DhwBT3StopEco;
    ELSIF(DhwMode = DHWMode#COMFORT) THEN
        ActPmpStrtSetP := DhwSetPointStrtCom;
        ActPmpStopSetP := DhwSetPointStpCom;
        ActPmpStrtTemp := DhwBT3StrtCom;
        ActPmpStopTemp := DhwBT3StopCom;
    ELSE
        ActPmpStrtSetP := DhwSetPointStrtNor;
        ActPmpStopSetP := DhwSetPointStpNor;
        ActPmpStrtTemp := DhwBT3StrtNor;
        ActPmpStopTemp := DhwBT3StopNor;
    END_IF;
    
    //Calculate pump regulation setpoint
    DhwPumpSetP := ScalerLimited(Ain_Data.Fct.DHWTopTemp, ActPmpStrtTemp, ActPmpStopTemp, ActPmpStrtSetP, ActPmpStopSetP);
    IF  Ain_Data.Fct.DHWTopTemp - DhwPumpSetP <= 7.5 THEN
        DHWPumpEstimate := Limit(0,(7.69 * ((WaterFlow_DHW * 1.257) - 0.85) -84.5) ,100);                         //low Delta T formula BF2 vs GP2 flow y = 1.257 x - 0.85
    ELSIF Ain_Data.Fct.DHWTopTemp - DhwPumpSetP > 7.5 AND Ain_Data.Fct.DHWTopTemp - DhwPumpSetP <=15 THEN
        DHWPumpEstimate := Limit(0,(7.69 * ((WaterFlow_DHW * 0.8621) - 0.2994)- 84.5) ,100);                      //Medium Delta T formula BF2 vs GP2 flow y = 0.8621 x - 0.2994
    ELSE        
        DHWPumpEstimate := Limit(0,(7.69 * ((WaterFlow_DHW * 0.696) - 0.1418) - 84.5 ),100);                      //High Delta T formula BF2 vs GP2 flow y = 0.696 x - 0.1418
    END_IF;
            
    //DHWPumpEstimate :=0;
    DhwPumpMin1 := Limit(0, DHWPumpEstimate - 40, 100);   //Calculated pumpflow vs pump pwm 0-100% y= 7.69 x -84.5 
    DhwPumpMax1 := Limit(0, DHWPumpEstimate + 10, 100);
    
    (*Preset regulator*)
    R_Edge_Dhw(
    In := TON_CircPump1.Q,
    Reset := R_Edge_Dhw.Out
    );
    
    (*Temperature regulation PID*)
    DHWTempRegPID(
        RegVar_PID_Incr       := Ain_Data.Fct.DHWSupplyTemp,
        SetP_PID_Incr         := DhwPumpSetP,
        DeadBand_PID_Incr     := 0.0,
        Kp_PID_Incr           := TO_REAL(DhwPumpPGain),
        Ti_PID_Incr           := TO_UINT(DhwPumpIGain),
        Td_PID_Incr           := TO_UINT(DhwPumpDGain),
        LowLim_PID_Incr       := TO_REAL(DhwPumpMin),//DhwPumpMin1,//
        HighLim_PID_Incr      := TO_REAL(DhwPumpMax),//DhwPumpMax1,//
        Period_PID_Incr       := 100,
        CoolHeatMode_PID_Incr := ENUM_COOL_HEAT#Heat,
        RevMode_PID_Incr      := ENUM_REV_MODE#Direct,
        ManModeVal_PID_Incr   := TO_REAL(PumpSpdGP2Man),
        En_PrePos_PID_Incr    := R_Edge_Dhw.Out,
        En_ManMode_PID_Incr   := PumpSpdGP2Man > 0,
        En_PID_Incr           := TON_CircPump1.Q
    );
    
    DHWPumpReg := DHWTempRegPID.Req_PID_Incr;
    
    TON_GP1_LowSpd(
        IN := TON_CircPump1.Q AND NOT(TON_GP1_LowSpd.Q),
        PT := 1
        );
    
    IF UnitOff OR NOT TON_CircPump1.Q  THEN // OLD: OR (DHWTempRegPID.Req_PID_Incr = 0)
        //Unit off or PID reg 0 then pump shall be set to off
        DhwCircPumpReq :=97;
    ELSIF (DHWTempRegPID.Req_PID_Incr = 0) AND TON_CircPump1.Q THEN
        IF TON_GP1_LowSpd.Q THEN
            DhwCircPumpReq := 85;
        ELSE
            DhwCircPumpReq := 97;
        END_IF;
    ELSE
        //Pump regulation
        DhwCircPumpReq := Scaler(DHWTempRegPID.Req_PID_Incr, 0, 100, 84, 10); //0-100 input is reversed and pump PWM input is scaled to fit the correct PWM curve.
    END_IF;
    
    Circ2Act_Req := DhwCircPumpReq;
    
    {ENDREGION}
    
END_PROGRAM