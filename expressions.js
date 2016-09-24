expRemoveAll();
expAdd ("gMotorVars");
expAdd ("gMotorVars.UserErrorCode");
expAdd ("gMotorVars.CtrlVersion");
expAdd ("gMotorVars.Flag_enableSys", getDecimal());
expAdd ("gMotorVars.Flag_Run_Identify", getDecimal());
expAdd ("gMotorVars.Flag_enableUserParams", getDecimal());
expAdd ("gMotorVars.Flag_enableRsRecalc", getDecimal());
expAdd ("gMotorVars.Flag_enableForceAngle", getDecimal());
expAdd ("gMotorVars.Flag_enableOffsetcalc", getDecimal());
expAdd ("gMotorVars.Flag_enablePowerWarp", getDecimal());

expAdd ("gMotorVars.CtrlState");
expAdd ("gMotorVars.EstState");

expAdd ("gMotorVars.IqRef_A", getQValue(24));
expAdd ("gMotorVars.Speed_krpm", getQValue(24));
expAdd ("gMotorVars.Torque_Nm", getQValue(24));

expAdd ("gMotorVars.MagnCurr_A");
expAdd ("gMotorVars.Rr_Ohm");
expAdd ("gMotorVars.Rs_Ohm");
expAdd ("gMotorVars.Lsd_H");
expAdd ("gMotorVars.Lsq_H");
expAdd ("gMotorVars.Flux_VpHz");

expAdd ("gMotorVars.VdcBus_kV", getQValue(24));
expAdd ("gMotorVars.IdcBus", getQValue(24));

expAdd ("gDrvSpi8301Vars");
