/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Wang Jingyi's PID.
 */
public class MyPID {
    private double kLastError = 0;
    private double kP, kI, kD, kF;
    private double kMaxOutput = 1.0, kMinOutput = -1.0; 
    private String kName;
    private boolean kShowDebug = true;
    private double kErrorTotal = 0;
    public MyPID(String name){
        kName = name;
    } 
    public void setPID(double P, double I, double D, double F){
        kP = P;
        kI = I;
        kD = D;
        kF = F;
    }
    public void setOutputRange(double maxOutput, double minOutput){
        kMaxOutput = maxOutput;
        kMinOutput = minOutput;
    }
    public double calc(double setPoint, double processVar){
        double error = setPoint - processVar;
        double errorRate = 0;
        if(kLastError != 0){
            errorRate = error - kLastError;
        }
        kLastError = error;
        kErrorTotal += error;
        double output = error * kP + kErrorTotal * kI + errorRate * kD + kF;
        output = output > kMaxOutput ? kMaxOutput : output;
        output = output < kMinOutput ? kMinOutput : output;
        if(kShowDebug){
            SmartDashboard.putNumber(kName+"'s "+"P", kP);
            SmartDashboard.putNumber(kName+"'s "+"I", kI);
            SmartDashboard.putNumber(kName+"'s "+"D", kD);
            SmartDashboard.putNumber(kName+"'s "+"processVar", processVar);
            SmartDashboard.putNumber(kName+"'s "+"setPoint", setPoint);
            SmartDashboard.putNumber(kName+"'s "+"error", error);
            SmartDashboard.putNumber(kName+"'s "+"errorRate", errorRate);
            SmartDashboard.putNumber(kName+"'s "+"kErrorTotal", kErrorTotal);
            SmartDashboard.putNumber(kName+"'s "+"output", output);
        }
        return output;
    }
    public void resetError(){
        kErrorTotal = 0;
    }
}
