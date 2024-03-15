// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Pivot extends SubsystemBase
{
    //Declare our motors here first
    CANSparkMax pivotMotor1 = new CANSparkMax(35,MotorType.kBrushless);
    CANSparkMax pivotMotor2 = new CANSparkMax(36, MotorType.kBrushless);
    
private Encoder throughBore = new Encoder(1,2,3,false);

/*
//fixed angles 
private static final double horizontalAngle = x;
  private static final double angledSubwooferAngle = x;
  private static final double shallowerAngle = x; 
  private static final double verticalAngle = x;
*/
public void pivot(boolean xButtonPressed,boolean yButtonPressed, boolean getRightBumper)
{
    if(xButtonPressed)
    {
        double currentAngle =  throughBore.getDistance();
        double expectedAngle = throughBore.getDistance()+15;
    }
    else if(yButtonPressed)
    {
        double currentAngle = throughBore.getDistance();
        double expectedAngle = throughBore.getDistance()-15;
    }

    else if (getRightBumper)
    {
        stopPivot();
    }
    
}

public void stopPivot()
{
    pivotMotor1.stopMotor();
    pivotMotor2.stopMotor();
}

public void pivotUp(Double currentAngle,Double expectedAngle)
{
    if(currentAngle==expectedAngle)
    {
        return;
    }
    else
    {
        pivotMotor2.follow(pivotMotor1,true);
        pivotMotor1.setVoltage((double)((expectedAngle-currentAngle)/15)*4); //change the number multiplied based on requirement
        pivotUp(throughBore.getDistance(),expectedAngle);
    }
}

public void pivotDown(double currentAngle,double expectedAngle)
{
    if(currentAngle==expectedAngle)
    {
        return;
    }
    else
    {
        pivotMotor2.follow(pivotMotor1,true);
        pivotMotor1.setVoltage(((double)((currentAngle-expectedAngle)/15)*4)*-2);  //change the number multiplied based on requirement (both for this case)
        pivotUp(throughBore.getDistance(),expectedAngle);
    }
}
}

/*

// old working code 


public void pivot(boolean upPivot, boolean downPivot)
{
     pivotMotor2.follow(pivotMotor1,true);
     if(upPivot)
     {
         SmartDashboard.putBoolean("Voltage ",true);
         pivotMotor1.setVoltage(-4.5);
     }
     else if (downPivot) 
     {
           
         SmartDashboard.putBoolean("Voltage ",true);
         pivotMotor1.setVoltage(4.5);
     }

}

*/

