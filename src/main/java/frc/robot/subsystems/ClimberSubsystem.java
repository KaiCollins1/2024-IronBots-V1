// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.management.relation.Relation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberSubsystemConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private RelativeEncoder leftHallSensor;
  private RelativeEncoder rightHallSensor;
  private PIDController rightPID;
  private PIDController leftPID;

  public ClimberSubsystem() {

    leftMotor = new CANSparkMax(ClimberSubsystemConstants.kLeftMotorID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ClimberSubsystemConstants.kRightMotorID, MotorType.kBrushless);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setInverted(ClimberSubsystemConstants.kIsLeftInverted);
    rightMotor.setInverted(ClimberSubsystemConstants.kIsRightInverted);
    leftMotor.setSmartCurrentLimit(ClimberSubsystemConstants.kMotorCurrentLimit_AMP);
    rightMotor.setSmartCurrentLimit(ClimberSubsystemConstants.kMotorCurrentLimit_AMP);

    leftHallSensor = leftMotor.getEncoder();
    rightHallSensor = rightMotor.getEncoder();
    leftHallSensor.setPositionConversionFactor(ClimberSubsystemConstants.kPositionScalingFactor);
    rightHallSensor.setPositionConversionFactor(ClimberSubsystemConstants.kPositionScalingFactor);

    leftPID = new PIDController(ClimberSubsystemConstants.kP, ClimberSubsystemConstants.kI, ClimberSubsystemConstants.kD);
    rightPID = new PIDController(ClimberSubsystemConstants.kP, ClimberSubsystemConstants.kI, ClimberSubsystemConstants.kD);


  }

  private double kSpeed = 0.2;

  public Command tempClimberControlCommand(BooleanSupplier up, BooleanSupplier down){
    return (run(() -> {
      if(up.getAsBoolean()){
        leftMotor.set(kSpeed);
        rightMotor.set(kSpeed);
      }else if(down.getAsBoolean()){
        leftMotor.set(-kSpeed);
        rightMotor.set(-kSpeed);
      }else{
        leftMotor.set(0);
        rightMotor.set(0);
      }
    }));
  }

  @Override
  public void periodic() {
    
  }
}
