// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.management.relation.Relation;
import javax.swing.plaf.basic.BasicSplitPaneUI.KeyboardDownRightHandler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double leftSetpoint = ClimberSubsystemConstants.kBottomPosition_IN;
  private double rightSetpoint = ClimberSubsystemConstants.kBottomPosition_IN;

  public ClimberSubsystem() {

    leftMotor = new CANSparkMax(ClimberSubsystemConstants.kLeftMotorID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ClimberSubsystemConstants.kRightMotorID, MotorType.kBrushless);
    leftMotor.setIdleMode(ClimberSubsystemConstants.kMotorMode);
    rightMotor.setIdleMode(ClimberSubsystemConstants.kMotorMode);
    leftMotor.setInverted(ClimberSubsystemConstants.kIsLeftInverted);
    rightMotor.setInverted(ClimberSubsystemConstants.kIsRightInverted);
    leftMotor.setSmartCurrentLimit(ClimberSubsystemConstants.kMotorCurrentLimit_AMP);
    rightMotor.setSmartCurrentLimit(ClimberSubsystemConstants.kMotorCurrentLimit_AMP);

    leftHallSensor = leftMotor.getEncoder();
    rightHallSensor = rightMotor.getEncoder();
    leftHallSensor.setPositionConversionFactor(ClimberSubsystemConstants.kPositionScalingFactor);
    rightHallSensor.setPositionConversionFactor(ClimberSubsystemConstants.kPositionScalingFactor);
    leftHallSensor.setPosition(0);
    rightHallSensor.setPosition(0);

    leftPID = new PIDController(ClimberSubsystemConstants.kP, ClimberSubsystemConstants.kI, ClimberSubsystemConstants.kD);
    rightPID = new PIDController(ClimberSubsystemConstants.kP, ClimberSubsystemConstants.kI, ClimberSubsystemConstants.kD);


  }

  public Command lowerClimber(){
    return runOnce(() -> {
      // leftSetpoint = ClimberSubsystemConstants.kBottomPosition_IN;
      // rightSetpoint = ClimberSubsystemConstants.kBottomPosition_IN;
      leftSetpoint = 0;
      rightSetpoint = 0;
    });
  }

  public Command raiseClimber(){
    return runOnce(() -> {
      leftSetpoint = 0;
      rightSetpoint = 0;
    });
  }


  @Override
  public void periodic() {
    leftMotor.setVoltage(leftPID.calculate(leftHallSensor.getPosition(), leftSetpoint));
    rightMotor.setVoltage(rightPID.calculate(rightHallSensor.getPosition(), rightSetpoint));
  }
}
