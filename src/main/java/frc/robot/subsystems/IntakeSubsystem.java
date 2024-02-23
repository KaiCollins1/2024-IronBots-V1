// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax movementMotor;
  private DutyCycleEncoder movementAbsEncoder;
  private RelativeEncoder movementHallSensor;
  private PIDController movementPID;

  private CANSparkMax rollerMotor;
  private RelativeEncoder rollerHallSensor;
  private PIDController rollerPID;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    rollerMotor = new CANSparkMax(IntakeSubsystemConstants.kRollerMotorID, MotorType.kBrushless);
    movementMotor = new CANSparkMax(IntakeSubsystemConstants.kMovementMotorID, MotorType.kBrushless);

    rollerMotor.setSmartCurrentLimit(IntakeSubsystemConstants.kMotorCurrentLimit);
    movementMotor.setSmartCurrentLimit(IntakeSubsystemConstants.kMotorCurrentLimit);
    
    rollerHallSensor = rollerMotor.getEncoder();
    rollerHallSensor.setPositionConversionFactor(IntakeSubsystemConstants.kRollerHallSensorPositionConversionFactor);
    rollerHallSensor.setVelocityConversionFactor(IntakeSubsystemConstants.kRollerHallSensorVelcityConversionFactor);

    movementHallSensor = movementMotor.getEncoder();
    movementHallSensor.setPositionConversionFactor(IntakeSubsystemConstants.kMovementHallSensorPositionConversionFactor);
    movementHallSensor.setVelocityConversionFactor(IntakeSubsystemConstants.kMovementHallSensorVelocityConversionFactor);

    movementAbsEncoder = new DutyCycleEncoder(IntakeSubsystemConstants.kMovementAbsEncoderPin);
    movementAbsEncoder.setDistancePerRotation(IntakeSubsystemConstants.kMovementAbsEncoderDistancePerRoatation);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
