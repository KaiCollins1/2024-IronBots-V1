// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax movementMotor;
  private DutyCycleEncoder movementAbsEncoder;
  private RelativeEncoder movementHallSensor;
  private PIDController movementPID;

  private CANSparkMax rollerMotor;
  private RelativeEncoder rollerHallSensor;
  private PIDController rollerPID;
  private SimpleMotorFeedforward rollerFeedforward;

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

    rollerPID = new PIDController(
      IntakeSubsystemConstants.kRP, 
      IntakeSubsystemConstants.kRI, 
      IntakeSubsystemConstants.kRD
    );

    movementPID = new PIDController(
      IntakeSubsystemConstants.kMP,
      IntakeSubsystemConstants.kMI, 
      IntakeSubsystemConstants.kMD
    );

    rollerFeedforward = new SimpleMotorFeedforward(
      IntakeSubsystemConstants.kRS,
      IntakeSubsystemConstants.kRV,
      IntakeSubsystemConstants.kRA
    );

  }


  private Command rawrollerSpeedCommand(double speed){
    return (
      IntakeSubsystemConstants.kUseSmartMoveNRollDrive ? 
      run(() -> 
        rollerMotor.setVoltage(
          rollerFeedforward.calculate(speed)+
          rollerPID.calculate(rollerHallSensor.getVelocity(), speed)
        )).withName("setRollerSpeedSmart") 
      :
      run(() -> 
        rollerMotor.set(speed)
      ).withName("setRollerSpeedDumb")
    );
  }

  // public Command tempMovementCommand(BooleanSupplier upBtn, BooleanSupplier dwnBtn){
  //   return run(() ->{
  //     movementMotor.set(
  //       upBtn.getAsBoolean() ? 0.25 : (dwnBtn.getAsBoolean() ? -0.25 : 0.0)
  //     );
  //   });
  // }

  // public Command tempRollerCommand(BooleanSupplier inBtn, BooleanSupplier outBtn){
  //   return run(() ->{
  //     rollerMotor.set(
  //       inBtn.getAsBoolean() ? 0.25 : (outBtn.getAsBoolean() ? -0.25 : 0.0)
  //     );
  //   });
  // }

  public Command tempDefaultCommand(BooleanSupplier upBtn, BooleanSupplier dwnBtn, BooleanSupplier inBtn, BooleanSupplier outBtn){
    return run(() ->{
      rollerMotor.set(
        inBtn.getAsBoolean() ? 0.4 : (outBtn.getAsBoolean() ? -0.8 : 0.0)
      );
      movementMotor.set(
        upBtn.getAsBoolean() ? 0.18 : (dwnBtn.getAsBoolean() ? -0.2 : 0.0)
      );
    });
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
