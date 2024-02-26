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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private SimpleMotorFeedforward rollerFeedforward;

  private double intakeSetpoint_DEG = IntakeSubsystemConstants.kInsideBotPos;
  private double rollerSetpoint_MPS = IntakeSubsystemConstants.kInsideBotPos;

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


  private Command prepHandoff(){
    return run(() -> {
      intakeSetpoint_DEG = IntakeSubsystemConstants.kInsideBotPos;
      rollerSetpoint_MPS = 0;
    }).withName("prep4Handoff"); 
  }
  
  private Command setPosIdle(){
    return run(() -> {
      intakeSetpoint_DEG = IntakeSubsystemConstants.kInsideBotPos;
      rollerSetpoint_MPS = 0;
    }).withName("idling"); 
  }

  public Command handoff(){
    return run(() -> {
      intakeSetpoint_DEG = IntakeSubsystemConstants.kInsideBotPos;
      rollerSetpoint_MPS = IntakeSubsystemConstants.kGoalIntakeSpeed;
    }).withName("handing off"); 
  }

  private Command intakeCommand(){
    return run(() -> {
      intakeSetpoint_DEG = IntakeSubsystemConstants.kIntakingPos;
      rollerSetpoint_MPS = IntakeSubsystemConstants.kGoalIntakeSpeed;
    }).withName("intaking");
  }

  private double getAngle(){
    return IntakeSubsystemConstants.kUseAbsoluteEncoder ? movementAbsEncoder.getDistance() : movementHallSensor.getPosition();
  }

  public Command tempDefaultCommand(BooleanSupplier upBtn, BooleanSupplier dwnBtn, BooleanSupplier inBtn, BooleanSupplier outBtn){
    return run(() ->{
      rollerMotor.set(
        inBtn.getAsBoolean() ? 0.4 : (outBtn.getAsBoolean() ? -.8 : 0.0)
      );
      movementMotor.set(
        upBtn.getAsBoolean() ? 0.18 : (dwnBtn.getAsBoolean() ? -0.2 : 0.0)
      );
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // movementMotor.setVoltage(movementPID.calculate(getAngle(), intakeSetpoint_DEG));
    // rollerMotor.setVoltage(
    //  rollerFeedforward.calculate(rollerSetpoint_MPS)+
    //  rollerPID.calculate(rollerHallSensor.getVelocity(), rollerSetpoint_MPS)
    // );

    SmartDashboard.putNumber("Intake Angle", getAngle());
    SmartDashboard.putNumber("Intake Speed", rollerHallSensor.getVelocity());

  }
}
