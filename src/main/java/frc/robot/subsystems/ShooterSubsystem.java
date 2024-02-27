// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterSubsystemConstants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;

  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;
  
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    ShooterSubsystemConstants.kS, 
    ShooterSubsystemConstants.kV, 
    ShooterSubsystemConstants.kA
  );

  private PIDController topPID = new PIDController(
    ShooterSubsystemConstants.kP, 
    ShooterSubsystemConstants.kI,
    ShooterSubsystemConstants.kD
  );

  private PIDController bottomPID = new PIDController(
    ShooterSubsystemConstants.kP, 
    ShooterSubsystemConstants.kI,
    ShooterSubsystemConstants.kD
  );

  private double speedSetpoint_MPS = 0;

  // Create the URCL compatable SysId routine
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.per(Seconds).of(0.8), Volts.of(7), Seconds.of(10)),//we are not using advantage kit so we can just leave this empty, //we are not using advantage kit so we can just leave this empty
    new SysIdRoutine.Mechanism(
      (Measure<Voltage> volts) -> {
        topMotor.setVoltage(volts.in(Volts));
        bottomMotor.setVoltage(volts.in(Volts));
      },
      null, // No log consumer, since data is recorded by URCL
      this
    )
  );
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    motorConfig();

  }

  @Override
  public void periodic() {
    topMotor.setVoltage(
      feedforward.calculate(speedSetpoint_MPS)+
      topPID.calculate(topEncoder.getVelocity(), speedSetpoint_MPS)
    );
    bottomMotor.setVoltage(
      feedforward.calculate(speedSetpoint_MPS)+
      bottomPID.calculate(bottomEncoder.getVelocity(), speedSetpoint_MPS)
    );
  }

  public Command setFireHigh(){
    return run(() -> speedSetpoint_MPS = ShooterSubsystemConstants.kGoalSpeedHigh_MPS).withName("fireHigh");
  }

  public Command setFireLow(){
    return run(() -> speedSetpoint_MPS = ShooterSubsystemConstants.kGoalSpeedLow_MPS).withName("fireLow");
  }

  public Command setHandoffAllowance(){
    return run(() -> 
      speedSetpoint_MPS = ShooterSubsystemConstants.kHandoffAllowanceSpeed_MPS
    ).withTimeout(
      ShooterSubsystemConstants.kHandoffAllowanceTime_SEC
    ).withName("handoffPrep");
  }

  public Command setDisabled(){
    return run(() -> speedSetpoint_MPS = 0).withName("diabled");
  }

  public Command tempSetShooterSpeed(BooleanSupplier highSpeedEnabled, BooleanSupplier lowSpeedEnabled){
    return (run(() -> 
      speedSetpoint_MPS = (
        highSpeedEnabled.getAsBoolean() ? ShooterSubsystemConstants.kGoalSpeedHigh_MPS :
        (lowSpeedEnabled.getAsBoolean() ? ShooterSubsystemConstants.kGoalSpeedLow_MPS : 0)
      )
    ));
  }


  public boolean velocityAboveGoal(boolean isGoalHighSpeed){
    return (
      getAvgSpeed()
      >= 
      (isGoalHighSpeed ? ShooterSubsystemConstants.kGoalSpeedHigh_MPS : ShooterSubsystemConstants.kGoalSpeedLow_MPS) - 1
    );
  }

  public double getAvgSpeed(){
    return (topEncoder.getVelocity()+bottomEncoder.getVelocity())/2;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  private void motorConfig(){
    topMotor = new CANSparkMax(ShooterSubsystemConstants.kTopRollerMotorID, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(ShooterSubsystemConstants.kBottomRollerMotorID, MotorType.kBrushless);
    topMotor.setInverted(ShooterSubsystemConstants.kIsTopReversed);
    bottomMotor.setInverted(ShooterSubsystemConstants.kIsBottomReversed);
    topMotor.setSmartCurrentLimit(ShooterSubsystemConstants.kMotorCurrentLimit_AMP);
    bottomMotor.setSmartCurrentLimit(ShooterSubsystemConstants.kMotorCurrentLimit_AMP);

    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();
    topEncoder.setPositionConversionFactor(ShooterSubsystemConstants.kEncoderPositionScalingFactor);
    bottomEncoder.setPositionConversionFactor(ShooterSubsystemConstants.kEncoderPositionScalingFactor);
    topEncoder.setVelocityConversionFactor(ShooterSubsystemConstants.kEncoderVelocityScalingFactor);
    bottomEncoder.setVelocityConversionFactor(ShooterSubsystemConstants.kEncoderVelocityScalingFactor);
    topEncoder.setAverageDepth(ShooterSubsystemConstants.kFilterDepth_CNT);
    topEncoder.setMeasurementPeriod(ShooterSubsystemConstants.kFilterPeriod_MS);
    bottomEncoder.setAverageDepth(ShooterSubsystemConstants.kFilterDepth_CNT);
    bottomEncoder.setMeasurementPeriod(ShooterSubsystemConstants.kFilterPeriod_MS);
  }

}
