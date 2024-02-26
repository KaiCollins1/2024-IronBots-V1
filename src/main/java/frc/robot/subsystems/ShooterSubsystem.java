// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

  private double speedSetpoint = 0;

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

    topMotor = new CANSparkMax(ShooterSubsystemConstants.kTopRollerMotorID, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(ShooterSubsystemConstants.kBottomRollerMotorID, MotorType.kBrushless);
    topMotor.setInverted(ShooterSubsystemConstants.kIsTopReversed);
    bottomMotor.setInverted(ShooterSubsystemConstants.kIsBottomReversed);
    topMotor.setSmartCurrentLimit(ShooterSubsystemConstants.kMotorCurrentLimit);
    bottomMotor.setSmartCurrentLimit(ShooterSubsystemConstants.kMotorCurrentLimit);

    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();
    topEncoder.setPositionConversionFactor(ShooterSubsystemConstants.kEncoderPositionScalingFactor);
    bottomEncoder.setPositionConversionFactor(ShooterSubsystemConstants.kEncoderPositionScalingFactor);
    topEncoder.setVelocityConversionFactor(ShooterSubsystemConstants.kEncoderVelocityScalingFactor);
    bottomEncoder.setVelocityConversionFactor(ShooterSubsystemConstants.kEncoderVelocityScalingFactor);
    topEncoder.setAverageDepth(ShooterSubsystemConstants.kFilterDepth);
    topEncoder.setMeasurementPeriod(ShooterSubsystemConstants.kFilterPeriod);
    bottomEncoder.setAverageDepth(ShooterSubsystemConstants.kFilterDepth);
    bottomEncoder.setMeasurementPeriod(ShooterSubsystemConstants.kFilterPeriod);

  }

  @Override
  public void periodic() {
    topMotor.setVoltage(
      feedforward.calculate(speedSetpoint)+
      topPID.calculate(topEncoder.getVelocity(), speedSetpoint)
    );
    bottomMotor.setVoltage(
      feedforward.calculate(speedSetpoint)+
      bottomPID.calculate(bottomEncoder.getVelocity(), speedSetpoint)
    );
  }

  public Command tempSetShooterSpeed(BooleanSupplier highSpeedEnabled, BooleanSupplier lowSpeedEnabled){
    return (
      tempRawSetSpeedCommand(
        () -> highSpeedEnabled.getAsBoolean() ? ShooterSubsystemConstants.kGoalSpeedHigh:
        (lowSpeedEnabled.getAsBoolean() ? ShooterSubsystemConstants.kGoalSpeedLow : 0 )
      )
    );
  }

  private Command tempRawSetSpeedCommand(DoubleSupplier speed){
    return (
      ShooterSubsystemConstants.kUseSetSpeedSmart ? 
      run(() -> {
        topMotor.setVoltage(
          feedforward.calculate(speed.getAsDouble())+
          topPID.calculate(topEncoder.getVelocity(), speed.getAsDouble())
        );
        bottomMotor.setVoltage(
          feedforward.calculate(speed.getAsDouble())+
          bottomPID.calculate(bottomEncoder.getVelocity(), speed.getAsDouble())
        );
      }).withName("setSpeedSmart") 
      :
      run(() -> {
        topMotor.set(speed.getAsDouble());
        bottomMotor.set(speed.getAsDouble());
      }).withName("setSpeedDumb")
    );
  }


  public boolean velocityAboveGoal(boolean isGoalHighSpeed){
    return (
      ((topEncoder.getVelocity() + bottomEncoder.getVelocity()) / 2)
      >= 
      (isGoalHighSpeed ? ShooterSubsystemConstants.kGoalSpeedHigh : ShooterSubsystemConstants.kGoalSpeedLow)
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

}
