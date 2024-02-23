// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  // Create the URCL compatable SysId routine
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(), //we are not using advantage kit so we can just leave this empty
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

  }

  @Override
  public void periodic() {
    
  }

  public Command setShooterSpeed(BooleanSupplier highSpeedEnabled, BooleanSupplier lowSpeedEnabled){
    return (
      rawSetSpeedCommand(
        highSpeedEnabled.getAsBoolean() ? ShooterSubsystemConstants.kGoalSpeedHigh:
        (lowSpeedEnabled.getAsBoolean() ? ShooterSubsystemConstants.kGoalSpeedLow : 0 )
      ) 
    );
  }

  private Command rawSetSpeedCommand(double speed){
    return (
      ShooterSubsystemConstants.kUseSetSpeedSmart ? 
      run(() -> {
        topMotor.setVoltage(
          feedforward.calculate(speed)+
          topPID.calculate(topEncoder.getVelocity(), speed)
        );
        bottomMotor.setVoltage(
          feedforward.calculate(speed)+
          bottomPID.calculate(bottomEncoder.getVelocity(), speed)
        );
      }).withName("setSpeedSmart") 
      :
      run(() -> {
        topMotor.set(speed);
        bottomMotor.set(speed);
      }).withName("setSpeedDumb")
    );
  }

  // private Command setSpeedSmart(double speed_MPS){
  //   return run(
  //     () -> topMotor.setVoltage(
  //       feedforward.calculate(speed_MPS)+
  //       topPID.calculate(getAverageVelocity_MPS(), speed_MPS)
  //     )
  //   ).withName("setSpeedSmart");
  // }
  //
  // private Command setSpeedDumb(double speed_DUTY){
  //   return run(
  //     () -> topMotor.set(speed_DUTY)
  //   ).withName("setSpeedDumb");
  // }

  public boolean velocityAboveGoal(boolean isGoalHighSpeed){
    return (
      ((topEncoder.getVelocity() + bottomEncoder.getVelocity()) / 2)
      >= 
      (isGoalHighSpeed ? ShooterSubsystemConstants.kGoalSpeedHigh : ShooterSubsystemConstants.kGoalSpeedLow)
    );
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

}