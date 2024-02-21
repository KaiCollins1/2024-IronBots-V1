// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  // private PIDController bottomPID = new PIDController(
  //   ShooterSubsystemConstants.kP, 
  //   ShooterSubsystemConstants.kI,
  //   ShooterSubsystemConstants.kD
  // );
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    topMotor = new CANSparkMax(ShooterSubsystemConstants.kTopRollerMotorID, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(ShooterSubsystemConstants.kBottomRollerMotorID, MotorType.kBrushless);

    topMotor.setInverted(ShooterSubsystemConstants.kIsTopReversed);
    bottomMotor.follow(topMotor, true);

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

  public Command setShooterSpeed(BooleanSupplier isHighSpeed, BooleanSupplier isDisabled){
    return (
      isDisabled.getAsBoolean() ? rawSetSpeedCommand(0) : (
        rawSetSpeedCommand(isHighSpeed.getAsBoolean()? 
          ShooterSubsystemConstants.kGoalSpeedHigh:
          ShooterSubsystemConstants.kGoalSpeedLow
        )
      ) 
    );
  }

  private Command rawSetSpeedCommand(double speed){
    return (
      ShooterSubsystemConstants.kUseSetSpeedSmart ? 
      run(
        () -> topMotor.setVoltage(
          feedforward.calculate(speed)+
          topPID.calculate(getAverageVelocity_MPS(), speed)
        )
      ).withName("setSpeedSmart")
      : 
      run(
      () -> topMotor.set(speed)
      ).withName("setSpeedDumb")
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

  public double getAverageVelocity_MPS(){
    return (topEncoder.getVelocity() + bottomEncoder.getVelocity())/2;
  }

}
