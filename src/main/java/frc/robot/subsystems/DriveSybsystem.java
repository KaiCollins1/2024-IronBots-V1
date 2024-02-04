// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveSubsystemConstants;

public class DriveSybsystem extends SubsystemBase {



private CANSparkMax leftFrontMotor = new CANSparkMax(DriveSubsystemConstants.kLeftFrontMotorID, MotorType.kBrushless);
private CANSparkMax leftBackMotor = new CANSparkMax(DriveSubsystemConstants.kLeftBackMotorID, MotorType.kBrushless);
private CANSparkMax rightFrontMotor = new CANSparkMax(DriveSubsystemConstants.kRightFrontMotorID, MotorType.kBrushless);
private CANSparkMax rightBackMotor = new CANSparkMax(DriveSubsystemConstants.kRightBackMotorID, MotorType.kBrushless);

private DifferentialDrive drive;

private PIDController leftDrivePidController = new PIDController(DriveSubsystemConstants.kP, 0, DriveSubsystemConstants.kD);
private PIDController rightDrivePidController = new PIDController(DriveSubsystemConstants.kP, 0, DriveSubsystemConstants.kD);
private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DriveSubsystemConstants.kS, DriveSubsystemConstants.kV, DriveSubsystemConstants.kA);
private DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(DriveSubsystemConstants.kTrackWidth_M);

private RelativeEncoder leftFrontEncoder;
private RelativeEncoder leftBackEncoder;
private RelativeEncoder rightFrontEncoder;
private RelativeEncoder rightBackEncoder;




  /** Creates a new DriveSybsystem. */
  public DriveSybsystem() {

    leftFrontMotor.setInverted(DriveSubsystemConstants.kIsLeftInverted);
    leftFrontMotor.setInverted(DriveSubsystemConstants.kIsRightInverted);
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);
    drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    

    //grab motor encoders as an easy to use object
    leftFrontEncoder = leftFrontMotor.getEncoder();
    leftBackEncoder = leftBackMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();
    rightBackEncoder = rightBackMotor.getEncoder();

    //Encoder setup (inversion and scaling from rotations and rpm to meters and m/s)
    leftFrontEncoder.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);
    leftFrontEncoder.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);
    leftFrontEncoder.setInverted(DriveSubsystemConstants.kIsLeftInverted);
    leftBackEncoder.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);
    leftBackEncoder.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);
    leftBackEncoder.setInverted(DriveSubsystemConstants.kIsLeftInverted);
    rightFrontEncoder.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);
    rightFrontEncoder.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);
    leftFrontEncoder.setInverted(DriveSubsystemConstants.kIsRightInverted);
    rightBackEncoder.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);
    leftBackEncoder.setInverted(DriveSubsystemConstants.kIsRightInverted);
    rightBackEncoder.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);

    this.zeroEncoders(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //reset all of the encoders to zero
  public void zeroEncoders(boolean recalcOdometry) {
    leftFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
    //if(recalcOdometry) driveOdometry.calculate(gyro.getRotation2d(), 0, 0, drivePose);
  }

  //get average values of encoders
  public double getAvgLeftPosition() {
    return (leftFrontEncoder.getPosition()+leftBackEncoder.getPosition())/2;
  }

  public double getAvgLeftVelocity() {
    return (leftFrontEncoder.getVelocity()+leftBackEncoder.getVelocity())/2;
  }

  public double getAvgRightPosition() {
    return (rightFrontEncoder.getPosition()+rightBackEncoder.getPosition())/2;
  }

  public double getAvgRightVelocity() {
    return (rightFrontEncoder.getVelocity()+rightBackEncoder.getVelocity())/2;
  }


  public void chassisSpeedDrive(ChassisSpeeds speed) {
    leftFrontMotor.setVoltage(
      DriveSubsystemConstants.kMaxMotorOutput_Volts* 
      MathUtil.clamp(
        driveFeedforward.calculate(
          driveKinematics.toWheelSpeeds(speed).leftMetersPerSecond
        )+
        rightDrivePidController.calculate(
          getAvgLeftVelocity(), 
          driveKinematics.toWheelSpeeds(speed).leftMetersPerSecond
        ),
        -1,
        1
      )
    );

    rightFrontMotor.setVoltage(
      DriveSubsystemConstants.kMaxMotorOutput_Volts* 
      MathUtil.clamp(
        driveFeedforward.calculate(
          driveKinematics.toWheelSpeeds(speed).rightMetersPerSecond
        )+
        rightDrivePidController.calculate(
          getAvgLeftVelocity(), 
          driveKinematics.toWheelSpeeds(speed).rightMetersPerSecond
        ),
        -1,
        1
      )
    );
  }
}
