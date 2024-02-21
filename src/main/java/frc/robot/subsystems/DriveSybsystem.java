// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
//import static edu.wpi.first.units.MutableMeasure.mutable;
//import static edu.wpi.first.units.Units.Meters;
//import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
//import edu.wpi.first.units.Velocity;
//import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
//import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveSubsystemConstants;


public class DriveSybsystem extends SubsystemBase {

private CANSparkMax leftFrontMotor = new CANSparkMax(DriveSubsystemConstants.kLeftFrontMotorID, MotorType.kBrushless);
private CANSparkMax leftBackMotor = new CANSparkMax(DriveSubsystemConstants.kLeftBackMotorID, MotorType.kBrushless);
private CANSparkMax rightFrontMotor = new CANSparkMax(DriveSubsystemConstants.kRightFrontMotorID, MotorType.kBrushless);
private CANSparkMax rightBackMotor = new CANSparkMax(DriveSubsystemConstants.kRightBackMotorID, MotorType.kBrushless);

private DifferentialDrive drive;

private PIDController leftDrivePidController = new PIDController(
  DriveSubsystemConstants.kP, 
  0, 
  DriveSubsystemConstants.kD
);
private PIDController rightDrivePidController = new PIDController(
  DriveSubsystemConstants.kP, 
  0,
  DriveSubsystemConstants.kD
);
private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
  DriveSubsystemConstants.kS,
  DriveSubsystemConstants.kV,
  DriveSubsystemConstants.kA
);

private DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(DriveSubsystemConstants.kTrackWidth_M);

private RelativeEncoder leftFrontEncoder;
private RelativeEncoder leftBackEncoder;
private RelativeEncoder rightFrontEncoder;
private RelativeEncoder rightBackEncoder;

private AHRS gyro = new AHRS(SPI.Port.kMXP);

private DifferentialDriveOdometry driveOdometry;
private Pose2d drivePose = new Pose2d(new Translation2d(0,0), new Rotation2d(0));

// Create the SysId routine
private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
  new SysIdRoutine.Config(), //we are not using advantage kit so we can just leave this empty
  new SysIdRoutine.Mechanism(
    (Measure<Voltage> volts) -> {
      leftFrontMotor.setVoltage(volts.in(Volts));
      rightFrontMotor.setVoltage(volts.in(Volts));
    },
    null, // No log consumer, since data is recorded by URCL
    this
  )
);

// //Old non URCL SysID
// //Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
// private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
// // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
// private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
// // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
// private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
// private final SysIdRoutine sysIdRoutine =
//       new SysIdRoutine(
//           // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
//           new SysIdRoutine.Config(),
//           new SysIdRoutine.Mechanism(
//               // Tell SysId how to plumb the driving voltage to the motors.
//               (Measure<Voltage> volts) -> {
//                 leftFrontMotor.setVoltage(volts.in(Volts));
//                 rightFrontMotor.setVoltage(volts.in(Volts));
//               },
//               // Tell SysId how to record a frame of data for each motor on the mechanism being
//               // characterized.
//               log -> {
//                 // Record a frame for the left motors.  Since these share an encoder, we consider
//                 // the entire group to be one motor.
//                 log.motor("drive-left")
//                     .voltage(
//                         m_appliedVoltage.mut_replace(
//                             leftFrontMotor.get() * RobotController.getBatteryVoltage(), Volts))
//                     .linearPosition(m_distance.mut_replace(this.getAvgLeftPosition(), Meters))
//                     .linearVelocity(
//                         m_velocity.mut_replace(this.getAvgLeftVelocity(), MetersPerSecond));
//                 // Record a frame for the right motors.  Since these share an encoder, we consider
//                 // the entire group to be one motor.
//                 log.motor("drive-right")
//                     .voltage(
//                         m_appliedVoltage.mut_replace(
//                             rightFrontMotor.get() * RobotController.getBatteryVoltage(), Volts))
//                     .linearPosition(m_distance.mut_replace(this.getAvgRightPosition(), Meters))
//                     .linearVelocity(
//                         m_velocity.mut_replace(this.getAvgRightPosition(), MetersPerSecond));
//               },
//               // Tell SysId to make generated commands require this subsystem, suffix test state in
//               // WPILog with this subsystem's name ("drive")
//               this));

  /** Creates a new DriveSybsystem. */
  public DriveSybsystem() {

    motorConfig();

    driveOdometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);
    zeroEncoders(false);
    zeroGyro(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivePose = driveOdometry.update(gyro.getRotation2d(), getAvgLeftPosition(), getAvgRightPosition());
  }

  public Command smartArcadeDriveCommand(DoubleSupplier fwdSupplier, DoubleSupplier rotSupplier) {
    return run(
      () -> chassisSpeedDrive(
        new ChassisSpeeds(
          DriveSubsystemConstants.kMaxDriveVelocity_Mps * fwdSupplier.getAsDouble(),
          0, //vy is always zero because we use tank drive and it cannot move sideways
          DriveSubsystemConstants.kMaxDriveRotations_Radps * rotSupplier.getAsDouble()
        )
      )
    ).withName("smartArcadeDrive");
  }

  public Command dumbArcadeDriveCommand(DoubleSupplier fwdSupplier, DoubleSupplier rotSupplier) {
    return run(
      () -> drive.arcadeDrive(
        fwdSupplier.getAsDouble(), 
        rotSupplier.getAsDouble(), 
        true
      )
    ).withName("dumbArcadeDrive");
  }

  // private void smartArcadeDrive(DoubleSupplier fwdSupplier, DoubleSupplier rotSupplier) {
  //   this.chassisSpeedDrive(
  //     new ChassisSpeeds(
  //       DriveSubsystemConstants.kMaxDriveVelocity_Mps * fwdSupplier.getAsDouble(),
  //       0, //vy is always zero because we use tank drive and it cannot move sideways
  //       DriveSubsystemConstants.kMaxDriveRotations_Radps * rotSupplier.getAsDouble()
  //     )
  //   );
  // }

  // private void dumbArcadeDrive(DoubleSupplier fwdSupplier, DoubleSupplier rotSupplier) {
  //   drive.arcadeDrive(fwdSupplier.getAsDouble(), rotSupplier.getAsDouble());
  // }

  public void chassisSpeedDrive(ChassisSpeeds speed) {

    leftFrontMotor.setVoltage(
      driveFeedforward.calculate(
        driveKinematics.toWheelSpeeds(speed).leftMetersPerSecond
      )+
      leftDrivePidController.calculate(
        getAvgLeftVelocity(), 
        driveKinematics.toWheelSpeeds(speed).leftMetersPerSecond
      )
    );

    rightFrontMotor.setVoltage(
      driveFeedforward.calculate(
        driveKinematics.toWheelSpeeds(speed).rightMetersPerSecond
      )+
      rightDrivePidController.calculate(
        getAvgLeftVelocity(), 
        driveKinematics.toWheelSpeeds(speed).rightMetersPerSecond
      )
    );

  }

  public void setPose2d(Pose2d newPose) {
    drivePose = newPose;
  }

  public Pose2d getPose2d() {
    return drivePose;
  }

  //get average (avg) values of encoders
  public double getAvgLeftPosition() {
    return (leftFrontEncoder.getPosition() + leftBackEncoder.getPosition()) / 2;
  }

  public double getAvgLeftVelocity() {
    return (leftFrontEncoder.getVelocity() + leftBackEncoder.getVelocity()) / 2;
  }

  public double getAvgRightPosition() {
    return (rightFrontEncoder.getPosition() + rightBackEncoder.getPosition()) / 2;
  }

  public double getAvgRightVelocity() {
    return (rightFrontEncoder.getVelocity() + rightBackEncoder.getVelocity()) / 2;
  }

  //reset all of the encoders to zero
  public void zeroEncoders(boolean recalcOdometry) {
    leftFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
    if(recalcOdometry) driveOdometry.resetPosition(gyro.getRotation2d(), 0, 0, drivePose);
  }

  public void zeroGyro(boolean recalcOdometry) {
    gyro.reset();
    if(recalcOdometry) driveOdometry.resetPosition(new Rotation2d(0), getAvgLeftPosition(), getAvgRightPosition(), drivePose);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  private void motorConfig(){
    //motor config
    leftFrontMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit);
    leftBackMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit);
    rightFrontMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit);
    rightBackMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit);
    leftFrontMotor.setInverted(DriveSubsystemConstants.kIsLeftInverted);
    leftBackMotor.setInverted(DriveSubsystemConstants.kIsLeftInverted);
    rightFrontMotor.setInverted(DriveSubsystemConstants.kIsRightInverted);
    rightBackMotor.setInverted(DriveSubsystemConstants.kIsRightInverted);

    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    
    //grab motor encoders as an easy to use object
    leftFrontEncoder = leftFrontMotor.getEncoder();
    leftBackEncoder = leftBackMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();
    rightBackEncoder = rightBackMotor.getEncoder();

    //Encoder config (inversion and scaling from rotations and rpm to meters and m/s)
    leftFrontEncoder.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);
    leftBackEncoder.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);
    rightFrontEncoder.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);
    rightBackEncoder.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);

    leftFrontEncoder.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);
    leftBackEncoder.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);
    rightFrontEncoder.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);
    rightBackEncoder.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);

  }

}
