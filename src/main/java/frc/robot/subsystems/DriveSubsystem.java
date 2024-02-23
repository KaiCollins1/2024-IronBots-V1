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
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveSubsystemConstants;


public class DriveSubsystem extends SubsystemBase {

private CANSparkMax leftLeaderMotor = new CANSparkMax(DriveSubsystemConstants.kLeftFrontMotorID, MotorType.kBrushless);
private CANSparkMax leftFollowerMotor = new CANSparkMax(DriveSubsystemConstants.kLeftBackMotorID, MotorType.kBrushless);
private CANSparkMax rightLeaderMotor = new CANSparkMax(DriveSubsystemConstants.kRightFrontMotorID, MotorType.kBrushless);
private CANSparkMax rightFollowerMotor = new CANSparkMax(DriveSubsystemConstants.kRightBackMotorID, MotorType.kBrushless);

private DifferentialDrive drive;

private PIDController leftDrivePID = new PIDController(
  DriveSubsystemConstants.kP, 
  0, 
  DriveSubsystemConstants.kD
);
private PIDController rightDrivePID = new PIDController(
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

private RelativeEncoder leftLeaderHallSensor;
private RelativeEncoder leftFollowerHallSensor;
private RelativeEncoder rightLeaderHallSensor;
private RelativeEncoder rightFollowerHallSensor;

private Encoder leftQuadEncoder;
private Encoder rightQuadEncoder;

private AHRS gyro = new AHRS(SPI.Port.kMXP);

private DifferentialDriveOdometry driveOdometry;
private Pose2d drivePose = new Pose2d(new Translation2d(0,0), new Rotation2d(0));

// Create the URCL compatable SysId routine
private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
  new SysIdRoutine.Config(), //we are not using advantage kit so we can just leave this empty
  new SysIdRoutine.Mechanism(
    (Measure<Voltage> volts) -> {
      leftLeaderMotor.setVoltage(volts.in(Volts));
      rightLeaderMotor.setVoltage(volts.in(Volts));
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
//                 leftLeaderMotor.setVoltage(volts.in(Volts));
//                 rightLeaderMotor.setVoltage(volts.in(Volts));
//               },
//               // Tell SysId how to record a frame of data for each motor on the mechanism being
//               // characterized.
//               log -> {
//                 // Record a frame for the left motors.  Since these share an encoder, we consider
//                 // the entire group to be one motor.
//                 log.motor("drive-left")
//                     .voltage(
//                         m_appliedVoltage.mut_replace(
//                             leftLeaderMotor.get() * RobotController.getBatteryVoltage(), Volts))
//                     .linearPosition(m_distance.mut_replace(this.getAvgLeftPosition(), Meters))
//                     .linearVelocity(
//                         m_velocity.mut_replace(this.getAvgLeftVelocity(), MetersPerSecond));
//                 // Record a frame for the right motors.  Since these share an encoder, we consider
//                 // the entire group to be one motor.
//                 log.motor("drive-right")
//                     .voltage(
//                         m_appliedVoltage.mut_replace(
//                             rightLeaderMotor.get() * RobotController.getBatteryVoltage(), Volts))
//                     .linearPosition(m_distance.mut_replace(this.getAvgRightPosition(), Meters))
//                     .linearVelocity(
//                         m_velocity.mut_replace(this.getAvgRightPosition(), MetersPerSecond));
//               },
//               // Tell SysId to make generated commands require this subsystem, suffix test state in
//               // WPILog with this subsystem's name ("drive")
//               this));

  /** Creates a new DriveSybsystem. */
  public DriveSubsystem() {

    //motor config
    leftLeaderMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit);
    leftFollowerMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit);
    rightLeaderMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit);
    rightFollowerMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit);
    leftLeaderMotor.setInverted(DriveSubsystemConstants.kIsLeftInverted);
    leftFollowerMotor.setInverted(DriveSubsystemConstants.kIsLeftInverted);
    rightLeaderMotor.setInverted(DriveSubsystemConstants.kIsRightInverted);
    rightFollowerMotor.setInverted(DriveSubsystemConstants.kIsRightInverted);

    leftFollowerMotor.follow(leftLeaderMotor);
    rightFollowerMotor.follow(rightLeaderMotor);

    drive = new DifferentialDrive(leftLeaderMotor, rightLeaderMotor);

    //deals with selection of neo encoders or quad encoders
    encoderConfig();


    zeroEncoders(false);
    zeroGyro(false);
    driveOdometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivePose = driveOdometry.update(gyro.getRotation2d(), getAvgLeftPosition(), getAvgRightPosition());
  }

  public Command teleopDriveCommand(DoubleSupplier fwdSupplier, DoubleSupplier rotSupplier){
    return(
      DriveSubsystemConstants.kUseSmartTeleopDrive ?
      run(
        () -> rawChassisSpeedDrive(
          new ChassisSpeeds(
            DriveSubsystemConstants.kMaxDriveVelocity_Mps * fwdSupplier.getAsDouble(),
            0, //vy is always zero because we use tank drive and it cannot move sideways
            DriveSubsystemConstants.kMaxDriveRotations_Radps * rotSupplier.getAsDouble()
          )
        )
      ).withName("arcadeDriveSmart"):
      run(
        () -> drive.arcadeDrive(
          fwdSupplier.getAsDouble(), 
          rotSupplier.getAsDouble(), 
          true
        )
      ).withName("arcadeDriveDumb")
    );
  }

  // private Command smartArcadeDriveCommand(DoubleSupplier fwdSupplier, DoubleSupplier rotSupplier) {
  //   return run(
  //     () -> rawChassisSpeedDrive(
  //       new ChassisSpeeds(
  //         DriveSubsystemConstants.kMaxDriveVelocity_Mps * fwdSupplier.getAsDouble(),
  //         0, //vy is always zero because we use tank drive and it cannot move sideways
  //         DriveSubsystemConstants.kMaxDriveRotations_Radps * rotSupplier.getAsDouble()
  //       )
  //     )
  //   ).withName("smartArcadeDrive");
  // }
  //
  // private Command dumbArcadeDriveCommand(DoubleSupplier fwdSupplier, DoubleSupplier rotSupplier) {
  //   return run(
  //     () -> drive.arcadeDrive(
  //       fwdSupplier.getAsDouble(), 
  //       rotSupplier.getAsDouble(), 
  //       true
  //     )
  //   ).withName("dumbArcadeDrive");
  // }

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

  public void rawChassisSpeedDrive(ChassisSpeeds speed) {
    double leftSpeed = driveKinematics.toWheelSpeeds(speed).leftMetersPerSecond;
    double rightSpeed = driveKinematics.toWheelSpeeds(speed).rightMetersPerSecond;
    leftLeaderMotor.setVoltage(
      driveFeedforward.calculate(leftSpeed)+
      leftDrivePID.calculate(getAvgLeftVelocity(), leftSpeed)
    );
    rightLeaderMotor.setVoltage(
      driveFeedforward.calculate(rightSpeed)+
      rightDrivePID.calculate(getAvgRightVelocity(), rightSpeed)
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
    return (
      DriveSubsystemConstants.kUseQuadEncoders?
      leftQuadEncoder.getDistance():
      (leftLeaderHallSensor.getPosition() + leftFollowerHallSensor.getPosition()) / 2
    );
  }
  public double getAvgLeftVelocity() {
    return (
      DriveSubsystemConstants.kUseQuadEncoders?
      leftQuadEncoder.getRate():
      (leftLeaderHallSensor.getVelocity() + leftFollowerHallSensor.getVelocity()) / 2
    );
  }
  public double getAvgRightPosition() {
    return (
      DriveSubsystemConstants.kUseQuadEncoders?
      rightQuadEncoder.getDistance():
      (rightLeaderHallSensor.getPosition() + rightFollowerHallSensor.getPosition()) / 2
    );
  }
  public double getAvgRightVelocity() {
    return (
      DriveSubsystemConstants.kUseQuadEncoders?
      rightQuadEncoder.getRate():
      (rightLeaderHallSensor.getVelocity() + rightFollowerHallSensor.getVelocity()) / 2
    );
  }

  //reset all of the encoders to zero
  public void zeroEncoders(boolean recalcOdometry) {
    if(!DriveSubsystemConstants.kUseQuadEncoders){
      leftLeaderHallSensor.setPosition(0);
      leftFollowerHallSensor.setPosition(0);
      rightLeaderHallSensor.setPosition(0);
      rightFollowerHallSensor.setPosition(0);
    }else{
      leftQuadEncoder.reset();
      rightQuadEncoder.reset();
    }

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

  private void encoderConfig(){
    
    if(!DriveSubsystemConstants.kUseQuadEncoders){

      //grab motor encoders as an easy to use object
      leftLeaderHallSensor = leftLeaderMotor.getEncoder();
      leftFollowerHallSensor = leftFollowerMotor.getEncoder();
      rightLeaderHallSensor = rightLeaderMotor.getEncoder();
      rightFollowerHallSensor = rightFollowerMotor.getEncoder();

      //Encoder config (inversion and scaling from rotations and rpm to meters and m/s)
      leftLeaderHallSensor.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);
      leftFollowerHallSensor.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);
      rightLeaderHallSensor.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);
      rightFollowerHallSensor.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);

      leftLeaderHallSensor.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);
      leftFollowerHallSensor.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);
      rightLeaderHallSensor.setVelocityConversionFactor(DriveSubsystemConstants.kEncoderVelocityScalingFactor);
      rightFollowerHallSensor.setPositionConversionFactor(DriveSubsystemConstants.kEncoderPositionScalingFactor);

    }else{

      leftQuadEncoder = new Encoder(
        DriveSubsystemConstants.kLeftEncoderPortA,
        DriveSubsystemConstants.kLeftEncoderPortB,
        DriveSubsystemConstants.kIsLeftInverted, 
        EncodingType.k2X
      );
      rightQuadEncoder = new Encoder(
        DriveSubsystemConstants.kRightEncoderPortA,
        DriveSubsystemConstants.kRightEncoderPortB,
        DriveSubsystemConstants.kIsRightInverted, 
        EncodingType.k2X
      );

      leftQuadEncoder.setDistancePerPulse(DriveSubsystemConstants.kDistancePerPulse);
      rightQuadEncoder.setDistancePerPulse(DriveSubsystemConstants.kDistancePerPulse);
    }

  }

}