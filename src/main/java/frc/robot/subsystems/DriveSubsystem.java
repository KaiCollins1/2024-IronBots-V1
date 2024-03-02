// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

private SlewRateLimiter driveLimiter;

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
private Field2d fieldPose = new Field2d();

// Create the URCL compatable SysId routine
private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
  // new SysIdRoutine.Config(Volts.of(0.5), Volts.of(7), Seconds.of(10)),
  new SysIdRoutine.Config(Volts.per(Seconds).of(0.75), Volts.of(7), Seconds.of(10)),//we are not using advantage kit so we can just leave this empty
  new SysIdRoutine.Mechanism(
    (Measure<Voltage> volts) -> {
      leftLeaderMotor.setVoltage(volts.in(Volts));
      rightLeaderMotor.setVoltage(volts.in(Volts));
    },
    null, // No log consumer, since data is recorded by URCL
    this
  )
);


  /** Creates a new DriveSybsystem. */
  public DriveSubsystem(){
    motorConfig();

    //max output divided by time to accelerate = dO/s, acceleration
    driveLimiter = new SlewRateLimiter((1/0.75));

    zeroEncoders(false);
    zeroGyro(false);
    driveOdometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);

    AutoBuilder.configureRamsete(
      () -> drivePose, // Robot pose supplier
      this::setPose2d, // Method to reset odometry (will be called if your auto has a starting pose)
      () -> new ChassisSpeeds( // Current ChassisSpeeds supplier
        getAvgVelocity(),
        0,
        Units.DegreesPerSecond.of(gyro.getRate()).in(RadiansPerSecond)
      ),
      this::rawChassisSpeedDrive, // Method that will drive the robot given ChassisSpeeds
      new ReplanningConfig(), // Default path replanning config. See the API for the options here
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivePose = driveOdometry.update(gyro.getRotation2d(), getAvgLeftPosition(), getAvgRightPosition());
    //fieldPose.setRobotPose(drivePose);
    SmartDashboard.putNumber("driveAngle", drivePose.getRotation().getDegrees());
    //SmartDashboard.putData("FieldPosition", fieldPose);
  }

  public Command teleopDriveCommand(DoubleSupplier fwdSupplier, DoubleSupplier rotSupplier){
    return(
      DriveSubsystemConstants.kUseSmartTeleopDrive ?
      run(
        () -> rawChassisSpeedDrive(
          new ChassisSpeeds(
            DriveSubsystemConstants.kMaxDriveVelocity_MPS * fwdSupplier.getAsDouble(),
            0, //vy is always zero because we use tank drive and it cannot move sideways
            DriveSubsystemConstants.kMaxDriveRotations_RADPS * rotSupplier.getAsDouble()
          )
        )
      ).withName("arcadeDriveSmart"):
      run(
        () -> drive.arcadeDrive(
          driveLimiter.calculate(fwdSupplier.getAsDouble()), 
          0.7 * rotSupplier.getAsDouble(), 
          true
        )
      ).withName("arcadeDriveDumb")
    );
  }

  public Command doNothing(){
    return run(() -> drive.arcadeDrive(0, 0));
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

  public Command confirmShootingPosition(){
    return run(() -> 
      drive.arcadeDrive(DriveSubsystemConstants.kConfirmShootDriveSpeed_PCT, 0)
    ).repeatedly().withTimeout(DriveSubsystemConstants.kConfirmShootDriveLength_SEC);
  }

  private void setPose2d(Pose2d newPose) {
    driveOdometry.resetPosition(
      gyro.getRotation2d(),
      new DifferentialDriveWheelPositions(getAvgLeftPosition(), getAvgRightPosition()),
      newPose
    );
  }

  // public Pose2d getPose2d() {
  //   return drivePose;
  // }

  //get average (avg) values of encoders
  private double getAvgLeftPosition() {
    return (
      DriveSubsystemConstants.kUseQuadEncoders?
      leftQuadEncoder.getDistance():
      (leftLeaderHallSensor.getPosition() + leftFollowerHallSensor.getPosition()) / 2
    );
  }

  private double getAvgLeftVelocity() {
    return (
      DriveSubsystemConstants.kUseQuadEncoders?
      leftQuadEncoder.getRate():
      (leftLeaderHallSensor.getVelocity() + leftFollowerHallSensor.getVelocity()) / 2
    );
  }

  private double getAvgRightPosition() {
    return (
      DriveSubsystemConstants.kUseQuadEncoders?
      rightQuadEncoder.getDistance():
      (rightLeaderHallSensor.getPosition() + rightFollowerHallSensor.getPosition()) / 2
    );
  }

  private double getAvgRightVelocity() {
    return (
      DriveSubsystemConstants.kUseQuadEncoders?
      rightQuadEncoder.getRate():
      (rightLeaderHallSensor.getVelocity() + rightFollowerHallSensor.getVelocity()) / 2
    );
  }

  // private double getAvgPosition(){
  //   return (
  //     DriveSubsystemConstants.kUseQuadEncoders?
  //     (rightQuadEncoder.getDistance() + leftQuadEncoder.getDistance())/2:
  //     (rightLeaderHallSensor.getPosition() + rightFollowerHallSensor.getPosition()+ leftLeaderHallSensor.getPosition() + leftFollowerHallSensor.getPosition()) / 4
  //   );
  // }

  private double getAvgVelocity(){
    return (
      DriveSubsystemConstants.kUseQuadEncoders?
      (rightQuadEncoder.getRate() + leftQuadEncoder.getRate())/2:
      (rightLeaderHallSensor.getVelocity() + rightFollowerHallSensor.getVelocity()+ leftLeaderHallSensor.getVelocity() + leftFollowerHallSensor.getVelocity()) / 4
    );
  }

  //reset all of the encoders to zero
  private void zeroEncoders(boolean recalcOdometry) {
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

  private void zeroGyro(boolean recalcOdometry) {
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

    leftLeaderMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit_AMP);
    leftFollowerMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit_AMP);
    rightLeaderMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit_AMP);
    rightFollowerMotor.setSmartCurrentLimit(DriveSubsystemConstants.kMotorCurrentLimit_AMP);
    leftLeaderMotor.setInverted(DriveSubsystemConstants.kIsLeftInverted);
    leftFollowerMotor.setInverted(DriveSubsystemConstants.kIsLeftInverted);
    rightLeaderMotor.setInverted(DriveSubsystemConstants.kIsRightInverted);
    rightFollowerMotor.setInverted(DriveSubsystemConstants.kIsRightInverted);

    leftLeaderMotor.setIdleMode(IdleMode.kBrake);
    leftFollowerMotor.setIdleMode(IdleMode.kBrake);
    rightLeaderMotor.setIdleMode(IdleMode.kBrake);
    rightFollowerMotor.setIdleMode(IdleMode.kBrake);

    // leftLeaderMotor.setIdleMode(IdleMode.kCoast);
    // leftFollowerMotor.setIdleMode(IdleMode.kCoast);
    // rightLeaderMotor.setIdleMode(IdleMode.kCoast);
    // rightFollowerMotor.setIdleMode(IdleMode.kCoast);

    leftFollowerMotor.follow(leftLeaderMotor);
    rightFollowerMotor.follow(rightLeaderMotor);

    drive = new DifferentialDrive(leftLeaderMotor, rightLeaderMotor);

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
