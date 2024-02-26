// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AutonConstants {
    public static final int kMaxDrtiveMotorOutput_Volts = 10;
  }

  public static class DriveSubsystemConstants {

    //major drive config changes
    public static final boolean kUseSmartTeleopDrive = false;
    public static final boolean kUseQuadEncoders = false;
    

    //feedforward constants
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final int kResponseTimescale_Milis = 1000;
    //velocity PID constants
    public static final double kP = 0;
    public static final double kD = 0;
    public static final double kMaxVelocityError = 0;
    public static final double kMaxControlEffort = 0;

    //Relative Encoder constants
    //converts between revolutions/rpm to meters and m/s
    public static final double kEncoderPositionScalingFactor = (Units.inchesToMeters(6)*Math.PI)/(8.46);
    public static final double kEncoderVelocityScalingFactor = (Units.inchesToMeters(6)*Math.PI)/(8.46*60);

    //Quad Encoder constants
    public static final int kLeftEncoderPortA = 0;
    public static final int kLeftEncoderPortB = 1;
    public static final int kRightEncoderPortA = 4;
    public static final int kRightEncoderPortB = 5;

    public static final double kDistancePerPulse = (Units.inchesToMeters(6)*Math.PI)/(2048);

    //drive/motor constants
    public static final int kLeftFrontMotorID = 4;
    public static final int kLeftBackMotorID = 5;
    public static final int kRightFrontMotorID = 6;
    public static final int kRightBackMotorID = 7;
    public static final double kTrackWidth_M = Units.inchesToMeters(21.869);
    public static final boolean kIsLeftInverted = true;
    public static final boolean kIsRightInverted = false;
    //meters per second max velocity when at full throttle, and radians per second max rotation when at full throttle
    public static final double kMaxDriveVelocity_Mps = 0;
    public static final double kMaxDriveRotations_Radps = 0 * 2 * Math.PI;
    //max current for driving
    public static final int kMotorCurrentLimit = 55;


  }

  public static class ShooterSubsystemConstants {

    public static final boolean kUseSetSpeedSmart = true;

    //speed constants
    public static final double kGoalSpeedLow = kUseSetSpeedSmart ? 11.5 : 0.6;
    public static final double kGoalSpeedHigh = kUseSetSpeedSmart ? 13.5 : 1;

    //motor constants
    public static final int kTopRollerMotorID = 8;
    public static final int kBottomRollerMotorID = 9;
    public static final boolean kIsTopReversed = false;
    public static final boolean kIsBottomReversed = !kIsTopReversed;
    public static final int kMotorCurrentLimit = 60;

    //feedforward constants
    public static final double kS = 0.38361;
    public static final double kV = 0.45433;
    public static final double kA = 0.25736;
    //public static final int kResponseTimescale_Milis = 1000;

    //velocity PID constants
    public static final double kP = 0.68552;
    public static final double kD = 0;
    public static final double kI = 0;
    //public static final double kMaxVelocityError = 0;
    //public static final double kMaxControlEffort = 0;

    //encoder constants
    //converts between revolutions/rpm to meters and m/s
    public static final double kEncoderPositionScalingFactor = (Units.inchesToMeters(4)*Math.PI);
    public static final double kEncoderVelocityScalingFactor = (Units.inchesToMeters(4)*Math.PI)/60;
    public static final int kFilterDepth = 8;
    public static final int kFilterPeriod = 8;


  }

  public static class IntakeSubsystemConstants {

    //subsystem config
    public static final boolean kUseAbsoluteEncoder = true;
    public static final boolean kUseSmartMoveNRollDrive = false;
    public static final double kGoalIntakeSpeed = kUseSmartMoveNRollDrive ? 3.0 : 0.75;
    public static final double kGoalHandoffSpeed = kUseSmartMoveNRollDrive ? 4.0 : 0.90;

    //roller feedforward constants
    public static final double kRS = 0;
    public static final double kRV = 0;
    public static final double kRA = 0;
    public static final int kRResponseTimescale_Milis = 1000;
    //roller PID constants
    public static final double kRP = 0;
    public static final double kRD = 0;
    public static final double kRI = 0;
    public static final double kRMaxVelocityError = 0;
    public static final double kRMaxControlEffort = 0;
    //movement PID constants
    public static final double kMP = 0;
    public static final double kMD = 0;
    public static final double kMI = 0;
    public static final double kMMaxVelocityError = 0;
    public static final double kMMaxControlEffort = 0;

    //motor constants
    public static final int kMovementMotorID = 10;
    public static final int kRollerMotorID = 11;
    public static final int kMotorCurrentLimit = 30;

    //encoder constants
    public static final double kRollerHallSensorVelcityConversionFactor = (Units.inchesToMeters(2)*Math.PI)/(5);
    public static final double kRollerHallSensorPositionConversionFactor = (Units.inchesToMeters(2)*Math.PI)/(5*60);
    public static final double kMovementHallSensorPositionConversionFactor = 360/(20);
    public static final double kMovementHallSensorVelocityConversionFactor = 360/(20*60);
    public static final int kMovementAbsEncoderPin = 7;
    public static final double kMovementAbsEncoderDistancePerRoatation = 360;
    public static final double kInsideBotPos = 0;
    public static final double kIntakingPos = 0;
    public static final double kIdlePos = 0;

  }
}
