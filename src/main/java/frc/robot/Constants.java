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

    //encoder constants
    //converts between revolutions/rpm to meters and m/s
    public static final double kEncoderPositionScalingFactor = (Units.inchesToMeters(6)*Math.PI)/(8.46);
    public static final double kEncoderVelocityScalingFactor = (Units.inchesToMeters(6)*Math.PI)/(8.46*60);

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
    public static final int kMotorCurrentLimit = 50;
    public static final boolean kUseSmartTeleopDrive = false;
  }

  public static class ShooterSubsystemConstants {

    public static final int kMovementMotorID = 8;
    public static final int kRotationMotorID = 9;

    public static final double kP = 0;
    public static final double kD = 0;
    public static final double kI = 0;

    public static final double kGoalSpeed1_MPS = 8;
    public static final double kGoalSpeed2_MPS = 10;

  }

}
