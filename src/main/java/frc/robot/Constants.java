// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

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
  public static class GeneralConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AutonConstants {
    public static final int kMaxDrtiveMotorOutput_VOLT = 10;
  }

  public static class DriveSubsystemConstants {

    //major drive config changes
    public static final boolean kUseSmartTeleopDrive = false;
    public static final boolean kUseQuadEncoders = false;
    public static final double kConfirmShootDriveSpeed_PCT = 0.2;
    public static final double kConfirmShootDriveLength_SEC = 1.5;
    public static final double kMaxDriveVelocity_MPS = 4*(0.2);
    public static final double kMaxDriveRotations_RADPS = 0.15;
    public static final int kMotorCurrentLimit_AMP = 58;
    

    // //feedforward constants
    // public static final double kS = 0.035675;
    // public static final double kV = 2.0692;
    // public static final double kA = 1.0056;
    // //feedforward constants
    // public static final double kS = 0.28753;
    // public static final double kV = 1.7784;
    // public static final double kA = 1.2748;
    //left feedforward constants
    // public static final double kLS = (0.061834 + 0.13059)/2;
    // public static final double kLV = (2.1738 + 2.2335)/2;
    // public static final double kLA = (1.0994 + 0.87323)/2;
    // //left velocity PID constants
    // public static final double kLP = (1.3617 + 1.0058)/2;
    // public static final double kLD = 0;
    // public static final double kLI = 0;
    // //right feedforward constants
    // public static final double kRS = (0.13186 + 0.078695)/2;// - 0.08;
    // public static final double kRV = (2.2076 + 1.8034)/2 - 0.13;
    // public static final double kRA = (0.96977 + 1.7498)/2;
    // //right velocity PID constants
    // public static final double kRP = (1.2113 + 1.6714)/2;
    // public static final double kRD = 0;
    // public static final double kRI = 0;
    //left feedforward constants
    public static final double kLS = 0.061834;
    public static final double kLV = 2.1738;
    public static final double kLA = 1.0994;
    //left velocity PID constants
    public static final double kLP = 1.3617;
    public static final double kLD = 0;
    public static final double kLI = 0;
    //right feedforward constants
    public static final double kRS = 0.13186;//-0.08;
    public static final double kRV = 2.2076;//-0.2;
    public static final double kRA = 0.96977;
    //right velocity PID constants
    public static final double kRP = 1.2113;
    public static final double kRD = 0;
    public static final double kRI = 0;

    //Relative Encoder constants
    //converts between revolutions to meters and rpm and mps
    public static final double kEncoderPositionScalingFactor = (((Units.inchesToMeters(6)*Math.PI)/(8.46))*(109/129));
    public static final double kEncoderVelocityScalingFactor = (((Units.inchesToMeters(6)*Math.PI)/(8.46*60))*(109/129));
    //values for the encoder's filters. average depth must be a power of two, up to 8. default is 64 idk why it's different
    public static final int kEncoderAverageDepth = 1;
    //this has to be from 8 to 64
    public static final int kEncoderMeasurementPeriod_MS = 8;

    //Quad Encoder constants
    public static final int kLeftEncoderPortA = 0; //3pin with blue
    public static final int kLeftEncoderPortB = 1; //3pin with yellow
    public static final int kRightEncoderPortA = 2;
    public static final int kRightEncoderPortB = 3;

    public static final double kDistancePerPulse = (Units.inchesToMeters(6)*Math.PI)/(2048*2);

    //drive/motor constants
    public static final int kLeftFrontMotorID = 4;
    public static final int kLeftBackMotorID = 5;
    public static final int kRightFrontMotorID = 6;
    public static final int kRightBackMotorID = 7;
    public static final double kTrackWidth_M = Units.inchesToMeters(21.869);
    public static final boolean kIsLeftInverted = true;
    public static final boolean kIsRightInverted = !kIsLeftInverted;


  }

  public static class ShooterSubsystemConstants {

    //subsystem config
    public static final boolean kUseSetSpeedSmart = true;
    public static final double kHandoffAllowanceSpeed_MPS = 5;
    public static final double kHandoffAllowanceTime_SEC = 1;
    public static final double kGoalSpeedLow_MPS = 11.5;
    public static final double kGoalSpeedHigh_MPS = 13.5;

    //motor constants
    public static final int kTopRollerMotorID = 8;
    public static final int kBottomRollerMotorID = 9;
    public static final boolean kIsTopReversed = false;
    public static final boolean kIsBottomReversed = !kIsTopReversed;
    public static final int kMotorCurrentLimit_AMP = 60;

    //feedforward constants
    public static final double kS = 0.38361;
    public static final double kV = 0.46433;
    public static final double kA = 0.25736;
    //velocity PID constants
    public static final double kP = 0.68552;
    public static final double kD = 0;
    public static final double kI = 0;

    //encoder constants
    //converts between revolutions/rpm to meters and m/s
    public static final double kEncoderPositionScalingFactor = (Units.inchesToMeters(4)*Math.PI);
    public static final double kEncoderVelocityScalingFactor = (Units.inchesToMeters(4)*Math.PI)/60;
    public static final int kFilterDepth_CNT = 8;
    public static final int kFilterPeriod_MS = 8;


  }

  public static class IntakeSubsystemConstants {

    //subsystem config
    public static final boolean kUseAbsoluteEncoder = true;
    //public static final boolean kUseSmartMoveNRollDrive = false;
    public static final double kConfirmNoteOwningDelay_SEC = 0.1;
    // public static final double kGoalIntakeSpeed_MPS =  -0.25;
    // public static final double kGoalHandoffSpeed_MPS = .8;
    public static final double kHandoffTime_SEC = 1;
    public static final double kGoalIntakeSpeed_MPS =  -(1.5)*(60);
    public static final double kGoalHandoffSpeed_MPS = (5)*(60);

    public static final double kInsideBotPos_DEG = 280-1;
    public static final double kIntakingPos_DEG = 56+6;
    public static final double kIdlePos_DEG = 200;

    //roller feedforward constants
    public static final double kRS = 0.48732;
    public static final double kRV = 0.066998;
    public static final double kRA = 0.0063037;
    //roller PID constants
    public static final double kRP = 0.0017483;
    public static final double kRD = 0;
    public static final double kRI = 0;
    //movement PID constants
    public static final double kMP = 0.04;
    public static final double kMD = 0.0005;
    public static final double kMI = 0;

    //motor constants
    public static final boolean kMovementMotorReversed = false;
    public static final boolean kRollerMotorReversed = true;
    public static final int kMovementMotorID = 10;
    public static final int kRollerMotorID = 11;
    public static final int kMotorCurrentLimit_AMP = 30;

    //encoder constants
    public static final double kRollerHallSensorVelcityConversionFactor = (Units.inchesToMeters(2)*Math.PI)/(5);
    public static final double kRollerHallSensorPositionConversionFactor = (Units.inchesToMeters(2)*Math.PI)/(5*60);
    public static final double kMovementHallSensorPositionConversionFactor = 360/(20);
    public static final double kMovementHallSensorVelocityConversionFactor = 360/(20*60);
    public static final int kMovementAbsEncoderPort = 5; //3pin with white wire
    public static final double kMovementAbsEncoderDistancePerRoatation = 360;

    //sensor constants
    public static final int kRightLimitSwitchPort = 7;
    public static final int kMiddleLimitSwitchPort = 8;
    public static final int kLeftLimitSwitchPort = 9;

  }

  public static class ClimberSubsystemConstants {

    public static final int kRightMotorID = 13;
    public static final int kLeftMotorID = 12;

    public static final boolean kIsRightInverted = false;
    public static final boolean kIsLeftInverted = !kIsRightInverted;
    public static final int kMotorCurrentLimit_AMP = 30;

    public static final IdleMode kMotorMode = IdleMode.kBrake;

    public static final double kP = 15;
    public static final double kI = 0;
    public static final double kD = 0;
    
    public static final double kBottomPosition_IN = -13;
    public static final double kSpoolDiameter_IN = .75+(0.11);
    public static final double kClimbGearRatio = 20;
    public static final double kPositionScalingFactor = (kSpoolDiameter_IN*Math.PI)/(kClimbGearRatio);

    

  }
}
