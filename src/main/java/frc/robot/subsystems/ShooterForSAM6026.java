// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterForSAM6026 extends SubsystemBase {

  //please excuse me if I am being too obvious about certain things
  //I dont know your programming knowledge and I am trying to be extra clear
  //I am not intending to be condescending by noting something obvious

  //A lot of these constants (k...) will have to be changed to match your robot

  private CANSparkMax movementMotor;
  private static final int kMoveMotorCANID = 0; //change to your motors CAN ID
  private static final boolean kIsMoveMotorReversed = false; //change this if the motor is moving backwards
  private static final int kMoveMotorSmartCurrentLimit_AMP = 40; //this should be fine to have around 40A

  private DutyCycleEncoder movementEncoder;
  private static final int kMoveEncoderDIOPort = 0; //Change to whatever DIO you want. The cord you are looking for is red, black, and white
  //this will be very wrong right now. Put the movement encoder's position on the smartdashboard WITHOUT ENABLING ROBOT
  //change this value to the flattest position you can (except make the value negative). On the drawing you sent, its the furthest on the right
  private static final double kBottomZeroOffset_DEG = -0;
  //if your encoder values on smartdasboard are negative when you move your shooter "up" from its flattest position, make this -360
  private static final double kMoveEncoderDistancePerRotation = 360;
  private static final double kRangeOFMotion_DEG = 160;

  //a PID controller takes in the point you want the shooter to go to (setpoint) and where the shooter currently is
  //it uses the difference between those and a bunch of math to determine how to run your motor
  //please read the docs.wpilib.org pages and learn how it works
  private PIDController movementPID;
  //kP is how "hard" your shooter tries to move the further it is away from your setpoint
  //this should be tweaked during testing of your shooter, try teweaking it slowly
  private static final double kP = 0.015;   
  //kI would be best to keep at zero until you start to better understand PID as this can easily cause damage if used improperly
  private static final double kI = 0;
  //kD basically says "if shooter is moving very quickly as it gets close to the setpoint, slow it down a lot"
  //for initial testing and tweaking of kP you should have this very very small (or at zero)
  //this probably won't really be anything more than 1/100th of your kP value as it is very "strong"
  private static final double kD = 0;

  //this will be updated as you change position of your shooter through commands
  private double currentSetpoint_DEG = 0;



  /** Creates a new ShooterForSAM6026. */
  public ShooterForSAM6026() {

    movementMotor = new CANSparkMax(kMoveMotorCANID, MotorType.kBrushless);
    movementMotor.setInverted(kIsMoveMotorReversed);
    movementMotor.setSmartCurrentLimit(kMoveMotorSmartCurrentLimit_AMP);

    movementEncoder = new DutyCycleEncoder(kMoveEncoderDIOPort);
    movementEncoder.setDistancePerRotation(kMoveEncoderDistancePerRotation);
    movementEncoder.setPositionOffset(kBottomZeroOffset_DEG);

    movementPID = new PIDController(kP, kI, kD);

  }

  @Override
  public void periodic() {
    //if the angle you are trying to set the shooter to is outside of where you intend,
    //put the setpoint back in range
    if(currentSetpoint_DEG >= kRangeOFMotion_DEG){
      currentSetpoint_DEG = kRangeOFMotion_DEG;
    }else if(currentSetpoint_DEG <= 0){
      currentSetpoint_DEG = 0;
    }

    //set the voltage of the motor to the calculated voltage by the PID
    movementMotor.setVoltage(movementPID.calculate(movementEncoder.get(), currentSetpoint_DEG));
  }

  //at the start of testing, make sure to only do like 5 degrees, and then increase this to 10 degrees, 15, etc
  //you don't want to hurt the shooter
  public Command testingCommandIncreaseByDegrees(double degrees){
    return runOnce(() -> currentSetpoint_DEG = currentSetpoint_DEG + degrees);
  }

  public Command testingCommandDecreaseByDegrees(double degrees){
    return runOnce(() -> currentSetpoint_DEG = currentSetpoint_DEG - degrees);
  }

  //use this command to set your shooter to any angle you want
  public Command goToAngleCommand(double degrees){
    return run(() -> currentSetpoint_DEG = degrees);
  }
  
  //use a command like this to set your shooter to a predefined angle
  public Command goToXYZAngleCommand(){
    return run(() -> currentSetpoint_DEG = 0); //replace zero with your predefined angle
  }
  

  //schedule these commands something like this inside of RobotContainer
  /*
    CommandXboxController m_driverController = new CommandXboxController(kWhateverYourPortIs);

    m_driverController.a().onTrue(m_shooterSubsystem.testingCommandIncreaseByDegrees(5);
    
    m_driverController.b().whileTrue(m_shooterSubsystem.goToXYZAngleCommand(5);
   */

  //you can also make a default command that will run when no other command is being ran with the subsystem
  /*
    CommandScheduler.getInstance().setDefaultCommand(
      m_shooterSubsystem,
      m_shooterSubsystem.whateverDefaultCommand()
    );
   */

}
