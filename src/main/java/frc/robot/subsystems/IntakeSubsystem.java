// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {

  

  private CANSparkMax movementMotor;
  private DutyCycleEncoder movementAbsEncoder;
  private RelativeEncoder movementHallSensor;
  private PIDController movementPID = new PIDController(
    IntakeSubsystemConstants.kMP,
    IntakeSubsystemConstants.kMI, 
    IntakeSubsystemConstants.kMD
  );

  private CANSparkMax rollerMotor;
  private RelativeEncoder rollerHallSensor;
  private PIDController rollerPID = new PIDController(
    IntakeSubsystemConstants.kRP, 
    IntakeSubsystemConstants.kRI, 
    IntakeSubsystemConstants.kRD
  );
  private SimpleMotorFeedforward rollerFeedforward = new SimpleMotorFeedforward(
      IntakeSubsystemConstants.kRS,
      IntakeSubsystemConstants.kRV,
      IntakeSubsystemConstants.kRA
    );

  private double intakeSetpoint_DEG = IntakeSubsystemConstants.kIdlePos_DEG;
  private double rollerSetpoint_MPS = 0;

  private DigitalInput rightLimitSwitch;
  private DigitalInput middleLimitSwitch;
  private DigitalInput leftLimitSwitch;

  private Trigger hasNote;

  // Create the URCL compatable SysId routine
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.per(Seconds).of(0.8), Volts.of(7), Seconds.of(10)),//we are not using advantage kit so we can just leave this empty, //we are not using advantage kit so we can just leave this empty
    new SysIdRoutine.Mechanism(
      (Measure<Voltage> volts) -> {
        rollerMotor.setVoltage(volts.in(Volts));
      },
      null, // No log consumer, since data is recorded by URCL
      this
    )
  );

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    motorConfig();

    rightLimitSwitch = new DigitalInput(IntakeSubsystemConstants.kRightLimitSwitchPort);
    middleLimitSwitch = new DigitalInput(IntakeSubsystemConstants.kMiddleLimitSwitchPort);
    leftLimitSwitch = new DigitalInput(IntakeSubsystemConstants.kLeftLimitSwitchPort);

    hasNote = new Trigger(() -> 
      (rightLimitSwitch.get() || middleLimitSwitch.get() || leftLimitSwitch.get())
    ).debounce(IntakeSubsystemConstants.kConfirmNoteOwningDelay_SEC);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    movementMotor.setVoltage(movementPID.calculate(getAngle(), intakeSetpoint_DEG));
    // rollerMotor.setVoltage(
    //  rollerFeedforward.calculate(rollerSetpoint_MPS)+
    //  rollerPID.calculate(rollerHallSensor.getVelocity(), rollerSetpoint_MPS)
    // );
    rollerMotor.set(rollerSetpoint_MPS);

    SmartDashboard.putNumber("Intake Angle", getAngle());
    SmartDashboard.putNumber("Intake Speed", rollerHallSensor.getVelocity());

  }


  public Command setPrepHandoff(){
    return run(() -> {
      intakeSetpoint_DEG = IntakeSubsystemConstants.kInsideBotPos_DEG;
      rollerSetpoint_MPS = 0;
    }).withName("prep4Handoff"); 
  }
  
  public Command setIdling(){
    return run(() -> {
      intakeSetpoint_DEG = IntakeSubsystemConstants.kIdlePos_DEG;
      rollerSetpoint_MPS = 0;
    }).withName("idling"); 
  }

  public Command setHandoff(){
    return run(() -> {
      intakeSetpoint_DEG = IntakeSubsystemConstants.kInsideBotPos_DEG;
      rollerSetpoint_MPS = IntakeSubsystemConstants.kGoalHandoffSpeed_MPS;
    }).withName("handing off"); 
  }

  public Command setIntake(){
    return run(() -> {
      intakeSetpoint_DEG = IntakeSubsystemConstants.kIntakingPos_DEG;
      rollerSetpoint_MPS = IntakeSubsystemConstants.kGoalIntakeSpeed_MPS;
    }).withName("intaking");
  }

  // public Command autoCollectNote(){
  //   return setIntake().until(hasNote);
  // }

  public Command teleopNoteCollection(){
    return setIntake().until(hasNote).andThen(setPrepHandoff());
  }

  // public boolean getOwnsNote(){
  //   return hasNote.getAsBoolean();
  // }

  private double getAngle(){
    return IntakeSubsystemConstants.kUseAbsoluteEncoder ? movementAbsEncoder.getDistance() : movementHallSensor.getPosition();
  }

  // public Command tempDefaultCommand(BooleanSupplier upBtn, BooleanSupplier dwnBtn, BooleanSupplier inBtn, BooleanSupplier outBtn){
  //   return run(() ->{
  //     rollerMotor.set(
  //       inBtn.getAsBoolean() ? 0.4 : (outBtn.getAsBoolean() ? -.8 : 0.0)
  //     );
  //     movementMotor.set(
  //       upBtn.getAsBoolean() ? 0.18 : (dwnBtn.getAsBoolean() ? -0.2 : 0.0)
  //     );
  //   });
  // }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  private void motorConfig(){
    rollerMotor = new CANSparkMax(IntakeSubsystemConstants.kRollerMotorID, MotorType.kBrushless);
    movementMotor = new CANSparkMax(IntakeSubsystemConstants.kMovementMotorID, MotorType.kBrushless);

    rollerMotor.setSmartCurrentLimit(IntakeSubsystemConstants.kMotorCurrentLimit_AMP);
    movementMotor.setSmartCurrentLimit(IntakeSubsystemConstants.kMotorCurrentLimit_AMP);

    rollerMotor.setInverted(IntakeSubsystemConstants.kRollerMotorReversed);
    movementMotor.setInverted(IntakeSubsystemConstants.kMovementMotorReversed);
    
    rollerHallSensor = rollerMotor.getEncoder();
    rollerHallSensor.setPositionConversionFactor(IntakeSubsystemConstants.kRollerHallSensorPositionConversionFactor);
    rollerHallSensor.setVelocityConversionFactor(IntakeSubsystemConstants.kRollerHallSensorVelcityConversionFactor);

    movementHallSensor = movementMotor.getEncoder();
    movementHallSensor.setPositionConversionFactor(IntakeSubsystemConstants.kMovementHallSensorPositionConversionFactor);
    movementHallSensor.setVelocityConversionFactor(IntakeSubsystemConstants.kMovementHallSensorVelocityConversionFactor);

    movementAbsEncoder = new DutyCycleEncoder(IntakeSubsystemConstants.kMovementAbsEncoderPin);
    movementAbsEncoder.setDistancePerRotation(IntakeSubsystemConstants.kMovementAbsEncoderDistancePerRoatation);
  }
}
