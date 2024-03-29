// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class SubsystemContainer {

    private ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private DriveSubsystem driveSubsystem = new DriveSubsystem();
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private LEDSubsystem ledSubsystem = new LEDSubsystem();
    private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    public SubsystemContainer(){}

    //TELOP Commands, t_
    // public final Command t_intakeNote = intakeSubsystem.setIntake().alongWith(shooterSubsystem.setHandoffAllowance());
    public final Command t_climberUp = climberSubsystem.raiseClimber().repeatedly();
    public final Command t_climberDown = climberSubsystem.lowerClimber().repeatedly();
    public final Command t_adjustIntake = intakeSubsystem.adjustIntake();
    public final Command t_activatePain = intakeSubsystem.switchToPainMode();
    public final Command t_pausePain = intakeSubsystem.deactivatePainMove();
    
    public final Command t_handoffNote = 
    new ParallelCommandGroup(
        intakeSubsystem.setPrepHandoff(),
        shooterSubsystem.setHandoffAllowance()
    );

    public final Command t_intakeNote = 
    new SequentialCommandGroup(
      new ParallelCommandGroup(
        intakeSubsystem.setIntake(),
        shooterSubsystem.setHandoffAllowance()
      ),
      new ParallelCommandGroup(
        intakeSubsystem.setPrepHandoff(),
        shooterSubsystem.setHandoffAllowance()
      ).repeatedly()
    );

    public final Command t_shootNote =
    new ParallelCommandGroup(
        driveSubsystem.confirmShootingPosition().repeatedly(),
        shooterSubsystem.setFireLow().repeatedly(),
        new SequentialCommandGroup(
            new WaitCommand(4).until(shooterSubsystem.velocityAboveLowGoal()),
            intakeSubsystem.setHandoff().repeatedly()
        )
    );

    public final Command t_removeNote =
    new SequentialCommandGroup(
        intakeSubsystem.removeNote().repeatedly()
    );


    // AUTON Commands, a_
    public final Command a_shootStraight = 
    new ParallelRaceGroup(
      driveSubsystem.confirmShootingPosition().repeatedly().withTimeout(3.5),
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          shooterSubsystem.setFireLow().repeatedly().until(shooterSubsystem.velocityAboveLowGoal()).withTimeout(1),
          new WaitCommand(0.75)
        ),
        new ParallelCommandGroup(
          intakeSubsystem.setHandoff().repeatedly(),
          shooterSubsystem.setFireLow().repeatedly()
        ).withTimeout(1),
        new ParallelCommandGroup(
          shooterSubsystem.setDisabled(),
          intakeSubsystem.setPrepHandoff()
        )
      )
    );

    // public final Command a_shootStraight = 
    // new ParallelRaceGroup(
    //   driveSubsystem.confirmShootingPosition().repeatedly().withTimeout(3.5),
    //   new SequentialCommandGroup(
    //     shooterSubsystem.setFireLow().repeatedly().until(shooterSubsystem.velocityAboveLowGoal()),
    //     new ParallelCommandGroup(
    //       intakeSubsystem.removeNote().repeatedly(),
    //       shooterSubsystem.setDisabled().repeatedly()
    //     ).withTimeout(2.5),
    //     new ParallelCommandGroup(
    //       shooterSubsystem.setDisabled(),
    //       intakeSubsystem.setPrepHandoff()
    //     )
    //   )
    //  );

    public final Command a_satisfyDDMW =
        driveSubsystem.doNothing().repeatedly();
    
    public final Command a_intakeCollect =
    new SequentialCommandGroup(
      new ParallelCommandGroup(
        intakeSubsystem.setIntake(),
        shooterSubsystem.setHandoffAllowance()
      ),
      new ParallelCommandGroup(
        intakeSubsystem.setPrepHandoff(),
        shooterSubsystem.setHandoffAllowance()
      ).repeatedly()
    );
    
    public final Command a_intakeReturn = 
    new SequentialCommandGroup(
        new ParallelRaceGroup(
          intakeSubsystem.setPrepHandoff().repeatedly(),
          shooterSubsystem.setHandoffAllowance()
        ),
        new ParallelCommandGroup(
          intakeSubsystem.setPrepHandoff(),
          shooterSubsystem.setDisabled()
        )
    );
    public SendableChooser<Command> a_waitChooser(){

      SendableChooser<Command> waitChooser = new SendableChooser<>();
      waitChooser.setDefaultOption("0sec", driveSubsystem.doNothing());
      waitChooser.addOption("1", driveSubsystem.doNothing().repeatedly().withTimeout(1));
      waitChooser.addOption("2", driveSubsystem.doNothing().repeatedly().withTimeout(2));
      waitChooser.addOption("3", driveSubsystem.doNothing().repeatedly().withTimeout(3));
      waitChooser.addOption("4", driveSubsystem.doNothing().repeatedly().withTimeout(4));
      waitChooser.addOption("5", driveSubsystem.doNothing().repeatedly().withTimeout(5));
      return waitChooser;

    }

    public final Command a_tempGetNoteDrive = 
        driveSubsystem.goDirectionTimeout(2,0.45,false);
    
    public final Command a_tempReturnNoteDrive =
        driveSubsystem.goDirectionTimeout(2,0.5,true);



    //DEFAULT Commands, d_
    public final void d_setDefaultCommands(CommandXboxController driverController){
        CommandScheduler.getInstance().setDefaultCommand(
            driveSubsystem,
            driveSubsystem.teleopDriveCommand(
                () -> driverController.getLeftY(),
                () -> driverController.getRightX()
            )
        );
        CommandScheduler.getInstance().setDefaultCommand(shooterSubsystem, shooterSubsystem.setDisabled());
        CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem, intakeSubsystem.setPrepHandoff());
        CommandScheduler.getInstance().setDefaultCommand(climberSubsystem, t_climberDown);
    }

    //SYSID Commands, s_
    public void s_bindSysIDCommands(CommandXboxController sysIDController){
        // Bind full set of SysId routine tests to buttons; a complete routine should run each of these once.
        // sysIDController.a().whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // sysIDController.b().whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // sysIDController.x().whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // sysIDController.y().whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
        // sysIDController.a().whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // sysIDController.b().whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // sysIDController.x().whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // sysIDController.y().whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // sysIDController.a().whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // sysIDController.b().whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // sysIDController.x().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // sysIDController.y().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

}
