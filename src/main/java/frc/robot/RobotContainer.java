// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.GeneralConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final CommandXboxController m_driverController = new CommandXboxController(GeneralConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    // //collects the note!
    // NamedCommands.registerCommand("intakeCollect", 
    //   m_intakeSubsystem.setIntake().repeatedly().withTimeout(2)
    //   .andThen(m_shooterSubsystem.setHandoffAllowance())
    // );
    NamedCommands.registerCommand("intakeCollect", 
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          m_intakeSubsystem.setPrepHandoff(),
          m_shooterSubsystem.setDisabled()
        ),
        new SequentialCommandGroup(
          m_intakeSubsystem.setIntake().repeatedly().withTimeout(2),
          m_shooterSubsystem.setHandoffAllowance()
        ).withTimeout(2.5),
        new ParallelCommandGroup(
          m_intakeSubsystem.setPrepHandoff(),
          m_shooterSubsystem.setDisabled()
        ),
        new WaitCommand(0.25)
      )
    );
    //scores the note
    // NamedCommands.registerCommand("straightShoot", 
    //   m_driveSubsystem.confirmShootingPosition().repeatedly()
    //   .alongWith(
    //     m_shooterSubsystem.setFireLow().repeatedly().until(m_shooterSubsystem.velocityAboveLowGoal())
    //     .andThen(m_intakeSubsystem.setHandoff().repeatedly()
    //     .alongWith(m_shooterSubsystem.setFireLow().repeatedly()))
    //   ).withTimeout(3).andThen(
    //     m_shooterSubsystem.setDisabled()
    //     .alongWith(m_intakeSubsystem.setPrepHandoff())
    //   )
    // );
    NamedCommands.registerCommand("straightShoot",
     new ParallelCommandGroup(
      m_driveSubsystem.confirmShootingPosition().repeatedly().withTimeout(3.5),
      new SequentialCommandGroup(
        m_shooterSubsystem.setFireLow().repeatedly().until(m_shooterSubsystem.velocityAboveLowGoal()),
        new ParallelCommandGroup(
          m_intakeSubsystem.setHandoff().repeatedly(),
          m_shooterSubsystem.setFireLow().repeatedly()
        ).withTimeout(2.5),
        new ParallelCommandGroup(
          m_shooterSubsystem.setDisabled(),
          m_intakeSubsystem.setPrepHandoff()
        ).withTimeout(0.5)
      )
     )
    );

    NamedCommands.registerCommand("TempGetNoteDrive", 
      m_driveSubsystem.goDirectionTimeout(2,0.45,false)
    );
    NamedCommands.registerCommand("TempReturnNoteDrive", 
      m_driveSubsystem.goDirectionTimeout(2,0.5,true)
    );

    // new ParallelCommandGroup(
    //   m_driveSubsystem.confirmShootingPosition().repeatedly().withTimeout(3.5),
    //   new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //       m_intakeSubsystem.setPrepHandoff(),
    //       m_shooterSubsystem.setDisabled()
    //     ),
    //     new SequentialCommandGroup(
    //       m_intakeSubsystem.setIntake().repeatedly().withTimeout(2),
    //       m_shooterSubsystem.setHandoffAllowance()
    //     ).withTimeout(2.5),
    //     new ParallelCommandGroup(
    //       m_intakeSubsystem.setPrepHandoff(),
    //       m_shooterSubsystem.setDisabled()
    //     ),
    //     new WaitCommand(0.25)
    //   )
    //  );
    //satisfies differential drive motor watchdog DDMW for when the bot is doing nothing
    NamedCommands.registerCommand("satisfyDDMW", 
      m_driveSubsystem.doNothing().repeatedly()
    );

    CameraServer.startAutomaticCapture();

    //AutoBuilder gets these from the deploy directory. To clear it, follow these directions:
    //To FTP to the roboRIO, open a Windows Explorer window. In the address bar, 
    //type ftp://roboRIO-4983-frc.local and press enter. You can now browse the 
    //roboRIO file system just like you would browse files on your computer.
    //https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ftp.html#ftp
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }


  private void configureBindings() {
 
    CommandScheduler.getInstance().setDefaultCommand(
      m_driveSubsystem,
      m_driveSubsystem.teleopDriveCommand(
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getRightX()
      )
    );

    CommandScheduler.getInstance().setDefaultCommand(
      m_shooterSubsystem, 
      m_shooterSubsystem.setDisabled()
    );

    CommandScheduler.getInstance().setDefaultCommand(
      m_intakeSubsystem,
      m_intakeSubsystem.setPrepHandoff()
    );

    m_driverController.rightBumper().whileTrue(
      m_intakeSubsystem.setIntake()
    ).onFalse(
      m_shooterSubsystem.setHandoffAllowance()
    );

    m_driverController.leftBumper().whileTrue(
      m_shooterSubsystem.setFireLow().repeatedly().until(m_shooterSubsystem.velocityAboveLowGoal())
      .deadlineWith(
        m_driveSubsystem.confirmShootingPosition().repeatedly()
      )
      .andThen(
        m_intakeSubsystem.setHandoff().repeatedly()
        .alongWith(m_shooterSubsystem.setFireLow().repeatedly())
        .alongWith(m_driveSubsystem.confirmShootingPosition().repeatedly())
      )
    );

    m_driverController.x().whileTrue(
      m_intakeSubsystem.setIdling().repeatedly().withTimeout(0.2)
      .andThen(m_intakeSubsystem.removeNote().repeatedly())
    );


    // m_driverController.a().onTrue(m_intakeSubsystem.setIntake());
    // m_driverController.b().onTrue(m_intakeSubsystem.setIdling());
    // m_driverController.x().onTrue(m_intakeSubsystem.prepHandoff());


    // // Bind full set of SysId routine tests to buttons; a complete routine should run each of these once.
    // m_driverController.a().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.b().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.x().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.y().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    
    // m_driverController.a().whileTrue(m_shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.b().whileTrue(m_shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.x().whileTrue(m_shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.y().whileTrue(m_shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // m_driverController.a().whileTrue(m_intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.b().whileTrue(m_intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.x().whileTrue(m_intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.y().whileTrue(m_intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

  }

  //@return the command to run in autonomous
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return null;
    // return m_driveSubsystem.doNothing().withTimeout(0.5).andThen(
    // m_shooterSubsystem.setFireLow().repeatedly().until(m_shooterSubsystem.velocityAboveLowGoal())
    //   .deadlineWith(
    //     m_driveSubsystem.confirmShootingPosition().repeatedly()
    //   )
    //   .andThen(
    //     m_intakeSubsystem.setHandoff().repeatedly()
    //     .alongWith(m_shooterSubsystem.setFireLow().repeatedly())
    //     .alongWith(m_driveSubsystem.confirmShootingPosition().repeatedly()).withTimeout(2)
    //   )
    //   .andThen(m_driveSubsystem.teleopDriveCommand(() -> -0.5, () -> 0).repeatedly().withTimeout(2)));
  }

  public void updateSchedulerTelemetry() {
    SmartDashboard.putData(m_driveSubsystem);
    SmartDashboard.putData(m_shooterSubsystem);
    SmartDashboard.putData(m_intakeSubsystem);
    SmartDashboard.putNumber("avgShooterSpeed", m_shooterSubsystem.getAvgSpeed());
  }

}
