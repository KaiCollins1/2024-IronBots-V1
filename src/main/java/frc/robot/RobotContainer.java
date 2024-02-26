// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

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
      m_shooterSubsystem.setShooterSpeed(
        m_driverController.a(), //high speed
        m_driverController.b() //low speed
      )
    );

    CommandScheduler.getInstance().setDefaultCommand(
      m_intakeSubsystem,
      m_intakeSubsystem.tempDefaultCommand(
        m_driverController.povUp(),     //move up
        m_driverController.povDown(),   //move down
        m_driverController.povLeft(),   //move in
        m_driverController.povRight()   //move out
      )
    );



    // Bind full set of SysId routine tests to buttons; a complete routine should run each of these once.
    // m_driverController.a().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.b().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.x().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.y().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // m_driverController.a().whileTrue(m_shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.b().whileTrue(m_shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.x().whileTrue(m_shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.y().whileTrue(m_shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

  }

  //@return the command to run in autonomous
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void updateSchedulerTelemetry() {
    SmartDashboard.putData(m_driveSubsystem);
    SmartDashboard.putData(m_shooterSubsystem);
    SmartDashboard.putData(m_intakeSubsystem);
    SmartDashboard.putNumber("avgShooterSpeed", m_shooterSubsystem.getAvgSpeed());
  }

}
