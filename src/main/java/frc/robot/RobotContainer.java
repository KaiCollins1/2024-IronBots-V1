// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.multi.MultiButtonUI;

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
import frc.robot.Constants.GeneralConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final MultiSystemCommands systemCommands = new MultiSystemCommands();

  private final CommandXboxController m_driverController = new CommandXboxController(GeneralConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    configureBindings();
    systemCommands.d_setDefaultCommands(m_driverController);

    NamedCommands.registerCommand("intakeCollect", systemCommands.a_intakeCollect);
    NamedCommands.registerCommand("straightShoot", systemCommands.a_shootStraight);
    NamedCommands.registerCommand("TempGetNoteDrive", systemCommands.a_tempGetNoteDrive);
    NamedCommands.registerCommand("TempReturnNoteDrive", systemCommands.a_tempReturnNoteDrive);
    NamedCommands.registerCommand("satisfyDDMW", systemCommands.a_satisfyDDMW);

    //AutoBuilder gets these from the deploy directory. To clear it, follow these directions:
    //To FTP to the roboRIO, open a Windows Explorer window. In the address bar, 
    //type ftp://roboRIO-4983-frc.local and press enter. You can now browse the 
    //roboRIO file system just like you would browse files on your computer.
    //https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ftp.html#ftp
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }


  private void configureBindings() {

    m_driverController.rightBumper().whileTrue(systemCommands.t_intakeNote).onFalse(systemCommands.t_handoffNote);
    m_driverController.leftBumper().whileTrue(systemCommands.t_shootNote);
    m_driverController.x().whileTrue(systemCommands.t_removeNote);

    systemCommands.s_bindSysIDCommands(m_driverController);

  }

  //@return the command to run in autonomous
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void updateSchedulerTelemetry() {
  }

}
