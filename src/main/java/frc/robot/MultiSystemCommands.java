// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
public class MultiSystemCommands {

    private ClimberSubsystem m_climberSubsystem;
    private DriveSubsystem m_driveSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private LEDSubsystem m_ledSubsystem;
    private ShooterSubsystem m_shooterSubsystem;

    public MultiSystemCommands(){
        m_climberSubsystem = new ClimberSubsystem();
        m_driveSubsystem = new DriveSubsystem();
        m_intakeSubsystem = new IntakeSubsystem();
        m_ledSubsystem = new LEDSubsystem();
        m_shooterSubsystem = new ShooterSubsystem();
    }

    //TELOP Commands, t_
    public final Command t_intakeNote = m_intakeSubsystem.setIntake();
    public final Command t_handoffNote = 
    new ParallelCommandGroup(
        m_intakeSubsystem.setPrepHandoff(),
        m_shooterSubsystem.setHandoffAllowance()
    );

    public final Command t_shootNote =
    new ParallelCommandGroup(
        m_driveSubsystem.confirmShootingPosition().repeatedly(),
        m_shooterSubsystem.setFireLow().repeatedly(),
        new SequentialCommandGroup(
            new WaitCommand(4).until(m_shooterSubsystem.velocityAboveLowGoal()),
            m_intakeSubsystem.setHandoff().repeatedly()
        )
    );

    public final Command t_removeNote =
    new SequentialCommandGroup(
        m_intakeSubsystem.setIdling().repeatedly().withTimeout(0.2),
        m_intakeSubsystem.removeNote().repeatedly()
    );


    //AUTON Commands, a_
    public final Command a_shootStraight = 
    new ParallelRaceGroup(
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
        )
      )
     );

    public final Command a_satisfyDDMW =
        m_driveSubsystem.doNothing().repeatedly();
    
    public final Command a_intakeCollect =
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
        new WaitCommand(0.15)
      );

    public final Command a_tempGetNoteDrive = 
        m_driveSubsystem.goDirectionTimeout(2,0.45,false);
    
    public final Command a_tempReturnNoteDrive =
        m_driveSubsystem.goDirectionTimeout(2,0.5,true);



    //DEFAULT Commands, d_
    public final void d_setDefaultCommands(CommandXboxController driverController){
        CommandScheduler.getInstance().setDefaultCommand(
            m_driveSubsystem,
            m_driveSubsystem.teleopDriveCommand(
                () -> driverController.getLeftY(),
                () -> driverController.getRightX()
            )
        );
        CommandScheduler.getInstance().setDefaultCommand(m_shooterSubsystem, m_shooterSubsystem.setDisabled());
        CommandScheduler.getInstance().setDefaultCommand(m_intakeSubsystem, m_intakeSubsystem.setPrepHandoff());
    }

    //SYSID Commands, s_
    public void s_bindSysIDCommands(CommandXboxController sysIDController){
        // Bind full set of SysId routine tests to buttons; a complete routine should run each of these once.
        sysIDController.a().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysIDController.b().whileTrue(m_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysIDController.x().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysIDController.y().whileTrue(m_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
        // sysIDController.a().whileTrue(m_shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // sysIDController.b().whileTrue(m_shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // sysIDController.x().whileTrue(m_shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // sysIDController.y().whileTrue(m_shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // sysIDController.a().whileTrue(m_intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // sysIDController.b().whileTrue(m_intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // sysIDController.x().whileTrue(m_intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // sysIDController.y().whileTrue(m_intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

}
