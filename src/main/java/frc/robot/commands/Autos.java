// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import static frc.robot.Constants.FuelConstants.FEEDER_RPMS;
//import static frc.robot.Constants.FuelConstants.INTAKE_RPMS;
import static frc.robot.Constants.AutoConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANLauncherSubsystem;
import frc.robot.subsystems.CANFeederSubsystem;
import frc.robot.subsystems.CANIntakeSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public final class Autos {
  public static final Command mainAuto(CANDriveSubsystem driveSubsystem, CANLauncherSubsystem launcherSubsystem, CANFeederSubsystem feederSubsystem, CANIntakeSubsystem intakeSubsystem) {
    return new SequentialCommandGroup(
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(SmartDashboard.getNumber("Auto Delay", INITIAL_DELAY)),
        driveSubsystem.driveArcade(() -> SmartDashboard.getNumber("Backup Speed", BACKUP_SPEED), () -> 0).withTimeout(SmartDashboard.getNumber("Backup Time", BACKUP_TIME)),
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(SmartDashboard.getNumber("Return Delay", RETURN_DELAY)),
        launcherSubsystem.launcherRunOutCommand().withTimeout(2),
        Commands.parallel(
          launcherSubsystem.launcherRunOutCommand().withTimeout(SmartDashboard.getNumber("Shooter Time", SHOOTER_TIME)),
          intakeSubsystem.intakeRunInCommand().withTimeout(SmartDashboard.getNumber("Shooter Time", SHOOTER_TIME)),
          feederSubsystem.feederRunOutCommand().withTimeout(SmartDashboard.getNumber("Shooter Time", SHOOTER_TIME))
        ).finallyDo((interrupted) -> {
          intakeSubsystem.intakeStop();
          feederSubsystem.feederStop();
          launcherSubsystem.launcherStop();
        }),
        driveSubsystem.driveArcade(() -> SmartDashboard.getNumber("Forward Speed", FORWARD_SPEED), () -> 0).withTimeout(SmartDashboard.getNumber("Forward Time", FORWARD_TIME)).onlyIf(() -> SmartDashboard.getBoolean("Return to Start", RETURN_TO_START)),
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(0.1)
    );
  }

  public static final Command Delay2SecAuto(CANDriveSubsystem driveSubsystem, CANLauncherSubsystem launcherSubsystem, CANFeederSubsystem feederSubsystem, CANIntakeSubsystem intakeSubsystem) {
    return new SequentialCommandGroup(
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(2),
        driveSubsystem.driveArcade(() -> SmartDashboard.getNumber("Backup Speed", BACKUP_SPEED), () -> 0).withTimeout(SmartDashboard.getNumber("Backup Time", BACKUP_TIME)),
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(SmartDashboard.getNumber("Return Delay", RETURN_DELAY)),
        launcherSubsystem.launcherRunOutCommand().withTimeout(2),
        Commands.parallel(
          launcherSubsystem.launcherRunOutCommand().withTimeout(SmartDashboard.getNumber("Shooter Time", SHOOTER_TIME)),
          intakeSubsystem.intakeRunInCommand().withTimeout(SmartDashboard.getNumber("Shooter Time", SHOOTER_TIME)),
          feederSubsystem.feederRunOutCommand().withTimeout(SmartDashboard.getNumber("Shooter Time", SHOOTER_TIME))
        ).finallyDo((interrupted) -> {
          intakeSubsystem.intakeStop();
          feederSubsystem.feederStop();
          launcherSubsystem.launcherStop();
        }),
        driveSubsystem.driveArcade(() -> SmartDashboard.getNumber("Forward Speed", FORWARD_SPEED), () -> 0).withTimeout(2),
        driveSubsystem.driveArcade(() -> 0, () -> 0).withTimeout(0.1)
    );
  }

}
