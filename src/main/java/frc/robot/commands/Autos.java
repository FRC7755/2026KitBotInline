// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import static frc.robot.Constants.FuelConstants.FEEDER_RPMS;
//import static frc.robot.Constants.FuelConstants.INTAKE_RPMS;
//import static frc.robot.Constants.FuelConstants.LAUNCHER_RPMS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANLauncherSubsystem;
import frc.robot.subsystems.CANFeederSubsystem;
import frc.robot.subsystems.CANIntakeSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public final class Autos {
  public static final Command exampleAuto(CANDriveSubsystem driveSubsystem, CANLauncherSubsystem launcherSubsystem, CANFeederSubsystem feederSubsystem, CANIntakeSubsystem intakeSubsystem) {
    return new SequentialCommandGroup(
        // Drive backwards for .25 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
        driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(2),
        // Stop driving. This line uses the regular driveArcade command factory so it
        // ends immediately after commanding the motors to stop
        driveSubsystem.driveArcade(() -> 0, () -> 0),
        // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
        // total of 10 seconds
        launcherSubsystem.launcherRunOutCommand().withTimeout(1),
        //ParallelCommandGroup(
        Commands.parallel(
          launcherSubsystem.launcherRunOutCommand().withTimeout(9),
          intakeSubsystem.intakeRunInCommand().withTimeout(9),
          feederSubsystem.feederRunOutCommand().withTimeout(9)
        ).finallyDo((interrupted) -> {
          intakeSubsystem.intakeStop();
          feederSubsystem.feederStop();
          launcherSubsystem.launcherStop();
        })
    );
  }

  public static final Command newAuto(CANDriveSubsystem driveSubsystem, CANLauncherSubsystem launcherSubsystem, CANFeederSubsystem feederSubsystem, CANIntakeSubsystem intakeSubsystem) {
    Commands.sequence(
        // Drive backwards for .25 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
      driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(2),
        // Stop driving. This line uses the regular driveArcade command factory so it
        // ends immediately after commanding the motors to stop
      driveSubsystem.driveArcade(() -> 0, () -> 0),
        // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
        // total of 10 seconds
      launcherSubsystem.launcherRunOutCommand().withTimeout(1),
        //ParallelCommandGroup(
      Commands.parallel(
        launcherSubsystem.launcherRunOutCommand().withTimeout(9),
        intakeSubsystem.intakeRunInCommand().withTimeout(9),
        feederSubsystem.feederRunOutCommand().withTimeout(9)
        ).finallyDo((interrupted) -> {
          intakeSubsystem.intakeStop();
          feederSubsystem.feederStop();
          launcherSubsystem.launcherStop();
        })
    );
    return null;
  }

}
