// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.AutoConstants.*;
import frc.robot.commands.Autos;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANLauncherSubsystem;
import frc.robot.subsystems.CANFeederSubsystem;
//import frc.robot.subsystems.CANClimberSubsystem;
import frc.robot.subsystems.CANIntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The autonomous chooser
//  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANLauncherSubsystem launcherSubsystem = new CANLauncherSubsystem();
  private final CANFeederSubsystem feederSubsystem = new CANFeederSubsystem();
  //private final CANClimberSubsystem climberSubsystem = new CANClimberSubsystem();
  private final CANIntakeSubsystem intakeSubsystem = new CANIntakeSubsystem();
  
  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  // The operator's controller box
  private final CommandGenericHID boxController = new CommandGenericHID(BOX_CONTROLLER_PORT);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    SmartDashboard.putNumber("Drive scaling value", DRIVE_SCALING);
    SmartDashboard.putNumber("Rotation scaling value", ROTATION_SCALING);
    SmartDashboard.putNumber("Left scaling value", LEFT_SCALING);
    SmartDashboard.putNumber("Right scaling value", RIGHT_SCALING);

    SmartDashboard.putNumber("Auto Delay", INITIAL_DELAY);
    SmartDashboard.putNumber("Backup Time", BACKUP_TIME);
    SmartDashboard.putNumber("Backup Speed", BACKUP_SPEED);
    SmartDashboard.putNumber("Return Delay", RETURN_DELAY);
    SmartDashboard.putNumber("Shooter Delay", SHOOTER_DELAY);
    SmartDashboard.putNumber("Shooter Time", SHOOTER_TIME);
    SmartDashboard.putNumber("Forward Time", FORWARD_TIME);
    SmartDashboard.putNumber("Forward Speed", FORWARD_SPEED);
    SmartDashboard.putBoolean("Return to Start", RETURN_TO_START);


    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
/*    autoChooser.setDefaultOption("Main Auto", Autos.mainAuto(driveSubsystem, launcherSubsystem, feederSubsystem, intakeSubsystem));
    autoChooser.addOption("Delay 2 Sec", Autos.Delay2SecAuto(driveSubsystem, launcherSubsystem, feederSubsystem, intakeSubsystem));
    autoChooser.addOption("Delay 4 Sec", Autos.Delay4SecAuto(driveSubsystem, launcherSubsystem, feederSubsystem, intakeSubsystem));
    SmartDashboard.putData("Auto Selection", autoChooser);
*/
  autoChooser.setDefaultOption("Main", "MAIN");
  autoChooser.addOption("Delay 2", "DELAY2");
  autoChooser.addOption("Delay 4", "DELAY4");
  SmartDashboard.putData("Auto Selection", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //operatorController.leftBumper()
    //    .whileTrue(launcherSubsystem.runEnd(() -> launcherSubsystem.intake(), () -> launcherSubsystem.stop()));
    //driverController.leftBumper().whileTrue(launcherSubsystem.runEnd(() -> launcherSubsystem.intake(), () -> launcherSubsystem.stop()));

    //driverController.rightBumper().whileTrue(launcherSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS).andThen(launcherSubsystem.launchCommand()).finallyDo(() -> launcherSubsystem.stop()));

    //driverController.y().whileTrue(climberSubsystem.runEnd(() -> climberSubsystem.climberup(), () -> climberSubsystem.stop()));
    //driverController.a().whileTrue(climberSubsystem.runEnd(() -> climberSubsystem.climberdown(), () -> climberSubsystem.stop()));

    //boxController.button(1).whileTrue(launcherSubsystem.runEnd(() -> launcherSubsystem.intake(), () -> launcherSubsystem.stop()));
    //boxController.button(2).whileTrue(launcherSubsystem.runEnd(() -> launcherSubsystem.eject(), () -> launcherSubsystem.stop()));
    //boxController.button(3).whileTrue(launcherSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS).andThen(launcherSubsystem.launchCommand()).finallyDo(() -> launcherSubsystem.stop()));
    //boxController.button(4).whileTrue(launcherSubsystem.runEnd(() -> launcherSubsystem.setVelocity(SmartDashboard.getNumber("Target RPM", SPIN_RPMS)), () -> launcherSubsystem.stop()));
    //boxController.button(5).whileTrue(launcherSubsystem.runEnd(() -> launcherSubsystem.feed(), () -> launcherSubsystem.stop()));

    //operatorController.leftBumper().whileTrue(launcherSubsystem.runEnd(() -> launcherSubsystem.intake(), () -> launcherSubsystem.stop()));
    //operatorController.rightBumper().whileTrue(launcherSubsystem.runEnd(() -> launcherSubsystem.eject(), () -> launcherSubsystem.stop()));
    //operatorController.b().whileTrue(feederSubsystem.runEnd(() -> feederSubsystem.feedout(), () -> feederSubsystem.stop()));
    //operatorController.x().whileTrue(feederSubsystem.runEnd(() -> feederSubsystem.feedin(), () -> feederSubsystem.stop()));
    //operatorController.y().whileTrue(launcherSubsystem.runEnd(() -> 
    //      launcherSubsystem.launcherRun(SmartDashboard.getNumber("Launcher Target RPM", LAUNCHER_RPMS)),
    //      () -> launcherSubsystem.launcherStop()));


    // Intake
    boxController.button(1).whileTrue(
      Commands.sequence(
        Commands.parallel(
            intakeSubsystem.intakeRunInCommand(),
            feederSubsystem.feederRunInCommand(),
            launcherSubsystem.launcherRunSlowCommand()
        )
        .finallyDo((interrupted) -> {
          intakeSubsystem.intakeStop();
          feederSubsystem.feederStop();
          launcherSubsystem.launcherStop();
        })
      )
    );

    // Eject
    boxController.button(2).whileTrue(
      Commands.sequence(
        Commands.parallel(
            intakeSubsystem.intakeRunOutCommand(),
            feederSubsystem.feederRunOutCommand(),
            launcherSubsystem.launcherRunSlowCommand()
        )
        .finallyDo((interrupted) -> {
          intakeSubsystem.intakeStop();
          feederSubsystem.feederStop();
          launcherSubsystem.launcherStop();
        })
      )
    );

    // Launch
    boxController.button(3).whileTrue(
      Commands.sequence(
        Commands.parallel(
            intakeSubsystem.intakeRunInCommand(),
            launcherSubsystem.launcherRunOutCommand()
        )
        .finallyDo((interrupted) -> {
          intakeSubsystem.intakeStop();
          launcherSubsystem.launcherStop();
        })
      )
    );

    // Feeder In Only
    boxController.button(4).whileTrue(
      Commands.sequence(
        Commands.parallel(
            feederSubsystem.feederRunInCommand()
        )
        .finallyDo((interrupted) -> {
          feederSubsystem.feederStop();
        })
      )
    );

    // Feeder Out Only
    boxController.button(5).whileTrue(
      Commands.sequence(
        Commands.parallel(
            feederSubsystem.feederRunOutCommand()
        )
        .finallyDo((interrupted) -> {
          feederSubsystem.feederStop();
        })
      )
    );

    // Intake In Only - Test Row
    boxController.button(6).whileTrue(
      Commands.sequence(
        Commands.parallel(
            intakeSubsystem.intakeRunInCommand()
        )
        .finallyDo((interrupted) -> {
          intakeSubsystem.intakeStop();
        })
      )
    );

    // Intake Out Only - Test Row
    boxController.button(7).whileTrue(
      Commands.sequence(
        Commands.parallel(
            intakeSubsystem.intakeRunOutCommand()
        )
        .finallyDo((interrupted) -> {
          intakeSubsystem.intakeStop();
        })
      )
    );

    // Launcher In Only - Test Row
    boxController.button(8).whileTrue(
      Commands.sequence(
        Commands.parallel(
            launcherSubsystem.launcherRunInCommand()
        )
        .finallyDo((interrupted) -> {
          launcherSubsystem.launcherStop();
        })
      )
    );

    // Launcher Out Only - Test Row
    boxController.button(9).whileTrue(
      Commands.sequence(
        Commands.parallel(
            launcherSubsystem.launcherRunOutCommand()
        )
        .finallyDo((interrupted) -> {
          launcherSubsystem.launcherStop();
        })
      )
    );

    // Feeder In Only
    boxController.button(10).whileTrue(
      Commands.sequence(
        Commands.parallel(
            feederSubsystem.feederRunInCommand()
        )
        .finallyDo((interrupted) -> {
          feederSubsystem.feederStop();
        })
      )
    );

    // Feeder Out Only
    boxController.button(11).whileTrue(
      Commands.sequence(
        Commands.parallel(
            feederSubsystem.feederRunOutCommand()
        )
        .finallyDo((interrupted) -> {
          feederSubsystem.feederStop();
        })
      )
    );


    
    //driverController.b().whileTrue(driveSubsystem.alignWithTag());

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). The X-axis is also inverted so a positive value (stick to the right)
    // results in clockwise rotation (front of the robot turning right). Both axes
    // are also scaled down so the rotation is more easily controllable.
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveArcade(
            () -> -driverController.getLeftY() * SmartDashboard.getNumber("Drive scaling value", DRIVE_SCALING),
            () -> -driverController.getRightX() * SmartDashboard.getNumber("Rotation scaling value", ROTATION_SCALING)));

//            driveSubsystem.driveTank(
//            () -> -driverController.getLeftY() * SmartDashboard.getNumber("Left scaling value", LEFT_SCALING),
//            () -> -driverController.getRightY() * SmartDashboard.getNumber("Right scaling value", RIGHT_SCALING)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
/*  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  } */
  public Command getAutonomousCommand() {
  // Map the string to the command only when auto starts
  return switch (autoChooser.getSelected()) {
    case "DELAY2" -> Autos.Delay2SecAuto(driveSubsystem, launcherSubsystem, feederSubsystem, intakeSubsystem);
    case "DELAY4" -> Autos.Delay4SecAuto(driveSubsystem, launcherSubsystem, feederSubsystem, intakeSubsystem);
    default -> Autos.mainAuto(driveSubsystem, launcherSubsystem, feederSubsystem, intakeSubsystem);
  };
}
}
