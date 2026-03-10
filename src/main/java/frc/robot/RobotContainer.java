// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.FuelConstants.*;
import frc.robot.commands.Autos;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANFeederSubsystem;
import frc.robot.subsystems.CANClimberSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();
  private final CANFeederSubsystem feederSubsystem = new CANFeederSubsystem();
  private final CANClimberSubsystem climberSubsystem = new CANClimberSubsystem();
  
  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
  //private final CommandGenericHID boxController = new CommandGenericHID(BOX_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

  SmartDashboard.putNumber("Drive scaling value", DRIVE_SCALING);
  SmartDashboard.putNumber("Rotation scaling value", ROTATION_SCALING);

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(driveSubsystem, ballSubsystem));
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
    //    .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    //driverController.leftBumper().whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));

    //driverController.rightBumper().whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS).andThen(ballSubsystem.launchCommand()).finallyDo(() -> ballSubsystem.stop()));

    driverController.y().whileTrue(climberSubsystem.runEnd(() -> climberSubsystem.climberup(), () -> climberSubsystem.stop()));
    driverController.a().whileTrue(climberSubsystem.runEnd(() -> climberSubsystem.climberdown(), () -> climberSubsystem.stop()));

    //boxController.button(1).whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    //boxController.button(2).whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
    //boxController.button(3).whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS).andThen(ballSubsystem.launchCommand()).finallyDo(() -> ballSubsystem.stop()));
    //boxController.button(4).whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.setVelocity(SmartDashboard.getNumber("Target RPM", SPIN_RPMS)), () -> ballSubsystem.stop()));
    //boxController.button(5).whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.feed(), () -> ballSubsystem.stop()));

    operatorController.leftBumper().whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    operatorController.rightBumper().whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
    operatorController.a().whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS).andThen(ballSubsystem.launchCommand()).finallyDo(() -> ballSubsystem.stop()));
    operatorController.y().whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.launchVelocity(SmartDashboard.getNumber("Target RPM", SPIN_RPMS)), () -> ballSubsystem.stop()));
    operatorController.b().whileTrue(feederSubsystem.runEnd(() -> feederSubsystem.feedout(), () -> feederSubsystem.stop()));
    operatorController.x().whileTrue(feederSubsystem.runEnd(() -> feederSubsystem.feedin(), () -> feederSubsystem.stop()));

    driverController.b().whileTrue(driveSubsystem.alignWithTag());

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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
