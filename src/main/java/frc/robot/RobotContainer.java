// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrain;
  private final Joystick driveJoystick;
  private final Joystick steerJoystick;
  private boolean fieldCentric;

  public RobotContainer() {
    driveJoystick = new Joystick(0);
    steerJoystick = new Joystick(1);
    
    drivetrain = new DrivetrainSubsystem();

    drivetrain.setDefaultCommand(
      new DriveCommand(
          // Forward velocity supplier.
          driveJoystick::getY,
          // Sideways velocity supplier.
          driveJoystick::getX,
          // Rotation velocity supplier.
          steerJoystick::getX,
          () -> fieldCentric,
          drivetrain
      )
    );
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    JoystickButton zeroGyroButton = new JoystickButton(steerJoystick, 2);
    zeroGyroButton.onTrue(new InstantCommand(() -> drivetrain.zeroGyro(), drivetrain));

    JoystickButton driveModeToggleButton = new JoystickButton(steerJoystick, 5);
    driveModeToggleButton.onTrue(new InstantCommand( () -> fieldCentric = !fieldCentric));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
