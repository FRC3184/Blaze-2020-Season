/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    public final OI oi = new OI();

    private DrivetrainSubsystem m_drivetrainSubsystem;
    public DrivetrainCommand m_drivetrainCommand;

    public IntakeSubsystem m_intakeSubsystem;

    public IntakeCommand m_intakeCommand;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        //create commands/subsystems
        init();
    }

    private void init() {
        try {
            m_drivetrainSubsystem = new DrivetrainSubsystem();
        } catch (Exception e) {
            DriverStation.reportWarning("Drive Subsystem failed to instantiate with error:" + e.toString(), false);
        }

        try {
            m_drivetrainCommand = new DrivetrainCommand(oi, m_drivetrainSubsystem);
        } catch (Exception e) {
            DriverStation.reportWarning("Drive Command failed to instantiate with error:" + e.toString(), false);
        }

        try {
            m_intakeSubsystem = new IntakeSubsystem();
        } catch (Exception e) {
            DriverStation.reportWarning("Intake Subsystem failed to instantiate with error:" + e.toString(), false);
        }

        try {
            m_intakeCommand = new IntakeCommand(oi, m_intakeSubsystem);
        } catch (Exception e) {
            DriverStation.reportWarning("Intake Command failed to instantiate with error:" + e.toString(), false);
        }
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_autoCommand;
    }
}
