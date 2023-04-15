// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3647.frc2023.constants.LEDConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    public static final double kTenMSLoopTime = 0.01;
    public static final double kTwentyMSLoopTime = 0.02;

    private RobotContainer robotContainer = new RobotContainer();

    public Robot() {
        super(.02);
        addPeriodic(
                () -> robotContainer.superstructure.periodic(Timer.getFPGATimestamp()),
                kTwentyMSLoopTime,
                0.019);
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // // autonomous chooser on the dashboard.
        robotContainer.swerve.resetModuleAngle();
        PathPlannerServer.startServer(5811);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        robotContainer.LEDS.setAnimation(LEDConstants.BREATHE_RED);
        robotContainer.wrist.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        robotContainer.swerve.zeroPitch();
        autonomousCommand = robotContainer.getAutonomousCommand();
        robotContainer.wrist.setNeutralMode(NeutralMode.Brake);


        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        robotContainer.wrist.setNeutralMode(NeutralMode.Brake);

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.superstructure.enableCompressor().schedule();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        robotContainer.configTestCommands();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
