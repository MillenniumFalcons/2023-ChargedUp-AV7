package team3647.frc2023.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.LinkedList;
import java.util.List;

import team3647.frc2023.commands.SwerveDriveNoAim;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
// import team3647.lib.tracking.RobotTracker;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Joysticks mainController = new Joysticks(0);
//     private final Joysticks coController = new Joysticks(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        scheduler.registerSubsystem(
                m_swerve,
                m_printer);

        configureButtonBindings();
        configureDefaultCommands();
        configureSmartDashboardLogging();
        // chooseAuto();
        // rot 2d is the rotation of the robot relative to field during auto
        // m_swerve.setOdometry(startPosition, startPosition.getRotation());

        // m_swerve.setOdometry(
        //         PathPlannerTrajectories.startStateStraight,
        //         new Rotation2d(Units.degreesToRadians(-45)));
        // m_swerve.setOdometry(
        //         PathPlannerTrajectories.startStateSixBallBump1,
        //         new Rotation2d(Units.degreesToRadians(45)));
    }

    private void configureButtonBindings() {
        mainController.buttonA.onTrue(new InstantCommand(() -> m_swerve.zeroHeading()));
     }

    private void configureDefaultCommands() {
        m_swerve.setDefaultCommand(
                new SwerveDriveNoAim(
                        m_swerve,
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        // () -> new Pose2d(),
                        () -> true));
    }

    public void configureSmartDashboardLogging() {
        m_printer.addPose("Robot", m_swerve::getPose);
        // m_printer.addDouble("intake velocity brr", () -> m_intake.getVelocity()););
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }

    public void updateTapeTranslations(List<Translation2d> translations) {
        List<Pose2d> poses = new LinkedList<>();
        translations.stream()
                .map((translation) -> new Pose2d(translation, new Rotation2d()))
                .forEach(poses::add);
        m_printer.getField().getObject("Tapes").setPoses(poses);
    }

    public final SwerveDrive m_swerve =
            new SwerveDrive(
                    SwerveDriveConstants.kFrontLeftModule,
                    SwerveDriveConstants.kFrontRightModule,
                    SwerveDriveConstants.kBackLeftModule,
                    SwerveDriveConstants.kBackRightModule,
                    SwerveDriveConstants.kGyro);

    final Superstructure m_superstructure =
            new Superstructure(m_swerve::isStopped);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private final GroupPrinter m_printer = GroupPrinter.getInstance();
}
