// package team3647.lib.tracking;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Twist2d;
// import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// /** Inspired by (basically copied lol) 254 RobotState class */
// public class RobotTracker {
//     private final double kBufferLengthSeconds;

//     private final TimeInterpolatableBuffer<Pose2d> fieldToRobot;
//     private final TimeInterpolatableBuffer<Pose2d> robotToTurret;
//     private final Translation2d kRobotToTurretFixed;
//     private final Supplier<Pose2d> drivetrainGetPose;
//     private Pose2d previousPose;
//     private final DoubleSupplier drivetrainGetPoseTS;
//     private final Supplier<Rotation2d> turretGetRotation;
//     private final DoubleSupplier turretGetRotationTS;

//     private Twist2d measuredVelocity = new Twist2d();

//     public RobotTracker(
//             double bufferLengthSeconds,
//             Translation2d robotToTurretFixed,
//             Supplier<Pose2d> drivetrainGetPose,
//             DoubleSupplier drivetrainGetPoseTS,
//             Supplier<Rotation2d> turretGetRotation,
//             DoubleSupplier turretGetRotationTS) {
//         this.kBufferLengthSeconds = bufferLengthSeconds;
//         this.kRobotToTurretFixed = robotToTurretFixed;
//         fieldToRobot = TimeInterpolatableBuffer.createBuffer(kBufferLengthSeconds);
//         robotToTurret = TimeInterpolatableBuffer.createBuffer(kBufferLengthSeconds);
//         this.drivetrainGetPose = drivetrainGetPose;
//         this.drivetrainGetPoseTS = drivetrainGetPoseTS;
//         this.turretGetRotation = turretGetRotation;
//         this.turretGetRotationTS = turretGetRotationTS;
//         previousPose = drivetrainGetPose.get();
//     }

//     public void clear() {
//         this.fieldToRobot.clear();
//         this.robotToTurret.clear();
//     }

//     public void update() {
//         addFieldToRobotObservation(drivetrainGetPoseTS.getAsDouble(), drivetrainGetPose.get());
//         addTurretRotationObservation(turretGetRotationTS.getAsDouble(), turretGetRotation.get());
//     }

//     public synchronized void addFieldToRobotObservation(double timestamp, Pose2d observation) {
//         setRobotVelocityObservation(this.previousPose.log(observation));
//         this.previousPose = observation;
//         fieldToRobot.addSample(timestamp, observation);
//     }

//     public synchronized void addTurretRotationObservation(
//             double timestamp, Rotation2d observation) {
//         robotToTurret.addSample(timestamp, new Pose2d(kRobotToTurretFixed, observation));
//     }

//     public synchronized void setRobotVelocityObservation(Twist2d measuredVelocity) {
//         this.measuredVelocity = measuredVelocity;
//     }

//     public synchronized Pose2d getFieldToRobot(double timestamp) {
//         return fieldToRobot.getSample(timestamp);
//     }

//     public synchronized Pose2d getRobotToTurret(double timestamp) {
//         return robotToTurret.getSample(timestamp);
//     }

//     public Pose2d getFieldToTurret(double timestamp) {
//         var ftr = getFieldToRobot(timestamp);
//         var rtt = getRobotToTurret(timestamp);
//         if (ftr == null || rtt == null) {
//             return null;
//         }
//         SmartDashboard.putNumber("Robot To Turret rotation", rtt.getRotation().getDegrees());
//         return ftr.transformBy(new Transform2d(rtt.getTranslation(), rtt.getRotation()));
//     }

//     public Transform2d getTurretToTarget(double timestamp, Pose2d fieldToTarget) {
//         var ftTurret = getFieldToTurret(timestamp);
//         if (ftTurret == null || fieldToTarget == null) {
//             return null;
//         }
//         return fieldToTarget.minus(ftTurret);
//         // X->Y = Z->y - Z->X
//     }

//     public synchronized Twist2d getMeasuredVelocity() {
//         return measuredVelocity;
//     }
// }
