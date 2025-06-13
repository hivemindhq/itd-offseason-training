package dev.nullftc.wpiftc.localization.impl.pinpoint.v1;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.nullftc.wpiftc.localization.FTCPoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A {@link FTCPoseEstimator} for the Sparkfun OTOS sensor.
 */
public class PinpointPoseEstimator extends FTCPoseEstimator<Pose2D> {
    private ChassisSpeeds speeds = new ChassisSpeeds();

    public PinpointPoseEstimator(Pose2D pose2d, Pose2d initialPose) {
        this(new PinpointKinematics(), pose2d, initialPose, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.45, 0.45, 1.0e9));
    }

    private PinpointPoseEstimator(PinpointKinematics kinematics, Pose2D reading, Pose2d initialPose, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        super(kinematics, new PinpointOdometry(kinematics, reading, initialPose), stateStdDevs, visionMeasurementStdDevs);
    }

    public void updateVelocity(Pose2D velocity, Pose2D position) {
        double vX = velocity.getX(DistanceUnit.METER) * Math.cos(-position.getHeading(AngleUnit.RADIANS)) - velocity.getY(DistanceUnit.METER) * Math.sin(-velocity.getHeading(AngleUnit.RADIANS));
        double vY = velocity.getX(DistanceUnit.METER) * Math.sin(-position.getHeading(AngleUnit.RADIANS)) + velocity.getY(DistanceUnit.METER) * Math.cos(-velocity.getHeading(AngleUnit.RADIANS));
        speeds = new ChassisSpeeds(vX, vY, velocity.getHeading(AngleUnit.RADIANS));
    }

    public void update(Pose2D wheelPositions) {
        super.update(new Rotation2d(wheelPositions.getHeading(AngleUnit.RADIANS)), wheelPositions);
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getEstimatedPosition().getRotation());
    }

    public ChassisSpeeds getRobotVelocity() {
        return speeds;
    }
}
