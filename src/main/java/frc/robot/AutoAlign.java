package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;
import com.therekrab.autopilot.Autopilot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlign extends Command {
    private APTarget m_target;

    private static final APConstraints kConstraints = new APConstraints()
            .withAcceleration(5.0)
            .withJerk(2.0);
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;

    private String branch;

    private static final APProfile kProfile = new APProfile(kConstraints)
            .withErrorXY(Centimeters.of(2))
            .withErrorTheta(Degrees.of(0.5))
            .withBeelineRadius(Centimeters.of(8));

    public static final Autopilot kAutopilot = new Autopilot(kProfile);

    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(4, 0, 0); /* tune this for your robot! */

    public AutoAlign(APTarget target, 
            String branch,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier) {
        this.m_target = target;
        this.branch = branch;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
    }

    public void execute() {





        // Find closest reef side for the chosen branch









    }

}
