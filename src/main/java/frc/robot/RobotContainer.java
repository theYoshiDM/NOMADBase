// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.HandS;

import frc.robot.subsystems.YAMSIntakePivot;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandXboxController joystick = new CommandXboxController(0);


    // public final IntakePivotS intakePivot = new IntakePivotS();



    public final HandS handRoller = new HandS();




    public final YAMSIntakePivot yIntakePivot = new YAMSIntakePivot();


    private Mechanism2d VISUALIZER;

    private final AutoChooser m_chooser = new AutoChooser();
    private final StateMachine stateMachine = new StateMachine();



   

    private void configureBindings() {


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.


        joystick.a().onTrue(
                stateMachine.stowCoral());


    }



    // TODO: add to state machine, delete
    public Command Stow() {
        return yIntakePivot.setAngle(YAMSIntakePivot.L1_ANGLE);
    }



    public Command getAutonomousCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
    }





}
