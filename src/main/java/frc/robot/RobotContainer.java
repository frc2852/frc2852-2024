package frc.robot;

import frc.robot.Constants.OperatorConstant;
import frc.robot.commands.AmpNoteDischarge;
import frc.robot.commands.SpeakerShot;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.PowerHub;
import frc.robot.subsystems.ClimbWheel;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Winch;
import frc.robot.util.DataTracker;
import frc.robot.util.constants.LogConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * RobotContainer is the class where the bulk of the robot's systems are
 * declared.
 * Here, subsystems, OI devices, and commands are set up and should be the only
 * place that this configuration exists in the code.
 */
public class RobotContainer {
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  // private final CommandXboxController sysIdController;

  // private final AprilTagDetectionSubsystem aprilTagDetectionSubsystem;
  // private final GamePieceDetectionSubsystem gamePieceDetectionSubsystem;

  private final PowerHub powerHubSubsystem;

  private final Drive driveSubsystem;
  private final Conveyor conveyorSubsystem;
  private final Elevator elevatorSubsystem;
  private final Intake intakeSubsystem;
  private final Shooter shooterSubsystem;
  private final Winch winchSubsystem;
  private final ClimbWheel climbWheelSubsystem;
  
  /**
   * Constructs the container for the robot. Subsystems and command mappings are
   * initialized here.
   */
  public RobotContainer() {

    // Start data logger
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Port forwarding
    PortForwarder.add(5800, "photonvision.local", 5800);

    // Initialize data tracker
    DataTracker.putBoolean(LogConstants.ROBOT_SYSTEM, "Intialization", true, false);

    // Initialize controllers with distinct ports
    driverController = new CommandXboxController(OperatorConstant.DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OperatorConstant.OPERATOR_CONTROLLER_PORT);
    // sysIdController = new CommandXboxController(OperatorConstant.SYSID_CONTROLLER_PORT);

    // Initialize helpers
    powerHubSubsystem = new PowerHub();
    powerHubSubsystem.highBeamsOff();
    powerHubSubsystem.reset();

    // Initialize vision
    // aprilTagDetectionSubsystem = new AprilTagDetectionSubsystem(CameraTracking.APRIL_TAG_CAMERA_CONFIG);
    // gamePieceDetectionSubsystem = new GamePieceDetectionSubsystem(CameraTracking.GAME_PIECE_CAMERA_CONFIG, powerHubSubsystem);

    // Initialize subsystems
    driveSubsystem = new Drive(null);
    // driveSubsystem = new DriveSubsystem(aprilTagDetectionSubsystem);

    conveyorSubsystem = new Conveyor();
    elevatorSubsystem = new Elevator();
    intakeSubsystem = new Intake();
    shooterSubsystem = new Shooter();
    winchSubsystem = new Winch();
    climbWheelSubsystem = new ClimbWheel();

    // Configuration
    configureDriverBindings();
    configureOperatorBindings();
    configureSysIdBindings();
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureDriverBindings() {
    driverController.leftBumper().onTrue(
        new RunCommand(() -> driveSubsystem.lockDrive(), driveSubsystem));

    driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> driveSubsystem.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstant.DEAD_BAND),
                -MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstant.DEAD_BAND),
                -MathUtil.applyDeadband(driverController.getRightX(), OperatorConstant.DEAD_BAND),
                true, true),
            driveSubsystem));
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureOperatorBindings() {

    // Set elevator to drive position, really shouldn't be used unless the operator messed up and pressed the climb button
    operatorController.povDown().onTrue(new RunCommand(() -> elevatorSubsystem.drivePosition(), elevatorSubsystem)
        .until(() -> elevatorSubsystem.isElevatorAtPosition()));

    operatorController.povLeft().onTrue(new RunCommand(() -> elevatorSubsystem.decreasePosition(), elevatorSubsystem)
        .until(() -> elevatorSubsystem.isElevatorAtPosition()));

    operatorController.povRight().onTrue(new RunCommand(() -> elevatorSubsystem.increasePosition(), elevatorSubsystem)
        .until(() -> elevatorSubsystem.isElevatorAtPosition()));

    // Intake note
    operatorController.a().onTrue(new ToggleIntake(intakeSubsystem));

    // AMP note discharge
    operatorController.b().onTrue(new AmpNoteDischarge(intakeSubsystem, conveyorSubsystem, shooterSubsystem, elevatorSubsystem));

    // Speaker note shooting
    operatorController.y().onTrue(new SpeakerShot(intakeSubsystem, conveyorSubsystem, shooterSubsystem));

    // Lift arms up and run conveyor until game piece is ready
    operatorController.leftBumper().onTrue(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                // Move the note up to the amp position
                new ParallelCommandGroup(
                    new RunCommand(() -> conveyorSubsystem.runConveyorForward(), conveyorSubsystem),
                    new RunCommand(() -> intakeSubsystem.runIntake(true), intakeSubsystem),
                    new RunCommand(() -> shooterSubsystem.divertGamePiece(), shooterSubsystem))
                    .until(() -> conveyorSubsystem.isGamePieceAmpReady()),

                // Stop the conveyor and shooter
                new ParallelCommandGroup(
                    new InstantCommand(() -> shooterSubsystem.stopShooter(), conveyorSubsystem),
                    new InstantCommand(() -> conveyorSubsystem.stopConveyor(), intakeSubsystem),
                    new InstantCommand(() -> intakeSubsystem.stopIntake(), shooterSubsystem)),

                // Then, move the elevator to climb position
                new RunCommand(() -> elevatorSubsystem.climbPosition(), elevatorSubsystem)
                    .until(() -> elevatorSubsystem.isElevatorAtPosition()))));

    // Climb and trap score
    operatorController.rightBumper().onTrue(
        new SequentialCommandGroup(
            // Move the elevator to drive position to hook the chain
            new RunCommand(() -> elevatorSubsystem.drivePosition(), elevatorSubsystem)
                .until(() -> elevatorSubsystem.isElevatorAtPosition()),

            // Run the conveyor and shooter in parallel until the game piece is ready
            new ParallelCommandGroup(
                new RunCommand(() -> climbWheelSubsystem.runClimbWheels(), climbWheelSubsystem),
                new RunCommand(() -> winchSubsystem.raiseRobot(), winchSubsystem),
                new RunCommand(() -> elevatorSubsystem.trapPosition(), elevatorSubsystem))
                .until(() -> elevatorSubsystem.isElevatorAtPosition() && winchSubsystem.isRobotAtPosition()),

            // Run the conveyor and shooter again to discharge the game piece
            new ParallelCommandGroup(
                new RunCommand(() -> climbWheelSubsystem.stopClimbWheels(), climbWheelSubsystem),
                new RunCommand(() -> shooterSubsystem.divertGamePiece(), shooterSubsystem),
                new RunCommand(() -> conveyorSubsystem.runConveyorForward(), conveyorSubsystem))
                .until(() -> !conveyorSubsystem.isGamePieceAmpReady()),

            // Finally, stop the conveyor and shooter
            new ParallelCommandGroup(
                new InstantCommand(() -> conveyorSubsystem.stopConveyor(), conveyorSubsystem),
                new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem))));
  }

  private void configureSysIdBindings() {
    // Intake
    // sysIdController.a().whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // sysIdController.b().whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // sysIdController.x().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // sysIdController.y().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Gets the command to run in autonomous mode.
   *
   * @return The autonomous command to run.`
   */
  public Command getAutonomousCommand() {
    return null;
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    // AutoConstants.MAX_SPEED_METERS_PER_SECOND,
    // AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(SwerveDrive.DRIVE_KINEMATICS);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(3, 0, new Rotation2d(0)),
    // config);

    // var thetaController = new ProfiledPIDController(
    // AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    // exampleTrajectory,
    // driveSubsystem::getPose, // Functional interface to feed supplier
    // SwerveDrive.DRIVE_KINEMATICS,

    // // Position controllers
    // new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
    // new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
    // thetaController,
    // driveSubsystem::setModuleStates,
    // driveSubsystem);

    // // Reset odometry to the starting pose of the trajectory.
    // driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, false, false));
  }
}
