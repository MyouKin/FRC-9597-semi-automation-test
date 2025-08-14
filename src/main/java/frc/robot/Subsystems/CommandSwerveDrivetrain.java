package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.MathUtils;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* vision */
    public  boolean kUseVision = true;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /* Auto */
    // PathPlanner config constants
    private static final Mass ROBOT_MASS = Kilogram.of(74);
    private static final MomentOfInertia ROBOT_MOI =KilogramSquareMeters.of(6.8);
    private static final double WHEEL_COF = 1.2;
    public static final SwerveModuleConstants SWERVE_MODULE_CONSTANTS = TunerConstants.FrontLeft;
    public static final Translation2d[] SWERVE_MODULE_OFFSETS =
    new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };

    // PID controller for translation to target position
    private final PIDController pidLineup = new PIDController(4, 0, 0), angleController = new PIDController(4, 0, 0);
    private boolean inPidTranslate = false;
    private static final double PID_TRANSLATION_SPEED_MPS = 1.5;// 最大线速度（m/s）
    private static final double PID_ROTATION_RAD_PER_SEC = Math.PI;// 最大角速度（rad/s）
    private static final double AUTON_PATH_CANCEL_RADIUS_M = 0.8; //到点容差半径，主要用在自动路径任务中作为提前结束的条件

    @AutoLogOutput
    private String closestReefName = "";
    private boolean reefTargetIsRight = true;
    // private PhotonTrackedTarget closestReefTag = ;

    /* Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =new SwerveRequest.ApplyRobotSpeeds();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants .
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        
        pidLineup.setTolerance(0.03);//m
        angleController.setTolerance(Units.degreesToRadians(1));//°

        angleController.enableContinuousInput(0, 2 * Math.PI);

        // Robot config
        RobotConfig robotConfig = null; // Initialize with null in case of exception
        try {
            robotConfig =
                    RobotConfig.fromGUISettings(); // Takes config from Robot Config on Pathplanner
            // Settings
        } catch (Exception e) {
            e.printStackTrace(); // Fallback to a default configuration
        }


        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> this.setControl(
                    m_pathApplyRobotSpeeds
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                robotConfig,//load the parameters from the gui
                () -> {

                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
        AutoBuilder.resetOdom(Constants.Vision.m_initialPose);

    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /*
     * Periodically update the operator perspective:
     * 1. If it has never been applied before (e.g., after a restart), apply immediately
     *    regardless of Driver Station state to ensure correct driving direction mid-match.
     * 2. If already applied, only update when the Driver Station is disabled
     *    to avoid sudden changes in driving direction during operation,
     *    ensuring consistent control behavior during testing.
     * 
     * Vision processing:
     * Execute vision data processing every cycle to keep perception information up to date.
     */
    @Override
    public void periodic() {

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /**
     * Translates the robot to a target position and orientation using PID control.
     * <p>
     * This command uses a linear PID controller for translation and a separate
     * controller for rotation to move the robot from its current pose to the
     * specified {@code pose}. It calculates the required field-relative chassis
     * speeds based on the distance and angle to the target, and terminates when
     * the position error is within the PID tolerance.
     *
     * @param pose The target pose (position and orientation) in field coordinates.
     * @return A {@link Command} that drives the robot until it reaches the target pose.
     */
    @SuppressWarnings("removal")
    public Command translateToPositionWithPID(Pose2d pose) {
        DoubleSupplier theta = () -> new Pose2d(pose.getTranslation(), new Rotation2d())//计算目标方向角
                .relativeTo(new Pose2d(getPose().getTranslation(), new Rotation2d()))
                .getTranslation().getAngle().getRadians();
        
        DoubleSupplier driveYaw = () -> (getRotation().getRadians() + 2 * Math.PI) % (2 * Math.PI);//获取当前朝向角
        
        DoubleSupplier distanceToTarget = () -> -new Pose2d(pose.getTranslation(), new Rotation2d())
                .relativeTo(new Pose2d(getPose().getTranslation(), new Rotation2d()))
                .getTranslation().getNorm();//当前位置与目标的距离
        
        return new PIDCommand(
            pidLineup, // PID控制器
            distanceToTarget, // 测量值供应商：当前距离
            0, // 设定值：目标距离为0
            (pidOutput) -> { // 控制输出处理
                inPidTranslate = true;
                
                // 使用PID输出计算底盘速度
                runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                    MathUtils.clamp(pidOutput * Math.cos(theta.getAsDouble()), 
                        -PID_TRANSLATION_SPEED_MPS, PID_TRANSLATION_SPEED_MPS),//计算Vx速度
                    MathUtils.clamp(pidOutput * Math.sin(theta.getAsDouble()), 
                        -PID_TRANSLATION_SPEED_MPS, PID_TRANSLATION_SPEED_MPS),//计算Vy速度
                    MathUtils.clamp(
                        angleController.calculate(driveYaw.getAsDouble(), pose.getRotation().getRadians()),
                        -PID_ROTATION_RAD_PER_SEC, PID_ROTATION_RAD_PER_SEC)),//计算角速度
                    getRotation()));
            },
            this // 子系统需求
        )
        .until(() -> pidLineup.atSetpoint()) // 使用until判断命令退出条件
        .andThen(() -> {
                // 命令结束后重置所有参数
                inPidTranslate = false;
                pidLineup.reset();
                angleController.reset();
                stop();
                System.out.println("translateToPositionWithPID command completed");
            });
    }

    /**
     * Returns a {@link BooleanSupplier} indicating whether both the translation and rotation
     * PID controllers are within their respective setpoint tolerances.
     *
     * @return A boolean supplier that is {@code true} when both PID controllers have reached
     *         their setpoints.
     */
    public BooleanSupplier translatePidInPosition() {
        return () -> pidLineup.atSetpoint() && angleController.atSetpoint();
    }

    /**
     * Creates a command that uses pathfinding to move the robot to the given target pose.
     * <p>
     * This command uses predefined motion constraints to ensure the robot moves within
     * safe velocity and acceleration limits.
     *
     * @param pose The desired target pose (position and orientation) on the field.
     * @return A {@link Command} that will pathfind to the target pose.
     */
    public Command pathfindToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, new PathConstraints(2, 2, Math.PI, 2 * Math.PI));
    }

    /**
     * Determines the closest scoring position (reef pose) based on AprilTag vision data.
     * <p>
     * This method processes vision data from multiple cameras to identify valid AprilTag targets,
     * selects the nearest reef tag, and calculates the optimal robot scoring position relative to it.
     * If no valid tags are detected, a default tag ID is used as a fallback.
     * <p>
     * The scoring position is calculated by applying a transformation to the detected tag's pose,
     * taking into account the robot's scoring side radius and branch offset.
     *
     * @param cameraEstimators A map of {@link PhotonCamera} to {@link PhotonPoseEstimator}, representing
     *                         the cameras and their respective pose estimators.
     * @return The closest reef scoring {@link Pose2d} relative to the field. If no valid tag is found,
     *         the robot's current pose is returned as a fallback.
     */
    public Pose2d closestReefPose(Map<PhotonCamera, PhotonPoseEstimator> cameraEstimators) {
        List<PhotonTrackedTarget> allValidTargets = new ArrayList<>();
        
        // 遍历所有相机，获取并合并所有有效的AprilTag目标
        cameraEstimators.forEach((camera, estimator) -> {
            // 使用getLatestResult()替代getAllUnreadResults()
            PhotonPipelineResult latestResult = camera.getLatestResult();
            
            // 检查结果是否有效且包含目标
            if (latestResult == null || !latestResult.hasTargets()) {
                System.out.println("No valid targets for camera: " + camera.getName());
                return;
            }
            
            // 筛选有效的reef tag并添加到合并列表中
            latestResult.getTargets().stream()
                .filter(target -> target.getFiducialId() != -1)
                .filter(target -> Constants.Vision.reefTagNames.containsKey(target.getFiducialId()))
                .forEach(allValidTargets::add);
        });
        
        // 从所有相机的结果中找到最近的有效AprilTag
        PhotonTrackedTarget closestTag = allValidTargets.stream()
            .min(Comparator.comparingDouble(target -> {
                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                return tagPose.map(pose -> pose.toPose2d().getTranslation().getDistance(getPose().getTranslation()))
                            .orElse(Double.MAX_VALUE);
            }))
            .orElse(null);

        // 如果没有找到有效tag，返回机器人当前位置
        if (closestTag == null) {
            System.out.println("No valid reef tags found, staying at current position");
            return getPose();
        }

        int targetTagId = closestTag.getFiducialId();
        closestReefName = Constants.Vision.reefTagNames.get(targetTagId);
        
        // 获取tag的场地位置
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(targetTagId);
        if (!tagPose.isPresent()) {
            // 如果找不到tag位置，返回当前机器人位置作为fallback
            System.err.println("Warning: Could not find pose for tag ID " + targetTagId);
            return getPose();
        }
        
        Pose2d tagPose2d = tagPose.get().toPose2d();
        
        // 计算得分位置
        Pose2d closestPose = tagPose2d
                .transformBy(new Transform2d(
                    Units.inchesToMeters(Constants.Vision.SCORING_SIDE_RADIUS_ROBOT_IN),
                    ((reefTargetIsRight ? Constants.Vision.TAG_TO_BRANCH_OFFSET_M : -Constants.Vision.TAG_TO_BRANCH_OFFSET_M)),
                    Rotation2d.kZero));
        


        return new Pose2d(closestPose.getTranslation(),
                closestPose.getRotation().plus(Constants.Vision.SCORING_SIDE_FROM_FRONT_ROT));
    }

    /**
     * Sets whether the reef target is located on the right side.
     *
     * @param reefTargetIsRight true if the target is on the right, false if on the left
     */
    public void setReefTargetIsRight(boolean reefTargetIsRight) {
        this.reefTargetIsRight = reefTargetIsRight;
    }

    /**
     * Sends the given robot-relative chassis speeds to the drivetrain.
     *
     * @param robotRelativeSpeeds The desired chassis speeds in the robot's reference frame
     */
    private void runVelocity(ChassisSpeeds robotRelativeSpeeds) {
        // m_pathApplyRobotSpeeds 是你类里已存在的 ApplyRobotSpeeds 实例
        this.setControl(m_pathApplyRobotSpeeds.withSpeeds(robotRelativeSpeeds));
    }

    /**
     * Stops the drivetrain by setting all chassis velocities to zero.
     */
    private void stop() {
        runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    /**
     * Initializes and starts a high-frequency simulation thread for drivetrain physics updates.
     * <p>
     * This method:
     * <ul>
     *   <li>Stores the current simulation time</li>
     *   <li>Creates a {@link Notifier} that repeatedly calls {@link #updateSimState} at a fixed interval</li>
     *   <li>Runs the simulation loop at a faster rate so PID gains behave more realistically</li>
     * </ul>
     */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Gets the current pose of the robot.
     *
     * @return the current robot pose as a {@link Pose2d} object.
     */
    public Pose2d getPose() {
        return this.getState().Pose;
    }

    /**
     * Returns the current rotation of the robot from odometry.
     *
     * @return the current robot rotation as a {@link Rotation2d}.
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /**
     * Creates a command to reset the robot's pose.
     *
     * @param pose the new pose to set for the robot.
     * @return a {@link Command} that resets the pose when executed.
     */
    public Command resetpose(Pose2d pose){
        return runOnce(
            ()->resetPose(pose));
    }

    /**
     * Gets the current robot-relative chassis speeds.
     *
     * @return the current speeds in the robot's frame of reference
     *         as a {@link ChassisSpeeds} object.
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getKinematics().toChassisSpeeds(this.getState().ModuleStates);
    }

    /**
     * Determines the starting state of the elevator system
     * based on whether vision is being used.
     *
     * @return {@link Constants.Elevator.State#START_AUTO} if vision is enabled,
     *         otherwise {@link Constants.Elevator.State#START_OPERATED}.
     */
    public Constants.Elevator.State Get_Auto_State(){
        if(kUseVision){
            return Constants.Elevator.State.START_AUTO;
        }
        else{
            return Constants.Elevator.State.START_OPERATED;
        }
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /**
     * Toggles the usage of vision data in the system.
     * <p>
     * When executed, this command inverts the current {@code kUseVision} status:
     * if vision data is currently enabled, it will be disabled, and vice versa.
     * This allows switching between vision-assisted and non-vision modes at runtime.
     *
     * @return a {@link Command} that performs the vision status toggle once when run
     */
    public Command ChangeVisionDataStatus(){
        return  runOnce(()->{
            //System.out.println("vision status"+kUseVision);
            kUseVision = !kUseVision;
        });
    }

}

