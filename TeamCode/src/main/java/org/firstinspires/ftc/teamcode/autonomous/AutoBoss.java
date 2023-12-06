package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.GetCurrentRobotType;
import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.opencv.TeamPropDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.impl.ClawSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.LiftSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.RotatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.TumblerSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AutoBoss", group = "Auto")
public class AutoBoss extends LinearOpMode
{
	private MecanumDrive mecanumDrive;
	private TumblerSystem tumblerSystem;
	private RotatorSystem rotatorSystem;
	private ClawSystem clawSystem;
	private IntakeSystem intakeSystem;
	private LiftSystem liftSystem;

	private OpenCvCamera camera;
	private TeamPropDetectionPipeline detectionPipeline;

	private Utilities.DetectionCase detectionCase;
	private Utilities.Alliance currentAlliance = null;
	private Utilities.PathType currentPath = null;

	@Override
	public void runOpMode()
	{
		Init();
		Detection();
		Run();
	}

	private void Detection()
	{
		while (!isStarted()) {
			detectionCase = detectionPipeline.getDetectionCase();
			telemetry.addData("Case: ", detectionPipeline.getDetectionCase().name());
			telemetry.update();
		}
	}

	private void Init()
	{
		Utilities.RobotType robotType = GetCurrentRobotType(hardwareMap, telemetry, gamepad1, gamepad2);
		Constants.Init(robotType);

		telemetry.addLine("Please select an alliance");
		telemetry.addLine("Press A for Red Alliance");
		telemetry.addLine("Press B for Blue Alliance");
		telemetry.update();
		while (currentAlliance == null && !isStopRequested()) {
			if (gamepad1.a || gamepad2.a) currentAlliance = Utilities.Alliance.RED;
			else if (gamepad1.b || gamepad2.b) currentAlliance = Utilities.Alliance.BLUE;
		}

		telemetry.addLine("Please select a path");
		telemetry.addLine("Press X for Short Path");
		telemetry.addLine("Press Y for Long Path");
		telemetry.update();
		while (currentPath == null && !isStopRequested()) {
			if (gamepad1.x || gamepad2.x) currentPath = Utilities.PathType.SHORT;
			else if (gamepad1.y || gamepad2.y) currentPath = Utilities.PathType.LONG;
		}

		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), robotType);

		liftSystem = new LiftSystem(hardwareMap.get(DcMotorEx.class, "lift1"), hardwareMap.get(DcMotorEx.class, "lift2"));
		tumblerSystem = new TumblerSystem(hardwareMap.get(DcMotorEx.class, "tumbler"));
		intakeSystem = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"));
		rotatorSystem = new RotatorSystem(hardwareMap.get(Servo.class, "rotator"));
		clawSystem = new ClawSystem(hardwareMap.get(Servo.class, "claw"));

		tumblerSystem.setRobotType(robotType);

		liftSystem.Init();
		tumblerSystem.Init();
		intakeSystem.Init();
		rotatorSystem.Init();
		clawSystem.Init();

		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		detectionPipeline = new TeamPropDetectionPipeline();
		detectionPipeline.setDebug(true); // Comment this out to disable debug mode
		camera.setPipeline(detectionPipeline);
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				camera.startStreaming(Constants.Camera.CAMERA_WIDTH, Constants.Camera.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode)
			{
				telemetry.addData("Camera Error", errorCode);
			}
		});

		telemetry.setMsTransmissionInterval(50);
	}

	private void RunBlueShort()
	{
		Actions.runBlocking(clawSystem.MoveToPosition(Constants.getClawBusy()));
		switch (detectionPipeline.getDetectionCase()) {
			case RIGHT:
				Actions.runBlocking(RunSequentially(
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, -centimetersToInches(5), 0))
										.lineToXSplineHeading(-centimetersToInches(70), -Math.PI / 2)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						WaitFor(0.2),
						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(70), -centimetersToInches(5), -Math.PI / 2))
										.splineToSplineHeading(new Pose2d(-centimetersToInches(80), -centimetersToInches(100), Math.PI / 2), Math.PI / 2)
										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
								intakeSystem.RunIntakeFor(1)
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
						RunInParallel(
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.8),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 1)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5, Utilities.DelayDirection.BOTH),
						// Reset positions and Park
						RunInParallel(
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(80), -centimetersToInches(100), Math.PI / 2))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), -centimetersToInches(80)), Math.PI / 2)
										.build()
						)
				));
				break;
			// =========================================================================================================================================
			case CENTER:
				Actions.runBlocking(RunSequentially(
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(60), -centimetersToInches(10)), 0)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						WaitFor(0.2),
						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(60), -centimetersToInches(10), 0))
										.splineToLinearHeading(new Pose2d(-centimetersToInches(65), -centimetersToInches(100), Math.PI / 2), Math.PI / 2)
										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
								intakeSystem.RunIntakeFor(1)
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
						RunInParallel(
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5, Utilities.DelayDirection.BOTH),
						// Reset positions and Park
						RunInParallel(
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(65), -centimetersToInches(100), Math.PI / 2))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), -centimetersToInches(80)), Math.PI / 2)
										.build()
						)
				));
				break;
			// =========================================================================================================================================
			case LEFT:
				Actions.runBlocking(RunSequentially(
								// Place purple
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
												.splineToLinearHeading(new Pose2d(-centimetersToInches(90), -centimetersToInches(60), -Math.PI / 2), Math.PI)
												.build(),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
								),
								clawSystem.ReleaseAtRest(mecanumDrive),
								WaitFor(0.2),
								// Place yellow
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(90), -centimetersToInches(60), -Math.PI / 2))
												.splineToSplineHeading(new Pose2d(-centimetersToInches(40), -centimetersToInches(100), Math.PI / 2), Math.PI / 2)
												.build(),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
										intakeSystem.RunIntakeFor(1)
								),
								clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
								RunInParallel(
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
								),
								clawSystem.ReleaseAtRest(mecanumDrive),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5, Utilities.DelayDirection.BOTH),
								// Reset positions and Park
								RunInParallel(
										rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
										mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(40), -centimetersToInches(100), Math.PI / 2))
												.splineToConstantHeading(new Vector2d(-centimetersToInches(120), -centimetersToInches(80)), Math.PI / 2)
												.build()
								)
						)
				);
				break;
			case NONE:
				telemetry.addLine("Invalid case! (NONE)");
				telemetry.update();
				break;
		}
	}

	private void RunRedShort()
	{
		Actions.runBlocking(clawSystem.MoveToPosition(Constants.getClawBusy()));
		switch (detectionPipeline.getDetectionCase()) {
			case LEFT:
				Actions.runBlocking(RunSequentially(
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.lineToXSplineHeading(-centimetersToInches(70), Math.PI / 2)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						WaitFor(0.2),
						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(70), 0, Math.PI / 2))
										.splineToLinearHeading(new Pose2d(-centimetersToInches(80), centimetersToInches(95), -Math.PI / 2), Math.PI / 2)
										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
								intakeSystem.RunIntakeFor(1)
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
						RunInParallel(
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.8),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 1)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5, Utilities.DelayDirection.BOTH),
						// Reset positions and Park
						RunInParallel(
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(80), centimetersToInches(95), -Math.PI / 2))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(80)), -Math.PI / 2)
										.build()
						)
				));
				break;
			// =========================================================================================================================================
			case CENTER:
				Actions.runBlocking(RunSequentially(
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(60), centimetersToInches(10)), 0)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						WaitFor(0.2),
						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(60), centimetersToInches(10), 0))
										.splineToLinearHeading(new Pose2d(-centimetersToInches(65), centimetersToInches(95), -Math.PI / 2), Math.PI / 2)
										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
								intakeSystem.RunIntakeFor(1)
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
						RunInParallel(
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5, Utilities.DelayDirection.BOTH),
						// Reset positions and Park
						RunInParallel(
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(65), centimetersToInches(95), -Math.PI / 2))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(80)), -Math.PI / 2)
										.build()
						)
				));
				break;
			// =========================================================================================================================================
			case RIGHT:
				Actions.runBlocking(RunSequentially(
								// Place purple
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
												.splineToLinearHeading(new Pose2d(-centimetersToInches(90), centimetersToInches(60), Math.PI / 2), Math.PI)
												.build(),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
								),
								clawSystem.ReleaseAtRest(mecanumDrive),
								WaitFor(0.2),
								// Place yellow
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(90), centimetersToInches(60), Math.PI / 2))
												.splineToLinearHeading(new Pose2d(-centimetersToInches(50), centimetersToInches(95), -Math.PI / 2), Math.PI / 2)
												.build(),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
										intakeSystem.RunIntakeFor(1)
								),
								clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
								RunInParallel(
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
								),
								clawSystem.ReleaseAtRest(mecanumDrive),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5, Utilities.DelayDirection.BOTH),
								// Reset positions and Park
								RunInParallel(
										rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
										mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(50), centimetersToInches(95), -Math.PI / 2))
												.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(80)), -Math.PI / 2)
												.build()
								)
						)
				);
				break;
			case NONE:
				telemetry.addLine("Invalid case! (NONE)");
				telemetry.update();
				break;
		}
	}

	public void RunRedLong()
	{
		Actions.runBlocking(clawSystem.MoveToPosition(Constants.getClawBusy()));
		switch (detectionPipeline.getDetectionCase()) {
			case RIGHT:
				Actions.runBlocking(RunSequentially(
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.lineToXSplineHeading(-centimetersToInches(70), Math.PI / 2)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						WaitFor(0.2),
						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(70), centimetersToInches(5), Math.PI / 2))
										.splineToSplineHeading(new Pose2d(-centimetersToInches(80), centimetersToInches(100), -Math.PI / 2), Math.PI / 2)
										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
								intakeSystem.RunIntakeFor(1)
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
						RunInParallel(
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.8),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 1)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5, Utilities.DelayDirection.BOTH),
						// Reset positions and Park
						RunInParallel(
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(80), centimetersToInches(100), -Math.PI / 2))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(80)), -Math.PI / 2)
										.build()
						)
				));
				break;
			// =========================================================================================================================================
			case CENTER:
				Actions.runBlocking(RunSequentially(
						// Place purple
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(60), centimetersToInches(10)), 0)
										.build(),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						WaitFor(0.2),
						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(60), centimetersToInches(10), 0))
										.splineToLinearHeading(new Pose2d(-centimetersToInches(65), centimetersToInches(100), -Math.PI / 2), Math.PI / 2)
										.build(),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
								intakeSystem.RunIntakeFor(1)
						),
						clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
						RunInParallel(
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
						),
						clawSystem.ReleaseAtRest(mecanumDrive),
						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5, Utilities.DelayDirection.BOTH),
						// Reset positions and Park
						RunInParallel(
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(65), centimetersToInches(100), -Math.PI / 2))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(80)), -Math.PI / 2)
										.build()
						)
				));
				break;
			// =========================================================================================================================================
			case LEFT:
				Actions.runBlocking(RunSequentially(
								// Place purple
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(0, 0, 0))
												.splineToLinearHeading(new Pose2d(-centimetersToInches(90), -centimetersToInches(60), -Math.PI / 2), Math.PI)
												.build(),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
								),
								clawSystem.ReleaseAtRest(mecanumDrive),
								WaitFor(0.2),
								// Place yellow
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(90), -centimetersToInches(60), -Math.PI / 2))
												.splineToSplineHeading(new Pose2d(-centimetersToInches(40), centimetersToInches(100), -Math.PI / 2), Math.PI / 2)
												.build(),
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
										intakeSystem.RunIntakeFor(1)
								),
								clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
								RunInParallel(
										rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
								),
								clawSystem.ReleaseAtRest(mecanumDrive),
								tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5, Utilities.DelayDirection.BOTH),
								// Reset positions and Park
								RunInParallel(
										rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
										tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.5),
										mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(40), centimetersToInches(100), -Math.PI / 2))
												.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(80)), -Math.PI / 2)
												.build()
								)
						)
				);
				break;
			case NONE:
				telemetry.addLine("Invalid case! (NONE)");
				telemetry.update();
				break;
		}
	}

	public void RunBlueLong()
	{

	}

	private void Run()
	{
		switch (currentAlliance) {
			case RED:
				switch (currentPath) {
					case SHORT:
						RunRedShort();
						break;
					case LONG:
						RunRedLong();
						break;
				}
				break;
			case BLUE:
				switch (currentPath) {
					case SHORT:
						RunBlueShort();
						break;
					case LONG:
						RunBlueLong();
						break;
				}
				break;
		}
	}
}