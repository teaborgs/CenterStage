package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.GetCurrentRobotType;
import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
		Constants.Init(Utilities.GetCurrentRobotType(hardwareMap));

		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

		liftSystem = new LiftSystem(hardwareMap.get(DcMotorEx.class, "lift1"), hardwareMap.get(DcMotorEx.class, "lift2"));
		tumblerSystem = new TumblerSystem(hardwareMap.get(DcMotorEx.class, "tumbler"));
		intakeSystem = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"));
		rotatorSystem = new RotatorSystem(hardwareMap.get(Servo.class, "rotator"));
		clawSystem = new ClawSystem(hardwareMap.get(Servo.class, "claw"));

		tumblerSystem.setRobotType(GetCurrentRobotType(hardwareMap));

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

	private void Run()
	{
//		double desiredHeading = detectionCase == Utilities.DetectionCase.LEFT ? Math.PI / 2 : detectionCase == Utilities.DetectionCase.RIGHT ? -Math.PI / 2 : 0;
//		int offset = detectionCase == Utilities.DetectionCase.LEFT ? 10 : detectionCase == Utilities.DetectionCase.RIGHT ? -30 : -10;
//		Pose2d destination = new Pose2d(-centimetersToInches(70), 0, 0);
//		TrajectoryActionBuilder spikeMarkTrajectory = mecanumDrive.actionBuilder(mecanumDrive.pose).lineToXSplineHeading(-centimetersToInches(70), desiredHeading);
//		if (detectionCase != Utilities.DetectionCase.LEFT) {
//			destination = new Pose2d(-centimetersToInches(detectionCase == Utilities.DetectionCase.RIGHT ? 70 : 50), -centimetersToInches(20), 0);
//			spikeMarkTrajectory = mecanumDrive.actionBuilder(mecanumDrive.pose).setTangent(Math.PI).splineToSplineHeading(destination, desiredHeading);
//		}
//		Actions.runBlocking(RunSequentially(
//				// Place purple
//				RunInParallel(
//						clawSystem.MoveToPosition(Constants.getClawBusy()),
//						spikeMarkTrajectory.build(),
//						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerSpikeMark(), 0.5),
//						rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.75)
//				),
//				clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.2, Utilities.DelayDirection.BOTH),
//
//				// Place yellow
//				RunInParallel(
//						mecanumDrive.actionBuilder(destination)
//								.splineToSplineHeading(new Pose2d(-centimetersToInches(70 + offset), centimetersToInches(100), -Math.PI / 2), Math.PI / 2)
//								.build(),
//						rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.5),
//						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.8),
//						intakeSystem.RunIntakeFor(1)
//				),
//				clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.1, Utilities.DelayDirection.BOTH),
//				RunInParallel(
//						rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5),
//						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerBackdrop(), 0.8)
//				),
//				clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.1, Utilities.DelayDirection.BOTH),
//
//				// Reset positions and Park
//				RunInParallel(
//						rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
//						tumblerSystem.MoveToPositionWithDelay(Constants.getTumblerLoad(), 0.2),
//						mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(70 + offset), centimetersToInches(100), -Math.PI / 2))
//								.splineToConstantHeading(new Vector2d(-centimetersToInches(100), centimetersToInches(70)), -Math.PI / 2)
//								.build()
//				)
//		));


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
						clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.1, Utilities.DelayDirection.BOTH),

						// Place yellow
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(70), 0, Math.PI / 2))
										.splineToLinearHeading(new Pose2d(-centimetersToInches(80), centimetersToInches(100), -Math.PI / 2), Math.PI / 2)
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
						clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.15, Utilities.DelayDirection.BOTH),
						tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
						WaitFor(0.3),

						// Reset positions and Park
						RunInParallel(
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(80), centimetersToInches(100), -Math.PI / 2))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(70)), -Math.PI / 2)
										.build()
						)
				));
				break;


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
						clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.1, Utilities.DelayDirection.BOTH),

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
						clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.15, Utilities.DelayDirection.BOTH),
						tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
						WaitFor(0.3),

						// Reset positions and Park
						RunInParallel(
								rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(65), centimetersToInches(100), -Math.PI / 2))
										.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(70)), -Math.PI / 2)
										.build()
						)
				));
				break;

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
								clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.1, Utilities.DelayDirection.BOTH),

								// Place yellow
								RunInParallel(
										mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(90), centimetersToInches(60), Math.PI/2))
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
								clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.25, Utilities.DelayDirection.BOTH),
								tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
								WaitFor(0.35),

								// Reset positions and Park
								RunInParallel(
										rotatorSystem.MoveToPosition(Constants.getRotatorIdle()),
										mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(50), centimetersToInches(95), -Math.PI / 2))
												.splineToConstantHeading(new Vector2d(-centimetersToInches(120), centimetersToInches(70)), -Math.PI / 2)
												.build()
								)
						)
				);

				break;

			case NONE:
				telemetry.addLine("Invalid case! (NONE)");
				telemetry.update();
		}
	}
}