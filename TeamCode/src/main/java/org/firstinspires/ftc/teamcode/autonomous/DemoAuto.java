package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.Utilities.GetCurrentRobotType;
import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.centimetersToInches;
import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.subsystems.impl.ClawSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.LiftSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.RotatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.TumblerSystem;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Demo Auto", group = "Auto")
public class DemoAuto extends LinearOpMode
{
	private MecanumDrive mecanumDrive;
	private TumblerSystem tumblerSystem;
	private RotatorSystem rotatorSystem;
	private ClawSystem clawSystem;
	private IntakeSystem intakeSystem;
	private LiftSystem liftSystem;

	@Override
	public void runOpMode()
	{
		Init();
		waitForStart();
		Run();
	}

	private void Init()
	{
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

		telemetry.setMsTransmissionInterval(50);
	}

	private void Run()
	{
		Actions.runBlocking(
				RunSequentially(
						clawSystem.MoveToPosition(Constants.getClawBusy()),
						WaitFor(0.5),
						RunInParallel(
								mecanumDrive.actionBuilder(mecanumDrive.pose)
										.lineToX(-centimetersToInches(60))
										.build(),
								tumblerSystem.MoveToPosition(Constants.getTumblerSpikeMark()),
								rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.1)
						),
						clawSystem.MoveToPosition(Constants.getClawIdle()),
						rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorIdle(), 0.1),
						tumblerSystem.MoveToPosition(Constants.getTumblerLoad() + 150),
						RunInParallel(
								mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(60), 0, 0))
										.setTangent(Math.PI / 2)
										.lineToYSplineHeading(centimetersToInches(100), -Math.PI / 2)
										.build(),
								RunSequentially(
										intakeSystem.RunIntakeFor(1.5),
										tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
										clawSystem.MoveToPositionWithDelay(Constants.getClawBusy(), 0.5, Utilities.DelayDirection.BOTH),
										RunInParallel(
												RunInParallel(
														tumblerSystem.MoveToPosition(Constants.getTumblerBackdrop()),
														rotatorSystem.MoveToPositionWithDelay(Constants.getRotatorBusy(), 0.5)
												),
												liftSystem.MoveToPosition(Constants.getLiftLevel1())
										)
								)
						),
						WaitFor(0.5),
						clawSystem.MoveToPositionWithDelay(Constants.getClawIdle(), 0.5, Utilities.DelayDirection.BOTH),
						tumblerSystem.MoveToPosition(Constants.getTumblerLoad()),
						mecanumDrive.actionBuilder(new Pose2d(-centimetersToInches(60), centimetersToInches(100), -Math.PI / 2))
								.setTangent(0)
								.lineToX(centimetersToInches(5))
								.build()
				)
		);
	}
}