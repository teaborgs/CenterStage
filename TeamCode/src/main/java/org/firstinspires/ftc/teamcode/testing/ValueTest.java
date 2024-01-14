package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

/*
 * Test the encoder value of a motor
 */
@TeleOp(name = "Value Test", group = "Testing")
public class ValueTest extends BaseOpMode
{
	private RobotHardware robotHardware;

	@Override
	protected void OnInitialize()
	{
		Globals.ValidateConfig(hardwareMap, telemetry, gamepad1, gamepad2);
		Constants.Init();
		robotHardware = new RobotHardware(hardwareMap, true);
		telemetry.setMsTransmissionInterval(50);
	}

	@Override
	protected void OnRun()
	{
		telemetry.addData("[DEBUG] Current Robot", Globals.GetCurrentRobotType().name());
		telemetry.addData("[DEBUG] Robot X", robotHardware.mecanumDrive.pose.position.x);
		telemetry.addData("[DEBUG] Robot Y", robotHardware.mecanumDrive.pose.position.y);
		telemetry.addData("[DEBUG] Robot Heading", robotHardware.mecanumDrive.pose.heading.toDouble());
		telemetry.addData("[DEBUG] Lift 1", robotHardware.liftMotor1.getCurrentPosition());
		telemetry.addData("[DEBUG] Lift 2", robotHardware.liftMotor2.getCurrentPosition());
		telemetry.addData("[DEBUG] Tumbler", robotHardware.tumblerMotor.getCurrentPosition());
		telemetry.addData("[DEBUG] Rotator", robotHardware.rotatorServo.getPosition());
		telemetry.addData("[DEBUG] Claw", robotHardware.clawServo.getPosition());
		telemetry.addData("[DEBUG] Antenna", robotHardware.antennaServo.getPosition());
		telemetry.addData("[DEBUG] Plane Level", robotHardware.planeLevelServo.getPosition());
		telemetry.addData("[DEBUG] Plane Release", robotHardware.planeShooterServo.getPosition());
		telemetry.update();
	}
}
