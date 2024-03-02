package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Globals;
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
		robotHardware = new RobotHardware(hardwareMap);

		robotHardware.clawSystem1.SetPosition(Constants.getClawBusy());
		robotHardware.clawSystem2.SetPosition(Constants.getClawBusy());
		robotHardware.rotatorSystem.SetPosition(Constants.getRotatorBusy(), 1.2);
		robotHardware.tumblerSystem.SetPosition(Constants.getTumblerBackdrop(), 1);
	}

	@Override
	protected void OnRun()
	{
		telemetry.addData("[DEBUG] Robot X", robotHardware.mecanumDrive.pose.position.x);
		telemetry.addData("[DEBUG] Robot Y", robotHardware.mecanumDrive.pose.position.y);
		telemetry.addData("[DEBUG] Robot Heading", robotHardware.mecanumDrive.pose.heading.toDouble());
		telemetry.addData("[DEBUG] Lift Position", robotHardware.liftSystem.GetCurrentPosition());
		telemetry.update();
	}
}