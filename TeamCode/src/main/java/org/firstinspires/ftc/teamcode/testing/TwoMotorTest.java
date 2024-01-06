package org.firstinspires.ftc.teamcode.testing;

import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BaseOpMode;

/*
 * Test two motors by controlling them with the right stick
 */

@Photon
@TeleOp(name = "Two Motor Test", group = "Testing")
public class TwoMotorTest extends BaseOpMode
{
	PhotonDcMotor m1, m2;

	@Override
	protected void OnInitialize()
	{
		m1 = hardwareMap.get(PhotonDcMotor.class, "slot0");
		m2 = hardwareMap.get(PhotonDcMotor.class, "slot1");
		m1.setMode(PhotonDcMotor.RunMode.RUN_WITHOUT_ENCODER);
		m2.setMode(PhotonDcMotor.RunMode.RUN_WITHOUT_ENCODER);
		m1.setZeroPowerBehavior(PhotonDcMotor.ZeroPowerBehavior.BRAKE);
		m2.setZeroPowerBehavior(PhotonDcMotor.ZeroPowerBehavior.BRAKE);
	}

	@Override
	protected void OnRun()
	{
		m1.setPower(gamepad1.right_stick_y);
		m2.setPower(gamepad1.right_stick_y);

		Telemetry();
	}

	private void Telemetry()
	{
		telemetry.addData("Motor 1 Power: ", m1.getPower());
		telemetry.addData("Motor 2 Power: ", m2.getPower());
		telemetry.update();
	}
}