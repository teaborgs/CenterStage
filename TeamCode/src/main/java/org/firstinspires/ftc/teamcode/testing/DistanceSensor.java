package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BaseOpMode;

@TeleOp(name = "DistanceSensorTest", group = "Testing")
public class DistanceSensor extends BaseOpMode
{
	private Rev2mDistanceSensor distanceSensor;

	@Override
	protected void OnInitialize()
	{
		distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

		distanceSensor.getDistance(DistanceUnit.CM);
		if (distanceSensor.didTimeoutOccur())
		{
			telemetry.clear();
			telemetry.addLine("Distance sensor fault!");
			telemetry.update();
			while (!isStopRequested());
		}
	}

	@Override
	protected void OnRun()
	{
		telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
		telemetry.update();
	}
}