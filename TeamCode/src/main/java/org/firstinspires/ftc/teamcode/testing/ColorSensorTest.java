package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.BaseOpMode;

@TeleOp(name = "Color Sensor Test", group = "Testing")
public class ColorSensorTest extends BaseOpMode
{
	private ColorSensor colorSensor;
	@Override
	protected void OnInitialize()
	{
		colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
	}

	@Override
	protected void OnRun()
	{
		telemetry.addData("Red", colorSensor.red());
		telemetry.addData("Green", colorSensor.green());
		telemetry.addData("Blue", colorSensor.blue());
		telemetry.addData("Alpha", colorSensor.alpha());
		float average = colorSensor.red() + colorSensor.green() + colorSensor.blue() + colorSensor.alpha();
		telemetry.addData("Average", average / 4);
		telemetry.update();
	}
}
