package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseOpMode extends LinearOpMode
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		telemetry.addLine("[STATUS] Initializing...");
		telemetry.update();

		// Initialize the robot
		OnInitialize();

		// Wait for the start button to be pressed
		waitForStart();

		// Run the robot
		while (!isStopRequested())
			OnRun();

		// Stop the robot
		telemetry.clearAll();
		telemetry.addLine("[STATUS] Stopping...");
		telemetry.update();
		OnStop();
	}

	protected abstract void OnInitialize();

	protected abstract void OnRun();

	protected void OnStop() {
		// Override this method to add custom stop behavior
	}
}
