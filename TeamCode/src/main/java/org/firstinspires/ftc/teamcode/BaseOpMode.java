package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseOpMode extends LinearOpMode
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		// Initialize the robot
		OnInitialize();

		// Wait for the start button to be pressed
		waitForStart();

		// Run the robot
		while (!isStopRequested())
			OnRun();

		// Stop the robot
		telemetry.clearAll();
		telemetry.addData("Status", "Stopping...");
		telemetry.update();
		OnStop();
	}

	protected abstract void OnInitialize();

	protected abstract void OnRun();

	protected void OnStop() {
		// Override this method to add custom stop behavior
	}
}
