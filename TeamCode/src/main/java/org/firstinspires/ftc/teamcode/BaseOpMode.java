package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseOpMode extends LinearOpMode
{
	private boolean internal_IsAutonomous = false;

	@Override
	public void runOpMode()
	{
		this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		telemetry.setMsTransmissionInterval(50);
		telemetry.addLine("[STATUS] Initializing...");
		telemetry.update();

		// Initialize the robot
		OnInitialize();

		// Wait for the start button to be pressed
		while (!isStarted() && !isStopRequested())
			WhileWaitingForStart();

		// Run the robot
		while (!isStopRequested()) {
			OnRun();
			if(internal_IsAutonomous) break;
		}

		// Stop the robot
		telemetry.clear();
		telemetry.addLine("[STATUS] Stopping...");
		telemetry.update();
		OnStop();
	}

	protected abstract void OnInitialize();

	protected abstract void OnRun();

	protected void WhileWaitingForStart() {
		// Override this method to add custom behavior while waiting for start
	}

	protected void OnStop() {
		// Override this method to add custom stop behavior
	}

	public void setAutonomous(boolean internal_IsAutonomous)
	{
		this.internal_IsAutonomous = internal_IsAutonomous;
	}
}
