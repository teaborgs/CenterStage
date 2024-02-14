package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class BaseOpMode extends LinearOpMode
{
	private boolean internal_IsAutonomous = false;

	private ElapsedTime internal_ElapsedTimeSinceInit;
	private ElapsedTime internal_ElapsedTimeSinceStart;

	@Override
	public void runOpMode()
	{
		internal_ElapsedTimeSinceInit = new ElapsedTime();
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
		internal_ElapsedTimeSinceStart = new ElapsedTime();
		OnStart();
		while (!isStopRequested()) {
			OnRun();
			if(internal_IsAutonomous) break;
		}
		OnStop();

		// Stop the robot
		telemetry.clear();
		telemetry.addLine("[STATUS] Stopping...");
		telemetry.update();
	}

	protected abstract void OnInitialize();

	protected abstract void OnRun();

	protected void WhileWaitingForStart() {
		// Override this method to add custom behavior while waiting for start
	}

	protected void OnStart() {
		// Override this method to add custom start behavior
	}

	protected void OnStop() {
		// Override this method to add custom stop behavior
	}

	protected void setAutonomous(boolean internal_IsAutonomous)
	{
		this.internal_IsAutonomous = internal_IsAutonomous;
	}

	protected double getElapsedTimeSinceInit()
	{
		if(internal_ElapsedTimeSinceInit == null) return -1;
		return internal_ElapsedTimeSinceInit.seconds();
	}

	protected double getElapsedTimeSinceStart()
	{
		if(internal_ElapsedTimeSinceStart == null) return -1;
		return internal_ElapsedTimeSinceStart.seconds();
	}
}
