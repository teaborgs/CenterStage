package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.Utilities;

public abstract class SystemEx
{
	protected boolean internal_Enabled = true;
	protected boolean internal_Initialized = false;

	/**
	 * Sets up the system, this is called before the system is initialized
	 */
	public abstract void Setup();
	/**
	 * Initializes the system and sets it to a known start state
	 */
	public abstract void Init();
	/**
	 * Disables the system
	 */
	public abstract void Disable();
	/**
	 * Enables the system
	 */
	public abstract void Enable();
	public Action MoveToPosition(double position) { return MoveToPositionWithDelay(position, 0); }
	public abstract Action MoveToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection);
	public Action MoveToPositionWithDelay(double position, double delay) { return MoveToPositionWithDelay(position, delay, Utilities.DelayDirection.BEFORE); }
}