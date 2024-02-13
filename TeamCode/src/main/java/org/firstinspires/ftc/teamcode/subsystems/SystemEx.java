package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.Utilities;

public abstract class SystemEx
{
	protected boolean internal_Enabled = true;
	protected boolean internal_Initialized = false;
	public abstract void Setup();
	public abstract void Init();
	public abstract void Disable();
	public abstract void Enable();
	public Action MoveToPosition(double position) { return MoveToPositionWithDelay(position, 0); }
	public abstract Action MoveToPositionWithDelay(double position, double delay, Utilities.DelayDirection delayDirection);
	public Action MoveToPositionWithDelay(double position, double delay) { return MoveToPositionWithDelay(position, delay, Utilities.DelayDirection.BEFORE); }
}