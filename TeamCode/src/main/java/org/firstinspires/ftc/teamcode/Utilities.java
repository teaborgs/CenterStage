package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Utilities
{
	public static void setTimeout(Runnable runnable, int delay)
	{
		new Thread(() -> {
			try {
				Thread.sleep(delay);
				runnable.run();
			} catch (Exception e) {
				System.err.println(e);
			}
		}).start();
	}

	public static void CutPower(DcMotorEx... devices)
	{
		for (DcMotorEx device : devices) device.setMotorDisable();
	}

	public static void RestorePower(DcMotorEx... devices)
	{
		for (DcMotorEx device : devices) device.setMotorEnable();
	}

	public enum State {
		IDLE,
		BUSY
	}

	public enum PickupMode {
		LOAD,
		STACK
	}

	public enum DetectionCase
	{
		NONE, LEFT, CENTER, RIGHT
	}
}
