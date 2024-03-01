package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.impl.ClawSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.DroneSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.LiftSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.RotatorSystem;
import org.firstinspires.ftc.teamcode.subsystems.impl.TumblerSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RobotHardware
{
	public final MecanumDrive mecanumDrive;

	public final DroneSystem droneSystem;
	public final IntakeSystem intakeSystem;
	public final ClawSystem clawSystem1;
	public final ClawSystem clawSystem2;
	public final RotatorSystem rotatorSystem;
	public final TumblerSystem tumblerSystem;
	public final LiftSystem liftSystem;

	public final Rev2mDistanceSensor distanceSensor;
	public OpenCvCamera camera = null;

	private final HardwareMap hardwareMap;
	private Telemetry telemetry;

	public RobotHardware(HardwareMap hardwareMap, boolean noLogic)
	{
		this.hardwareMap = hardwareMap;
		mecanumDrive = new MecanumDrive(hardwareMap);
		distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

		droneSystem = new DroneSystem(hardwareMap.get(Servo.class, "leveler"), hardwareMap.get(Servo.class, "shooter"));
		intakeSystem = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"), hardwareMap.get(Servo.class, "antenna"));
		clawSystem1 = new ClawSystem(hardwareMap.get(Servo.class, "claw1"));
		clawSystem2 = new ClawSystem(hardwareMap.get(Servo.class, "claw2"));
		rotatorSystem = new RotatorSystem(hardwareMap.get(Servo.class, "rotator"));
		tumblerSystem = new TumblerSystem(hardwareMap.get(Servo.class, "tumbler"));
		liftSystem = new LiftSystem(hardwareMap.get(DcMotorEx.class, "lift1"), hardwareMap.get(DcMotorEx.class, "lift2"));

		try {
			int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
			camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		} catch (Exception e) {
			// We catch the exception because we don't want to crash the robot if the camera is not available
		}

		if (noLogic)
		{
			droneSystem.Setup();
			intakeSystem.Setup();
			clawSystem1.Setup();
			clawSystem2.Setup();
			rotatorSystem.Setup();
			tumblerSystem.Setup();
			liftSystem.Setup();
			return;
		}

		droneSystem.Init();
		intakeSystem.Init();
		clawSystem1.Init();
		clawSystem2.Init();
		rotatorSystem.Init();
		tumblerSystem.Init();
		liftSystem.Init();
	}

	public RobotHardware(HardwareMap hardwareMap) { this(hardwareMap, false); }

	public void setTelemetry(Telemetry telemetry)
	{
		this.telemetry = telemetry;
	}

	public void setCameraPipeline(OpenCvPipeline pipeline)
	{
		camera.setPipeline(pipeline);
	}

	public void openCameraAsync()
	{
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				camera.startStreaming(Constants.Camera.CAMERA_WIDTH, Constants.Camera.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode)
			{
				if(telemetry != null) {
					telemetry.addLine("[ERROR] Camera failed to open with error code " + errorCode);
					telemetry.update();
				}
			}
		});
	}

	public void closeCamera()
	{
		camera.stopStreaming();
		camera.closeCameraDevice();
	}

	/**
	 * Test the distance sensor
	 * @return Whether the distance sensor is working
	 */
	public boolean testDistanceSensor()
	{
		// Test reading the distance sensor
		distanceSensor.getDistance(DistanceUnit.CM);
		return !distanceSensor.didTimeoutOccur();
	}

	public VoltageSensor getVoltageSensor()
	{
		return hardwareMap.voltageSensor.iterator().next();
	}
}
