package org.firstinspires.ftc.teamcode.testing;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.opencv.VideoWriterPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Media Recorder Test", group = "Testing")
public class MediaRecorderTest extends BaseOpMode
{
	private OpenCvCamera camera;
	private InputSystem inputSystem;
	private VideoWriterPipeline videoWriterPipeline;

	private static InputSystem.Key START_RECORDING = new InputSystem.Key("a");
	private static InputSystem.Key STOP_RECORDING = new InputSystem.Key("b");

	@Override
	protected void OnInitialize()
	{
		inputSystem = new InputSystem(gamepad1);
		videoWriterPipeline = new VideoWriterPipeline(Environment.getExternalStorageDirectory().getAbsolutePath() + "/video.mp4");
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		camera.setPipeline(videoWriterPipeline);
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
				telemetry.addData("[ERROR] Camera failed to open with error code", errorCode);
				telemetry.update();
			}
		});
	}

	@Override
	protected void OnRun()
	{
		if (inputSystem.wasPressedThisFrame(START_RECORDING) && !videoWriterPipeline.isRecording()) {
			videoWriterPipeline.startRecording();
			telemetry.addData("Recording?", "Started???????????");
		} else if (inputSystem.wasPressedThisFrame(STOP_RECORDING) && videoWriterPipeline.isRecording()) {
			videoWriterPipeline.stopRecording();
			telemetry.addData("Recording?", "Stopped???????????");
		}

		telemetry.addData("Recording", videoWriterPipeline.isRecording());
		telemetry.update();
	}
}