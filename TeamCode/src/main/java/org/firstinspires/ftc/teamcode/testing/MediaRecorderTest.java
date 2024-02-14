package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.Utilities.MakeVideoFile;

import android.Manifest;
import android.content.pm.PackageManager;
import android.media.AudioDeviceInfo;
import android.media.AudioManager;
import android.os.Environment;

import androidx.core.content.ContextCompat;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opencv.MediaRecorderPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

@TeleOp(name = "Media Recorder Test", group = "Testing")
public class MediaRecorderTest extends BaseOpMode
{
	private OpenCvCamera camera;
	private MediaRecorderPipeline mediaRecorderPipeline;

	@Override
	protected void OnInitialize()
	{
		mediaRecorderPipeline = new MediaRecorderPipeline(MakeVideoFile("tests"));
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		camera.setPipeline(mediaRecorderPipeline);
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
		telemetry.addData("Recording", mediaRecorderPipeline.isRecording());
		telemetry.addData("Voice Recording", mediaRecorderPipeline.isVoiceRecording());
		telemetry.addData("Recording Time", mediaRecorderPipeline.getRecordTime());
		telemetry.addData("FPS", camera.getFps());
		telemetry.update();
	}

	@Override
	protected void OnStart()
	{
		if (ContextCompat.checkSelfPermission(hardwareMap.appContext, Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED)
			System.out.println("Microphone permission not granted");
		mediaRecorderPipeline.startRecording();
	}

	@Override
	protected void OnStop()
	{
		mediaRecorderPipeline.stopRecording();
	}
}