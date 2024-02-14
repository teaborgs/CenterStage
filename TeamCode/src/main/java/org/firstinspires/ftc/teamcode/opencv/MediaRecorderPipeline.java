package org.firstinspires.ftc.teamcode.opencv;

import static org.firstinspires.ftc.teamcode.Constants.Camera.CAMERA_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.Camera.CAMERA_WIDTH;

import android.annotation.SuppressLint;
import android.media.MediaCodec;
import android.media.MediaExtractor;
import android.media.MediaFormat;
import android.media.MediaMuxer;
import android.media.MediaRecorder;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.nio.ByteBuffer;
import java.util.concurrent.TimeUnit;

public class MediaRecorderPipeline extends OpenCvPipeline
{
	private final VideoWriter videoWriter;

	private final String filename;
	private MediaRecorder mediaRecorder;
	private ElapsedTime elapsedTime;

	private boolean isVoiceRecording = false;

	public MediaRecorderPipeline(String filename)
	{
		this.filename = filename;
		videoWriter = new VideoWriter();
	}

	private final Mat converted = new Mat();

	@Override
	public Mat processFrame(Mat input)
	{
		if (!videoWriter.isOpened())
			return input;

		converted.release();
		Imgproc.cvtColor(input, converted, Imgproc.COLOR_RGBA2BGR);
		videoWriter.write(converted);
		return input;
	}

	public void startRecording()
	{
		new Thread(() -> {
			try {
				mediaRecorder = new MediaRecorder();
				mediaRecorder.setAudioSource(MediaRecorder.AudioSource.CAMCORDER);
				mediaRecorder.setOutputFormat(MediaRecorder.OutputFormat.THREE_GPP);
				mediaRecorder.setAudioEncoder(MediaRecorder.AudioEncoder.AMR_NB);
				mediaRecorder.setMaxDuration(-1);
				mediaRecorder.setMaxFileSize(-1);
				mediaRecorder.setOutputFile(filename.replace(".mp4", ".3gp"));

				mediaRecorder.setOnInfoListener((mr, what, extra) -> {
					System.out.println("MediaRecorder info: " + what + ", " + extra);
				});

				mediaRecorder.setOnErrorListener((mr, what, extra) -> {
					System.out.println("MediaRecorder error: " + what + ", " + extra);
				});

				isVoiceRecording = false;

				try {
					mediaRecorder.prepare();
				} catch (Exception e) {
					System.out.println("Failed to initialize voice recorder");
				}

				Thread.sleep(1000);

				try {
					mediaRecorder.start();
					isVoiceRecording = true;
				} catch (Exception e) {
					System.out.println("Failed to start voice recording: " + e.getMessage());
					e.printStackTrace();
				}

				if (!videoWriter.open(filename, VideoWriter.fourcc('H', '2', '6', '4'), 15, new Size(CAMERA_WIDTH, CAMERA_HEIGHT), true))
					System.out.println("Failed to open video writer");

				elapsedTime = new ElapsedTime();
			} catch (Exception e) {
				System.out.println("Failed to start recording: " + e.getMessage());
				e.printStackTrace();
			}
		}).start();
	}

	public void stopRecording()
	{
		new Thread(() -> {
			boolean voiceOk = true;
			boolean videoOk = true;

			if (videoWriter.isOpened()) videoWriter.release();
			else videoOk = false;

			try {
				if (isVoiceRecording) {
					mediaRecorder.stop();
					mediaRecorder.release();
					isVoiceRecording = false;
				} else voiceOk = false;
			} catch (Exception e) {
				voiceOk = false;
				System.out.println("Failed to stop recording: " + e.getMessage());
				e.printStackTrace();
			}

			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				throw new RuntimeException(e);
			}

			if (videoOk)
				mux(filename, filename.replace(".mp4", ".3gp"), filename.replace(".mp4", "_final.mp4"), !voiceOk);
			else System.out.println("Failed to mux video. Are you sure the video file exists?");
		}).start();
	}

	@SuppressLint("WrongConstant")
	private void mux(String videoFile, String audioFile, String outputFile, boolean videoOnly)
	{
		try {
			System.out.println("Muxing " + videoFile + " and " + audioFile + " to " + outputFile);
			MediaExtractor videoExtractor = new MediaExtractor();
			videoExtractor.setDataSource(videoFile);

			MediaExtractor audioExtractor = null;

			if (!videoOnly) {
				audioExtractor = new MediaExtractor();
				audioExtractor.setDataSource(audioFile);
			}

			MediaMuxer muxer = new MediaMuxer(outputFile, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4);

			int videoTrackIndex = -1;
			int audioTrackIndex = -1;

			int originalVideoTrackIndex = -1;
			int originalAudioTrackIndex = -1;

			for (int i = 0; i < videoExtractor.getTrackCount(); i++) {
				MediaFormat format = videoExtractor.getTrackFormat(i);
				String mime = format.getString(MediaFormat.KEY_MIME);
				if (mime == null) continue;
				if (mime.startsWith("video/")) {
					System.out.println("Video format: " + format);
					originalVideoTrackIndex = i;
					videoTrackIndex = muxer.addTrack(format);
				}
			}

			if (!videoOnly) {
				for (int i = 0; i < audioExtractor.getTrackCount(); i++) {
					MediaFormat format = audioExtractor.getTrackFormat(i);
					String mime = format.getString(MediaFormat.KEY_MIME);
					if (mime == null) continue;
					if (mime.startsWith("audio/")) {
						System.out.println("Audio format: " + format);
						originalAudioTrackIndex = i;
						audioTrackIndex = muxer.addTrack(format);
					}
				}
			}

			muxer.start();

			ByteBuffer videoBuffer = ByteBuffer.allocate(1024 * 1024);
			ByteBuffer audioBuffer = ByteBuffer.allocate(1024 * 1024);

			videoExtractor.selectTrack(originalVideoTrackIndex);
			MediaCodec.BufferInfo videoBufferInfo = new MediaCodec.BufferInfo();
			while (true) {
				int videoSampleSize = videoExtractor.readSampleData(videoBuffer, 0);
				if (videoSampleSize < 0) break;
				videoBufferInfo.offset = 0;
				videoBufferInfo.size = videoSampleSize;
				videoBufferInfo.presentationTimeUs = videoExtractor.getSampleTime();
				videoBufferInfo.flags = videoExtractor.getSampleFlags();
				muxer.writeSampleData(videoTrackIndex, videoBuffer, videoBufferInfo);
				videoExtractor.advance();
			}

			if (!videoOnly) {
				audioExtractor.selectTrack(originalAudioTrackIndex);
				MediaCodec.BufferInfo audioBufferInfo = new MediaCodec.BufferInfo();
				while (true) {
					int audioSampleSize = audioExtractor.readSampleData(audioBuffer, 0);
					if (audioSampleSize < 0) break;
					audioBufferInfo.offset = 0;
					audioBufferInfo.size = audioSampleSize;
					audioBufferInfo.presentationTimeUs = audioExtractor.getSampleTime();
					audioBufferInfo.flags = audioExtractor.getSampleFlags();
					muxer.writeSampleData(audioTrackIndex, audioBuffer, audioBufferInfo);
					audioExtractor.advance();
				}
			}

			muxer.stop();
			muxer.release();
			videoExtractor.release();
			if (!videoOnly) audioExtractor.release();
			System.out.println("Mux succeeded!");
		} catch (Exception e) {
			System.out.println("Mux failed. Are you sure the video and audio files exist?");
			e.printStackTrace();
		}

		cleanArtifacts();
	}

	private void cleanArtifacts()
	{
		try {
			File audioFile = new File(filename.replace(".mp4", ".3gp"));
			if (audioFile.exists()) {
				if (!audioFile.delete()) System.out.println("Failed to delete audio file");
				else System.out.println("Deleted audio file");
			} else System.out.println("Audio file does not exist");
			File videoFile = new File(filename);
			if (videoFile.exists()) {
				if (!videoFile.delete()) System.out.println("Failed to delete video file");
				else System.out.println("Deleted video file");
			} else System.out.println("Video file does not exist");
		} catch (Exception e) {
			System.out.println("Fatal error: failed to clean artifacts");
			e.printStackTrace();
		}
	}

	public boolean isRecording()
	{
		return videoWriter.isOpened();
	}

	public boolean isVoiceRecording()
	{
		return isVoiceRecording;
	}

	public double getRecordTime()
	{
		return elapsedTime == null ? 0 : elapsedTime.time(TimeUnit.SECONDS);
	}
}