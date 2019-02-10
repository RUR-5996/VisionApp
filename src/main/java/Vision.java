import java.awt.image.ImageProducer;
import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
	public static void main(String[] args)  {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		MjpegServer inputStream = new MjpegServer("MJPEG Server", 1185);
		HttpCamera camera = new HttpCamera("Camera", /*"http://10.59.96.11/video/stream.mjpg"*/ "http://roborio-5996-frc.local:1181/?action=stream");
		//inputStream.setSource(camera);
		CvSink imageSink = new CvSink("Image Grabber");
		imageSink.setSource(camera);
		CvSource imageSource = new CvSource("Image Source", VideoMode.PixelFormat.kMJPEG, 320, 240, 30);
		//MjpegServer cvStream = new MjpegServer("Image Stream", 1186);
		//cvStream.setSource(imageSource);
		
		NetworkTable dataTable = inst.getTable("vision");
		NetworkTable tapeTable = dataTable.getSubTable("tapeTable");
		
		Mat inputImage = new Mat();
		Mat hsv = new Mat();
		Mat streamed = new Mat();
		
		ArrayList<MatOfPoint> tapeContours;
		TapePipeline tape = new TapePipeline();
		
		while(true) {
			hsv.copyTo(streamed);
			
			long frameTime = imageSink.grabFrame(inputImage);
			if(frameTime == 0) {
				continue;
			}
			
			tape.process(inputImage);
			tapeContours = tape.convexHullsOutput();
			
			ArrayList<RotatedRect> minRect = new ArrayList<RotatedRect>();
			for(MatOfPoint t : tapeContours) {
				minRect.add(Imgproc.minAreaRect(new MatOfPoint2f(t.toArray())));
			}
			
			double[] angles = new double[minRect.size()];
			double[] height = new double[minRect.size()];
			double[] width = new double[minRect.size()];
			double[] centerX = new double[minRect.size()];
			double[] centerY = new double[minRect.size()];
			
			for(int i = 0; i < minRect.size(); i++) {
				angles[i] = minRect.get(i).angle;
				height[i] = minRect.get(i).size.height;
				width[i] = minRect.get(i).size.width;
				centerX[i] = minRect.get(i).center.x;
				centerY[i] = minRect.get(i).center.y;
			}
			
			tapeTable.getEntry("tapeAngles").setDoubleArray(angles);
			tapeTable.getEntry("tapeAngles").setDoubleArray(height);
			tapeTable.getEntry("tapeAngles").setDoubleArray(width);
			tapeTable.getEntry("tapeAngles").setDoubleArray(centerX);
			tapeTable.getEntry("tapeAngles").setDoubleArray(centerY);
			
		}
	}
}