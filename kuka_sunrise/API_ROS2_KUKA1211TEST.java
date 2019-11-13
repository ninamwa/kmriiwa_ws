package com.kuka.roboticsAPI;


	// JAVA

	import javax.inject.Inject;
	import javax.inject.Named;

	import org.apache.log4j.BasicConfigurator;

	import java.io.BufferedReader;
	import java.io.IOException;
	import java.io.InputStreamReader;
	import java.io.PrintWriter;
	import java.net.DatagramPacket;
	import java.net.DatagramSocket;
	import java.net.InetAddress;
	import java.net.InetSocketAddress;
	import java.nio.charset.Charset;

	//COMMON
	import com.kuka.common.ThreadUtil;
	import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
	import com.kuka.roboticsAPI.executionModel.ICommandContainer;

	//DEVICE MODEL
	import com.kuka.roboticsAPI.deviceModel.MobilePlatform;
	import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
	import com.kuka.roboticsAPI.controllerModel.Controller;
	import com.kuka.roboticsAPI.DataController;


	//MOTION MODEL
	import com.kuka.roboticsAPI.motionModel.kmp.*;
	import com.kuka.task.ITaskLogger;
	//SENSOR
	import com.kuka.roboticsAPI.controllerModel.sunrise.state.kmp.io.ScannerIOGroup;
	import com.kuka.nav.Pose;
	import com.kuka.nav.fdi.FDIConnection;
	import com.kuka.nav.fdi.data.Odometry;
	import com.kuka.nav.fdi.internal.protocol.msg.OdometryMsg;



	public class API_ROS2_KUKA1211TEST extends RoboticsAPIApplication{

		@Inject
		@Named("[KMPOmniMove200]")
		public KmpOmniMove kmp;

		public ScannerIOGroup scanner;

		public Controller controller;

		public double[] current_pose;
		public double[] current_vel;

		public long LastReceivedTime = System.currentTimeMillis();

		public boolean RUN = true;  // Is socket Connected and app running?

		// TEST
		public FDIConnection fdi;
		public DataController listener;

		public String CommandStr; // Command String
		String []strParams = new String[20];
		float []params = new float[10];


		private ICommandContainer _currentMotion;
	    	private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");



		public void initialize() {
			BasicConfigurator.configure();
			getController("KUKA_Sunrise_Cabinet_1");
			getLogger().info("Initializing robot");
			kmp = getContext().getDeviceFromType(KmpOmniMove.class);
			kmp.setName("KMPOmniMove200");

			controller = kmp.getController();
			
			// FOR FDI
			String serverIP = "127.0.1.1";
			int port = 34001;
			InetSocketAddress adr = new InetSocketAddress(serverIP,port);
			fdi = new FDIConnection(adr);
			ITaskLogger log = getLogger();
			listener = new DataController(log);
			fdi.addDataListener(listener);		
			fdi.connect();
			getLogger().info("connected fdi");
			
		}


		private void getOdometry() {
			Odometry msg = new Odometry();
			msg = fdi.getLastOdometry();
			String odom_string = msg.toString();
			getLogger().info(odom_string);
			
			//String odom_string1 = fdi.getNewOdometry().toString();
			//getLogger().info(odom_string1);
		}

		private void getLaserScan() {
			int laserId1 = 1801;
			int laserId2 = 1802;
			getLogger().info("Initializing robot");
			String scan_string1 = fdi.getNewLaserScan(laserId1).toString();
			getLogger().info(scan_string1);
			
			//String scan_string4 = fdi.getLastLaserScan(laserId2).toString();
			//getLogger().info(scan_string4);
		}
		
		
		
		public Thread Send_KMP_data = new Thread(){
		    public void run(){
		    	while (RUN)
		    	{
		    		//getLaserScan();
		    		//getOdometry();

		    		ThreadUtil.milliSleep(100);
		    	}
		    }
		};


		public void run() throws IOException {
			Send_KMP_data.start();
		}



		public static void main(String[] args){
			API_ROS2_KUKA1211TEST  app = new API_ROS2_KUKA1211TEST();
			app.runApplication();
		}
	}
