package com.kuka.roboticsAPI;

	//JAVA
	import java.util.Arrays;
	import java.util.List;
	import javax.inject.Inject;
	import javax.inject.Named;

	import org.apache.log4j.BasicConfigurator;

	import java.io.IOException;

	//UDP
	import java.net.DatagramPacket;
	import java.net.DatagramSocket;
	import java.net.InetAddress;
	import java.nio.charset.Charset;

	//COMMON
	import com.kuka.common.ThreadUtil;
	import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
	import com.kuka.roboticsAPI.executionModel.ICommandContainer;


	//DEVICE MODEL
	import com.kuka.roboticsAPI.deviceModel.Device;
	import com.kuka.roboticsAPI.deviceModel.JointPosition;
	import com.kuka.roboticsAPI.deviceModel.LBR;
	import com.kuka.roboticsAPI.deviceModel.MobilePlatform;
	import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
	import com.kuka.roboticsAPI.controllerModel.Controller;


	//MOTION MODEL
	import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
	import com.kuka.roboticsAPI.motionModel.ErrorHandlingAction;
	import com.kuka.roboticsAPI.motionModel.IMotionContainer;
	import com.kuka.roboticsAPI.motionModel.IErrorHandler;
	import com.kuka.roboticsAPI.motionModel.MotionBatch;
	import com.kuka.roboticsAPI.motionModel.kmp.*;
	//SENSOR
	import com.kuka.roboticsAPI.controllerModel.sunrise.state.kmp.io.ScannerIOGroup;
	import com.kuka.roboticsAPI.controllerModel.sunrise.mapping.CommandMapper;
	import com.kuka.roboticsAPI.ioModel.AbstractIO;
	import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
	import com.kuka.roboticsAPI.ioModel.Output;
	import com.kuka.roboticsAPI.controllerModel.sunrise.state.kmp.IMobilePlatformSafetyState;
	import com.kuka.roboticsAPI.controllerModel.sunrise.state.kmp.IMobilePlatformSafetyState.SafetyState;

	//FOR TESTING:
	import com.kuka.roboticsAPI.controllerModel.sunrise.predefinedCompounds.kmp.*;
	import com.kuka.roboticsAPI.controllerModel.sunrise.api.OutPort;
	import com.kuka.roboticsAPI.controllerModel.sunrise.api.SPR;
	import com.kuka.roboticsAPI.requestModel.IRequestHandler;
	import com.kuka.roboticsAPI.requestModel.ReadIORequest;
	import com.kuka.nav.geometry.DetectionModel;
	import com.kuka.nav.robot.MobileRobotManager;


	public class API_ROS2_KUKA1211roboticsapi extends RoboticsAPIApplication{
		
		@Inject
		@Named("[KMPOmniMove200]")
		public KmpOmniMove kmp;

		public ScannerIOGroup scanner;

		public Controller controller;
		
		public double[] current_pose;
		public double[] current_vel;

		public long LastReceivedTime = System.currentTimeMillis();
		private ICommandContainer _currentMotion;
		
		// TEST
		public OutPort odom_port;
		public AbstractIO ab_io;
		public ReadIORequest read_io_req;
		public IRequestHandler req_handler;
		public DetectionModel detection_mod;
		public MobileRobotManager mr_manager;
		public MobilePlatformVelocity mp_vel;
		public MobilePlatformPosition mp_pose;
		public boolean RUN = true;  // Is socket Connected and app running?


		public String CommandStr; // Command String
		String []strParams = new String[20];
		float []params = new float[10];
		
		public OutPort odomPort;
		
		MobilePlatformPosition pose;
		MobilePlatformVelocity vel;


		public void initialize() {
			BasicConfigurator.configure();
			getController("KUKA_Sunrise_Cabinet_1");
			
			getLogger().info("Initializing robot");
		
			kmp = getContext().getDeviceFromType(KmpOmniMove.class);
			kmp.setName("KMPOmniMove200");
			
			controller = kmp.getController();

					
			// Init ports to read odometry  - usikker på init av denne.
			OmniMoveObserver observer = new OmniMoveObserver("ObsGroup",new SPR(null,kmp.getName()));
			odom_port = observer.getOdoMsr();
			getLogger().info(observer.getOutPorts().toString());
		
			// Init scannergroup to read lasers
			List<String> sensors = Arrays.asList("FrontLaser", "RearLaser");
			scanner = new ScannerIOGroup(controller, sensors);

			pose = new MobilePlatformPosition();
			vel = new MobilePlatformVelocity();
		}


		private void getOdometry() {
			// FROM PORT:
			String odom_string = odom_port.getValue().toString(); // get Odometry from port as string
			getLogger().info(odom_string);

			// From MobilePlatformPosition
			current_pose = pose.getValues();
			current_vel = vel.getValues();
			
			getLogger().info(current_pose.toString());
			getLogger().info(current_vel.toString());


		}

		private void getLaserScan() {
			// ScannerIOGroup:
			String scan_str= scanner.getOutputs().toString();
			getLogger().info(scan_str);

			
		}

		
		public Thread Send_KMP_data = new Thread(){
		    public void run(){
		    	while (RUN)
		    	{
		    		getOdometry();
		    		getLaserScan();

		    		ThreadUtil.milliSleep(100);
		    	}
		    }
		};


		public void run() {
			Send_KMP_data.start();
		}



		public static void main(String[] args){
			API_ROS2_KUKA1211roboticsapi app = new API_ROS2_KUKA1211roboticsapi();
			app.runApplication();

		}

	
}