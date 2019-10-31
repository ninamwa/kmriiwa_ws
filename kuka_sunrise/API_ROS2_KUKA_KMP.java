package com.kuka.roboticsAPI;

//JAVA
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
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
import com.kuka.nav.core.swb.laserView.controller.LaserDataController;
import com.kuka.nav.fdi.FDIConnection;

//FOR TESTING:
import com.kuka.roboticsAPI.controllerModel.sunrise.predefinedCompounds.kmp.*;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.ObserverModule;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.DeviceObserverModule;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.OutPort;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.Port;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SPR;
import com.kuka.roboticsAPI.capabilities.interfaces.*;
import com.kuka.roboticsAPI.requestModel.IRequestHandler;
import com.kuka.roboticsAPI.requestModel.ReadIORequest;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSRFactory;
import com.kuka.roboticsAPI.controllerModel.sunrise.state.kmp.provider.MobilePlatformStateProvider;
import com.kuka.nav.geometry.DetectionModel;
import com.kuka.nav.robot.MobileRobotManager;


public class API_ROS2_KUKA_KMP extends RoboticsAPIApplication{
	
	@Inject
	@Named("[Name in StationSetup]")
	public KmpOmniMove kmp;

	public ScannerIOGroup scanner;

	public Controller controller;
	
	public double[] current_pose;
	public double[] current_vel;

	public long LastReceivedTime = System.currentTimeMillis();
	private IMotionContainer _currentMotion;
	
	// TEST
	public OutPort odom_port;
	public OutPort vel_port;
	public AbstractIO ab_io;
	public ReadIORequest read_io_req;
	public IRequestHandler req_handler;
	public DetectionModel detection_mod;
	public LaserDataController laser_data_controller;
	public MobilePlatformStateProvider state_provider;
	public MobileRobotManager mr_manager;
	public MobilePlatformVelocity mp_vel;
	public MobilePlatformPosition mp_pose;
	public FDIConnection fdi;
	private List<String> sensors;

	public String CommandStr; // Command String
	String []strParams = new String[20];
	float []params = new float[10];
	
	public OmniMoveObserver observer;

	// FOR SOCKET CONNECTION:
	public DatagramPacket output_packet;
	public DatagramPacket input_packet;
	private DatagramSocket socket; // UDP socket
	public boolean RUN = false;  // Is socket Connected and app running?
	private byte[] buf;
	private InetAddress ros_inetaddress;
	private int ros_IPport;
    private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");



	public void initialize() {
		getController("KUKA_Sunrise_Cabinet_1");

		kmp = getContext().getDeviceFromType(KmpOmniMove.class);
		kmp.setName("kmp1");
		
		controller = kmp.getController();
		socket = this.socketConnection();
		
		//state_provider = new MobilePlatformStateProvider(controller); //
		//state_provider.getMobilePlatformSafetyState(); // SKAL PRINTE ANTALL SENSORER i logger
		//observer = new OmniMoveObserver("ObsGroup",new SPR(null,"kmp1"));
		//odom_port = observer.getOdoMsr();
		//vel_port = observer.getOdoSpeedMsr();
		//sensors = Arrays.asList("foo", "bar");
		//scanner = new ScannerIOGroup(kmp.getController(), sensors);

		//mp_pose = new MobilePlatformPosition();
		//mp_vel = new MobilePlatformVelocity();
	}

	public DatagramSocket socketConnection()  // Connecting to server at 172.31.1.50 Port:1234
	{
		while (true){
			try{
		socket = new DatagramSocket(); 
		ros_inetaddress = InetAddress.getByName("charlotte-MacBookAir"); 
        // default for controller 172.31.1.10 defaultport: 8090
		ros_IPport = 20001;
		int buffer_size = 1024;
        byte init_msg[] = "Hello".getBytes();
        byte buf[] = new byte[buffer_size]; 

        // connect() method - connect to ROS
		System.out.println("Connecting to server at Port : " + ros_IPport);
        socket.connect(ros_inetaddress, ros_IPport);
	    System.out.println("KMR iiwa is connected to the server.");

        
        output_packet = new DatagramPacket(init_msg, buffer_size, ros_inetaddress, ros_IPport); 
        input_packet = new DatagramPacket(buf, buffer_size); 
        
        // send() method 
        socket.send(output_packet); 
        System.out.println("...packet sent successfully...."); 
        
        // receive() method 
        socket.receive(input_packet); 
        String instr = decode(input_packet);//= new String(receive_package.getData(), StandardCharsets.US_ASCII);
        System.out.println("Received packet data : " + instr);  
		    	break;
			}
			catch(IOException e1){
		        System.out.println("ERROR connecting to the server!");
			}
		}
		RUN = true;
	}
	
    static String decode(DatagramPacket pack) {
    	byte[] data = pack.getData();
        return new String(data, UTF8_CHARSET);
    }

    static byte[] encode(String string) {
        return string.getBytes(UTF8_CHARSET);
    }
    
	public void send_package(DatagramPacket output_packet, String msg)
	{
     output_packet.setData(msg.getBytes());

     try {
			socket.send(output_packet);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public String getLine(DatagramPacket input_packet)
	{
		String line;
		try{
			//HELP! was: while(!inputStream.ready()){}
			while(!RUN) {}
			socket.receive(input_packet);;
			line = decode(input_packet);
	    	System.out.println("Message received: " + line);
	    	return line;
		}
		catch(Exception e){
			return "Error command";
		}
	}

	private void getOperationMode(DatagramPacket output_packet) {
		String opMode = kmp.getOperationMode().toString();
		send_package(output_packet, ">"+"OperationMode " + opMode);
	}

	private void getOdometry() {
		String odom_string = odom_port.getValue().toString(); // get Odometry from port as string
		//String vel_string = vel_port.getValue().toString();

		// From MobilePlatformPosition
		//current_pose = pose.getValues();
		//current_vel = vel.getValues();
		send_package(output_packet, ">"+"odometry " + odom_string);
	}

	private void getLaserScan(DatagramPacket output_packet) {
		String scan_string = odom_port.getValue().toString(); // get Odometry from port as string
		send_package(output_packet, ">"+"laserScan " + scan_string);
	}

	public void setMobilePlatformVelocity(String data) {
		String []lineSplt = CommandStr.split(" ");
		if (strParams.length==4){
			double vx = Double.parseDouble(lineSplt[1]);
			double vy = Double.parseDouble(lineSplt[2]);
			double vTheta = Double.parseDouble(lineSplt[3]);
			double override = 100;

			MobilePlatformVelocityMotion vel = new MobilePlatformVelocityMotion(vx,vy,vTheta,override);
			kmp.move(vel);
		}else{
			getLogger().info("Unacceptable Velocity command!");
		}
	}

	public void getIsFinished(DatagramPacket output_packet)
	{
		LastReceivedTime = System.currentTimeMillis();
		try{
			if(this._currentMotion.isFinished())
	        	send_package(output_packet, ">"+"isFinished " + "true");
			else
	        	send_package(output_packet, ">"+"isFinished " + "false");
		}
		catch(Exception e){
     	send_package(output_packet, ">"+"isFinished " + "true");
		};
	}
	public void getHasError(DatagramPacket output_packet)
	{
		LastReceivedTime = System.currentTimeMillis();
		try{
                 if(this._currentMotion.hasError())
                     send_package(output_packet, ">"+"hasError " + "true");
                 else
                     send_package(output_packet, ">"+"hasError " + "false");
		}
		catch(Exception e){
                 send_package(output_packet, ">"+"hasError " + "true");
		};
	}
	public Thread Send_KMP_data = new Thread(){
	    public void run(){
	    	while (RUN)
	    	{
	    		getOperationMode(output_packet);

	    		ThreadUtil.milliSleep(100);
	    	}
	    }
	};

	public Thread MonitorWorkspace = new Thread(){

	    public void run(){
	    	while (RUN)
	    	{

	    		ThreadUtil.milliSleep(10);
	    	}
	    }
	};

	public void run() {
		socketConnection();
		Send_KMP_data.start();
		MonitorWorkspace.start();

		while( RUN )
		{   //DECODE? Nav sol p 56
	    	CommandStr = getLine(input_packet);
	    	String []lineSplt = CommandStr.split(" ");

			if( (lineSplt[0]).equals("setVelocity"))
				setMobilePlatformVelocity(CommandStr);

			if(  !socket.isConnected() || socket.isClosed()) //|| socket.isOutputShutdown() ||   socket.isInputShutdown())
			{
				try {
					socket.close();
				} catch (IOException e) {
					System.out.println("ERROR closing the port!");
				}
				RUN = false;
			}
		}

		System.out.println("- - - APPLICATION TERMINATED - - -");
	}



	public static void main(String[] args){
		API_ROS2_KUKA_KMP app = new API_ROS2_KUKA_KMP();
		app.runApplication();

	}

}