package com.kuka.roboticsAPI;

// JAVA
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;


// COMMON
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;

// DEVICE MODEL
import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.MobilePlatform;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.controllerModel.Controller;


// MOTION MODEL
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.motionModel.ErrorHandlingAction;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IErrorHandler;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.kmp.*;

// SENSOR
import com.kuka.roboticsAPI.controllerModel.sunrise.state.kmp.io.ScannerIOGroup;
import com.kuka.roboticsAPI.controllerModel.sunrise.mapping.CommandMapper;

import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;

// FOR TESTING:
import com.kuka.roboticsAPI.controllerModel.sunrise.predefinedCompounds.kmp.*;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.OutPort;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.Port;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SPR;
import com.kuka.roboticsAPI.capabilities.interfaces.*;


public class API_ROS2_KUKA_KMP extends RoboticsAPIApplication{
	
	public KmpOmniMove kmp;
	
	public ScannerIOGroup scanner;
	
	public Controller controller;
	
	public MobilePlatformPosition pose;
	public OutPort odom_port;
	public OutPort vel_port;

	
	public double[] current_pose;
	public MobilePlatformVelocity vel;
	public double[] current_vel;
	
	public OmniMoveObserver observer;

	// FOR SOCKET CONNECTION:
	public DatagramPacket output_packet;
	public DatagramPacket input_packet;
    private DatagramSocket socket; // UDP socket
	public boolean RUN = false;  // Is socket Connected and app running?
    private byte[] buf;
	private InetAddress inetaddress;
	private int IPport;
	
	private List<String> sensors;
    
    public String CommandStr; // Command String
    String []strParams = new String[20];
    float []params = new float[10];
    
    public long LastReceivedTime = System.currentTimeMillis();
    private IMotionContainer _currentMotion;
    

	public void initialize() {
		
	getController("KUKA_Sunrise_Cabinet_1");
	
	kmp = getContext().getDeviceFromType(KmpOmniMove.class);
	kmp.setName("kmp1");
	observer = new OmniMoveObserver("ObsGroup",new SPR(null,"kmp1"));
	odom_port = observer.getOdoMsr();
	vel_port = observer.getOdoSpeedMsr();
	
	scanner = new ScannerIOGroup(kmp.getController(), sensors);
	
	controller = kmp.getController();
	pose = new MobilePlatformPosition();
	vel = new MobilePlatformVelocity();
	
	}
	
	public void socketConnection()  // Connecting to server at 172.31.1.50 Port:1234
	{
		int buffer_size = 1024;
		buf = new byte[buffer_size];
		System.out.println("Connecting to server at Port:xxx");
		
		while (true){
			try{
			    socket = new DatagramSocket(); // Modify the IP and port depending on the system which is running the ROS-KUKA node server if it is required.
			    inetaddress = InetAddress.getByName("localhost"); 
		        IPport = 5252; 
		        output_packet = new DatagramPacket(buf, buf.length, inetaddress, IPport); 
		        input_packet = new DatagramPacket(buf, buf.length); 
		          
		        // connect() method 
		        socket.connect(inetaddress, IPport); 
		        
			    System.out.println("KUKA iiwa is connected to the server.");
		    	break;
			}
			catch(IOException e1){
		        System.out.println("ERROR connecting to the server!");	        
			}
		}
		RUN = true;
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
			//HELP
			//while(!inputStream.ready()){}
			while(!RUN) {}
			socket.receive(input_packet);;
			line = new String(input_packet.getData(),0,input_packet.getLength());
	    	System.out.println("Command received: " + line);
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
		current_pose = pose.getValues();
		current_vel = vel.getValues();
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

			if( (lineSplt[0].toString()).equals("setPosition".toString()))
				//setPosition(CommandStr);
			
			if( socket.isInputShutdown() || !socket.isConnected() || socket.isOutputShutdown() || socket.isClosed())
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
