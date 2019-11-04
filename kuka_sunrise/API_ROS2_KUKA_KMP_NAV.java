package com.kuka.roboticsAPI;

// JAVA
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

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
import com.kuka.nav.fdi.FDIConnection;

//FOR TESTING:



public class API_ROS2_KUKA_KMP_NAV extends RoboticsAPIApplication{

	@Inject
	@Named("[KMPOmniMove200]")
	public KmpOmniMove kmp;

	public ScannerIOGroup scanner;

	public Controller controller;

	public double[] current_pose;
	public double[] current_vel;

	public long LastReceivedTime = System.currentTimeMillis();


	// TEST
	public FDIConnection fdi;

	public String CommandStr; // Command String
	String []strParams = new String[20];
	float []params = new float[10];


	// FOR SOCKET CONNECTION:
	public DatagramPacket output_packet;
	public DatagramPacket input_packet;
	private DatagramSocket socket; // UDP socket
	public boolean RUN = false;  // Is socket Connected and app running?
	private byte[] buf;
	private InetAddress ros_inetaddress;
	private int ros_IPport;

	private ICommandContainer _currentMotion;
    	private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");



	public void initialize() {
		BasicConfigurator.configure();
		getController("KUKA_Sunrise_Cabinet_1");
		getLogger().info("Initializing robot");
		kmp = getContext().getDeviceFromType(KmpOmniMove.class);
		kmp.setName("KMPOmniMove200");

		controller = kmp.getController();
		socket = this.socketConnection();
		System.out.println("Connected!");

	}

	public DatagramSocket socketConnection()  // Connecting to server at 172.31.1.50 Port:1234
	{
		while (true){
			try{
		socket = new DatagramSocket();
		ros_inetaddress = InetAddress.getByName("charlotte-MacBookAir");
        // default for controller 172.31.1.10 defaultport: 8090
		ros_IPport = 20001;  // 30,000 to 30,010
		int buffer_size = 1024;
        byte init_msg[] = "Hello from KUKA".getBytes();
        byte buf[] = new byte[buffer_size];

        // connect() method - connect to ROS
		System.out.println("Connecting to server at Port : " + ros_IPport);
        socket.connect(ros_inetaddress, ros_IPport);
	    System.out.println("KMR iiwa is connected to the server.");


        output_packet = new DatagramPacket(init_msg, init_msg.length, ros_inetaddress, ros_IPport); 
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
		return socket;
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
     output_packet.setLength(msg.length());

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
			//was: while(!inputStream.ready()){}
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

		send_package(output_packet, ">"+"odometry " + odom_string);
	}

	private void getLaserScan(DatagramPacket output_packet) {
		send_package(output_packet, ">"+"laserScan " + scan_string);
	}

	public void setMobilePlatformVelocity(String data) {
		String []lineSplt = data.split(" ");
		if (strParams.length==4){
			double vx = Double.parseDouble(lineSplt[1]);
			double vy = Double.parseDouble(lineSplt[2]);
			double vTheta = Double.parseDouble(lineSplt[3]);
			double override = 1;

			MobilePlatformVelocityMotion vel = new MobilePlatformVelocityMotion(vx,vy,vTheta,override);
		if(kmp.isReadyToMove()) {
				this._currentMotion =  kmp.moveAsync(MP_vel_motion);
			}
			else {
				getLogger().warn("Kmp is not ready to move!");
			}
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
		API_ROS2_KUKA_KMP_NAV app = new API_ROS2_KUKA_KMP_NAV();
		app.runApplication();

	}

}
