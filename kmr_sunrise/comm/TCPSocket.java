package API_ROS2_Sunrise;


import java.io.IOException;

import java.net.DatagramPacket;

import java.nio.charset.Charset;
import java.net.Socket;
    
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;

import API_ROS2_Sunrise.ISocket;

public class TCPSocket implements ISocket{
	
	
	public volatile boolean isConnected;
	public Socket TCPConn;
	DatagramPacket package_out;
	DatagramPacket package_in;
	public PrintWriter outputStream;
	public BufferedReader inputStream;
	private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");
	int COMport;
	String nodename;
	
	public TCPSocket(int port, String node_name) {
		isConnected = false;
		COMport = port;
		TCPConn = connect();
		this.nodename=node_name;

	}
	
	public Socket connect()
	{
		while (true){
			try{
				String remotePC = "192.168.10.117";
				//String NUC = "192.168.10.120";

				TCPConn = new Socket(remotePC,COMport);
				TCPConn.setReuseAddress(true);
				System.out.println(this.nodename + " connecting to ROS over TCP on port: "+ COMport);
				break;
			}
			catch(IOException e1){
				System.out.println("Could not connect "+ this.nodename+ " to ROS over TCP on port : "+ this.COMport + " Error: " +e1);
			return null;
			}
		}
		try{
			outputStream = new PrintWriter(TCPConn.getOutputStream(),true);
			inputStream = new BufferedReader(new InputStreamReader(TCPConn.getInputStream()));
			isConnected=true;
			return TCPConn;
		}catch(Exception e){
			System.out.println("Error creating I/O ports for TCP communication for  "+ this.nodename+ " on port: "+ this.COMport + " Error: " +e);
			return null;
		}
		
		}
	
	public void send_message(String buffer){
		int len = (this.encode(buffer)).length;
		String send_string = String.format("%010d", len) + " "+buffer;
		outputStream.write(send_string);
		outputStream.flush();
	}
	
	@Override
	public String receive_message(){
		String line;
		try{
			while(!this.inputStream.ready()){}
			line=this.inputStream.readLine();
			return line;
		
			}catch(Exception e){
				System.out.println(this.nodename+ " could not receive message from TCP connection on port: "+ this.COMport + " Error: " +e);
				return "error";
			}
	}	

    
		        
	public void close(){
		try {
			TCPConn.close();
			System.out.println("TCP connection to ROS closed port: " + this.COMport);
			isConnected=false;
		} catch (Exception e) {
			System.out.println("ERROR closing the TCP communication of  "+ this.nodename+ "  on port: " + this.COMport + " error: " + e);
		}
	}
	

	public String decode(byte[] data) {
		String message = new String(data,0,data.length, UTF8_CHARSET);
        return message;
	}

	@Override
	public byte[] encode(String string) {
		return string.getBytes(UTF8_CHARSET);
	}

	@Override
	public boolean isConnected() {
		return this.isConnected;
	}

}
