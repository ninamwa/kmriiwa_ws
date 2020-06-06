package API_ROS2_Sunrise;


import java.net.BindException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.charset.Charset;

import API_ROS2_Sunrise.ISocket;


public class UDPSocket implements ISocket{
	
	public volatile boolean isConnected;
	DatagramSocket udpConn;
	DatagramPacket package_out;
	DatagramPacket package_in;
	private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");
	int COMport;
    static BindException b;
    String nodename;

	
	public UDPSocket(int port, String node_name) {
		isConnected = false;
		this.COMport = port;
		udpConn = connect();
		this.nodename=node_name;
	}
	
	public DatagramSocket connect()
	{
		while (!isConnected){
			try{
				System.out.println("Connecting  "+ this.nodename+ " to ROS over UDP on port: " + COMport); 
		    	int kuka_port = this.COMport; // change this if cannot bind error
		        InetSocketAddress socket_address = new InetSocketAddress(kuka_port);
		    	
		    	String ros_host = "192.168.10.117"; // REMOTE PC

		    	int ros_port = this.COMport;
		        InetAddress address = InetAddress.getByName(ros_host);

		        
		        udpConn = new DatagramSocket(null); 
		        udpConn.setReuseAddress(true);
		        udpConn.bind(socket_address);
		        
		   
		        
		        byte buf[] = "HALLA ROS".getBytes();
		        byte buf1[] = new byte[1024]; 
		        
		        package_out = new DatagramPacket(buf, buf.length, address, ros_port); 
		        package_in = new DatagramPacket(buf1, buf1.length); 
		      
		        // send() method 
		        udpConn.send(package_out); 
		  
		        // receive() method 
		        udpConn.setSoTimeout(3000); //ms

		        udpConn.receive(package_in); 
		        String s = decode(package_in);
		        System.out.println(this.nodename+ " received packet data over UDP on port : " + COMport + " Message: " +s);  
		        
		        if(s.length()<1){
		    	   udpConn.close();
		       		isConnected=false;
		       		System.out.println( this.nodename+ "  did not receive any message in 3 seconds, shutting off");
		       		break;
		       	 }
		        udpConn.setSoTimeout(0);
		        isConnected=true;
			}
			catch(Exception e1){
		        System.out.println("ERROR connecting  "+ this.nodename+ " to ROS over UDP on port: " + this.COMport + " Error: " + e1);
		        isConnected=false;
		        close();
				b = new BindException();
		        if(e1.getClass().equals(b.getClass())){
		        	e1.printStackTrace();
		        	break;
		        }
		        break;
		        }
			}
		return udpConn;
	}
	
	public String decode(DatagramPacket pack)  {
    	byte[] data = pack.getData();
    	String message = new String(data,0,pack.getLength(), UTF8_CHARSET);
        return message;

    }

	@Override
    public byte[] encode(String string) {
        return string.getBytes(UTF8_CHARSET);
    }
    
	@Override
    public void send_message(String msg)
	{
    	byte[] bytlist = msg.getBytes(UTF8_CHARSET);
    	package_out.setData(bytlist);
    	package_out.setLength(bytlist.length);
     try {
			udpConn.send(package_out);
		} catch (Exception e) {
			System.out.println( this.nodename+ " could not send package over UDP on port: "  + this.COMport + " error: " + e);
		}
	}
    @Override
	public String receive_message()
	{
		String line;
		try{
			udpConn.receive(package_in);
			line = decode(package_in);
	    	return line;
		}
		catch(Exception e){
			System.out.println("Error receiving package  "+ this.nodename+ " over UDP on port: " + this.COMport + " error: " + e);
			return " ";
		}
	}
	
    @Override
	public void close(){
		try {
			udpConn.close();
			System.out.println("Connection to ROS closed on port: " + COMport);
			isConnected=false;
		} catch (Exception e) {
			System.out.println("ERROR closing the UDP communication for "+ this.nodename+ " on port to ROS: " + COMport);
		}
	}


	@Override
	public boolean isConnected() {	
		return this.isConnected;
	}

}
