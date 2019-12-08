package API_0612;

import java.io.IOException;
import java.net.DatagramPacket;
import java.nio.charset.Charset;
import java.net.Socket;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;

public class TCPsocket {
	public boolean isConnected;
	public Socket TCPConn;
	DatagramPacket package_out;
	DatagramPacket package_in;
	public PrintWriter outputStream;
	public BufferedReader inputStream;
	private final static Charset UTF8_CHARSET = Charset.forName("UTF-8");

	
	public TCPsocket() {
		isConnected = false;
		TCPConn = tcpsocketConnection();
	}
	
	public Socket tcpsocketConnection()
	{
		while (true){
			try{
				int COMport = 30008;
				TCPConn = new Socket("192.168.10.116",COMport);
				TCPConn.setReuseAddress(true);
				System.out.println("KUKA connecting to ROS over TCP");
				break;
			}
			catch(IOException e1){
				System.out.println("Could not connect to server: " +e1);
			return null;
			}
		}
		try{
			outputStream = new PrintWriter(TCPConn.getOutputStream(),true);
			inputStream = new BufferedReader(new InputStreamReader(TCPConn.getInputStream()));
			isConnected=true;
			return TCPConn;
		}catch(Exception e){
			System.out.println("Error creating I/O ports: " +e);
			return null;
		}
		
		}
	
	public void send_message(String buffer){
		int len = (this.encode(buffer)).length;
		String send_string = String.format("%010d", len) + ">" + " "+buffer;
		outputStream.write(send_string);
		outputStream.flush();
	}
	
	public String receive_message(){
		String line;
		try{
			while(!this.inputStream.ready()){}
			line=this.inputStream.readLine();
			System.out.println(line);
			return line;
		
			}catch(Exception e){
				System.out.println("Could not read message: " +e);
				return "error";
			}
	}	

	static String decode(byte[] data)  {
    	String message = new String(data,0,data.length, UTF8_CHARSET);
        return message;

    }

    static byte[] encode(String string) {
        return string.getBytes(UTF8_CHARSET);
    }
    
		        
	public void close(){
		try {
			TCPConn.close();
			System.out.println("Connection to ROS closed");
			isConnected=false;
		} catch (Exception e) {
			System.out.println("ERROR closing the communication port to ROS!");
		}
	}

}
