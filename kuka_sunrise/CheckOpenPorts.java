package testwithrobot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

public class CheckOpenPorts extends RoboticsAPIApplication{
	
	public static void main(String[] args) {
		CheckOpenPorts nr = new CheckOpenPorts();
		nr.runApplication();
		//nr.run();
	}


	public void run(){
		String s = "";
				try {
					Process p = Runtime.getRuntime().exec("netstat -aop \"UDP\"  "); 
					BufferedReader stdInput = new BufferedReader(new InputStreamReader(p.getInputStream()));
					BufferedReader stdError = new BufferedReader(new InputStreamReader(p.getErrorStream()));
		 
					// read the output from the command
					System.out.println("Following ports are closing: \n");
					while ((s = stdInput.readLine()) != null) {
						if(s.contains("30001")){
							System.out.println(s);
							String[] pid30001 = s.split("                                    ");
							String PID1 = pid30001[1];
							Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID1); 
							}
						if(s.contains("30002")){
							System.out.println(s);
							String[] pid30002 = s.split("                                    ");
							String PID2 = pid30002[1];
							//Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID2); 
							}
						if(s.contains("30003")){
							System.out.println(s);
							String[] pid30003 = s.split("                                    ");
							String PID3 = pid30003[1];
							//Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID3); 
							}
						if(s.contains("30004")){
							System.out.println(s);
							String[] pid30004 = s.split("                                    ");
							String PID4 = pid30004[1];
							//Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID4); 
							}
						if(s.contains("30005")){
							System.out.println(s);
							String[] pid30005 = s.split("                                    ");
							String PID5 = pid30005[1];
							//Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID5); 
							}
						if(s.contains("30006")){
							System.out.println(s);
							String[] pid30006= s.split("                                    ");
							String PID6 = pid30006[1];
							//Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID6); 
							}
						if(s.contains("30007")){
							System.out.println(s);
							String[] pid30007 = s.split("                                    ");
							String PID7 = pid30007[1];
							//Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID7); 
							}
						}
	
					// read any errors from the attempted command
					System.out.println("Here is the standard error of the command (if any):\n");
					while ((s = stdError.readLine()) != null) {
						System.out.println(s);
					}
					dispose();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

			}
	}


