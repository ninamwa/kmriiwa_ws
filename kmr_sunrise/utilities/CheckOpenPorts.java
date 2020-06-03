package API_ROS2_Sunrise;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;


public class CheckOpenPorts{
	
//	public static void main(String[] args) {
//		CheckOpenPorts nr = new CheckOpenPorts();
////		nr.runApplication();
//		nr.run();
//	}


	public boolean run(){
		String s = "";
		boolean openports = false;
				try {
					Process p = Runtime.getRuntime().exec("netstat -aop \"TCP\"  "); 

					BufferedReader stdInput = new BufferedReader(new InputStreamReader(p.getInputStream()));
					BufferedReader stdError = new BufferedReader(new InputStreamReader(p.getErrorStream()));
		 
					// read the output from the command
					System.out.println("Following ports are open: \n");
					while ((s = stdInput.readLine()) != null) {
						if(s.contains("30001")){
							System.out.println(s);
							String[] pid30001 = s.split("                                    ");
							String PID1 = pid30001[1];
							openports = true;
							Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID1); 
							}

						if(s.contains("30002")){
							System.out.println(s);
							String[] pid30002 = s.split("                                    ");
							String PID2 = pid30002[1];
							openports = true;
							Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID2); 
							}
						if(s.contains("30003")){
							System.out.println(s);
							String[] pid30003 = s.split("                                    ");
							String PID3 = pid30003[1];
							openports = true;
//							Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID3); 
							}
						if(s.contains("30004")){
							System.out.println(s);
							String[] pid30004 = s.split("                                    ");
							String PID4 = pid30004[1];
							openports = true;
//							Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID4); 
							}
						if(s.contains("30005")){
							System.out.println(s);
							String[] pid30005 = s.split("                                    ");
							String PID5 = pid30005[1];
							openports = true;
//							Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID5); 
							}
						if(s.contains("30006")){
							System.out.println(s);
							String[] pid30006= s.split("                                    ");
							String PID6 = pid30006[1];
							openports = true;
//							Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID6); 
							}
						if(s.contains("30007")){
							System.out.println(s);
							String[] pid30007 = s.split("                                    ");
							String PID7 = pid30007[1];
							openports = true;
//							Process killpid = Runtime.getRuntime().exec("taskkill /PID "+ PID7); 
							}
						}
	
					// read any errors from the attempted command
					while ((s = stdError.readLine()) != null) {
						System.out.println(s);
					}
					//dispose();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				return openports;

			}

	}


