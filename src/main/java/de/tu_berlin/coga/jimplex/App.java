package de.tu_berlin.coga.jimplex;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.text.ParseException;


/**
 * Program entry point
 * 
 */
public class App {
	
	/**
	 * Starts the lp solver
	 * @param args
	 */
	public static void main(String[] args) {
		if (args.length > 0) {
			final String fileName = args[0];
			LPReader lpReader = new LPReader(fileName);
			
			try {
				lpReader.readLP();
				
				System.out.println("DONE!");
			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (ParseException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}		
	}
}
