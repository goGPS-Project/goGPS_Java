/*
 * Copyright (c) 2010, Eugenio Realini, Mirko Reguzzoni, Cryms sagl, Daisuke Yoshida (Osaka City Univ.). All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 *
 * goGPS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * goGPS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with goGPS.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
package org.gogpsproject.producer.parser.rinex;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.text.ParseException;
import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.ephemeris.EphGps;
import org.gogpsproject.ephemeris.EphemerisSystem;
import org.gogpsproject.positioning.Coordinates;
import org.gogpsproject.positioning.SatellitePosition;
import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.StreamResource;
import org.gogpsproject.producer.parser.IonoGps;

/**
 * <p>
 * Class for parsing RINEX navigation files
 * </p>
 *
 * @author Eugenio Realini, Cryms.com
 */
public class RinexNavigationParser extends EphemerisSystem implements NavigationProducer{

	private File fileNav;
	private FileInputStream streamNav;
	private InputStreamReader inStreamNav;
	private BufferedReader buffStreamNav;

	private FileOutputStream cacheOutputStream;
	private OutputStreamWriter cacheStreamWriter;

	public static String newline = System.getProperty("line.separator");

	private ArrayList<EphGps> eph = new ArrayList<EphGps>(); /* GPS broadcast ephemerides */
	//private double[] iono = new double[8]; /* Ionosphere model parameters */
	private IonoGps iono = null; /* Ionosphere model parameters */
	//	private double A0; /* Delta-UTC parameters: A0 */
	//	private double A1; /* Delta-UTC parameters: A1 */
	//	private double T; /* Delta-UTC parameters: T */
	//	private double W; /* Delta-UTC parameters: W */
	//	private int leaps; /* Leap seconds */


	// RINEX Read constructors
	public RinexNavigationParser(File fileNav) {
		this.fileNav = fileNav;
	}

	// RINEX Read constructors
	public RinexNavigationParser(InputStream is, File cache) {
		this.inStreamNav = new InputStreamReader(is);
		if(cache!=null){
			File path = cache.getParentFile();
			if(!path.exists()) path.mkdirs();
			try {
				cacheOutputStream = new FileOutputStream(cache);
				cacheStreamWriter = new OutputStreamWriter(cacheOutputStream);
			} catch (FileNotFoundException e) {
				System.err.println("Exception writing "+cache);
				e.printStackTrace();
			}
		}
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.Navigation#init()
	 */
	@Override
	public void init() {
		open();
		int ver = parseHeaderNav();
		if(ver != 0){

			if (ver == 2){

				//				System.out.println("Ver. 2.x");
				parseDataNavV2();

			} else if (ver == 212){

				//				System.out.println("Ver. 2.12");
				parseDataNavV2();

			} else if (ver == 3){

				//				System.out.println("Ver. 3.01");
				parseDataNavV3();

			}
	    close();
		}
		else{
		  close();
      throw new RuntimeException( fileNav.toString() + " is invalid ");
		}
	}



	/* (non-Javadoc)
	 * @see org.gogpsproject.Navigation#release()
	 */
	@Override
	public void release(boolean waitForThread, long timeoutMs) throws InterruptedException {

	}

	/**
	 *
	 */
	public void open() {
		try {

			if(fileNav!=null) streamNav = new FileInputStream(fileNav);
			if(streamNav!=null) inStreamNav = new InputStreamReader(streamNav);
			if(inStreamNav!=null) buffStreamNav = new BufferedReader(inStreamNav);

		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		}
	}

	public void close() {
		try {
			if(cacheStreamWriter!=null){
				cacheStreamWriter.flush();
				cacheStreamWriter.close();
			}
			if(cacheOutputStream!=null){
				cacheOutputStream.flush();
				cacheOutputStream.close();
			}
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		} catch (IOException e2) {
			e2.printStackTrace();
		}
		try {

			if(buffStreamNav!=null) buffStreamNav.close();
			if(inStreamNav!=null) inStreamNav.close();
			if(streamNav!=null) streamNav.close();


		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		} catch (IOException e2) {
			e2.printStackTrace();
		}
	}

	/**
	 *
	 */
	public int parseHeaderNav() {

		//Navigation.iono = new double[8];
		String sub;
		int ver = 0;

		try {

			while (buffStreamNav.ready()) {

				try {
					String line = buffStreamNav.readLine();
					if(cacheStreamWriter!=null){
						cacheStreamWriter.write(line);
						cacheStreamWriter.write(newline);
					}

					String typeField = line.substring(60, line.length());
					typeField = typeField.trim();

					if (typeField.equals("RINEX VERSION / TYPE")) {

						if (!line.substring(20, 21).equals("N")) {

							// Error if navigation file identifier was not found
							System.err.println("Navigation file identifier is missing in file " + fileNav.toString() + " header");
							return ver = 0;

						} else if (line.substring(5, 7).equals("3.")){

							//							System.out.println("Ver. 3.01");
							ver = 3;

						} else if (line.substring(5, 9).equals("2.12")){

							//							System.out.println("Ver. 2.12");
							ver = 212;

						} else {

							//							System.out.println("Ver. 2.x");
							ver = 2;
						}

					}

					switch (ver){ 	
					/* RINEX ver. 2.x */
					case 2:

						if (typeField.equals("ION ALPHA")) {

							float a[] = new float[4];
							sub = line.substring(3, 14).replace('D', 'e');
							//Navigation.iono[0] = Double.parseDouble(sub.trim());
							a[0] = Float.parseFloat(sub.trim());

							sub = line.substring(15, 26).replace('D', 'e');
							//Navigation.iono[1] = Double.parseDouble(sub.trim());
							a[1] = Float.parseFloat(sub.trim());

							sub = line.substring(27, 38).replace('D', 'e');
							//Navigation.iono[2] = Double.parseDouble(sub.trim());
							a[2] = Float.parseFloat(sub.trim());

							sub = line.substring(39, 50).replace('D', 'e');
							//Navigation.iono[3] = Double.parseDouble(sub.trim());
							a[3] = Float.parseFloat(sub.trim());

							if(iono==null) iono = new IonoGps();
							iono.setAlpha(a);

						} else if (typeField.equals("ION BETA")) {

							float b[] = new float[4];

							sub = line.substring(3, 14).replace('D', 'e');
							//Navigation.iono[4] = Double.parseDouble(sub.trim());
							//setIono(4, Double.parseDouble(sub.trim()));
							b[0] = Float.parseFloat(sub.trim());


							sub = line.substring(15, 26).replace('D', 'e');
							//Navigation.iono[5] = Double.parseDouble(sub.trim());
							//setIono(5, Double.parseDouble(sub.trim()));
							b[1] = Float.parseFloat(sub.trim());

							sub = line.substring(27, 38).replace('D', 'e');
							//Navigation.iono[6] = Double.parseDouble(sub.trim());
							//setIono(6, Double.parseDouble(sub.trim()));
							b[2] = Float.parseFloat(sub.trim());

							sub = line.substring(39, 50).replace('D', 'e');
							//Navigation.iono[7] = Double.parseDouble(sub.trim());
							//setIono(7, Double.parseDouble(sub.trim()));
							b[3] = Float.parseFloat(sub.trim());

							if(iono==null) iono = new IonoGps();
							iono.setBeta(b);

						} else if (typeField.equals("DELTA-UTC: A0,A1,T,W")) {

							if(iono==null) iono = new IonoGps();

							sub = line.substring(3, 22).replace('D', 'e');
							//setA0(Double.parseDouble(sub.trim()));
							iono.setUtcA0(Double.parseDouble(sub.trim()));

							sub = line.substring(22, 41).replace('D', 'e');
							//setA1(Double.parseDouble(sub.trim()));
							iono.setUtcA1(Double.parseDouble(sub.trim()));

							sub = line.substring(41, 50).replace('D', 'e');
							//setT(Integer.parseInt(sub.trim()));
							// TODO need check
							iono.setUtcWNT(Integer.parseInt(sub.trim()));

							sub = line.substring(50, 59).replace('D', 'e');
							//setW(Integer.parseInt(sub.trim()));
							// TODO need check
							iono.setUtcTOW(Integer.parseInt(sub.trim()));

						} else if (typeField.equals("LEAP SECONDS")) {
							if(iono==null) iono = new IonoGps();
							sub = line.substring(0, 6).trim().replace('D', 'e');
							//setLeaps(Integer.parseInt(sub.trim()));
							// TODO need check
							iono.setUtcLS(Integer.parseInt(sub.trim()));

						} else if (typeField.equals("END OF HEADER")) {	
							return ver;
						}
						break;


						/* RINEX ver. 2.12 */
					case 212:

						//							System.out.println("Ver. 2.12");

						String typeField2 = line.substring(0, 4);
						typeField2 = typeField2.trim();

						if (typeField2.equals("GPSA")) {

							float a[] = new float[4];
							sub = line.substring(6, 17).replace('D', 'e');
							//Navigation.iono[0] = Double.parseDouble(sub.trim());
							a[0] = Float.parseFloat(sub.trim());

							sub = line.substring(18, 29).replace('D', 'e');
							//Navigation.iono[1] = Double.parseDouble(sub.trim());
							a[1] = Float.parseFloat(sub.trim());

							sub = line.substring(30, 41).replace('D', 'e');
							//Navigation.iono[2] = Double.parseDouble(sub.trim());
							a[2] = Float.parseFloat(sub.trim());

							sub = line.substring(42, 53).replace('D', 'e');
							//Navigation.iono[3] = Double.parseDouble(sub.trim());
							a[3] = Float.parseFloat(sub.trim());

							if(iono==null) iono = new IonoGps();
							iono.setAlpha(a);

						} else if (typeField2.equals("GPSB")) {

							float b[] = new float[4];

							sub = line.substring(6, 17).replace('D', 'e');
							//Navigation.iono[4] = Double.parseDouble(sub.trim());
							//setIono(4, Double.parseDouble(sub.trim()));
							b[0] = Float.parseFloat(sub.trim());

							sub = line.substring(18, 29).replace('D', 'e');
							//Navigation.iono[5] = Double.parseDouble(sub.trim());
							//setIono(5, Double.parseDouble(sub.trim()));
							b[1] = Float.parseFloat(sub.trim());

							sub = line.substring(30, 41).replace('D', 'e');
							//Navigation.iono[6] = Double.parseDouble(sub.trim());
							//setIono(6, Double.parseDouble(sub.trim()));
							b[2] = Float.parseFloat(sub.trim());

							sub = line.substring(42, 53).replace('D', 'e');
							//Navigation.iono[7] = Double.parseDouble(sub.trim());
							//setIono(7, Double.parseDouble(sub.trim()));
							b[3] = Float.parseFloat(sub.trim());

							if(iono==null) iono = new IonoGps();
							iono.setBeta(b);

						} else if (typeField.equals("END OF HEADER")) {	
							return ver;
						}
						break;

						/* RINEX ver. 3.01 */
					case 3: 

						String typeField3 = line.substring(0, 4);
						typeField3 = typeField3.trim();

						//						String typeField3 = line.substring(60, line.length());
						//						typeField3 = typeField3.trim();

						//						System.out.println(typeField2);


						if (typeField3.equals("GPSA")) {

							//							System.out.println("GPSA");

							float a[] = new float[4];
							sub = line.substring(7, 17).replace('D', 'e');
							//Navigation.iono[0] = Double.parseDouble(sub.trim());
							a[0] = Float.parseFloat(sub.trim());

							sub = line.substring(18, 29).replace('D', 'e');
							//Navigation.iono[1] = Double.parseDouble(sub.trim());
							a[1] = Float.parseFloat(sub.trim());

							sub = line.substring(30, 41).replace('D', 'e');
							//Navigation.iono[2] = Double.parseDouble(sub.trim());
							a[2] = Float.parseFloat(sub.trim());

							sub = line.substring(42, 53).replace('D', 'e');
							//Navigation.iono[3] = Double.parseDouble(sub.trim());
							a[3] = Float.parseFloat(sub.trim());

							if(iono==null) iono = new IonoGps();
							iono.setAlpha(a);
							//							
							//							System.out.println(a[0]);
							//							System.out.println(a[1]);
							//							System.out.println(a[2]);
							//							System.out.println(a[3]);


						} else if (typeField3.equals("GPSB")) {

							//							System.out.println("GPSB");

							float b[] = new float[4];

							sub = line.substring(7, 17).replace('D', 'e');
							//Navigation.iono[4] = Double.parseDouble(sub.trim());
							//setIono(4, Double.parseDouble(sub.trim()));
							b[0] = Float.parseFloat(sub.trim());


							sub = line.substring(18, 29).replace('D', 'e');
							//Navigation.iono[5] = Double.parseDouble(sub.trim());
							//setIono(5, Double.parseDouble(sub.trim()));
							b[1] = Float.parseFloat(sub.trim());

							sub = line.substring(30, 41).replace('D', 'e');
							//Navigation.iono[6] = Double.parseDouble(sub.trim());
							//setIono(6, Double.parseDouble(sub.trim()));
							b[2] = Float.parseFloat(sub.trim());

							sub = line.substring(42, 53).replace('D', 'e');
							//Navigation.iono[7] = Double.parseDouble(sub.trim());
							//setIono(7, Double.parseDouble(sub.trim()));
							b[3] = Float.parseFloat(sub.trim());

							if(iono==null) iono = new IonoGps();
							iono.setBeta(b);

							//							System.out.println(b[0]);
							//							System.out.println(b[1]);
							//							System.out.println(b[2]);
							//							System.out.println(b[3]);				


						} else if (typeField.equals("END OF HEADER")) {	
							//							System.out.println("END OF HEADER");

							return ver;
						}


						break;
					}  // End of Switch 

				} catch (StringIndexOutOfBoundsException e) {
					// Skip over blank lines
				}
			}

			// Display an error if END OF HEADER was not reached
			System.err.println("END OF HEADER was not found in file "
					+ fileNav.toString());
			
		} catch (IOException e) {
			e.printStackTrace();
		}
		return 0;
	}

	/**
	 * Read all navigation data
	 */
	public void parseDataNavV2() {
		try {

			// Resizable array
			//Navigation.eph = new ArrayList<EphGps>();

			//			int j = 0;

			EphGps eph = null;

			while (buffStreamNav.ready()) {

				String sub;
				char satType = 'G'; 				

				eph = new EphGps();
				addEph(eph);
				eph.setSatType(satType);

				// read 8 lines
				for (int i = 0; i < 8; i++) {

					String line = buffStreamNav.readLine();
					if(cacheStreamWriter!=null){
						cacheStreamWriter.write(line);
						cacheStreamWriter.write(newline);
					}

					try {

						int len = line.length();

						if (len != 0) {							

							if (i == 0) { // LINE 1

								//Navigation.eph.get(j).refTime = new Time();


								//Navigation.eph.add(eph);
								//								addEph(eph);

								// Get satellite ID
								sub = line.substring(0, 2).trim();
								eph.setSatID(Integer.parseInt(sub));

								// Get and format date and time string
								String dT = line.substring(2, 22);
								dT = dT.replace("  ", " 0").trim();
								dT = "20" + dT;
								//								System.out.println(dT);


								try {
									//Time timeEph = new Time(dT);
									// Convert String to UNIX standard time in
									// milliseconds
									//timeEph.msec = Time.dateStringToTime(dT);
									Time toc = new Time(dT);
									eph.setRefTime(toc);
									eph.setToc(toc.getGpsWeekSec());

									// sets Iono reference time
									if(iono!=null && iono.getRefTime()==null) iono.setRefTime(new Time(dT));

								} catch (ParseException e) {
									System.err.println("Time parsing failed");
								}

								sub = line.substring(22, 41).replace('D', 'e');
								eph.setAf0(Double.parseDouble(sub.trim()));

								sub = line.substring(41, 60).replace('D', 'e');
								eph.setAf1(Double.parseDouble(sub.trim()));

								sub = line.substring(60, len).replace('D', 'e');
								eph.setAf2(Double.parseDouble(sub.trim()));

							} else if (i == 1) { // LINE 2

								sub = line.substring(3, 22).replace('D', 'e');
								double iode = Double.parseDouble(sub.trim());
								// TODO check double -> int conversion ?
								eph.setIode((int) iode);

								sub = line.substring(22, 41).replace('D', 'e');
								eph.setCrs(Double.parseDouble(sub.trim()));

								sub = line.substring(41, 60).replace('D', 'e');
								eph.setDeltaN(Double.parseDouble(sub.trim()));

								sub = line.substring(60, len).replace('D', 'e');
								eph.setM0(Double.parseDouble(sub.trim()));

							} else if (i == 2) { // LINE 3

								sub = line.substring(0, 22).replace('D', 'e');
								eph.setCuc(Double.parseDouble(sub.trim()));

								sub = line.substring(22, 41).replace('D', 'e');
								eph.setE(Double.parseDouble(sub.trim()));

								sub = line.substring(41, 60).replace('D', 'e');
								eph.setCus(Double.parseDouble(sub .trim()));

								sub = line.substring(60, len).replace('D', 'e');
								eph.setRootA(Double.parseDouble(sub.trim()));

							} else if (i == 3) { // LINE 4

								sub = line.substring(0, 22).replace('D', 'e');
								eph.setToe(Double.parseDouble(sub.trim()));

								sub = line.substring(22, 41).replace('D', 'e');
								eph.setCic(Double.parseDouble(sub.trim()));

								sub = line.substring(41, 60).replace('D', 'e');
								eph.setOmega0(Double.parseDouble(sub.trim()));

								sub = line.substring(60, len).replace('D', 'e');
								eph.setCis(Double.parseDouble(sub.trim()));

							} else if (i == 4) { // LINE 5

								sub = line.substring(0, 22).replace('D', 'e');
								eph.setI0(Double.parseDouble(sub.trim()));

								sub = line.substring(22, 41).replace('D', 'e');
								eph.setCrc(Double.parseDouble(sub.trim()));

								sub = line.substring(41, 60).replace('D', 'e');
								eph.setOmega(Double.parseDouble(sub.trim()));

								sub = line.substring(60, len).replace('D', 'e');
								eph.setOmegaDot(Double.parseDouble(sub.trim()));

							} else if (i == 5) { // LINE 6

								sub = line.substring(0, 22).replace('D', 'e');
								eph.setiDot(Double.parseDouble(sub.trim()));

								sub = line.substring(22, 41).replace('D', 'e');
								double L2Code = Double.parseDouble(sub.trim());
								eph.setL2Code((int) L2Code);

								sub = line.substring(41, 60).replace('D', 'e');
								double week = Double.parseDouble(sub.trim());
								eph.setWeek((int) week);

								sub = line.substring(60, len).replace('D', 'e');
								double L2Flag = Double.parseDouble(sub.trim());
								eph.setL2Flag((int) L2Flag);

							} else if (i == 6) { // LINE 7

								sub = line.substring(0, 22).replace('D', 'e');
								double svAccur = Double.parseDouble(sub.trim());
								eph.setSvAccur((int) svAccur);

								sub = line.substring(22, 41).replace('D', 'e');
								double svHealth = Double.parseDouble(sub.trim());
								eph.setSvHealth((int) svHealth);

								sub = line.substring(41, 60).replace('D', 'e');
								eph.setTgd(Double.parseDouble(sub.trim()));

								sub = line.substring(60, len).replace('D', 'e');
								double iodc = Double.parseDouble(sub.trim());
								eph.setIodc((int) iodc);

							} else if (i == 7) { // LINE 8

								sub = line.substring(0, 22).replace('D', 'e');
								eph.setTom(Double.parseDouble(sub.trim()));

								if (len > 22) {
									sub = line.substring(22, 41).replace('D', 'e');
									eph.setFitInt((long) Double.parseDouble(sub.trim()));

								} else {
									eph.setFitInt(0);
								}
							}
						} else {
							i--;
						}
					} catch (NullPointerException e) {
						// Skip over blank lines
					}
				}

				// Increment array index
				//				j++;
				// Store the number of ephemerides
				//Navigation.n = j;
			}

		} catch (IOException e) {
			e.printStackTrace();
		} catch (NullPointerException e) {
			e.printStackTrace();
		}
	}


	public void parseDataNavV3() {

		try {

			// Resizable array
			//Navigation.eph = new ArrayList<EphGps>();

			//			int j = 0;

			EphGps eph = null;

			while (buffStreamNav.ready()) {

				String sub;
				char satType;

				satType = (char)buffStreamNav.read();
				if(cacheStreamWriter!=null){
					cacheStreamWriter.write(satType);
				}
				//				System.out.println(s);

				if (satType != 'R' && satType != 'S'){  // other than GLONASS and SBAS data
					//						System.out.println(satType);

					// read 8 lines
					for (int i = 0; i < 8; i++) {

						String line = buffStreamNav.readLine();
						if(cacheStreamWriter!=null){
							cacheStreamWriter.write(line);
							cacheStreamWriter.write(newline);
						}

						try {
							int len = line.length();

							if (len != 0) {
								if (i == 0) { // LINE 1

									//Navigation.eph.get(j).refTime = new Time();

									eph = new EphGps();
									//Navigation.eph.add(eph);
									addEph(eph);

									eph.setSatType(satType);

									// Get satellite ID
									sub = line.substring(0, 2).trim();
									//										System.out.println(sub);
									eph.setSatID(Integer.parseInt(sub));

									// Get and format date and time string
									String dT = line.substring(3, 22);
									//								dT = dT.replace("  ", " 0").trim();
									dT = dT + ".0";
									//										System.out.println(dT);

									try {
										//Time timeEph = new Time(dT);
										// Convert String to UNIX standard time in
										// milliseconds
										//timeEph.msec = Time.dateStringToTime(dT);
										Time toc = new Time(dT);
										eph.setRefTime(toc);
										eph.setToc(toc.getGpsWeekSec());

										// sets Iono reference time
										if(iono!=null && iono.getRefTime()==null) iono.setRefTime(new Time(dT));

									} catch (ParseException e) {
										System.err.println("Time parsing failed");
									}

									sub = line.substring(22, 41).replace('D', 'e');
									eph.setAf0(Double.parseDouble(sub.trim()));

									sub = line.substring(41, 60).replace('D', 'e');
									eph.setAf1(Double.parseDouble(sub.trim()));

									sub = line.substring(60, len).replace('D', 'e');
									eph.setAf2(Double.parseDouble(sub.trim()));

								} else if (i == 1) { // LINE 2

									sub = line.substring(4, 23).replace('D', 'e');
									double iode = Double.parseDouble(sub.trim());
									// TODO check double -> int conversion ?
									eph.setIode((int) iode);

									sub = line.substring(23, 42).replace('D', 'e');
									eph.setCrs(Double.parseDouble(sub.trim()));

									sub = line.substring(42, 61).replace('D', 'e');
									eph.setDeltaN(Double.parseDouble(sub.trim()));

									sub = line.substring(61, len).replace('D', 'e');
									eph.setM0(Double.parseDouble(sub.trim()));

								} else if (i == 2) { // LINE 3

									sub = line.substring(4, 23).replace('D', 'e');
									eph.setCuc(Double.parseDouble(sub.trim()));

									sub = line.substring(23, 42).replace('D', 'e');
									eph.setE(Double.parseDouble(sub.trim()));

									sub = line.substring(42, 61).replace('D', 'e');
									eph.setCus(Double.parseDouble(sub .trim()));

									sub = line.substring(61, len).replace('D', 'e');
									eph.setRootA(Double.parseDouble(sub.trim()));

								} else if (i == 3) { // LINE 4

									sub = line.substring(4, 23).replace('D', 'e');
									eph.setToe(Double.parseDouble(sub.trim()));

									sub = line.substring(23, 42).replace('D', 'e');
									eph.setCic(Double.parseDouble(sub.trim()));

									sub = line.substring(42, 61).replace('D', 'e');
									eph.setOmega0(Double.parseDouble(sub.trim()));

									sub = line.substring(61, len).replace('D', 'e');
									eph.setCis(Double.parseDouble(sub.trim()));

								} else if (i == 4) { // LINE 5

									sub = line.substring(4, 23).replace('D', 'e');
									eph.setI0(Double.parseDouble(sub.trim()));

									sub = line.substring(23, 42).replace('D', 'e');
									eph.setCrc(Double.parseDouble(sub.trim()));

									sub = line.substring(42, 61).replace('D', 'e');
									eph.setOmega(Double.parseDouble(sub.trim()));

									sub = line.substring(61, len).replace('D', 'e');
									eph.setOmegaDot(Double.parseDouble(sub.trim()));

								} else if (i == 5) { // LINE 6

									sub = line.substring(4, 23).replace('D', 'e');
									eph.setiDot(Double.parseDouble(sub.trim()));

									sub = line.substring(23, 42).replace('D', 'e');
									double L2Code = Double.parseDouble(sub.trim());
									eph.setL2Code((int) L2Code);

									sub = line.substring(42, 61).replace('D', 'e');
									double week = Double.parseDouble(sub.trim());
									eph.setWeek((int) week);

									sub = line.substring(61, len).replace('D', 'e');
									if (!sub.trim().isEmpty()) {
										double L2Flag = Double.parseDouble(sub.trim());
										eph.setL2Flag((int) L2Flag);
									} else {
										eph.setL2Flag(0);
									}

								} else if (i == 6) { // LINE 7

									sub = line.substring(4, 23).replace('D', 'e');
									double svAccur = Double.parseDouble(sub.trim());
									eph.setSvAccur((int) svAccur);

									sub = line.substring(23, 42).replace('D', 'e');
									double svHealth = Double.parseDouble(sub.trim());
									eph.setSvHealth((int) svHealth);

									sub = line.substring(42, 61).replace('D', 'e');
									eph.setTgd(Double.parseDouble(sub.trim()));

									sub = line.substring(61, len).replace('D', 'e');
									double iodc = Double.parseDouble(sub.trim());
									eph.setIodc((int) iodc);

								} else if (i == 7) { // LINE 8

									sub = line.substring(4, 23).replace('D', 'e');
									eph.setTom(Double.parseDouble(sub.trim()));

									if (line.trim().length() > 22) {
										sub = line.substring(23, 42).replace('D', 'e');
										eph.setFitInt(Long.parseLong(sub.trim()));

									} else {
										eph.setFitInt(0);
									}
								}
							} else {
								i--;
							}


						} catch (NullPointerException e) {
							// Skip over blank lines
						}



					}  // End of for


				} else if (satType == 'R') {   // In case of GLONASS data
					//						System.out.println("satType: " + satType);

					for (int i = 0; i < 4; i++) {
						String line = buffStreamNav.readLine();
						if(cacheStreamWriter!=null){
							cacheStreamWriter.write(line);
							cacheStreamWriter.write(newline);
						}

						try {
							int len = line.length();

							if (len != 0) {	
								if (i == 0) { // LINE 1

									//Navigation.eph.get(j).refTime = new Time();

									eph = new EphGps();
									addEph(eph);

									eph.setSatType(satType);

									// Get satellite ID
									sub = line.substring(0, 2).trim();
									//										System.out.println("ID: "+sub);
									eph.setSatID(Integer.parseInt(sub));

									// Get and format date and time string
									String dT = line.substring(3, 22);
									//								dT = dT.replace("  ", " 0").trim();
									dT = dT + ".0";

									//										System.out.println("dT: " + dT);

									try {
										//Time timeEph = new Time(dT);
										// Convert String to UNIX standard time in
										// milliseconds
										//timeEph.msec = Time.dateStringToTime(dT);


										Time dtoc = new Time(dT);
										eph.setRefTime(dtoc);
										int toc = dtoc.getGpsWeekSec();
										//												System.out.println("toc: " + toc);																		
										eph.setToc(toc);

										int week = dtoc.getGpsWeek();
										//												System.out.println("week: " + week);																		
										eph.setWeek(week);

										double toe = toc;
										//												System.out.printf("%.3f\n", gTime);
										//												System.out.println("timeEph: " + toe);
										eph.setToe(toe);

										// sets Iono reference time
										if(iono!=null && iono.getRefTime()==null) iono.setRefTime(new Time(dT));

									} catch (ParseException e) {
										System.err.println("Time parsing failed");
									}

									/* TauN */ 
									sub = line.substring(22, 41).replace('D', 'e');
									//										System.out.println(sub);
									eph.setTauN(Float.parseFloat(sub.trim()));

									/* GammaN */
									sub = line.substring(41, 60).replace('D', 'e');
									//										System.out.println(sub);
									eph.setGammaN(Float.parseFloat(sub.trim()));

									/* tb */
									sub = line.substring(60, len).replace('D', 'e');
									//										System.out.println("tb: " + sub);

									/* tb is a time interval within the current day (UTC + 3 hours)*/
									double tb = Double.parseDouble(sub.trim());
									double tk = tb - 10800;		
									//										System.out.println("tk: " + tk);
									eph.settk(tk);


									//										eph.settb(Double.parseDouble(sub.trim()));									

								} else if (i == 1) { // LINE 2

									/* X: satellite X coordinate at ephemeris reference time [m] */
									sub = line.substring(4, 23).replace('D', 'e');
									//										System.out.println(sub);
									eph.setX(Double.parseDouble(sub.trim())*1e3);									

									/* Xv: satellite velocity along X at ephemeris reference time [m/s] */
									sub = line.substring(23, 42).replace('D', 'e');
									//										System.out.println(sub);
									eph.setXv(Double.parseDouble(sub.trim())*1e3);									

									/* Xa: acceleration due to lunar-solar gravitational perturbation along X at ephemeris reference time [m/s^2] */
									sub = line.substring(42, 61).replace('D', 'e');
									//										System.out.println(sub);
									eph.setXa(Double.parseDouble(sub.trim())*1e3);									

									/* Bn */
									sub = line.substring(61, len).replace('D', 'e');
									//										System.out.println(sub);
									eph.setBn(Double.parseDouble(sub.trim()));									

								} else if (i == 2) { // LINE 3

									/* Y: satellite Y coordinate at ephemeris reference time [m] */
									sub = line.substring(4, 23).replace('D', 'e');
									//										System.out.println(sub);
									eph.setY(Double.parseDouble(sub.trim())*1e3);

									/* Yv: satellite velocity along Y at ephemeris reference time [m/s] */
									sub = line.substring(23, 42).replace('D', 'e');
									//										System.out.println(sub);
									eph.setYv(Double.parseDouble(sub.trim())*1e3);

									/* Ya: acceleration due to lunar-solar gravitational perturbation along Y at ephemeris reference time [m/s^2] */
									sub = line.substring(42, 61).replace('D', 'e');
									//										System.out.println(sub);
									eph.setYa(Double.parseDouble(sub.trim())*1e3);

									/* freq_num */
									sub = line.substring(61, len).replace('D', 'e');
									//										System.out.println(sub);
									eph.setfreq_num((int) Double.parseDouble(sub.trim()));

								} else if (i == 3) { // LINE 4

									/* Z: satellite Z coordinate at ephemeris reference time [m] */
									sub = line.substring(4, 23).replace('D', 'e');
									//										System.out.println(sub);
									eph.setZ(Double.parseDouble(sub.trim())*1e3);

									/* Zv: satellite velocity along Z at ephemeris reference time [m/s] */
									sub = line.substring(23, 42).replace('D', 'e');
									//										System.out.println(sub);
									eph.setZv(Double.parseDouble(sub.trim())*1e3);

									/* Za: acceleration due to lunar-solar gravitational perturbation along Z at ephemeris reference time [m/s^2]  */
									sub = line.substring(42, 61).replace('D', 'e');
									//										System.out.println(sub);
									eph.setZa(Double.parseDouble(sub.trim())*1e3);

									/* En */
									sub = line.substring(61, len).replace('D', 'e');
									//										System.out.println(sub);
									//										eph.setEn(Long.parseLong(sub.trim()));
									eph.setEn(Double.parseDouble(sub.trim()));


								} // End of if

							} else {
								i--;
							}
							//		}  // End of if


						} catch (NullPointerException e) {
							// Skip over blank lines
						}

					} // End of for

				} else { //SBAS data

					for (int i = 0; i < 4; i++) {
						String line = buffStreamNav.readLine();
						if(cacheStreamWriter!=null){
							cacheStreamWriter.write(line);
							cacheStreamWriter.write(newline);
						}
					}

				}  // End of GLO if


				// Increment array index
				//				j++;
				// Store the number of ephemerides
				//Navigation.n = j;
			} // End of while

		} catch (IOException e) {
			e.printStackTrace();
		} catch (NullPointerException e) {
			e.printStackTrace();
		}


	}

	private double gpsToUnixTime(Time toc, int tow) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * @param unixTime
	 * @param satID
	 * @return Reference ephemeris set for given time and satellite
	 */
	public EphGps findEph(long unixTime, int satID, char satType) {

		long dt = 0;
		long dtMin = 0;
		long dtMax = 0;
		long delta = 0;
		EphGps refEph = null;

		for (int i = 0; i < eph.size(); i++) {
			// Find ephemeris sets for given satellite
			if (eph.get(i).getSatID() == satID && eph.get(i).getSatType() == satType) {
				// Consider BeiDou time (BDT) for BeiDou satellites (14 sec difference wrt GPS time)
				if (satType == 'C') {
					delta = 14000;
					unixTime = unixTime - delta;
				}
				// Compare current time and ephemeris reference time
				dt = Math.abs(eph.get(i).getRefTime().getMsec() - unixTime /*getGpsTime() - gpsTime*/)/1000;

				// If it's the first round, set the minimum time difference and
				// select the first ephemeris set candidate; if the current ephemeris set
				// is closer in time than the previous candidate, select new candidate
//				System.out.println(satID + " " + " " + eph.get(i).getRefTime() + " " + 
//						eph.get(i).getRefTime().getMsec()/1000 + " " + unixTime/1000 + " "+ dt);
				
				if( refEph != null && dt>dtMin)
					break;
				
				if (refEph == null || dt < dtMin) {
					dtMin = dt;
					refEph = eph.get(i);
				}
			}
		}

    if( refEph == null )
      return null;

		if( refEph.getSvHealth() != 0) {
		  return EphGps.UnhealthyEph;
		}

		//maximum allowed interval from ephemeris reference time
		long fitInterval = refEph.getFitInt();
		
		if (fitInterval != 0) {
			dtMax = fitInterval*3600/2; 
		} else {
			switch (refEph.getSatType()) {
  			case 'R': dtMax = 950;
  			case 'J': dtMax = 3600;
  			default: dtMax = 7200;
			}
		}
		if(dtMin > dtMax) {
			refEph = null;
		}

		return refEph;
	}

	public int getEphSize(){
		return eph.size();
	}

	public void addEph(EphGps eph){
		this.eph.add(eph);
	}

	//	public void setIono(int i, double val){
	//		this.iono[i] = val;
	//	}
	public IonoGps getIono(long unixTime){
		return iono;
	}
	//	/**
	//	 * @return the a0
	//	 */
	//	public double getA0() {
	//		return A0;
	//	}
	//	/**
	//	 * @param a0 the a0 to set
	//	 */
	//	public void setA0(double a0) {
	//		A0 = a0;
	//	}
	//	/**
	//	 * @return the a1
	//	 */
	//	public double getA1() {
	//		return A1;
	//	}
	//	/**
	//	 * @param a1 the a1 to set
	//	 */
	//	public void setA1(double a1) {
	//		A1 = a1;
	//	}
	//	/**
	//	 * @return the t
	//	 */
	//	public double getT() {
	//		return T;
	//	}
	//	/**
	//	 * @param t the t to set
	//	 */
	//	public void setT(double t) {
	//		T = t;
	//	}
	//	/**
	//	 * @return the w
	//	 */
	//	public double getW() {
	//		return W;
	//	}
	//	/**
	//	 * @param w the w to set
	//	 */
	//	public void setW(double w) {
	//		W = w;
	//	}
	//	/**
	//	 * @return the leaps
	//	 */
	//	public int getLeaps() {
	//		return leaps;
	//	}
	//	/**
	//	 * @param leaps the leaps to set
	//	 */
	//	public void setLeaps(int leaps) {
	//		this.leaps = leaps;
	//	}

	public boolean isTimestampInEpocsRange(long unixTime){
		return eph.size()>0 &&
				eph.get(0).getRefTime().getMsec() <= unixTime /*&&
		unixTime <= eph.get(eph.size()-1).getRefTime().getMsec() missing interval +epochInterval*/;
	}


	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getGpsSatPosition(long, int, double)
	 */
	@Override
	public SatellitePosition getGpsSatPosition(Observations obs, int satID, char satType, double receiverClockError) {
		long unixTime = obs.getRefTime().getMsec();
		double range = obs.getSatByIDType(satID, satType).getPseudorange(0);
		
		if( range == 0 )
			return null;
					
		EphGps eph = findEph(unixTime, satID, satType);
		if (eph != null) {

	        if( eph.equals( EphGps.UnhealthyEph ))
	            return SatellitePosition.UnhealthySat;

			//			char satType = eph.getSatType();

			SatellitePosition sp = computePositionGps(obs, satID, satType, eph, receiverClockError);
			//			SatellitePosition sp = computePositionGps(unixTime, satType, satID, eph, range, receiverClockError);
			//if(receiverPosition!=null) earthRotationCorrection(receiverPosition, sp);
			return sp;// new SatellitePosition(eph, unixTime, satID, range);
		}
		return null;
	}

  public String getFileName() {
    if( fileNav == null )
      return null;
    else
      return fileNav.getName();
  }
}
