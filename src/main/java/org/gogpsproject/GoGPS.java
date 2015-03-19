/*
 * Copyright (c) 2010, Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
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
package org.gogpsproject;
import java.io.*;
import java.text.DecimalFormat;
import java.util.*;
import java.text.*;

import org.gogpsproject.parser.nvs.NVSFileReader;
import org.gogpsproject.parser.nvs.NVSFileReader2;
import org.gogpsproject.parser.rinex.RinexNavigation;
import org.gogpsproject.parser.rinex.RinexNavigationParser;
import org.gogpsproject.parser.rinex.RinexObservationParser;
import org.gogpsproject.parser.rtcm3.RTCM3Client;
import org.gogpsproject.parser.sp3.SP3Navigation;
import org.gogpsproject.parser.ublox.UBXSerialConnection;
import org.gogpsproject.parser.ublox.UBXFileReader;

// TODO: Auto-generated Javadoc
/**
 * The Class GoGPS.
 *
 * @author Eugenio Realini, Cryms.com
 */
public class GoGPS implements Runnable{

	// Frequency selector
	/** The Constant FREQ_L1. */
	public final static int FREQ_L1 = ObservationSet.L1;

	/** The Constant FREQ_L2. */
	public final static int FREQ_L2 = ObservationSet.L2;

	/** The freq. */
	private int freq = FREQ_L1;

	// Double-frequency flag
	/** The dual freq. */
	private boolean dualFreq = false;

	// Weighting strategy
	// 0 = same weight for all observations
	// 1 = weight based on satellite elevation
	// 2 = weight based on signal-to-noise ratio
	// 3 = weight based on combined elevation and signal-to-noise ratio
	/** The Constant WEIGHT_EQUAL. */
	public final static int WEIGHT_EQUAL = 0;

	/** The Constant WEIGHT_SAT_ELEVATION. */
	public final static int WEIGHT_SAT_ELEVATION = 1;

	/** The Constant WEIGHT_SIGNAL_TO_NOISE_RATIO. */
	public final static int WEIGHT_SIGNAL_TO_NOISE_RATIO = 2;

	/** The Constant WEIGHT_COMBINED_ELEVATION_SNR. */
	public final static int WEIGHT_COMBINED_ELEVATION_SNR = 3;

	/** The weights. */
	private int weights = WEIGHT_SAT_ELEVATION;


	/** The Constant DYN_MODEL_STATIC. */
	public final static int DYN_MODEL_STATIC = 1;

	/** The Constant DYN_MODEL_CONST_SPEED. */
	public final static int DYN_MODEL_CONST_SPEED = 2;

	/** The Constant DYN_MODEL_CONST_ACCELERATION. */
	public final static int DYN_MODEL_CONST_ACCELERATION = 3;
	// Kalman filter parameters
	/** The dynamic model. */
	private int dynamicModel = DYN_MODEL_CONST_SPEED;

	/** The st dev init. */
	private double stDevInit = 3;

	/** The st dev e. */
	private double stDevE = 0.5;

	/** The st dev n. */
	private double stDevN = 0.5;

	/** The st dev u. */
	private double stDevU = 0.1;

	/** The st dev code c. */
	private double stDevCodeC = 3;

	/** The st dev code p. */
	private double[] stDevCodeP;

	/** The st dev phase. */
	private double stDevPhase = 0.03;

	/** The st dev ambiguity. */
	private double stDevAmbiguity = 10;

	/** The min num sat. */
	private int minNumSat = 2;

	/** The cycle slip threshold. */
	private double cycleSlipThreshold = 3;

	/** The Constant AMBIGUITY_OBSERV. */
	public final static int AMBIGUITY_OBSERV = 0;

	/** The Constant AMBIGUITY_APPROX. */
	public final static int AMBIGUITY_APPROX = 1;

	/** The Constant AMBIGUITY_LS. */
	public final static int AMBIGUITY_LS = 2;

	/** The ambiguity strategy. */
	private int ambiguityStrategy = AMBIGUITY_APPROX;

	/** The Elevation cutoff. */
	private double cutoff = 15; // Elevation cutoff

	public final static int RUN_MODE_STANDALONE = 0;
	public final static int RUN_MODE_DOUBLE_DIFF = 1;
	public final static int RUN_MODE_KALMAN_FILTER = 2;

	private int runMode = -1;
	private Thread runThread=null;

	/** The navigation. */
	private NavigationProducer navigation;

	/** The rover in. */
	private ObservationsProducer roverIn;

	/** The master in. */
	private ObservationsProducer masterIn;

	/** The rover calculated position */
	private ReceiverPosition roverPos = null;

	/** The rover calculated position is valid */
	private boolean validPosition = false;

	private Vector<PositionConsumer> positionConsumers = new Vector<PositionConsumer>();


//	private boolean debug = false;
	private boolean debug = true;

	/**
	 * Instantiates a new go gps.
	 *
	 * @param navigation the navigation
	 * @param roverIn the rover in
	 * @param masterIn the master in
	 */
	public GoGPS(NavigationProducer navigation, ObservationsProducer roverIn, ObservationsProducer masterIn){

		stDevCodeP = new double[2];
		stDevCodeP[0] = 0.6;
		stDevCodeP[1] = 0.4;

		this.navigation = navigation;
		this.roverIn = roverIn;
		this.masterIn = masterIn;

		validPosition = false;
	}
	
	public GoGPS(NavigationProducer navigation, ObservationsProducer roverIn){

		stDevCodeP = new double[2];
		stDevCodeP[0] = 0.6;
		stDevCodeP[1] = 0.4;

		this.navigation = navigation;
		this.roverIn = roverIn;

		validPosition = false;
	}

	/**
	 * Run code standalone.
	 */
	public void runCodeStandalone() {

		try {
			runCodeStandalone(-1);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Run code standalone.
	 *
	 * @param getNthPosition the get nth position
	 * @return the coordinates
	 * @throws Exception
	 */
	public RoverPosition runCodeStandalone(double stopAtDopThreshold) throws Exception {

		
//		GoGPS goGPS = new GoGPS(navigation, roverIn);
//		roverPos = new ReceiverPosition(goGPS);

		// Create a new object for the rover position
		roverPos = new ReceiverPosition(this);


		try {
			Observations obsR = roverIn.getNextObservations();
			while (obsR!=null) { // buffStreamObs.ready()
//				if(debug) System.out.println("OK ");

				//try{
					// If there are at least four satellites
					if (obsR.getNumSat() >= 4) { // gps.length
						if(debug) System.out.println("OK "+obsR.getNumSat()+" satellites");

						// Compute approximate positioning by Bancroft algorithm
						roverPos.bancroft(obsR);

						// If an approximate position was computed
						if(debug) System.out.println("Valid Bancroft position? "+roverPos.isValidXYZ()+" x:"+roverPos.getX()+" y:"+roverPos.getY()+" z:"+roverPos.getZ());
						
						if (roverPos.isValidXYZ()) {
							// Select available satellites
							roverPos.selectSatellitesStandalone(obsR);
							
							if (roverPos.getSatAvailNumber() >= 4){
								if(debug) System.out.println("# Num. of selected satllites: " + roverPos.getSatAvailNumber());
								// Compute code stand-alone positioning (epoch-by-epoch solution)
								roverPos.codeStandalone(obsR, false);
							}
							else
								// Discard Bancroft positioning
								roverPos.setXYZ(0, 0, 0);
						}

						if(debug)System.out.println("Valid LS position? "+roverPos.isValidXYZ()+" x:"+roverPos.getX()+" y:"+roverPos.getY()+" z:"+roverPos.getZ());
						if (roverPos.isValidXYZ()) {
							if(!validPosition){
								notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
								validPosition = true;
							}else{
								RoverPosition coord = new RoverPosition(roverPos, RoverPosition.DOP_TYPE_STANDARD, roverPos.getpDop(), roverPos.gethDop(), roverPos.getvDop());

								if(positionConsumers.size()>0){
									coord.setRefTime(new Time(obsR.getRefTime().getMsec()));
									notifyPositionConsumerAddCoordinate(coord);
								}
								if(debug)System.out.println("-------------------- "+roverPos.getpDop());
								if(stopAtDopThreshold>0.0 && roverPos.getpDop()<stopAtDopThreshold){
									return coord;
								}
							}
						}
					}
//				}catch(Exception e){
//					System.out.println("Could not complete due to "+e);
//					e.printStackTrace();
//				}
				obsR = roverIn.getNextObservations();
			}
		} catch (Exception e) {
			e.printStackTrace();
			throw e;
		} finally {
			notifyPositionConsumerEvent(PositionConsumer.EVENT_END_OF_TRACK);
		}
		return null;
	}

	/**
	 * Run code double differences.
	 */
	public void runCodeDoubleDifferences() {

		// Create a new object for the rover position
		roverPos = new ReceiverPosition(this);

		try {
			Observations obsR = roverIn.getNextObservations();
			Observations obsM = masterIn.getNextObservations();

			while (obsR != null && obsM != null) {

				// Discard master epochs if correspondent rover epochs are
				// not available
				double obsRtime = obsR.getRefTime().getRoundedGpsTime();
				while (obsM!=null && obsR!=null && obsRtime > obsM.getRefTime().getRoundedGpsTime()) {
					obsM = masterIn.getNextObservations();
				}

				// Discard rover epochs if correspondent master epochs are
				// not available
				double obsMtime = obsM.getRefTime().getRoundedGpsTime();
				while (obsM!=null && obsR!=null && obsR.getRefTime().getRoundedGpsTime() < obsMtime) {
					obsR = roverIn.getNextObservations();
				}


				// If there are at least four satellites
				if (obsM!=null && obsR!=null){
					if(obsR.getNumSat() >= 4) {

						// Compute approximate positioning by Bancroft algorithm
						roverPos.bancroft(obsR);

						// If an approximate position was computed
						if (roverPos.isValidXYZ()) {

							// Select satellites available for double differences
							roverPos.selectSatellitesDoubleDiff(obsR,
									obsM, masterIn.getDefinedPosition());

							if (roverPos.getSatAvailNumber() >= 4)
								// Compute code double differences positioning
								// (epoch-by-epoch solution)
								roverPos.codeDoubleDifferences(obsR,
										obsM, masterIn.getDefinedPosition());
							else
								// Discard Bancroft positioning
								roverPos.setXYZ(0, 0, 0);
						}

						if (roverPos.isValidXYZ()) {
							if(!validPosition){
								notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
								validPosition = true;
							}else{
								RoverPosition coord = new RoverPosition(roverPos, RoverPosition.DOP_TYPE_STANDARD, roverPos.getpDop(), roverPos.gethDop(), roverPos.getvDop());

								if(positionConsumers.size()>0){
									coord.setRefTime(new Time(obsR.getRefTime().getMsec()));
									notifyPositionConsumerAddCoordinate(coord);
								}
								if(debug)System.out.println("-------------------- "+roverPos.getpDop());
							}
						}
					}
				}
				// get next epoch
				obsR = roverIn.getNextObservations();
				obsM = masterIn.getNextObservations();
			}
		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			notifyPositionConsumerEvent(PositionConsumer.EVENT_END_OF_TRACK);
		}
	}

	/**
	 * Run kalman filter.
	 */
	public void runKalmanFilter() {

		long timeRead = System.currentTimeMillis();
		long depRead = 0;

		long timeProc = 0;
		long depProc = 0;

		// Create a new object for the rover position
		roverPos = new ReceiverPosition(this);

		// Flag to check if Kalman filter has been initialized
		boolean kalmanInitialized = false;

		try {

			timeRead = System.currentTimeMillis() - timeRead;
			depRead = depRead + timeRead;

			Observations obsR = roverIn.getNextObservations();
			Observations obsM = masterIn.getNextObservations();

			while (obsR != null && obsM != null) {
//				System.out.println("obsR: " + obsR);

				
				if(debug)System.out.println("R:"+obsR.getRefTime().getMsec()+" M:"+obsM.getRefTime().getMsec());

				timeRead = System.currentTimeMillis();

				// Discard master epochs if correspondent rover epochs are
				// not available
//				Observations obsR = roverIn.nextObservations();
//				Observations obsM = masterIn.nextObservations();
				double obsRtime = obsR.getRefTime().getRoundedGpsTime();
				System.out.println("look for M "+obsRtime);
//				System.out.println("obsM_Time: " + obsM.getRefTime().getRoundedGpsTime());

				while (obsM!=null && obsR!=null && obsRtime > obsM.getRefTime().getRoundedGpsTime()) {
					
//					masterIn.skipDataObs();
//					masterIn.parseEpochObs();
					obsM = masterIn.getNextObservations();
					System.out.println("while obsM: " + obsM);
				}
//				System.out.println("found M "+obsRtime);

				// Discard rover epochs if correspondent master epochs are
				// not available
				double obsMtime = obsM.getRefTime().getRoundedGpsTime();
				System.out.println("##look for R "+obsMtime);
			
				while (obsM!=null && obsR!=null && obsR.getRefTime().getRoundedGpsTime() < obsMtime) {
					System.out.println("obsR_Time: " + obsR.getRefTime().getGpsTime() );
					
					obsR = roverIn.getNextObservations();
				}
//				System.out.println("found R "+obsMtime);

				System.out.println("obsM: " + obsM);
				System.out.println("obsR: " + obsR);


				if(obsM!=null && obsR!=null){
					timeRead = System.currentTimeMillis() - timeRead;
					depRead = depRead + timeRead;
					timeProc = System.currentTimeMillis();
//					System.out.println("Check!!");

					
					// If Kalman filter was not initialized and if there are at
					// least four satellites
					boolean valid = true;
					if (!kalmanInitialized && obsR.getNumSat() >= 4) {

						if(debug)System.out.print("Try to init with bancroft ");

						// Compute approximate positioning by Bancroft algorithm
						roverPos.bancroft(obsR);

						// If an approximate position was computed
						if (roverPos.isValidXYZ()) {

							// Initialize Kalman filter
							roverPos.kalmanFilterInit(obsR, obsM, masterIn.getDefinedPosition());

							if (roverPos.isValidXYZ())
								kalmanInitialized = true;

							if(debug)System.out.println("OK");
						}else{
							if(debug)System.out.println("....nope");
						}
					} else if (kalmanInitialized) {

						// Do a Kalman filter loop
						try{
							roverPos.kalmanFilterLoop(obsR,obsM, masterIn.getDefinedPosition());
						}catch(Exception e){
							e.printStackTrace();
							valid = false;
						}
					}

					timeProc = System.currentTimeMillis() - timeProc;
					depProc = depProc + timeProc;

					if(kalmanInitialized && valid){
						if(!validPosition){
							notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
							validPosition = true;
						}else
						if(positionConsumers.size()>0){
							RoverPosition coord = new RoverPosition(roverPos, RoverPosition.DOP_TYPE_KALMAN, roverPos.getKpDop(), roverPos.getKhDop(), roverPos.getKvDop());
							coord.setRefTime(new Time(obsR.getRefTime().getMsec()));
							notifyPositionConsumerAddCoordinate(coord);
						}

					}
					//System.out.println("--------------------");

					if(debug)System.out.println("-- Get next epoch ---------------------------------------------------");
					// get next epoch
					obsR = roverIn.getNextObservations();
					obsM = masterIn.getNextObservations();

				}else{
					if(debug)System.out.println("Missing M or R obs ");
				}
			}

		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			notifyPositionConsumerEvent(PositionConsumer.EVENT_END_OF_TRACK);
		}

		int elapsedTimeSec = (int) Math.floor(depRead / 1000);
		int elapsedTimeMillisec = (int) (depRead - elapsedTimeSec * 1000);
		if(debug)System.out.println("\nElapsed time (read): " + elapsedTimeSec
				+ " seconds " + elapsedTimeMillisec + " milliseconds.");

		elapsedTimeSec = (int) Math.floor(depProc / 1000);
		elapsedTimeMillisec = (int) (depProc - elapsedTimeSec * 1000);
		if(debug)System.out.println("\nElapsed time (proc): " + elapsedTimeSec
				+ " seconds " + elapsedTimeMillisec + " milliseconds.");

	}


	/**
	 * Gets the freq.
	 *
	 * @return the freq
	 */
	public int getFreq() {
		return freq;
	}

	/**
	 * Sets the freq.
	 *
	 * @param freq the freq to set
	 */
	public void setFreq(int freq) {
		this.freq = freq;
	}

	/**
	 * Checks if is dual freq.
	 *
	 * @return the dualFreq
	 */
	public boolean isDualFreq() {
		return dualFreq;
	}

	/**
	 * Sets the dual freq.
	 *
	 * @param dualFreq the dualFreq to set
	 */
	public void setDualFreq(boolean dualFreq) {
		this.dualFreq = dualFreq;
	}

	/**
	 * Gets the cutoff.
	 *
	 * @return the cutoff
	 */
	public double getCutoff() {
		return cutoff;
	}

	/**
	 * Sets the cutoff.
	 *
	 * @param cutoff the cutoff to set
	 */
	public void setCutoff(double cutoff) {
		this.cutoff = cutoff;
	}

	/**
	 * Gets the order.
	 *
	 * @return the order
	 */
	public int getOrder() {
		return dynamicModel;
	}

	/**
	 * Sets the order.
	 *
	 * @param order the order to set
	 */
	public void setOrder(int order) {
		this.dynamicModel = order;
	}

	/**
	 * Gets the st dev init.
	 *
	 * @return the stDevInit
	 */
	public double getStDevInit() {
		return stDevInit;
	}

	/**
	 * Sets the st dev init.
	 *
	 * @param stDevInit the stDevInit to set
	 */
	public void setStDevInit(double stDevInit) {
		this.stDevInit = stDevInit;
	}

	/**
	 * Gets the st dev e.
	 *
	 * @return the stDevE
	 */
	public double getStDevE() {
		return stDevE;
	}

	/**
	 * Sets the st dev e.
	 *
	 * @param stDevE the stDevE to set
	 */
	public void setStDevE(double stDevE) {
		this.stDevE = stDevE;
	}

	/**
	 * Gets the st dev n.
	 *
	 * @return the stDevN
	 */
	public double getStDevN() {
		return stDevN;
	}

	/**
	 * Sets the st dev n.
	 *
	 * @param stDevN the stDevN to set
	 */
	public void setStDevN(double stDevN) {
		this.stDevN = stDevN;
	}

	/**
	 * Gets the st dev u.
	 *
	 * @return the stDevU
	 */
	public double getStDevU() {
		return stDevU;
	}

	/**
	 * Sets the st dev u.
	 *
	 * @param stDevU the stDevU to set
	 */
	public void setStDevU(double stDevU) {
		this.stDevU = stDevU;
	}

	/**
	 * Gets the st dev code.
	 *
	 * @param roverObsSet the rover observation set
	 * @param masterObsSet the master observation set
	 * @param i the selected GPS frequency
	 * @return the stDevCode
	 */
	public double getStDevCode(ObservationSet obsSet, int i) {
		return obsSet.isPseudorangeP(i)?stDevCodeP[i]:stDevCodeC;
	}

	/**
	 * Gets the st dev code c.
	 *
	 * @return the stDevCodeC
	 */
	public double getStDevCodeC() {
		return stDevCodeC;
	}

	/**
	 * Sets the st dev code c.
	 *
	 * @param stDevCodeC the stDevCodeC to set
	 */
	public void setStDevCodeC(double stDevCodeC) {
		this.stDevCodeC = stDevCodeC;
	}

	/**
	 * Gets the st dev code p.
	 *
	 * @param i the selected GPS frequency
	 * @return the stDevCodeP
	 */
	public double getStDevCodeP(int i) {
		return stDevCodeP[i];
	}

	/**
	 * Sets the st dev code p.
	 *
	 * @param stDevCodeP the stDevCodeP to set
	 * @param i the selected GPS frequency
	 */
	public void setStDevCodeP(double stDevCodeP, int i) {
		this.stDevCodeP[i] = stDevCodeP;
	}

	/**
	 * Gets the st dev phase.
	 *
	 * @return the stDevPhase
	 */
	public double getStDevPhase() {
		return stDevPhase;
	}

	/**
	 * Sets the st dev phase.
	 *
	 * @param stDevPhase the stDevPhase to set
	 */
	public void setStDevPhase(double stDevPhase) {
		this.stDevPhase = stDevPhase;
	}

	/**
	 * Gets the st dev ambiguity.
	 *
	 * @return the stDevAmbiguity
	 */
	public double getStDevAmbiguity() {
		return stDevAmbiguity;
	}

	/**
	 * Sets the st dev ambiguity.
	 *
	 * @param stDevAmbiguity the stDevAmbiguity to set
	 */
	public void setStDevAmbiguity(double stDevAmbiguity) {
		this.stDevAmbiguity = stDevAmbiguity;
	}

	/**
	 * Gets the min num sat.
	 *
	 * @return the minNumSat
	 */
	public int getMinNumSat() {
		return minNumSat;
	}

	/**
	 * Sets the min num sat.
	 *
	 * @param minNumSat the minNumSat to set
	 */
	public void setMinNumSat(int minNumSat) {
		this.minNumSat = minNumSat;
	}

	/**
	 * Gets the cycle slip threshold.
	 *
	 * @return the cycle slip threshold
	 */
	public double getCycleSlipThreshold() {
		return cycleSlipThreshold;
	}

	/**
	 * Sets the cycle slip threshold.
	 *
	 * @param csThreshold the cycle slip threshold to set
	 */
	public void setCycleSlipThreshold(double csThreshold) {
		this.cycleSlipThreshold = csThreshold;
	}

	/**
	 * Gets the navigation.
	 *
	 * @return the navigation
	 */
	public NavigationProducer getNavigation() {
		return navigation;
	}

	/**
	 * Sets the navigation.
	 *
	 * @param navigation the navigation to set
	 */
	public void setNavigation(NavigationProducer navigation) {
		this.navigation = navigation;
	}

	/**
	 * Gets the weights.
	 *
	 * @return the weights
	 */
	public int getWeights() {
		return weights;
	}

	/**
	 * Sets the weights.
	 *
	 * @param weights the weights to set
	 */
	public void setWeights(int weights) {
		this.weights = weights;
	}

	/**
	 * Gets the dynamic model.
	 *
	 * @return the dynamicModel
	 */
	public int getDynamicModel() {
		return dynamicModel;
	}

	/**
	 * Sets the dynamic model.
	 *
	 * @param dynamicModel the dynamicModel to set
	 */
	public void setDynamicModel(int dynamicModel) {
		this.dynamicModel = dynamicModel;
	}

	/**
	 * Gets the ambiguity strategy.
	 *
	 * @return the ambiguityStrategy
	 */
	public int getAmbiguityStrategy() {
		return ambiguityStrategy;
	}

	/**
	 * Sets the ambiguity strategy.
	 *
	 * @param ambiguityStrategy the ambiguityStrategy to set
	 */
	public void setAmbiguityStrategy(int ambiguityStrategy) {
		this.ambiguityStrategy = ambiguityStrategy;
	}

	/**
	 * @return the validPosition
	 */
	public boolean isValidPosition() {
		return validPosition;
	}

	/**
	 * @return the roverPos
	 */
	public Coordinates getRoverPos() {
		return (Coordinates)roverPos.clone();
	}

	/**
	 * @return the positionConsumer
	 */
	public void cleanPositionConsumers() {
		positionConsumers.clear();
	}
	public void removePositionConsumer(PositionConsumer positionConsumer) {
		positionConsumers.remove(positionConsumer);
	}
	/**
	 * @param positionConsumer the positionConsumer to add
	 */
	public void addPositionConsumerListener(PositionConsumer positionConsumer) {
		this.positionConsumers.add(positionConsumer);
	}

	private void notifyPositionConsumerEvent(int event){
		for(PositionConsumer pc:positionConsumers){
			try{
				pc.event(event);
			}catch(Exception e){
				e.printStackTrace();
			}
		}
	}
	private void notifyPositionConsumerAddCoordinate(RoverPosition coord){
		for(PositionConsumer pc:positionConsumers){
			try{
				pc.addCoordinate(coord);
			}catch(Exception e){
				e.printStackTrace();
			}
		}
	}

	/**
	 * @return the runMode
	 */
	public int getRunMode() {
		return runMode;
	}

	/**
	 * @param runMode the run mode to use
	 */
	public void runThreadMode(int runMode) {
		this.runMode = runMode;
		this.runThread = new Thread(this);
		switch(runMode){
			case RUN_MODE_STANDALONE:
				this.runThread.setName("goGPS standalone");
				runCodeStandalone();
				break;
			case RUN_MODE_DOUBLE_DIFF:
				this.runThread.setName("goGPS double difference");
				break;
			case RUN_MODE_KALMAN_FILTER:
				this.runThread.setName("goGPS Kalman filter");
				break;
		}
		this.runThread.start();
	}

	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		if(this.runMode<0) return;

		switch(runMode){
			case RUN_MODE_STANDALONE:
				runCodeStandalone();
				break;
			case RUN_MODE_DOUBLE_DIFF:
				runCodeDoubleDifferences();
				break;
			case RUN_MODE_KALMAN_FILTER:
				runKalmanFilter();
				break;
		}

		notifyPositionConsumerEvent(PositionConsumer.EVENT_GOGPS_THREAD_ENDED);
	}

	/**
	 * @param debug the debug to set
	 */
	public void setDebug(boolean debug) {
		this.debug = debug;
	}

	/**
	 * @return the debug
	 */
	public boolean isDebug() {
		return debug;
	}
}
