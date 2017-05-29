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

import java.util.*;

import org.gogpsproject.consumer.PositionConsumer;
import org.gogpsproject.positioning.Coordinates;
import org.gogpsproject.positioning.KF_DD_code_phase;
import org.gogpsproject.positioning.KF_SA_code_phase;
import org.gogpsproject.positioning.KalmanFilter;
import org.gogpsproject.positioning.LS_DD_code;
import org.gogpsproject.positioning.LS_SA_code;
import org.gogpsproject.positioning.LS_SA_code_coarse_time;
import org.gogpsproject.positioning.LS_SA_code_snapshot;
import org.gogpsproject.positioning.LS_SA_dopplerPos;
import org.gogpsproject.positioning.MasterPosition;
import org.gogpsproject.positioning.RoverPosition;
import org.gogpsproject.positioning.Satellites;
import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.ObservationsProducer;

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
	private double stDevInit = 1;

	/** The st dev e. */
	private double stDevE = 0.5;

	/** The st dev n. */
	private double stDevN = 0.5;

	/** The st dev u. */
	private double stDevU = 0.1;

	/** The st dev code c. */
	private double stDevCodeC = 0.3;

	/** The st dev code p. */
	private double[] stDevCodeP;

	/** The st dev phase. */
	private double stDevPhase = 0.003;

	/** The st dev ambiguity. */
	private double stDevAmbiguity = 10;

	/** The min num sat. */
	private int minNumSat = 2;

	/** The cycle slip threshold. */
	private double cycleSlipThreshold = 1;
	
	/** The Constant APPROX_PSEUDORANGE. */
	public final static int APPROX_PSEUDORANGE = 0;

	/** The Constant DOPPLER_PREDICTED_PHASE_RANGE. */
	public final static int DOPPLER_PREDICTED_PHASE_RANGE = 1;
	
	/** The cycle-slip detection strategy. */
	private int cycleSlipDetectionStrategy = APPROX_PSEUDORANGE;

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
	public final static int RUN_MODE_KALMAN_FILTER_STANDALONE = 2;
	public final static int RUN_MODE_KALMAN_FILTER_DOUBLE_DIFF = 3;
  public final static int RUN_MODE_STANDALONE_SNAPSHOT = 10;
  public final static int RUN_MODE_STANDALONE_COARSETIME = 11;

	private int runMode = -1;
	
	private Thread runThread=null;
	private boolean running = false;
	
	/** The navigation. */
	private NavigationProducer navigation;

	/** The rover in. */
	private ObservationsProducer roverIn;

	/** The master in. */
	private ObservationsProducer masterIn;

	/** The rover calculated position */
	private final RoverPosition roverPos;

	 /** The master position */
  private final MasterPosition masterPos;

  /** Satellite State Information */
  private final Satellites satellites;
  
	/** The rover calculated position is valid */
	private boolean validPosition = false;

	/** coarse time error */
  private long offsetms = 0;

	private Vector<PositionConsumer> positionConsumers = new Vector<PositionConsumer>();


//	private boolean debug = false;
	private boolean debug = true;

	private boolean useDTM = false;
	
  public final static double MODULO1MS  = Constants.SPEED_OF_LIGHT /1000;     // check 1ms bit slip
  public final static double MODULO20MS = Constants.SPEED_OF_LIGHT * 20/1000; // check 20ms bit slip

  /** print additional debug information if the true position is known */
  public RoverPosition truePos;

  /** max position update for a valid fix */
  private long posLimit = 100000; // m

  /** max height for a valid fix */
  private long maxHeight = 10000;

  /** max hdop for a valid fix */
  private double hdopLimit = 20.0;

  /** max residual error to exclude a given range */
  private double residThreshold = 3.0;

  /** pos update limit for LMS iterations */
  final double POS_TOL = 1.0;    // meters
  
  /** time update limit for LMS iterations */
  final double TG_TOL = 1;  // milliseconds

	public RoverPosition getRoverPos(){
    return roverPos;
  }

  public GoGPS setRoverPos(Coordinates roverPos) {
    roverPos.cloneInto(this.roverPos);
    return this;
  }

  public MasterPosition getMasterPos(){
    return masterPos;
  }

  public GoGPS setMasterPos(Coordinates masterPos) {
    masterPos.cloneInto(this.masterPos);
    return this;
  }

  public Satellites getSats(){
    return satellites;
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
   * @return 
   */
  public GoGPS setFreq(int freq) {
  	this.freq = freq;
  	return this;
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
   * @return 
   */
  public GoGPS setDualFreq(boolean dualFreq) {
  	this.dualFreq = dualFreq;
    return this;
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
  public GoGPS setCutoff(double cutoff) {
  	this.cutoff = cutoff;
  	return this;
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
   * @return 
   */
  public GoGPS setOrder(int order) {
  	this.dynamicModel = order;
    return this;
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
   * @return 
   */
  public GoGPS setStDevInit(double stDevInit) {
  	this.stDevInit = stDevInit;
    return this;
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
   * @return 
   */
  public GoGPS setStDevE(double stDevE) {
  	this.stDevE = stDevE;
    return this;
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
   * @return 
   */
  public GoGPS setStDevN(double stDevN) {
  	this.stDevN = stDevN;
    return this;
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
   * @return 
   */
  public GoGPS setStDevU(double stDevU) {
  	this.stDevU = stDevU;
    return this;
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
   * @return 
   */
  public GoGPS setStDevCodeC(double stDevCodeC) {
  	this.stDevCodeC = stDevCodeC;
    return this;
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
   * @return 
   */
  public GoGPS setStDevCodeP(double stDevCodeP, int i) {
  	this.stDevCodeP[i] = stDevCodeP;
    return this;
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
   * @return 
   */
  public GoGPS setStDevPhase(double stDevPhase) {
  	this.stDevPhase = stDevPhase;
    return this;
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
   * @return 
   */
  public GoGPS setStDevAmbiguity(double stDevAmbiguity) {
  	this.stDevAmbiguity = stDevAmbiguity;
    return this;
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
   * @return 
   */
  public GoGPS setMinNumSat(int minNumSat) {
  	this.minNumSat = minNumSat;
    return this;
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
   * @return 
   */
  public GoGPS setCycleSlipThreshold(double csThreshold) {
  	this.cycleSlipThreshold = csThreshold;
    return this;
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
   * @return 
   */
  public GoGPS setNavigation(NavigationProducer navigation) {
  	this.navigation = navigation;
    return this;
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
   * @return 
   */
  public GoGPS setWeights(int weights) {
  	this.weights = weights;
    return this;
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
  public GoGPS setDynamicModel(int dynamicModel) {
  	this.dynamicModel = dynamicModel;
  	return this;
  }

  /**
   * Gets the cycle-slip detection strategy.
   *
   * @return the cycleSlipDetectionStrategy
   */
  public int getCycleSlipDetectionStrategy() {
  	return cycleSlipDetectionStrategy;
  }

  /**
   * Sets the cycle-slip detection strategy.
   *
   * @param cycleSlipDetectionStrategy the cycleSlipDetectionStrategy to set
   * @return 
   */
  public GoGPS setCycleSlipDetection(int cycleSlipDetectionStrategy) {
  	this.cycleSlipDetectionStrategy = cycleSlipDetectionStrategy;
    return this;
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
   * @return 
   */
  public GoGPS setAmbiguityStrategy(int ambiguityStrategy) {
  	this.ambiguityStrategy = ambiguityStrategy;
    return this;
  }

  /**
   * @return the validPosition
   */
  public boolean isValidPosition() {
  	return validPosition;
  }

  /**
   * @return the debug
   */
  public boolean isDebug() {
  	return debug;
  }

  /**
   * @param debug the debug to set
   * @return 
   */
  public GoGPS setDebug(boolean debug) {
  	this.debug = debug;
    return this;
  }

  public boolean useDTM(){
    return useDTM;
  }

  public GoGPS useDTM( boolean useDTM ){
    this.useDTM = useDTM;
    return this;
  }

  public long getMaxHeight() {
    return maxHeight;
  }

  public GoGPS setMaxHeight(long maxHeight) {
    this.maxHeight = maxHeight;
    return this;
  }

  public double getResidThreshold(){
    return this.residThreshold;
  }

  public GoGPS setResidThreshold(double residThreshold) {
    this.residThreshold = residThreshold;
    return this;
  }

  public double getHdopLimit(){
    return hdopLimit;
  }

  public GoGPS setHdopLimit(double hdopLimit) {
    this.hdopLimit  = hdopLimit;
    return this;
  }

  public long getPosLimit(){
    return posLimit;
  }

  public GoGPS setPosLimit(long maxPosUpdate) {
    this.posLimit  = maxPosUpdate;
    return this;
  }

  public long getOffssetMs(){
    return offsetms;
  }

  public GoGPS setOffsetMs( long offsetms){
    this.offsetms = offsetms;
    return this;
  }

  /**
	 * Instantiates a new GoGPS.
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
		
    roverPos = new RoverPosition();
    masterPos = new MasterPosition();
    satellites = new Satellites(this);
	}
	
	public GoGPS(NavigationProducer navigation, ObservationsProducer roverIn){
	  this(navigation,roverIn,null);
	}

	/**
	 * Run code standalone.
	 */
	public GoGPS runCodeStandalone() {
		try {
			return runCodeStandalone(-1);
		} catch (Exception e) {
			e.printStackTrace();
			return this;
		}
	}

	/**
	 * Run code standalone.
	 *
	 * @param getNthPosition the get nth position
	 * @return the coordinates
	 * @throws Exception
	 */
	public GoGPS runCodeStandalone(double stopAtDopThreshold) throws Exception {

    running = true;
		LS_SA_code sa = new LS_SA_code(this);
		
		RoverPosition coord = null;
		try {
			Observations obsR = roverIn.getNextObservations();
			while( obsR!=null && running ) { // buffStreamObs.ready()
//				if(debug) System.out.println("OK ");

				//try{
					// If there are at least four satellites
					if (obsR.getNumSat() >= 4) { // gps.length
						if(debug) System.out.println("Total number of satellites: "+obsR.getNumSat());

						// Compute approximate positioning by iterative least-squares
            if (!roverPos.isValidXYZ()) {
  						for (int iter = 0; iter < 3; iter++) {
  							// Select all satellites
  							satellites.selectStandalone( obsR, -100);
  							
  							if (satellites.getAvailNumber() >= 4) {
  								sa.codeStandalone( obsR, false, true);
  							}
  						}

						// If an approximate position was computed
  						if(debug) System.out.println("Valid approximate position? "+roverPos.isValidXYZ()+ " " + roverPos.toString());
            }
						if (roverPos.isValidXYZ()) {
							// Select available satellites
							satellites.selectStandalone( obsR );
							
							if (satellites.getAvailNumber() >= 4){
								if(debug) System.out.println("Number of selected satellites: " + satellites.getAvailNumber());
								// Compute code stand-alone positioning (epoch-by-epoch solution)
								sa.codeStandalone( obsR, false, false);
							}
							else
								// Discard approximate positioning
								roverPos.setXYZ(0, 0, 0);
						}

						if(debug)System.out.println("Valid LS position? "+roverPos.isValidXYZ()+ " " + roverPos.toString() );
						if (roverPos.isValidXYZ()) {
							if(!validPosition){
								notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
								validPosition = true;
							}
//							else 
							{
								coord = new RoverPosition(roverPos, RoverPosition.DOP_TYPE_STANDARD, roverPos.getpDop(), roverPos.gethDop(), roverPos.getvDop());

								if(positionConsumers.size()>0){
									coord.setRefTime(new Time(obsR.getRefTime().getMsec()));
									notifyPositionConsumerAddCoordinate(coord);
								}
								if(debug)System.out.println("PDOP: "+roverPos.getpDop());
								if(debug)System.out.println("------------------------------------------------------------");
								if(stopAtDopThreshold>0.0 && roverPos.getpDop()<stopAtDopThreshold){
									return this;
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
    return this;
	}

	/**
	 * Run code double differences.
	 */
	public GoGPS runCodeDoubleDifferences() {

		try {
	    LS_DD_code dd = new LS_DD_code(this);

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

						// Compute approximate positioning by iterative least-squares
						for (int iter = 0; iter < 3; iter++) {
							// Select all satellites
							satellites.selectStandalone( obsR, -100);
							
							if (satellites.getAvailNumber() >= 4) {
								dd.codeStandalone( obsR, false, true);
							}
						}

						// If an approximate position was computed
						if (roverPos.isValidXYZ()) {

							// Select satellites available for double differences
						  satellites.selectDoubleDiff( obsR, obsM, masterIn.getDefinedPosition());

							if (satellites.getAvailNumber() >= 4)
								// Compute code double differences positioning
								// (epoch-by-epoch solution)
								dd.codeDoubleDifferences( obsR, obsM, masterIn.getDefinedPosition());
							else
								// Discard approximate positioning
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
		return this;
	}

	/**
	 * Run kalman filter on code and phase standalone.
	 */
	public GoGPS runKalmanFilterCodePhaseStandalone() {

		long timeRead = System.currentTimeMillis();
		long depRead = 0;

		long timeProc = 0;
		long depProc = 0;

		KF_SA_code_phase kf = new KF_SA_code_phase(this);
		
		// Flag to check if Kalman filter has been initialized
		boolean kalmanInitialized = false;

		try {

			timeRead = System.currentTimeMillis() - timeRead;
			depRead = depRead + timeRead;

			Observations obsR = roverIn.getNextObservations();

			while (obsR != null) {

				if(debug)System.out.println("R:"+obsR.getRefTime().getMsec());

				timeRead = System.currentTimeMillis();
				depRead = depRead + timeRead;
				timeProc = System.currentTimeMillis();

				// If Kalman filter was not initialized and if there are at least four satellites
				boolean valid = true;
				if (!kalmanInitialized && obsR.getNumSat() >= 4) {

					// Compute approximate positioning by iterative least-squares
          for (int iter = 0; iter < 3; iter++) {
						// Select all satellites
					  satellites.selectStandalone( obsR, -100);
						
					  if (satellites.getAvailNumber() >= 4) {
							kf.codeStandalone( obsR, false, true);
						}
					}

					// If an approximate position was computed
					if (roverPos.isValidXYZ()) {

						// Initialize Kalman filter
						kf.init( obsR, null, roverIn.getDefinedPosition());

						if (roverPos.isValidXYZ()) {
							kalmanInitialized = true;
							if(debug)System.out.println("Kalman filter initialized.");
						} else {
							if(debug)System.out.println("Kalman filter not initialized.");
						}
					}else{
						if(debug)System.out.println("A-priori position (from code observations) is not valid.");
					}
				} else if (kalmanInitialized) {

					// Do a Kalman filter loop
					try{
						kf.loop( obsR, null, roverIn.getDefinedPosition());
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
		
		return this;
	}

	/**
	 * Run kalman filter on code and phase double differences.
	 */
	public GoGPS runKalmanFilterCodePhaseDoubleDifferences() {

		long timeRead = System.currentTimeMillis();
		long depRead = 0;

		long timeProc = 0;
		long depProc = 0;

		KalmanFilter kf = new KF_DD_code_phase(this);
		
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

					
					// If Kalman filter was not initialized and if there are at least four satellites
					boolean valid = true;
					if (!kalmanInitialized && obsR.getNumSat() >= 4) {

						// Compute approximate positioning by iterative least-squares
						
            for (int iter = 0; iter < 3; iter++) {
							// Select all satellites
							satellites.selectStandalone( obsR, -100);
							
							if (satellites.getAvailNumber() >= 4) {
								kf.codeStandalone( obsR, false, true );
							}
						}

						// If an approximate position was computed
						if (roverPos.isValidXYZ()) {
						  
							// Initialize Kalman filter
							kf.init(obsR, obsM, masterIn.getDefinedPosition());

							if (roverPos.isValidXYZ()) {
								kalmanInitialized = true;
								if(debug)System.out.println("Kalman filter initialized.");
							} else {
								if(debug)System.out.println("Kalman filter not initialized.");
							}
						}else{
							if(debug)System.out.println("A-priori position (from code observations) is not valid.");
						}
					} else if (kalmanInitialized) {

						// Do a Kalman filter loop
						try{
							kf.loop(obsR,obsM, masterIn.getDefinedPosition());
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
		
		return this;
	}

    void runElevationMethod(Observations obsR){
      roverPos.setGeod(0, 0, 0);
      roverPos.computeECEF();
      for (int iter = 0; iter < 500; iter++) {
        // Select all satellites
        System.out.println("////// Itr = " + iter);
        
        double correctionMag = satellites.selectPositionUpdate(obsR);
        if (satellites.getAvailNumber() < 6) {
          roverPos.status = Status.NoAprioriPos;
          break;
        }
        roverPos.status = Status.Valid;

//          if( correctionMag<MODULO/2)
        if( correctionMag<1)
          break;
      }
    }
    
    public GoGPS runCodeStandaloneSnapshot() {
      LS_SA_code_snapshot sa = new LS_SA_code_snapshot(this);
      
      Observations obsR = null;
      RoverPosition roverObs;
      Coordinates aPrioriPos = (Coordinates) roverPos.clone();

      Time refTime;
      int leapSeconds;
      obsR = roverIn.getCurrentObservations();
      
      notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
      while( obsR!=null && !Thread.interrupted() ) { // buffStreamObs.ready()
       try {
         if(debug) System.out.println("Index: " + obsR.index );
         roverPos.satsInUse = 0;

         // apply time offset
         refTime = obsR.getRefTime();
         obsR.setRefTime(new Time(obsR.getRefTime().getMsec() + offsetms ));

         // Add Leap Seconds, remove at the end
         leapSeconds = refTime.getLeapSeconds();
         Time GPSTime = new Time( refTime.getMsec() + leapSeconds * 1000);
         obsR.setRefTime(GPSTime);

         long refTimeMs = obsR.getRefTime().getMsec();

//         if( truePos != null ){
//           if(debug) System.out.println( String.format( "\r\n* True Pos: %8.4f, %8.4f, %8.4f", 
//               truePos.getGeodeticLatitude(),
//               truePos.getGeodeticLongitude(),
//               truePos.getGeodeticHeight()
//               ));
//           truePos.selectSatellitesStandaloneFractional( obsR, -100, MODULO1MS );
//         }
         
         if( !roverPos.isValidXYZ() && aPrioriPos != null && aPrioriPos.isValidXYZ()){
           aPrioriPos.cloneInto(roverPos);
         }
         else if( !roverPos.isValidXYZ() && obsR.getNumSat()>0 && !Float.isNaN(obsR.getSatByIdx(0).getDoppler(0))){
           roverPos.setXYZ(0, 0, 0);
           runElevationMethod(obsR);
           
           sa.dopplerPos(obsR);
           
           if( roverPos.isValidXYZ() )
             roverPos.cloneInto(aPrioriPos);
         }
         
         if( !roverPos.isValidXYZ() ){
           obsR = roverIn.getNextObservations();
           roverPos.status = Status.None;
           continue;
         }
             
         sa.tryOffset( aPrioriPos, obsR );

         if(debug) System.out.println("Valid position? "+roverPos.isValidXYZ()+" x:"+roverPos.getX()+" y:"+roverPos.getY()+" z:"+roverPos.getZ());
         if(debug) System.out.println(" lat:"+roverPos.getGeodeticLatitude()+" lon:"+roverPos.getGeodeticLongitude() );

         roverObs = new RoverPosition( roverPos, RoverPosition.DOP_TYPE_STANDARD, roverPos.getpDop(), roverPos.gethDop(), roverPos.getvDop());
         roverObs.index = obsR.index;
         roverObs.sampleTime = refTime;
         roverObs.obs = obsR;
         roverObs.satsInView = obsR.getNumSat();
         roverObs.satsInUse = roverPos.satsInUse;
         roverObs.eRes = roverPos.eRes;
         roverObs.status = roverPos.status;

         if( roverPos.isValidXYZ() ){
           
           offsetms = obsR.getRefTime().getMsec()-refTimeMs;

           // remove Leap Seconds
           obsR.setRefTime(new Time(obsR.getRefTime().getMsec() - leapSeconds * 1000));

           roverObs.status = Status.Valid;
           roverObs.cErrMS = offsetms;
           // update a priori location
           roverPos.cloneInto(aPrioriPos);

          if(debug)System.out.println("-------------------- "+roverPos.getpDop());
        }
         else if( roverPos.status != Status.EphNotFound && !Float.isNaN(obsR.getSatByIdx(0).getDoppler(0))){
           // invalidate aPrioriPos and recompute later
           aPrioriPos.setXYZ(0, 0, 0);
         }
        if(positionConsumers.size()>0){
          roverObs.setRefTime(new Time(obsR.getRefTime().getMsec()));
          notifyPositionConsumerAddCoordinate(roverObs);
        }

       } catch (Throwable e) {
         e.printStackTrace();
       } 
       finally {
        obsR = roverIn.getNextObservations();
        roverPos.status = Status.None;
      } 
    }
    notifyPositionConsumerEvent(PositionConsumer.EVENT_END_OF_TRACK);
      
    return this;
  }

  public RoverPosition runCodeStandaloneCoarseTime() {
    try {
      return runCodeStandaloneCoarseTime( MODULO20MS );
    } catch (Exception e) {
      e.printStackTrace();
    }
    return null;
  }
	
  GoGPS runCoarseTime(Observations obsR, final double MODULO ){
    for (int iter = 0; iter < 2000; iter++) {
      if(debug) System.out.println("\r\n////// itr = " + iter );
      long   updatems = obsR.getRefTime().getMsec();

//      if( truePos != null ){
//        System.out.println( String.format( "\r\n* True Pos: %8.4f, %8.4f, %8.4f", 
//            truePos.getGeodeticLatitude(),
//            truePos.getGeodeticLongitude(),
//            truePos.getGeodeticHeight()
//            ));
//        truePos.selectSatellitesStandaloneFractional( obsR, -100, MODULO20MS );
//      }
      
      System.out.println( String.format( "\r\n* Rover Pos: %8.4f, %8.4f, %8.4f", 
          roverPos.getGeodeticLatitude(),
          roverPos.getGeodeticLongitude(),
          roverPos.getGeodeticHeight()
          ));
      LS_SA_code_coarse_time sa = new LS_SA_code_coarse_time(this);
      sa.selectSatellites( obsR, -100, MODULO20MS );
      System.out.println();

      if (satellites.getAvailNumber() < 3) {
        if(debug) System.out.println("Not enough satellites" );
        roverPos.setXYZ(0, 0, 0);
        roverPos.status = Status.NotEnoughSats;
        break;
      }
      else {
        double correction_mag = 
            satellites.getAvailNumber() == 3?
            sa.codeStandaloneDTM(obsR, MODULO )
          : sa.codeStandaloneCoarseTime(obsR, MODULO );
        updatems = obsR.getRefTime().getMsec() - updatems;
        
        if( Math.abs( updatems/1000 )> 12*60*60 ){
          if(debug) System.out.println("Time update is too large: " + updatems/1000 + " s" );
          roverPos.setXYZ(0, 0, 0);
          if( roverPos.status == Status.None ){
            roverPos.status = Status.MaxCorrection;
          }
          break;
        }
          
        if( roverPos.status != Status.None && roverPos.status != Status.Valid )
          break;
        
        // if correction is small enough, we're done, exit loop
        if( correction_mag< POS_TOL && Math.abs(updatems) < TG_TOL) {
           roverPos.status = Status.Valid;
           break; 
        }
      }
    }
    return this;
  }
  
  public RoverPosition runCodeStandaloneCoarseTime( final double MODULO ) throws Exception {
    long index = 0;
    Observations obsR = null;
    Time refTime;
    int leapSeconds;
    
    // read the whole file
    List<Observations> obsl = new ArrayList<Observations>();
    do{
      obsR = roverIn.getNextObservations();
      if( obsR!=null )
        obsl.add(obsR);
    } while( obsR!=null);
    
      Coordinates aPrioriPos = roverIn.getDefinedPosition();
      if( aPrioriPos != null && aPrioriPos.isValidXYZ() ){
        roverPos.setXYZ( aPrioriPos.getX(), aPrioriPos.getY(), aPrioriPos.getZ() );
        roverPos.computeGeodetic();
      }
      else {
        aPrioriPos = Coordinates.globalGeodInstance(0, 0, 0);
        
        System.out.println("\r\nSearching for a priori position");
        
        long maxNumSat = 0;
        index = 0;
        long maxSatIdx = 0;
        Observations maxSatObs = null;
        // find the observation set with most satellites
        Iterator<Observations> it = obsl.iterator();
        while( it.hasNext() ) { // buffStreamObs.ready()
          obsR = it.next();
          if( obsR==null)
            break;
          // search for an observation with at least 6 satellites to produce an a priori position using the elevation method
          if( obsR.getNumSat()>maxNumSat){
            maxNumSat = obsR.getNumSat();
            maxSatObs = obsR;
            maxSatIdx = index;
          }
          index++;
        }
  
        roverPos.status = Status.NoAprioriPos;
        runElevationMethod(maxSatObs);
            
        if( roverPos.status == Status.Valid){
            // remember refTime
            refTime = maxSatObs.getRefTime();
            
            double thr = this.getResidThreshold();
            this.setResidThreshold(MODULO);
            runCoarseTime(maxSatObs, MODULO);
            // restore obsR refTime
            maxSatObs.setRefTime(refTime);
            this.setResidThreshold(thr);
            
            roverPos.cloneInto(aPrioriPos);
            roverPos.status = Status.None;
          }
      }

    // now process all the observation sets from the top of the file
    Iterator<Observations> it = obsl.iterator();
    notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
    index = 0;
    try {
      while( it.hasNext() ) { 
        obsR = it.next();
        if( obsR == null )
          break;
        
        index++;
        
        if(debug){
          System.out.println("==========================================================================================");
          System.out.println("Index = " + index );
          System.out.println("Processing " + obsR);
        }
        
        roverPos.status = Status.None;
        
        // apply offset
        refTime = obsR.getRefTime();
        
        // Add Leap Seconds, remove at the end
        leapSeconds = refTime.getLeapSeconds();
        Time GPSTime = new Time( refTime.getMsec() + leapSeconds * 1000);
        obsR.setRefTime(GPSTime);
        Time newTime = new Time( obsR.getRefTime().getMsec() + offsetms );
        obsR.setRefTime(newTime);
        long newTimeRefms = obsR.getRefTime().getMsec();
        
        if( !roverPos.isValidXYZ() ){
          if( obsR.getNumSat()<6){
            roverPos.status = Status.NoAprioriPos;
          }
          else {
            runElevationMethod(obsR);
          }
        }

        // If an approximate position was computed
        if( !roverPos.isValidXYZ() ){
          if(debug) System.out.println("Couldn't compute an approximate position at " + obsR.getRefTime());
          if( roverPos.status == Status.None ){
            roverPos.status = Status.NoAprioriPos;
          }
          continue;
        }
        else{
          
          if(debug) System.out.println("Approximate position at " + obsR.getRefTime() +"\r\n" + roverPos );
          
          runCoarseTime(obsR, MODULO);
        }
        RoverPosition roverObs = null;
            
        if( !roverPos.isValidXYZ() || roverPos.gethDop()>this.hdopLimit ){
          roverObs = new RoverPosition( roverPos, RoverPosition.DOP_TYPE_STANDARD, roverPos.getpDop(), roverPos.gethDop(), roverPos.getvDop());
          roverObs.index = index;
          roverObs.sampleTime = refTime;
          roverObs.obs = obsR;
          roverObs.satsInView = obsR.getNumSat();
          roverObs.satsInUse = roverPos.satsInUse;
          roverObs.status = roverPos.status;
          
          if( roverPos.isValidXYZ() && roverPos.gethDop()>this.hdopLimit ){
            System.out.println( String.format( "Excluding fix hdop = %3.1f > %3.1f (limit)", roverPos.gethDop(), this.hdopLimit ));
            roverObs.status = Status.MaxHDOP;
          }
          // restore a priori location
          if( aPrioriPos != null && aPrioriPos.isValidXYZ() ){
            aPrioriPos.cloneInto(roverPos);
          } 
        }
        else {
          double offsetUpdate = obsR.getRefTime().getMsec() - newTimeRefms;
          offsetms += offsetUpdate;

          // remove Leap Seconds
          obsR.setRefTime(new Time(obsR.getRefTime().getMsec() - leapSeconds * 1000));
        
          // update aPrioriPos
          roverPos.cloneInto(aPrioriPos);

          if(debug) System.out.println("Valid position? "+ roverPos.isValidXYZ() + "\r\n" + roverPos );
          if(debug) System.out.println(" lat:"+roverPos.getGeodeticLatitude()+" lon:"+roverPos.getGeodeticLongitude() );
          if(debug) System.out.println(" time offset update (ms): " +  offsetUpdate + "; Total time offset (ms): " + offsetms );  
        
          roverObs = new RoverPosition( roverPos, RoverPosition.DOP_TYPE_STANDARD, roverPos.getpDop(), roverPos.gethDop(), roverPos.getvDop());
          roverObs.index = index;
          roverObs.sampleTime = refTime;
          roverObs.obs = obsR;
          roverObs.satsInView = obsR.getNumSat();
          roverObs.satsInUse = roverPos.satsInUse;
          roverObs.eRes = roverPos.eRes;
          roverObs.status = Status.Valid;
          roverObs.cErrMS = obsR.getRefTime().getMsec() - roverObs.sampleTime.getMsec();
        }
        if(positionConsumers.size()>0){
          roverObs.setRefTime(new Time(obsR.getRefTime().getMsec()));
          notifyPositionConsumerAddCoordinate(roverObs);
        }
      }
    }
    catch (Throwable e) {
      e.printStackTrace();
    } finally {
      notifyPositionConsumerEvent(PositionConsumer.EVENT_END_OF_TRACK);
    }
    return new RoverPosition( roverPos, RoverPosition.DOP_TYPE_STANDARD, roverPos.getpDop(), roverPos.gethDop(), roverPos.getvDop());
  }

  /**
   * @param interval process fixes every interval minutes
   * @return
   * @throws Exception
   */
  public GoGPS runDopplerPos() {
    
    long index = 0;
    Observations obsR = null;
    
    LS_SA_dopplerPos sa = new LS_SA_dopplerPos(this);
    Time refTime;
    try {
      obsR = roverIn.getCurrentObservations();
      
      notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
      while( obsR!=null ) { // buffStreamObs.ready()

        refTime = obsR.getRefTime();

        // for test
        roverPos.setXYZ(0, 0, 0);
        
//        runElevationMethod(obsR);

        sa.dopplerPos(obsR);

        // If an approximate position was computed
        if(debug) System.out.println("Valid position? "+roverPos.isValidXYZ());
        
        RoverPosition coord2 = null;
        
        if( !roverPos.isValidXYZ() ){
//              coord2 = new ReceiverPosition( Coordinates.globalXYZInstance(0, 0, 0), ReceiverPosition.DOP_TYPE_NONE,0.0,0.0,0.0 );
//              coord2.status = false;
//              coord2.satsInView = obsR.getNumSat();
//              coord2.satsInUse = 0;
          obsR = roverIn.getNextObservations();
          continue;
        }
          else {
            if(debug) System.out.println("Valid position? "+roverPos.isValidXYZ()+" x:"+roverPos.getX()+" y:"+roverPos.getY()+" z:"+roverPos.getZ());
            if(debug) System.out.println(" lat:"+roverPos.getGeodeticLatitude()+" lon:"+roverPos.getGeodeticLongitude() );
              
              coord2 = new RoverPosition( roverPos, RoverPosition.DOP_TYPE_STANDARD, roverPos.getpDop(), roverPos.gethDop(), roverPos.getvDop());
//                coord2.status = true;
//                coord2.satsInView = obsR.getNumSat();
//                coord2.satsInUse = ((SnapshotReceiverPosition)roverPos).satsInUse;

              // set other things
              // "Index,Status, Date, UTC,Latitude [DD], Longitude [DD], 
              // HDOP,SVs in Use, SVs in View, SNR Avg [dB], 
              // Residual Error, Clock Error, Clock Error Total,\r\n" );
              
              if(debug)System.out.println("-------------------- "+roverPos.getpDop());
//                if(stopAtDopThreshold>0.0 && roverPos.getpDop()<stopAtDopThreshold){
//                  return coord;
//                }
          }
          if(positionConsumers.size()>0){
            coord2.setRefTime(new Time(obsR.getRefTime().getMsec()));
            notifyPositionConsumerAddCoordinate(coord2);
          }
//        }catch(Exception e){
//          System.out.println("Could not complete due to "+e);
//          e.printStackTrace();
//        }
        obsR = roverIn.getNextObservations();
      }
    } catch (Throwable e) {
      e.printStackTrace();
    } finally {
      notifyPositionConsumerEvent(PositionConsumer.EVENT_END_OF_TRACK);
    }
    return this;
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
	public GoGPS addPositionConsumerListener(PositionConsumer positionConsumer) {
		this.positionConsumers.add(positionConsumer);
		return this;
	}

  public GoGPS addPositionConsumerListeners(PositionConsumer... positionConsumers) {
    this.positionConsumers.addAll(Arrays.asList(positionConsumers));
    return this;
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
	public GoGPS runThreadMode(int runMode) {
		this.runMode = runMode;
		this.runThread = new Thread(this);
		switch(runMode){
			case RUN_MODE_STANDALONE:
				this.runThread.setName("goGPS standalone");
				break;
			case RUN_MODE_DOUBLE_DIFF:
				this.runThread.setName("goGPS double difference");
				break;
			case RUN_MODE_KALMAN_FILTER_STANDALONE:
				this.runThread.setName("goGPS Kalman filter standalone");
				break;
			case RUN_MODE_KALMAN_FILTER_DOUBLE_DIFF:
				this.runThread.setName("goGPS Kalman filter double difference");
				break;
      case RUN_MODE_STANDALONE_SNAPSHOT:
        this.runThread.setName("goGPS standalone snapshot");
        break;
      case RUN_MODE_STANDALONE_COARSETIME:
        this.runThread.setName("goGPS standalone coarse time");
        break;
		}
		this.running = true;
		this.runThread.start();
		return this;
	}

  public boolean isRunning() {
    return runThread != null && running;
  }
  
  public String getThreadName(){
    if( runThread != null )
      return runThread.getName();
    else 
      return "";
  }
  
  public void stopThreadMode(){
	  if( !running )
	    return;
	  
    if( runThread != null) {
      try {
        runThread.interrupt();
        runThread.join();
      } catch (InterruptedException e) {
        // FIXME Auto-generated catch block
        e.printStackTrace();
      }
    }
    running = false;
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
			case RUN_MODE_KALMAN_FILTER_STANDALONE:
				runKalmanFilterCodePhaseStandalone();
				break;
			case RUN_MODE_KALMAN_FILTER_DOUBLE_DIFF:
				runKalmanFilterCodePhaseDoubleDifferences();
				break;
      case RUN_MODE_STANDALONE_SNAPSHOT:
        runCodeStandaloneSnapshot();
        break;
      case RUN_MODE_STANDALONE_COARSETIME:
        runCodeStandaloneCoarseTime();
        break;
		}

		notifyPositionConsumerEvent(PositionConsumer.EVENT_GOGPS_THREAD_ENDED);
    running = false;
	}

  public GoGPS runUntilFinished() {
    for( PositionConsumer pc: positionConsumers ){
      if( pc instanceof Thread ){
        try {
          ((Thread)pc).join();
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
    return this;
  }

  public GoGPS runFor(int seconds ) {
    try {
      Thread.sleep(seconds*1000);
    } catch (InterruptedException e) {
      // FIXME Auto-generated catch block
      e.printStackTrace();
    }
    return this;
  }

}
