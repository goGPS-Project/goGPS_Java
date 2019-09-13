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
import org.gogpsproject.positioning.*;
import org.gogpsproject.producer.*;

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
  public static enum WeightingStrategy{
    EQUAL,
    SAT_ELEVATION,
    SIGNAL_TO_NOISE_RATIO,
    COMBINED_ELEVATION_SNR
  }
	
  /** The weights. */
  private WeightingStrategy weights = WeightingStrategy.SAT_ELEVATION;

  public static enum DynamicModel {
    STATIC(1),
    CONST_SPEED(2),
    CONST_ACCELERATION(3);
  
    private int order;
  
    public int getOrder(){
      return order;
    }
  
    DynamicModel( int order ){
      this.order = order;
    }
  }
	
  // Kalman filter parameters
  /** The dynamic model. */
  private DynamicModel dynamicModel = DynamicModel.CONST_SPEED;

  /** The cycle slip threshold. */
  private double cycleSlipThreshold = 1;
	
  public static enum CycleSlipDetectionStrategy {
    APPROX_PSEUDORANGE,
    DOPPLER_PREDICTED_PHASE_RANGE
  }
	
  /** The cycle-slip detection strategy. */
  private CycleSlipDetectionStrategy cycleSlipDetectionStrategy = CycleSlipDetectionStrategy.APPROX_PSEUDORANGE;

  public static enum AmbiguityStrategy {
    OBSERV,
    APPROX,
    LS
  }
	
  /** The ambiguity strategy. */
  private AmbiguityStrategy ambiguityStrategy = AmbiguityStrategy.APPROX;

  /** The Elevation cutoff. */
  private double cutoff = 15; // Elevation cutoff

  public static enum RunMode {
    CODE_STANDALONE,
	CODE_DOUBLE_DIFF,
	KALMAN_FILTER_CODE_PHASE_STANDALONE,
	KALMAN_FILTER_CODE_PHASE_DOUBLE_DIFF,
	CODE_STANDALONE_SNAPSHOT,
	CODE_STANDALONE_COARSETIME
  }

  private RunMode runMode;
	
  private Thread runThread=null;
	
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
  
  /** coarse time error */
  private long offsetms = 0;

  private Vector<PositionConsumer> positionConsumers = new Vector<PositionConsumer>();

//	private boolean debug = false;
  private boolean debug = true;

  private boolean useDTM = false;

  /** Use Doppler observations in standalone snapshot case */
  private boolean useDoppler = true;
	
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

  /** max code residual error to exclude a given range (m) */
  private double codeResidThreshold = 30;

  /** max code residual error to exclude a given range (m) */
  private double phaseResidThreshold = 0.05;

  private boolean searchForOutliers = false;
  
  /**
   * Instantiates a new GoGPS.
   *
   * @param navigation the navigation
   * @param roverIn the rover in
   * @param masterIn the master in
   */
  public GoGPS(NavigationProducer navigation, ObservationsProducer roverIn, ObservationsProducer masterIn){

    this.navigation = navigation;
    this.roverIn = roverIn;
    this.masterIn = masterIn;

    roverPos = new RoverPosition();
    masterPos = new MasterPosition();
    satellites = new Satellites(this);
  }
  
  /**
   * Instantiates a new GoGPS.
   *
   * @param navigation the navigation
   * @param roverIn the rover in
   */
  public GoGPS(NavigationProducer navigation, ObservationsProducer roverIn ){
    this( navigation, roverIn, null );
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


  
  public ObservationsProducer getRoverIn(){
    return roverIn;
  }

  public ObservationsProducer getMasterIn(){
    return masterIn;
  }
  
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
   * Gets the weights.
   *
   * @return the weights
   */
  public WeightingStrategy getWeights() {
  	return weights;
  }

  /**
   * Sets the weights.
   *
   * @param weights the weights to set
   * @return 
   */
  public GoGPS setWeights(WeightingStrategy weights) {
  	this.weights = weights;
    return this;
  }

  /**
   * Gets the dynamic model.
   *
   * @return the dynamicModel
   */
  public DynamicModel getDynamicModel() {
  	return dynamicModel;
  }

  /**
   * Sets the dynamic model.
   *
   * @param dynamicModel the dynamicModel to set
   */
  public GoGPS setDynamicModel(DynamicModel dynamicModel) {
  	this.dynamicModel = dynamicModel;
  	return this;
  }

  /**
   * Gets the cycle-slip detection strategy.
   *
   * @return the cycleSlipDetectionStrategy
   */
  public CycleSlipDetectionStrategy getCycleSlipDetectionStrategy() {
  	return cycleSlipDetectionStrategy;
  }

  /**
   * Sets the cycle-slip detection strategy.
   *
   * @param cycleSlipDetectionStrategy the cycleSlipDetectionStrategy to set
   * @return 
   */
  public GoGPS setCycleSlipDetection( CycleSlipDetectionStrategy cycleSlipDetectionStrategy ) {
  	this.cycleSlipDetectionStrategy = cycleSlipDetectionStrategy;
    return this;
  }

  /**
   * Gets the ambiguity strategy.
   *
   * @return the ambiguityStrategy
   */
  public AmbiguityStrategy getAmbiguityStrategy() {
  	return ambiguityStrategy;
  }

  /**
   * Sets the ambiguity strategy.
   *
   * @param ambiguityStrategy the ambiguityStrategy to set
   * @return 
   */
  public GoGPS setAmbiguityStrategy( AmbiguityStrategy ambiguityStrategy ) {
  	this.ambiguityStrategy = ambiguityStrategy;
    return this;
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

  public double getCodeResidThreshold(){
    return this.codeResidThreshold;
  }

  public GoGPS setCodeResidThreshold(double codeResidThreshold) {
    this.codeResidThreshold = codeResidThreshold;
    return this;
  }

  public double getPhaseResidThreshold(){
    return this.phaseResidThreshold;
  }

  public GoGPS setPhaseResidThreshold(double phaseResidThreshold) {
    this.phaseResidThreshold = phaseResidThreshold;
    return this;
  }

  public GoGPS searchForOutliers( boolean searchForOutliers ) {
	this.searchForOutliers = searchForOutliers;
	return this;
  }
  
  public boolean searchForOutliers() {
	return searchForOutliers ;
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

  public long getOffsetms(){
    return offsetms;
  }

  public GoGPS setOffsetms( long offsetms){
    this.offsetms = offsetms;
    return this;
  }

  /** Use Doppler observations in standalone snapshot case */
  public boolean useDoppler() {
    return useDoppler;
  }

  /** Use Doppler observations in standalone snapshot case */
  public GoGPS useDoppler(boolean useDoppler) {
    this.useDoppler = useDoppler;
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
	
	public Vector<PositionConsumer> getPositionConsumers() {
    return positionConsumers;
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
	
	public void notifyPositionConsumerEvent(int event){
		for(PositionConsumer pc:positionConsumers){
			try{
				pc.event(event);
			}catch(Exception e){
				e.printStackTrace();
			}
		}
	}
	
	public void notifyPositionConsumerAddCoordinate(RoverPosition coord){
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
	public RunMode getRunMode() {
		return runMode;
	}

	/**
	 * @param runMode the run mode to use
	 */
	public GoGPS runThreadMode( RunMode runMode ) {
		return run( runMode, true );
	}

  public GoGPS run( RunMode runMode ) {
    return run( runMode, false );
  }
  
  /**
   * @param runMode the run mode to use
   */
  public GoGPS run( RunMode runMode, boolean threadMode ) {
    this.runMode = runMode;
    
    if( threadMode ){
      runThread = new Thread(this);
      
      switch(runMode){
        case CODE_STANDALONE:
          runThread.setName("goGPS standalone");
          break;
        case CODE_DOUBLE_DIFF:
          runThread.setName("goGPS double difference");
          break;
        case KALMAN_FILTER_CODE_PHASE_STANDALONE:
          runThread.setName("goGPS Kalman filter standalone");
          break;
        case KALMAN_FILTER_CODE_PHASE_DOUBLE_DIFF:
          runThread.setName("goGPS Kalman filter double difference");
          break;
        case CODE_STANDALONE_SNAPSHOT:
          runThread.setName("goGPS standalone snapshot");
          break;
        case CODE_STANDALONE_COARSETIME:
          runThread.setName("goGPS standalone coarse time");
          break;
      }
      
      runThread.start();
    }
    else
      run();
    
    return this;
  }
	
  /* (non-Javadoc)
   * @see java.lang.Runnable#run()
   */
  @Override
  public void run() {
  	if( runMode == null ) 
  	  throw new RuntimeException( "runMode was not defined. Ex. goGPS.run( RunMode.CODE_STANDALONE )");
  
  	switch(runMode){
  		case CODE_STANDALONE:
  	    LS_SA_code.run(this, -1);
  			break;
  		case CODE_DOUBLE_DIFF:
  		  LS_DD_code.run(this);
  			break;
  		case KALMAN_FILTER_CODE_PHASE_STANDALONE:
  		  KF_SA_code_phase.run(this);
  			break;
  		case KALMAN_FILTER_CODE_PHASE_DOUBLE_DIFF:
  		  KF_DD_code_phase.run(this);
  			break;
      case CODE_STANDALONE_SNAPSHOT:
        LS_SA_code_snapshot.run(this);
        break;
      case CODE_STANDALONE_COARSETIME:
        LS_SA_code_coarse_time.run( this, MODULO20MS );
        break;
  	}
  
  	notifyPositionConsumerEvent(PositionConsumer.EVENT_GOGPS_THREAD_ENDED);
  }

  /**
   * Run code standalone.
   *
   * @param getNthPosition the get nth position
   * @return the coordinates
   * @throws Exception
   */
  public GoGPS runCodeStandalone(double stopAtDopThreshold) {
    LS_SA_code.run(this, stopAtDopThreshold);
    return this;
  }

  public boolean isRunning() {
    return runThread != null && runThread.isAlive();
  }
  
  public String getThreadName(){
    if( runThread != null )
      return runThread.getName();
    else 
      return "";
  }
  
  public void stopThreadMode(){
    if( isRunning() ) {
      try {
        runThread.interrupt();
        runThread.join();
      } catch (InterruptedException e) {}
    }
  }
	
	public GoGPS runUntilFinished() {
    for( PositionConsumer pc: positionConsumers ){
      if( pc instanceof Thread ){
        try {
          ((Thread)pc).join();
        } catch (InterruptedException e) {}
      }
    }
    
    return this;
  }

  public GoGPS runFor(int seconds ) {
    try {
      Thread.sleep(seconds*1000);
    } catch (InterruptedException e) { }
    
    return this;
  }

}
