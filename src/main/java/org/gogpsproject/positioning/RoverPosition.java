/*
 * Copyright (c) 2011 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. 
 * All Rights Reserved.
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
package org.gogpsproject.positioning;
import org.gogpsproject.Status;
import org.gogpsproject.producer.Observations;

/**
 * <p>
 * Receiver position class
 * </p>
 *
 * @author Eugenio Realini, Cryms.com, Daisuke Yoshida, Emanuele Ziglioli (Sirtrack Ltd)
 */
public class RoverPosition extends ReceiverPosition {

  public Time sampleTime;
  public Observations obs;
  public Status status = Status.None;

  /** Sats in use from an observation set */
  public long satsInUse = 0;

  /** coarse time clock error in ms */
  public long cErrMS = 0;
  
  /** standard subms clock error in s*/
  double clockError; 

  /** Clock error rate */
	public double clockErrorRate; 
	
  /** Average residual error for least-squares computation */
  public double eRes;
	
  /** Position dilution of precision (PDOP) */
	double pDop; 

	/** Horizontal dilution of precision (HDOP) */
	double hDop; 

  /** Vertical dilution of precision (VDOP) */
	double vDop; 
	
	/** Kalman-derived position dilution of precision (KPDOP) */
	double kpDop; 
	
	/** Kalman-derived horizontal dilution of precision (KHDOP) */
	double khDop; 
	
	/** Kalman-derived vertical dilution of precision (KVDOP) */
	double kvDop; 
	
  private int dopType = DOP_TYPE_NONE;
  public final static int DOP_TYPE_NONE = 0;
  public final static int DOP_TYPE_STANDARD = 1; /* Standard DOP values (satellite geometry only) */
  public final static int DOP_TYPE_KALMAN = 2; /* Kalman DOP values (KDOP), based on the Kalman filter error covariance matrix */
  
	public RoverPosition(){
		super();
		this.setXYZ(0.0, 0.0, 0.0);
		this.clockError = 0.0;
	}

  public RoverPosition(Coordinates c) {
    this(c,DOP_TYPE_NONE,0.0,0.0,0.0);
  }

  public RoverPosition(Coordinates c, int dopType, double pDop, double hDop, double vDop) {
    super();
    c.cloneInto(this);
    this.dopType = dopType;
    this.pDop = pDop;
    this.hDop = hDop;
    this.vDop = vDop;
  }
  
	public double getClockError() {
		return clockError;
	}

	public void setClockError(double clockError ) {
		this.clockError = clockError;
	}

  public double getClockErrorRate() {
    return clockErrorRate;
  }

  public void setClockErrorRate(double clockErrorRate) {
    this.clockErrorRate = clockErrorRate;
  }

	public double getpDop() {
		return pDop;
	}

	public void setpDop(double pDop) {
		this.pDop = pDop;
	}

	public double gethDop() {
		return hDop;
	}

	public void sethDop(double hDop) {
		this.hDop = hDop;
	}

	public double getvDop() {
		return vDop;
	}

	public void setvDop(double vDop) {
		this.vDop = vDop;
	}

	public double getKpDop() {
		return kpDop;
	}

	public void setKpDop(double kpDop) {
		this.kpDop = kpDop;
	}

	public double getKhDop() {
		return khDop;
	}

	public void setKhDop(double khDop) {
		this.khDop = khDop;
	}

	public double getKvDop() {
		return kvDop;
	}

	public void setKvDop(double kvDop) {
		this.kvDop = kvDop;
	}

  /**
   * @return the dopType
   */
  public int getDopType() {
    return dopType;
  }

  /**
   * @param dopType the dopType to set
   */
  public void setDopType(int dopType) {
    this.dopType = dopType;
  }
  
  public RoverPosition clone( Observations obs ){
    RoverPosition r = new RoverPosition( this, RoverPosition.DOP_TYPE_STANDARD, pDop, hDop, vDop );
    r.obs        = obs;
    r.sampleTime = sampleTime;
    r.satsInUse  = satsInUse;
    r.eRes       = eRes;
    r.status     = status;
    r.clockError = clockError;
    r.clockErrorRate = clockErrorRate;
    
    return r;
  }
}
