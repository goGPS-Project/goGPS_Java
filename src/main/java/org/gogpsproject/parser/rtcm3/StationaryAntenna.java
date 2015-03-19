/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
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
 */

package org.gogpsproject.parser.rtcm3;
public class StationaryAntenna {

	private int stationID;
	private int itrl; // realization year
	private int gpsIndicator;
	private int glonassIndicator;
	private int rgalileoIndicator; // reserved galileo indicator
	private int rstationIndicator; // reference station indicator
	private double antennaRefPointX; // antenna reference point ECEF-x
	private int sreceiverOscillator; // signle receiver oscillator
	private int reserved1;
	private double antennaRefPointY; // antenna reference point ECEF-y
	private int reserved2;
	private double antennaRefPointZ; // antenna reference point ECEF-Y
	private double antennaHeight;


	public double getAntennaRefPointX() {
		return antennaRefPointX;
	}

	public double getAntennaRefPointY() {
		return antennaRefPointY;
	}

	public double getAntennaRefPointZ() {
		return antennaRefPointZ;
	}

	public int getGlonassIndicator() {
		return glonassIndicator;
	}

	public int getGpsIndicator() {
		return gpsIndicator;
	}

	public int getItrl() {
		return itrl;
	}

	public int getReserved1() {
		return reserved1;
	}

	public int getReserved2() {
		return reserved2;
	}

	public int getRgalileoIndicator() {
		return rgalileoIndicator;
	}

	public int getRstationIndicator() {
		return rstationIndicator;
	}

	public int getSreceiverOscillator() {
		return sreceiverOscillator;
	}

	public int getStationID() {
		return stationID;
	}

	public void setAntennaRefPointX(double antennaRefPointX) {
		this.antennaRefPointX = antennaRefPointX;
	}

	public void setAntennaRefPointY(double antennaRefPointY) {
		this.antennaRefPointY = antennaRefPointY;
	}

	public void setAntennaRefPointZ(double antennaRefPointZ) {
		this.antennaRefPointZ = antennaRefPointZ;
	}

	public void setGlonassIndicator(int glonassIndicator) {
		this.glonassIndicator = glonassIndicator;
	}

	public void setGpsIndicator(int gpsIndicator) {
		this.gpsIndicator = gpsIndicator;
	}

	public void setItrl(int itrl) {
		this.itrl = itrl;
	}

	public void setReserved1(int reserved1) {
		this.reserved1 = reserved1;
	}

	public void setReserved2(int reserved2) {
		this.reserved2 = reserved2;
	}

	public void setRgalileoIndicator(int rgalileoIndicator) {
		this.rgalileoIndicator = rgalileoIndicator;
	}

	public void setRstationIndicator(int rstationIndicator) {
		this.rstationIndicator = rstationIndicator;
	}

	public void setSreceiverOscillator(int sreceiverOscillator) {
		this.sreceiverOscillator = sreceiverOscillator;
	}

	public void setStationID(int stationID) {
		this.stationID = stationID;
	}

	@Override
	public String toString() {
		return "StationaryAntenna [antennaRefPointX=" + antennaRefPointX
				+ ", antennaRefPointY=" + antennaRefPointY
				+ ", antennaRefPointZ=" + antennaRefPointZ
				+ ", glonassIndicator=" + glonassIndicator + ", gpsIndicator="
				+ gpsIndicator + ", itrl=" + itrl + ", reserved1=" + reserved1
				+ ", reserved2=" + reserved2 + ", rgalileoIndicator="
				+ rgalileoIndicator + ", rstationIndicator="
				+ rstationIndicator + ", sreceiverOscillator="
				+ sreceiverOscillator + ", stationID=" + stationID + "]";
	}

	/**
	 * @param antennaHeight the antennaHeight to set
	 */
	public void setAntennaHeight(double antennaHeight) {
		this.antennaHeight = antennaHeight;
	}

	/**
	 * @return the antennaHeight
	 */
	public double getAntennaHeight() {
		return antennaHeight;
	}

}
