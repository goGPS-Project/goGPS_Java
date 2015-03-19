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
public class AntennaDescriptor {

	private int stationID;
	private String antennaDescriptor;
	private int setupID;
	private String antennaSerial;

	public String getAntennaDescriptor() {
		return antennaDescriptor;
	}

	public int getSetupID() {
		return setupID;
	}

	public int getStationID() {
		return stationID;
	}

	public void setAntennaDescriptor(String antennaDescriptor) {
		this.antennaDescriptor = antennaDescriptor;
	}

	public void setSetupID(int setupID) {
		this.setupID = setupID;
	}

	public void setStationID(int stationID) {
		this.stationID = stationID;
	}

	@Override
	public String toString() {
		return "AntennaDescriptor [antennaDescriptor=" + antennaDescriptor
				+ ", setupID=" + setupID + ", stationID=" + stationID + " antennaSerial=" + antennaSerial + "]";
	}

	/**
	 * @return the antennaSerial
	 */
	public String getAntennaSerial() {
		return antennaSerial;
	}

	/**
	 * @param antennaSerial the antennaSerial to set
	 */
	public void setAntennaSerial(String antennaSerial) {
		this.antennaSerial = antennaSerial;
	}

}
