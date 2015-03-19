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
public class GlonassHeader {

	private int stationid; // 12
	private long epochTime; // 27
	private int flag; // 1
	private int numberOfSatellites; // 5
	private int smoothIndicator; // 1
	private int smoothInterval; // 3

	public long getEpochTime() {
		return epochTime;
	}

	public int getFlag() {
		return flag;
	}

	public int getNumberOfSatellites() {
		return numberOfSatellites;
	}

	public int getSmoothIndicator() {
		return smoothIndicator;
	}

	public int getSmoothInterval() {
		return smoothInterval;
	}

	public int getStationid() {
		return stationid;
	}

	public void setEpochTime(long epochTime) {
		this.epochTime = epochTime;
	}

	public void setFlag(int flag) {
		this.flag = flag;
	}

	public void setNumberOfSatellites(int numberOfSatellites) {
		this.numberOfSatellites = numberOfSatellites;
	}

	public void setSmoothIndicator(int smoothIndicator) {
		this.smoothIndicator = smoothIndicator;
	}

	public void setSmoothInterval(int smoothInterval) {
		this.smoothInterval = smoothInterval;
	}

	// 61;
	public void setStationid(int stationid) {
		this.stationid = stationid;
	}

	@Override
	public String toString() {
		return "Glonassheader [epochTime=" + epochTime + ", flag=" + flag
				+ ", numberOfSatellites=" + numberOfSatellites
				+ ", smoothIndicator=" + smoothIndicator + ", smoothInterval="
				+ smoothInterval + ", stationid=" + stationid + "]";
	}

}
