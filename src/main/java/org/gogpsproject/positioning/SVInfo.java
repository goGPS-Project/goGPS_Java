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
package org.gogpsproject.positioning;

/**
 * @author ZiglioUK
 */
public class SVInfo extends SatellitePosition{

//	public TopocentricCoordinates topocentric;
	public double az;
	public double el;
	
	/*
	 * svUsed
	 * diffCorr
	 * orbitAvail
	 * orpithEph
	 * unhealthy
	 * orbitAlm
	 * orbitAop
	 * smoothed
	 */
	int flags;

	/*
	 * 0: no signal
	 * 1: signal acquired
	 * 3: signal detected but unuseable
	 * 4. code locked and time synchronised
	 * 5, 6, 7: code and carrier locked and time synchronised
	 */
	int quality;
	
	int cno;
	
	/* pseudorange residual in cm*/
	int prRes;
	
	public SVInfo( long unixTime, int satID, char satType, double az, double el, int cno, int prRes, int flags, int quality ) {
		super( unixTime, satID, satType, 0, 0, 0);
//		topocentric = new TopocentricCoordinates(az, el);
		this.az = az;
		this.el = el;
	}
}

