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
public class RinexNavigationParserGlonass extends RinexNavigationParser{

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
	public RinexNavigationParserGlonass(File fileNav) {
		super(fileNav);
	}

	// RINEX Read constructors
	public RinexNavigationParserGlonass(InputStream is, File cache) {
		super(is,cache);
	}

	/* TODO all the rest*/
}
