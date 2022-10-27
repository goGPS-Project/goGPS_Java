///=================================================================================================
// Class Decode1260Msg
//      Author :  Antoine GRENIER
//        Date :  2019/09/06
///=================================================================================================
// This code is based on the GoGPS library - Geomatics Laboratory of Politecnico di Milano, Italy.
// The GoGPS library is licenced under the LGPL.
// Copyrights: Mirko Reguzzoni, Dominioni Teruzzi, Colla, Palmulli Tettamanti, Realini.
///=================================================================================================
/*
 * Copyright 2018(c) IFSTTAR - TeamGEOLOC
 *
 * This file is part of the GeolocPVT application.
 *
 * GeolocPVT is distributed as a free software in order to build a community of users, contributors,
 * developers who will contribute to the project and ensure the necessary means for its evolution.
 *
 * GeolocPVT is a free software; you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free Software Foundation; either version 3
 * of the License, or (at your option) any later version. Any modification of source code in this
 * LGPL software must also be published under the LGPL license.
 *
 * GeolocPVT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU Lesser General Public License along with GeolocPVT.
 * If not, see <https://www.gnu.org/licenses/lgpl.txt/>.
 */
///=================================================================================================
package org.gogpsproject.producer.parser.rtcm3;

import android.location.GnssStatus;
import android.util.Log;

import org.gogpsproject.Constants;
import org.gogpsproject.ephemeris.SatelliteCodeBiases;
import org.gogpsproject.util.Bits;

import java.util.LinkedHashMap;

public class Decode1260Msg implements Decode
{
    public Decode1260Msg() {

    }

    public Object decode(boolean[] bits, int referenceTS)
    {
        int i = 0;
        double tow = 0;
        int iod = 0;
        int nsat = 0;

        // Specific to the constellation
        int system = GnssStatus.CONSTELLATION_BEIDOU;
        int indexPrn = 6;
        int offsetPrn = 1;
        int ncode = 9;

        // Decode header
        i += 12;

        tow = (double) Bits.bitsToUInt(Bits.subset(bits, i, 20)); i += 20;
        i += 4;  // Update interval
        i += 1;  // Multiple Message Indicator
        iod = (int) Bits.bitsToUInt(Bits.subset(bits, i, 4)); i += 4;
        i += 16; // SSR Provider ID
        i += 4;  // SSR Solution ID
        nsat = (int) Bits.bitsToUInt(Bits.subset(bits, i, 6)); i += 6;

        SatelliteCodeBiases scb = new SatelliteCodeBiases();

        // Decode body
        for(int j = 0; j < nsat && (i + 5 + indexPrn <= bits.length*8); j++)
        {
            int prn = (int) Bits.bitsToUInt(Bits.subset(bits, i, indexPrn)) + offsetPrn; i += indexPrn;
            int nbias = (int) Bits.bitsToUInt(Bits.subset(bits, i, 5)); i += 5;

            LinkedHashMap<Integer, Double> biases = new LinkedHashMap<>();

            for(int k = 0; k < nbias && (i + 19 <= bits.length*8) ; k++)
            {
                int mode = (int) Bits.bitsToUInt(Bits.subset(bits, i, 5)); i += 5;
                Double bias = (double) Bits.bitsTwoComplement(Bits.subset(bits, i, 14)) * 0.01; i += 14;

                if(mode > ncode)
                {
                    Log.e("RTCM", "Unsupported code for DCB: " + mode);
                    continue;
                }

                int code = Constants.CODES_BDS[mode];

                biases.put(code, bias);
            }

            SatelliteCodeBiases.CodeBias cb = new SatelliteCodeBiases.CodeBias(system, prn, tow, iod, biases);

            scb.setCodeBias(cb);
        }

        return scb;
    }
}