///=================================================================================================
// Class Decode1042Msg
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
import org.gogpsproject.ephemeris.GNSSEphemeris;
import org.gogpsproject.util.Bits;


public class Decode1042Msg implements Decode {
    public Decode1042Msg() {

    }

    public Object decode(boolean[] bits, int referenceTS) {
        GNSSEphemeris mEph = new GNSSEphemeris();
        int i = 0;
        //int i = 24; //Found in RTKLIB, to be verified ...

        i += 12; //Bits of message number

        mEph.setGnssSystem(GnssStatus.CONSTELLATION_BEIDOU);

        try {
            //Start decoding
            mEph.setPrn((int) Bits.bitsToUInt(Bits.subset(bits, i, 6)));
            i += 6;
            mEph.setWeek((int) Bits.bitsToUInt(Bits.subset(bits, i, 13)));
            i += 13;
            i += 4; // SV URAI
            mEph.setIdot((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 14)) * Constants.P2_43 * Constants.PI_ORBIT);
            i += 14;
            mEph.setIode((int) Bits.bitsToUInt(Bits.subset(bits, i, 5)));
            i += 5;
            mEph.setToc((double) Bits.bitsToUInt(Bits.subset(bits, i, 17)) * 8.0);
            i += 17;
            mEph.setAf2((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 11)) * Constants.P2_66);
            i += 11;
            mEph.setAf1((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 22)) * Constants.P2_50);
            i += 22;
            mEph.setAf0((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 24)) * Constants.P2_33);
            i += 24;
            mEph.setIodc((int) Bits.bitsToUInt(Bits.subset(bits, i, 5)));
            i += 5;
            mEph.setCrs((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 18)) * Constants.P2_6);
            i += 18;
            mEph.setDeltaN((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 16)) * Constants.P2_43 * Constants.PI_ORBIT);
            i += 16;
            mEph.setM0((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 32)) * Constants.P2_31 * Constants.PI_ORBIT);
            i += 32;
            mEph.setCuc((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 18)) * Constants.P2_31);
            i += 18;
            mEph.setEc((double) Bits.bitsToUInt(Bits.subset(bits, i, 32)) * Constants.P2_33);
            i += 32;
            mEph.setCus((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 18)) * Constants.P2_31);
            i += 18;
            mEph.setSquareA((double) Bits.bitsToUInt(Bits.subset(bits, i, 32)) * Constants.P2_19);
            i += 32;
            mEph.setToe((double) Bits.bitsToUInt(Bits.subset(bits, i, 17)) * 8.0);
            i += 17;
            mEph.setCic((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 18)) * Constants.P2_31);
            i += 18;
            mEph.setOmega0((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 32)) * Constants.P2_31 * Constants.PI_ORBIT);
            i += 32;
            mEph.setCis((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 18)) * Constants.P2_31);
            i += 18;
            mEph.setI0((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 32)) * Constants.P2_31 * Constants.PI_ORBIT);
            i += 32;
            mEph.setCrc((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 18)) * Constants.P2_6);
            i += 18;
            mEph.setOmega((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 32)) * Constants.P2_31 * Constants.PI_ORBIT);
            i += 32;
            mEph.setOmegaDot((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 24)) * Constants.P2_43 * Constants.PI_ORBIT);
            i += 24;
            i += 10;  //tgd word
            i += 10;  //tgd word
            mEph.setSvh((int) Bits.bitsToUInt(Bits.subset(bits, i, 1)));
            i += 1;
        }
        catch (Exception e)
        {
            Log.e("STREAMS", "RTCM3 1045 error length");
            return -1;
        }

        return mEph;
    }
}