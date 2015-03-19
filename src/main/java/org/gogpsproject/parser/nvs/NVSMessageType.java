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

package org.gogpsproject.parser.nvs;

public class NVSMessageType {
	public final static int CLASS_NAV = 0;
	public final static int CLASS_RXM = 1;
	public final static int CLASS_INF = 2;
	public final static int CLASS_ACK = 3;
	public final static int CLASS_CFG = 4;
	public final static int CLASS_UPD = 5;
	public final static int CLASS_MON = 6;
	public final static int CLASS_AID = 7;
	public final static int CLASS_TIM = 8;
	public final static int CLASS_NMEA = 9;
	public final static int CLASS_PUBX = 10;

	public final static int ACK_ACK = 11;
	public final static int ACK_NAK = 12;


	public final static int AID_REQ = 13;
	public final static int AID_DATA = 14;
	public final static int AID_INI = 15;
	public final static int AID_HUI = 16;
	public final static int AID_ALM = 17;
	public final static int AID_EPH = 18;

	public final static int CFG_PRT = 19;
	public final static int CFG_USB = 20;
	public final static int CFG_MSG = 21;
	public final static int CFG_NMEA = 22;
	public final static int CFG_RATE = 23;
	public final static int CFG_CFG = 24;
	public final static int CFG_TP = 25;
	public final static int CFG_NAV2 = 26;
	public final static int CFG_DAT = 27;
	public final static int CFG_INF = 28;
	public final static int CFG_RST = 29;
	public final static int CFG_RXM = 30;
	public final static int CFG_ANT = 31;
	public final static int CFG_FXN = 32;
	public final static int CFG_SBAS = 33;
	public final static int CFG_LIC = 34;
	public final static int CFG_TM = 35;
	public final static int CFG_TM2 = 36;
	public final static int CFG_TMODE = 37;
	public final static int CFG_EKF = 38;

	public final static int INF_ERROR = 39;
	public final static int INF_WARNING = 40;
	public final static int INF_NOTICE = 41;
	public final static int INF_TEST = 42;
	public final static int INF_DEBUG = 43;
	public final static int INF_USER = 44;

	public final static int MON_SCHD = 45;
	public final static int MON_IO = 46;
	public final static int MON_MSGPP = 47;
	public final static int MON_RXBUF = 48;
	public final static int MON_TXBUF = 49;
	public final static int MON_HW = 50;
	public final static int MON_IPC = 51;
	public final static int MON_USB = 52;
	public final static int MON_VER = 53;
	public final static int MON_EXCEPT = 54;

	public final static int NAV_POSECEF = 55;
	public final static int NAV_POSLLH = 56;
	public final static int NAV_POSUTM = 57;
	public final static int NAV_DOP = 58;
	public final static int NAV_STATUS = 59;
	public final static int NAV_SOL = 60;
	public final static int NAV_VELECEF = 61;
	public final static int NAV_VELNED = 62;
	public final static int NAV_TIMEGPS = 63;
	public final static int NAV_TIMEUTC = 64;
	public final static int NAV_CLOCK = 65;
	public final static int NAV_SVINFO = 66;
	public final static int NAV_DGPS = 67;
	public final static int NAV_SBAS = 68;
	public final static int NAV_EKFSTATUS = 69;

	public final static int NMEA_GGA = 70;
	public final static int NMEA_GLL = 71;
	public final static int NMEA_GSA = 72;
	public final static int NMEA_GSV = 73;
	public final static int NMEA_RMC = 74;
	public final static int NMEA_VTG = 75;
	public final static int NMEA_GRS = 76;
	public final static int NMEA_GST = 77;
	public final static int NMEA_ZDA = 78;
	public final static int NMEA_GBS = 79;
	public final static int NMEA_DTM = 80;

	public final static int PUBX_A = 81;
	public final static int PUBX_B = 82;
	public final static int PUBX_C = 83;
	public final static int PUBX_D = 84;

	public final static int RXM_RAW = 85;
	public final static int RXM_SVSI = 86;
	public final static int RXM_SFRB = 87;
	public final static int RXM_ALM = 88;
	public final static int RXM_EPH = 89;
	public final static int RXM_POSREQ = 90;
	public final static int RXM_PMREQ = 98;

	public final static int TIM_TM = 90;
	public final static int TIM_TM2 = 91;
	public final static int TIM_TP = 92;
	public final static int TIM_SVIN = 93;

	public final static int UPD_DOWNL = 94;
	public final static int UPD_UPLOAD = 95;
	public final static int UPD_EXEC = 96;
	public final static int UPD_MEMCPY = 97;

	private static int classOut = 0;

	private static int idOut = 0;

	private static void setClassOut(int classOut) {
		NVSMessageType.classOut = classOut;
	}

	private static void setIdOut(int idOut) {
		NVSMessageType.idOut = idOut;
	}

	public NVSMessageType(int mclass, int msgtype) {
		getMsg(mclass, msgtype);
	}
	
	public NVSMessageType(String mclass, String msgtype) {
		getMsg(mclass, msgtype);
	}

	public int getClassOut() {
		return classOut;
	}

	public int getIdOut() {
		return idOut;
	}

	private int getMsg(int mclass, int msgtype) {
		try {
			switch (mclass) {

			case CLASS_NAV:
				setClassOut(0x01);
				switch (msgtype) {
				case NAV_POSECEF:
					setIdOut(0x01);
					return 0;
				case NAV_POSLLH:
					setIdOut(0x02);
					return 0;
				case NAV_POSUTM:
					setIdOut(0x08);
					return 0;
				case NAV_DOP:
					setIdOut(0x04);
					return 0;
				case NAV_STATUS:
					setIdOut(0x03);
					return 0;
				case NAV_SOL:
					setIdOut(0x06);
					return 0;
				case NAV_VELECEF:
					setIdOut(0x11);
					return 0;
				case NAV_VELNED:
					setIdOut(0x12);
					return 0;
				case NAV_TIMEGPS:
					setIdOut(0x20);
					return 0;
				case NAV_TIMEUTC:
					setIdOut(0x21);
					return 0;
				case NAV_CLOCK:
					setIdOut(0x22);
					return 0;
				case NAV_SVINFO:
					setIdOut(0x30);
					return 0;
				case NAV_DGPS:
					setIdOut(0x31);
					return 0;
				case NAV_SBAS:
					setIdOut(0x32);
					return 0;
				case NAV_EKFSTATUS:
					setIdOut(0x40);
					return 0;
				}

			case CLASS_RXM:
				setClassOut(0x02);
				switch (msgtype) {
					case RXM_RAW:
						setIdOut(0x10);
						return 0;
					case RXM_SVSI:
						setIdOut(0x20);
						return 0;
					case RXM_SFRB:
						setIdOut(0x11);
						return 0;
					case RXM_ALM:
						setIdOut(0x30);
						return 0;
					case RXM_EPH:
						setIdOut(0x31);
						return 0;
					case RXM_POSREQ:
						setIdOut(0x40);
						return 0;
					case RXM_PMREQ:
						setIdOut(0x41);
						return 0;
				}

			case CLASS_INF:
				setClassOut(0x04);
				switch (msgtype) {
				case INF_ERROR:
					setIdOut(0x00);
					return 0;
				case INF_WARNING:
					setIdOut(0x01);
					return 0;
				case INF_NOTICE:
					setIdOut(0x02);
					return 0;
				case INF_TEST:
					setIdOut(0x03);
					return 0;
				case INF_DEBUG:
					setIdOut(0x04);
					return 0;
				case INF_USER:
					setIdOut(0x07);
					return 0;
				}
			case CLASS_ACK:
				setClassOut(0x05);
				switch (msgtype) {
				case ACK_ACK:
					setIdOut(0x01);
					return 0;
				case ACK_NAK:
					setIdOut(0x00);
					return 0;
				}

			case CLASS_CFG:
				setClassOut(0x06);
				switch (msgtype) {
				case CFG_PRT:
					setIdOut(0x00);
					return 0;
				case CFG_USB:
					setIdOut(0x1B);
					return 0;
				case CFG_MSG:
					setIdOut(0x01);
					return 0;
				case CFG_NMEA:
					setIdOut(0x17);
					return 0;
				case CFG_RATE:
					setIdOut(0x08);
					return 0;
				case CFG_CFG:
					setIdOut(0x09);
					return 0;
				case CFG_TP:
					setIdOut(0x07);
					return 0;
				case CFG_NAV2:
					setIdOut(0x1A);
					return 0;
				case CFG_DAT:
					setIdOut(0x06);
					return 0;
				case CFG_INF:
					setIdOut(0x02);
					return 0;
				case CFG_RST:
					setIdOut(0x04);
					return 0;
				case CFG_RXM:
					setIdOut(0x11);
					return 0;
				case CFG_ANT:
					setIdOut(0x13);
					return 0;
				case CFG_FXN:
					setIdOut(0x0E);
					return 0;
				case CFG_SBAS:
					setIdOut(0x16);
					return 0;
				case CFG_LIC:
					setIdOut(0x80);
					return 0;
				case CFG_TM:
					setIdOut(0x10);
					return 0;
				case CFG_TM2:
					setIdOut(0x19);
					return 0;
				case CFG_TMODE:
					setIdOut(0x1D);
					return 0;
				case CFG_EKF:
					setIdOut(0x12);
					return 0;
				}

			case CLASS_UPD:
				setClassOut(0x09);
				switch (msgtype) {
				case UPD_DOWNL:
					setIdOut(0x01);
					return 0;
				case UPD_UPLOAD:
					setIdOut(0x02);
					return 0;
				case UPD_EXEC:
					setIdOut(0x03);
					return 0;
				case UPD_MEMCPY:
					setIdOut(0x04);
					return 0;
				}

			case CLASS_MON:
				setClassOut(0x0A);
				switch (msgtype) {
				case MON_SCHD:
					setIdOut(0x01);
					return 0;
				case MON_IO:
					setIdOut(0x02);
					return 0;
				case MON_MSGPP:
					setIdOut(0x06);
					return 0;
				case MON_RXBUF:
					setIdOut(0x07);
					return 0;
				case MON_TXBUF:
					setIdOut(0x08);
					return 0;
				case MON_HW:
					setIdOut(0x09);
					return 0;
				case MON_IPC:
					setIdOut(0x03);
					return 0;
				case MON_USB:
					setIdOut(0x0A);
					return 0;
				case MON_VER:
					setIdOut(0x04);
					return 0;
				case MON_EXCEPT:
					setIdOut(0x05);
					return 0;
				}

			case CLASS_AID:
				setClassOut(0x0B);
				switch (msgtype) {
				case AID_REQ:
					setIdOut(0x00);
					return 0;
				case AID_DATA:
					setIdOut(0x10);
					return 0;
				case AID_INI:
					setIdOut(0x01);
					return 0;
				case AID_HUI:
					setIdOut(0x02);
					return 0;
				case AID_ALM:
					setIdOut(0x30);
					return 0;
				case AID_EPH:
					setIdOut(0x31);
					return 0;
				}

			case CLASS_TIM:
				setClassOut(0x0D);
				switch (msgtype) {
				case TIM_TM:
					setIdOut(0x02);
					return 0;
				case TIM_TM2:
					setIdOut(0x03);
					return 0;
				case TIM_TP:
					setIdOut(0x01);
					return 0;
				case TIM_SVIN:
					setIdOut(0x04);
					return 0;
				}
			case CLASS_NMEA:
				setClassOut(0xF0);
				switch (msgtype) {
				case NMEA_GGA:
					setIdOut(0x00);
					return 0;
				case NMEA_GLL:
					setIdOut(0x01);
					return 0;
				case NMEA_GSA:
					setIdOut(0x02);
					return 0;
				case NMEA_GSV:
					setIdOut(0x03);
					return 0;
				case NMEA_RMC:
					setIdOut(0x04);
					return 0;
				case NMEA_VTG:
					setIdOut(0x05);
					return 0;
				case NMEA_GRS:
					setIdOut(0x06);
					return 0;
				case NMEA_GST:
					setIdOut(0x07);
					return 0;
				case NMEA_ZDA:
					setIdOut(0x08);
					return 0;
				case NMEA_GBS:
					setIdOut(0x09);
					return 0;
				case NMEA_DTM:
					setIdOut(0x0A);
					return 0;
				}

			case CLASS_PUBX:
				setClassOut(0xF1);
				switch (msgtype) {
				case PUBX_A:
					setIdOut(0x00);
					return 0;
				case PUBX_B:
					setIdOut(0x01);
					return 0;
				case PUBX_C:
					setIdOut(0x03);
					return 0;
				case PUBX_D:
					setIdOut(0x04);
					return 0;
				}
				// return 0;
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		return 1;

	}
	
	private int getMsg(String mclass, String msgtype) {
		try {
			if (mclass.equals("NAV")) {
				setClassOut(CLASS_NAV);
				if (msgtype.equals("POSECEF")) {
					setIdOut(NAV_POSECEF);
					return 0;
				} else if (msgtype.equals("POSLLH")) {
					setIdOut(NAV_POSLLH);
					return 0;
				} else if (msgtype.equals("POSUTM")) {
					setIdOut(NAV_POSUTM);
					return 0;
				} else if (msgtype.equals("DOP")) {
					setIdOut(NAV_DOP);
					return 0;
				} else if (msgtype.equals("STATUS")) {
					setIdOut(NAV_STATUS);
					return 0;
				} else if (msgtype.equals("SOL")) {
					setIdOut(NAV_SOL);
					return 0;
				} else if (msgtype.equals("VELECEF")) {
					setIdOut(NAV_VELECEF);
					return 0;
				} else if (msgtype.equals("VELNED")) {
					setIdOut(NAV_VELNED);
					return 0;
				} else if (msgtype.equals("TIMEGPS")) {
					setIdOut(NAV_TIMEGPS);
					return 0;
				} else if (msgtype.equals("TIMEUTC")) {
					setIdOut(NAV_TIMEUTC);
					return 0;
				} else if (msgtype.equals("CLOCK")) {
					setIdOut(NAV_CLOCK);
					return 0;
				} else if (msgtype.equals("SVINFO")) {
					setIdOut(NAV_SVINFO);
					return 0;
				} else if (msgtype.equals("DGPS")) {
					setIdOut(NAV_DGPS);
					return 0;
				} else if (msgtype.equals("SBAS")) {
					setIdOut(NAV_SBAS);
					return 0;
				} else if (msgtype.equals("EKFSTATUS")) {
					setIdOut(NAV_EKFSTATUS);
					return 0;
				}
			} else if (mclass.equals("RXM")) {
				setClassOut(CLASS_RXM);
				if (msgtype.equals("RAW")) {
					setIdOut(RXM_RAW);
					return 0;
				} else if (msgtype.equals("SVSI")) {
					setIdOut(RXM_SVSI);
					return 0;
				} else if (msgtype.equals("SFRB")) {
					setIdOut(RXM_SFRB);
					return 0;
				} else if (msgtype.equals("ALM")) {
					setIdOut(RXM_ALM);
					return 0;
				} else if (msgtype.equals("EPH")) {
					setIdOut(RXM_EPH);
					return 0;
				} else if (msgtype.equals("POSREQ")) {
					setIdOut(RXM_POSREQ);
					return 0;
				} else if (msgtype.equals("PMREQ")) {
					setIdOut(RXM_PMREQ);
					return 0;
				}

			} else if (mclass.equals("INF")) {
				setClassOut(CLASS_INF);
				if (msgtype.equals("ERROR")) {
					setIdOut(INF_ERROR);
					return 0;
				} else if (msgtype.equals("WARNING")) {
					setIdOut(INF_WARNING);
					return 0;
				} else if (msgtype.equals("NOTICE")) {
					setIdOut(INF_NOTICE);
					return 0;
				} else if (msgtype.equals("TEST")) {
					setIdOut(INF_TEST);
					return 0;
				} else if (msgtype.equals("DEBUG")) {
					setIdOut(INF_DEBUG);
					return 0;
				} else if (msgtype.equals("USER")) {
					setIdOut(INF_USER);
					return 0;
				}
			} else if (mclass.equals("ACK")) {
				setClassOut(CLASS_ACK);
				if (msgtype.equals("ACK")) {
					setIdOut(ACK_ACK);
					return 0;
				} else if (msgtype.equals("NAK")) {
					setIdOut(ACK_NAK);
					return 0;
				}

			} else if (mclass.equals("CFG")) {
				setClassOut(CLASS_CFG);
				if (msgtype.equals("PRT")) {
					setIdOut(CFG_PRT);
					return 0;
				} else if (msgtype.equals("USB")) {
					setIdOut(CFG_USB);
					return 0;
				} else if (msgtype.equals("MSG")) {
					setIdOut(CFG_MSG);
					return 0;
				} else if (msgtype.equals("NMEA")) {
					setIdOut(CFG_NMEA);
					return 0;
				} else if (msgtype.equals("RATE")) {
					setIdOut(CFG_RATE);
					return 0;
				} else if (msgtype.equals("CFG")) {
					setIdOut(CFG_CFG);
					return 0;
				} else if (msgtype.equals("TP")) {
					setIdOut(CFG_TP);
					return 0;
				} else if (msgtype.equals("NAV2")) {
					setIdOut(CFG_NAV2);
					return 0;
				} else if (msgtype.equals("DAT")) {
					setIdOut(CFG_DAT);
					return 0;
				} else if (msgtype.equals("INF")) {
					setIdOut(CFG_INF);
					return 0;
				} else if (msgtype.equals("RST")) {
					setIdOut(CFG_RST);
					return 0;
				} else if (msgtype.equals("RXM")) {
					setIdOut(CFG_RXM);
					return 0;
				} else if (msgtype.equals("ANT")) {
					setIdOut(CFG_ANT);
					return 0;
				} else if (msgtype.equals("FXN")) {
					setIdOut(CFG_FXN);
					return 0;
				} else if (msgtype.equals("SBAS")) {
					setIdOut(CFG_SBAS);
					return 0;
				} else if (msgtype.equals("LIC")) {
					setIdOut(CFG_LIC);
					return 0;
				} else if (msgtype.equals("TM")) {
					setIdOut(CFG_TM);
					return 0;
				} else if (msgtype.equals("TM2")) {
					setIdOut(CFG_TM2);
					return 0;
				} else if (msgtype.equals("TMODE")) {
					setIdOut(CFG_TMODE);
					return 0;
				} else if (msgtype.equals("EKF")) {
					setIdOut(CFG_EKF);
					return 0;
				}

			} else if (mclass.equals("UPD")) {
				setClassOut(CLASS_UPD);
				if (msgtype.equals("DOWNL")) {
					setIdOut(UPD_DOWNL);
					return 0;
				} else if (msgtype.equals("UPLOAD")) {
					setIdOut(UPD_UPLOAD);
					return 0;
				} else if (msgtype.equals("EXEC")) {
					setIdOut(UPD_EXEC);
					return 0;
				} else if (msgtype.equals("MEMCPY")) {
					setIdOut(UPD_MEMCPY);
					return 0;
				}

			} else if (mclass.equals("MON")) {
				setClassOut(CLASS_MON);
				if (msgtype.equals("SCHD")) {
					setIdOut(MON_SCHD);
					return 0;
				} else if (msgtype.equals("IO")) {
					setIdOut(MON_IO);
					return 0;
				} else if (msgtype.equals("MSGPP")) {
					setIdOut(MON_MSGPP);
					return 0;
				} else if (msgtype.equals("RXBUF")) {
					setIdOut(MON_RXBUF);
					return 0;
				} else if (msgtype.equals("TXBUF")) {
					setIdOut(MON_TXBUF);
					return 0;
				} else if (msgtype.equals("HW")) {
					setIdOut(MON_HW);
					return 0;
				} else if (msgtype.equals("IPC")) {
					setIdOut(MON_IPC);
					return 0;
				} else if (msgtype.equals("USB")) {
					setIdOut(MON_USB);
					return 0;
				} else if (msgtype.equals("VER")) {
					setIdOut(MON_VER);
					return 0;
				} else if (msgtype.equals("EXCEPT")) {
					setIdOut(MON_EXCEPT);
					return 0;
				}

			} else if (mclass.equals("AID")) {
				setClassOut(CLASS_AID);
				if (msgtype.equals("REQ")) {
					setIdOut(AID_REQ);
					return 0;
				} else if (msgtype.equals("DATA")) {
					setIdOut(AID_DATA);
					return 0;
				} else if (msgtype.equals("INI")) {
					setIdOut(AID_INI);
					return 0;
				} else if (msgtype.equals("HUI")) {
					setIdOut(AID_HUI);
					return 0;
				} else if (msgtype.equals("ALM")) {
					setIdOut(AID_ALM);
					return 0;
				} else if (msgtype.equals("EPH")) {
					setIdOut(AID_EPH);
					return 0;
				}

			} else if (mclass.equals("TIM")) {
				setClassOut(CLASS_TIM);
				if (msgtype.equals("TM")) {
					setIdOut(TIM_TM);
					return 0;
				} else if (msgtype.equals("TM2")) {
					setIdOut(TIM_TM2);
					return 0;
				} else if (msgtype.equals("TP")) {
					setIdOut(TIM_TP);
					return 0;
				} else if (msgtype.equals("SVIN")) {
					setIdOut(TIM_SVIN);
					return 0;
				}
			} else if (mclass.equals("NMEA")) {
				setClassOut(CLASS_NMEA);
				if (msgtype.equals("GGA")) {
					setIdOut(NMEA_GGA);
					return 0;
				} else if (msgtype.equals("GLL")) {
					setIdOut(NMEA_GLL);
					return 0;
				} else if (msgtype.equals("GSA")) {
					setIdOut(NMEA_GSA);
					return 0;
				} else if (msgtype.equals("GSV")) {
					setIdOut(NMEA_GSV);
					return 0;
				} else if (msgtype.equals("RMC")) {
					setIdOut(NMEA_RMC);
					return 0;
				} else if (msgtype.equals("VTG")) {
					setIdOut(NMEA_VTG);
					return 0;
				} else if (msgtype.equals("GRS")) {
					setIdOut(NMEA_GRS);
					return 0;
				} else if (msgtype.equals("GST")) {
					setIdOut(NMEA_GST);
					return 0;
				} else if (msgtype.equals("ZDA")) {
					setIdOut(NMEA_ZDA);
					return 0;
				} else if (msgtype.equals("GBS")) {
					setIdOut(NMEA_GBS);
					return 0;
				} else if (msgtype.equals("DTM")) {
					setIdOut(NMEA_DTM);
					return 0;
				}

			} else if (mclass.equals("PUBX")) {
				setClassOut(CLASS_PUBX);
				if (msgtype.equals("A")) {
					setIdOut(PUBX_A);
					return 0;
				} else if (msgtype.equals("B")) {
					setIdOut(PUBX_B);
					return 0;
				} else if (msgtype.equals("C")) {
					setIdOut(PUBX_C);
					return 0;
				} else if (msgtype.equals("D")) {
					setIdOut(PUBX_D);
					return 0;
				}
				// return 0;
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		return 1;

	}

}
