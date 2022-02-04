package org.gogpsproject.producer;

import org.gogpsproject.ephemeris.EphGps;

public interface RinexNavigationProducer extends NavigationProducer {

	public EphGps findEph(long unixTime, int satID, char satType);

}
