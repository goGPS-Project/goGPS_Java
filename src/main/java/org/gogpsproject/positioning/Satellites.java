package org.gogpsproject.positioning;

import java.util.ArrayList;

import org.gogpsproject.SatellitePosition;

public class Satellites {

  /** Absolute position of all visible satellites (ECEF) */
  SatellitePosition[] pos; 

  /** List of satellites available for processing */
  ArrayList<Integer> avail; 
 
  /** List of satellites available for processing */
  ArrayList<Integer> availPhase; 
  
  /** List of satellite Types available for processing */
  ArrayList<Character> typeAvail; 
  
  /** List of satellite Type available for processing */
  ArrayList<Character> typeAvailPhase; 
  
  /** List of satellite Types & Id available for processing */
  ArrayList<String> gnssAvail;  
  
  /** List of satellite Types & Id available for processing */
  ArrayList<String> gnssAvailPhase;  

  /** Index of the satellite with highest elevation in satAvail list */
  int pivot; 
  
  /** @return the number of available satellites */
  public int getAvailNumber() {
    return avail.size();
  }

  /** @return the number of available satellites (with phase) */
  public int getAvailPhaseNumber() {
    return availPhase.size();
  }
  
  public String getAvailGnssSystems(){
    if( typeAvail.isEmpty()) return "";
    String GnssSys = "";
    for(int i=0;i< typeAvail.size();i++) {
      if (GnssSys.indexOf(( typeAvail.get(i))) < 0)
        GnssSys = GnssSys + typeAvail.get(i);
    }
    return GnssSys;
  }
  
}
