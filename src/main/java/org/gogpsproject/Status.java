package org.gogpsproject;

/**
 * All the values that the goGPS object can be in during processing
 */
public enum Status {
  None,
  Valid,
  NoAprioriPos,
  EphNotFound,
  NotEnoughSats,
  Low_Sat, 
  MaxCorrection, 
  MaxHDOP, 
  MaxEres,
  aPosterioriPR, 
  Exception, /* Generic error, none of the above */
}