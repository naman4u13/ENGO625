package com.ENGO625.models;

public class Observation {

	// GPS time
	private int t;
	// Satellite PRN
	private int prn;
	// Pseudorange(in m)
	private double pseduorange;
	// L1 Carrier Phase(in meters)
	private double phaseL1;
	// L2 Carrier Phase(in cycles)
	private double phaseL2;
	// Doppler of L1
	private double dopplerL1;
	// ECEF(in m)
	private double[] ecef;
	// Vel(in m/s)
	private double[] vel;
	// Elevation angle
	private double elevAngle;
	// Azimuth Angle
	private double azmAngle;
	// Range Rate
	private double rangeRate;
	// GPS L1 frequency(in Hz)
	private static final double freqL1 = 1575.42e6;
	// Speed of Light
	private static final double SPEED_OF_LIGHT = 299792458;
	// GPS L1 wavelength
	private static final double wavelengthL1 = SPEED_OF_LIGHT / freqL1;
	// Phase Lock/ Cycle Slip indicator
	private boolean isPhaseLocked = false;
	// unit LOS - specifically for RTK/Baseline processing
	private double[] unitLOS;

	// Specifically for Baseline/RTK based estimation
	public Observation(int t, int prn, double pseduorange, double phaseL1, double[] unitLOS, boolean isPhaseLocked) {
		super();
		this.t = t;
		this.prn = prn;
		this.pseduorange = pseduorange;
		this.phaseL1 = phaseL1;
		this.unitLOS = unitLOS;
		this.isPhaseLocked = isPhaseLocked;

	}

	public Observation(double[] data) {
		super();
		this.prn = (int) data[0];
		this.t = (int) data[1];
		this.pseduorange = data[2];
		this.phaseL1 = -data[3] * wavelengthL1;
		this.dopplerL1 = data[4];
		this.phaseL2 = data[5];
		this.rangeRate = this.dopplerL1 * wavelengthL1;

	}

	public int getT() {
		return t;
	}

	public int getPrn() {
		return prn;
	}

	public double getPseduorange() {
		return pseduorange;
	}

	public double getPhaseL1() {
		return phaseL1;
	}

	public double getPhaseL2() {
		return phaseL2;
	}

	public double getDoppler() {
		return dopplerL1;
	}

	public double[] getEcef() {
		return ecef;
	}

	public void setEcef(double[] ecef) {
		this.ecef = ecef;
	}

	public double[] getVel() {
		return vel;
	}

	public void setVel(double[] vel) {
		this.vel = vel;
	}

	public double getElevAngle() {
		return elevAngle;
	}

	public void setElevAzmAngle(double[] elevAzm) {
		this.elevAngle = elevAzm[0];
		this.azmAngle = elevAzm[1];
	}

	public double getAzmAngle() {
		return azmAngle;
	}

	public void setPseduorange(double pseduorange) {
		this.pseduorange = pseduorange;
	}

	public double getRangeRate() {
		return rangeRate;
	}

	public boolean isPhaseLocked() {
		return isPhaseLocked;
	}

	public void setPhaseLocked(boolean isPhaseLocked) {
		this.isPhaseLocked = isPhaseLocked;
	}

	public double[] getUnitLOS() {
		return unitLOS;
	}

	public void setUnitLOS(double[] unitLOS) {
		this.unitLOS = unitLOS;
	}
}
