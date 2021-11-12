package com.ENGO625.models;

public class Observation {

	// GPS time
	private int t;
	// Satellite PRN
	private int prn;
	// Pseudorange(in m)
	private double pseduorange;
	// L1 Carrier Phase(in cycles)
	private double phaseL1;
	// L2 Carrier Phase(in cycles)
	private double phaseL2;
	// Doppler
	private double doppler;
	// ECEF(in m)
	private double[] ecef;
	// Vel(in m/s)
	private double[] vel;
	// Elevation angle
	private double elevAngle;
	// Azimuth Angle
	private double azmAngle;

	public Observation(int t, int prn, double pseduorange, double phaseL1, double phaseL2, double doppler) {
		super();
		this.t = t;
		this.prn = prn;
		this.pseduorange = pseduorange;
		this.phaseL1 = phaseL1;
		this.phaseL2 = phaseL2;
		this.doppler = doppler;
	}

	public Observation(double[] data) {
		super();
		this.prn = (int) data[0];
		this.t = (int) data[1];
		this.pseduorange = data[2];
		this.phaseL1 = data[3];
		this.phaseL2 = data[4];
		this.doppler = data[5];
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
		return doppler;
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
}
