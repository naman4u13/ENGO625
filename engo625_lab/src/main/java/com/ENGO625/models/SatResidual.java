package com.ENGO625.models;

public class SatResidual {

	// GPS Time
	private int t;
	// Elevation Angle(Radians)
	private double elevAngle;
	// Residual(meter)
	private double residual;

	public SatResidual(int t, double elevAngle, double residual) {
		super();
		this.t = t;
		this.elevAngle = elevAngle;
		this.residual = residual;
	}

	public int getT() {
		return t;
	}

	public double getElevAngle() {
		return elevAngle;
	}

	public double getResidual() {
		return residual;
	}

}
