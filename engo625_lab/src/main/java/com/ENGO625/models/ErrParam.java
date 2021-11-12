package com.ENGO625.models;

public class ErrParam {

	private double varUERE;
	// Diagonal elements of DOP matrix
	private double[] dopDiag;

	public ErrParam(double varUERE, double[] dopDiag) {
		super();
		this.varUERE = varUERE;
		this.dopDiag = dopDiag;
	}

	public double getVarUERE() {
		return varUERE;
	}

	public double[] getDopDiag() {
		return dopDiag;
	}
}
