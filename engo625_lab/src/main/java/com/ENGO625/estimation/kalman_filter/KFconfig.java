package com.ENGO625.estimation.kalman_filter;

import java.util.stream.IntStream;

import org.ejml.simple.SimpleMatrix;

import com.ENGO625.util.LatLonUtil;

public class KFconfig extends KF {

	private final double SpeedofLight = 299792458;
	private final double c2 = SpeedofLight * SpeedofLight;
	// Typical Allan Variance Coefficients for TCXO (low quality)
	private final double h0 = 2E-19;
	private final double h_2 = 2E-20;
	private final double sf = h0 / 2;
	private final double sg = 2 * Math.PI * Math.PI * h_2;

	public void config() {

		/*
		 * The process noise for position vector will be initialized in ENU frame and
		 * will then be changed to ECEF frame. Rotation matrix 'R' will be computed to
		 * perform the coordinate transform.
		 */
		int n = getState().getNumElements();
		double[][] F = new double[n][n];
		double[][] Q = new double[n][n];
		IntStream.range(0, n).forEach(i -> F[i][i] = 1);
		super.configure(F, Q);

	}

	/*
	 * Compute Rotation Matrix to transform from ENU to ECEF frame. Reference -
	 * https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#
	 * From_ENU_to_ECEF
	 */
	private SimpleMatrix getR(double[] ecef) {
		double[] llh = LatLonUtil.ecef2lla(ecef);
		double lat = Math.toRadians(llh[0]);
		double lon = Math.toRadians(llh[1]);
		double[][] r = new double[][] {
				{ -Math.sin(lon), -Math.sin(lat) * Math.cos(lon), Math.cos(lat) * Math.cos(lon) },
				{ Math.cos(lon), -Math.sin(lat) * Math.sin(lon), Math.cos(lat) * Math.sin(lon) },
				{ 0, Math.cos(lat), Math.sin(lat) } };
		SimpleMatrix R = new SimpleMatrix(r);
		return R;
	}

	// Perform transformation from ENU to ECEF
	private double[] enuToEcef(SimpleMatrix R, double[] _enuParam) {

		SimpleMatrix enuParam = new SimpleMatrix(3, 1, false, _enuParam);
		SimpleMatrix _ecefParam = R.mult(enuParam);
		double[] ecefParam = new double[] { _ecefParam.get(0), _ecefParam.get(1), _ecefParam.get(2) };
		return ecefParam;
	}

}
