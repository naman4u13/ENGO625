package com.ENGO625.util;

import org.ejml.simple.SimpleMatrix;

public class LatLonUtil {

	// All are WGS-84 params
	// Semi-major axis or Equatorial radius
	private static final double a = 6378137;
	// flattening
	private static final double f = 1 / 298.257223563;
	// Semi-minor axis or Polar radius
	private static final double b = 6356752.314245;
	private static final double e = Math.sqrt((Math.pow(a, 2) - Math.pow(b, 2)) / Math.pow(a, 2));
	private static final double e2 = Math.sqrt((Math.pow(a, 2) - Math.pow(b, 2)) / Math.pow(b, 2));

	public static double[] ecef2lla(double[] ECEF) {

		double x = ECEF[0];
		double y = ECEF[1];
		double z = ECEF[2];
		double[] lla = { 0, 0, 0 };
		double lat = 0, lon, height, N, theta, p;

		p = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

		theta = Math.atan((z * a) / (p * b));

		lon = Math.atan(y / x);

		if (x < 0) {
			if (y > 0) {
				lon = Math.PI + lon;
			} else {
				lon = -Math.PI + lon;
			}
		}
		for (int i = 0; i < 3; i++) {
			lat = Math.atan(((z + Math.pow(e2, 2) * b * Math.pow(Math.sin(theta), 3))
					/ ((p - Math.pow(e, 2) * a * Math.pow(Math.cos(theta), 3)))));
			theta = Math.atan((Math.tan(lat) * b) / (a));

		}

		N = a / (Math.sqrt(1 - (Math.pow(e, 2) * Math.pow(Math.sin(lat), 2))));

		height = (p * Math.cos(lat)) + ((z + (Math.pow(e, 2) * N * Math.sin(lat))) * Math.sin(lat)) - N;

		lon = lon * 180 / Math.PI;

		lat = lat * 180 / Math.PI;
		lla[0] = lat;
		lla[1] = lon;
		lla[2] = height;
		return lla;
	}

	public static double[] lla2ecef(double[] lla) {

		double lat = Math.toRadians(lla[0]);
		double lon = Math.toRadians(lla[1]);
		double h = lla[2];
		double N = a / Math.sqrt(1 - Math.pow(e * Math.sin(lat), 2));
		double x = (N + h) * Math.cos(lat) * Math.cos(lon);
		double y = (N + h) * Math.cos(lat) * Math.sin(lon);
		double z = ((N * Math.pow(b / a, 2)) + h) * Math.sin(lat);
		return new double[] { x, y, z };
	}

	// Reference -
	// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
	public static double[] ecef2enu(double[] ecef, double[] refEcef) {
		double[] _diff = new double[] { ecef[0] - refEcef[0], ecef[1] - refEcef[1], ecef[2] - refEcef[2] };
		SimpleMatrix diff = new SimpleMatrix(3, 1, false, _diff);
		double[] llh = ecef2lla(refEcef);
		double lat = Math.toRadians(llh[0]);
		double lon = Math.toRadians(llh[1]);
		double[][] _R = new double[][] { { -Math.sin(lon), Math.cos(lon), 0 },
				{ -Math.sin(lat) * Math.cos(lon), -Math.sin(lat) * Math.sin(lon), Math.cos(lat) },
				{ Math.cos(lat) * Math.cos(lon), Math.cos(lat) * Math.sin(lon), Math.sin(lat) } };
		SimpleMatrix R = new SimpleMatrix(_R);
		SimpleMatrix _enu = R.mult(diff);
		double[] enu = new double[] { _enu.get(0), _enu.get(1), _enu.get(2) };
		return enu;
	}

	public static double[][] getEcef2EnuRotMat(double[] refEcef) {
		double[] llh = ecef2lla(refEcef);
		double lat = Math.toRadians(llh[0]);
		double lon = Math.toRadians(llh[1]);
		double[][] R = new double[][] { { -Math.sin(lon), Math.cos(lon), 0 },
				{ -Math.sin(lat) * Math.cos(lon), -Math.sin(lat) * Math.sin(lon), Math.cos(lat) },
				{ Math.cos(lat) * Math.cos(lon), Math.cos(lat) * Math.sin(lon), Math.sin(lat) } };

		return R;
	}

	/*
	 * Compute Rotation Matrix to transform from ENU to ECEF frame. Reference -
	 * https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#
	 * From_ENU_to_ECEF
	 */
	public static SimpleMatrix getEnu2EcefRotMat(double[] ecef) {
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

}
