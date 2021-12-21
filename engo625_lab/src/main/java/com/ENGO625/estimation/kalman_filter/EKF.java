package com.ENGO625.estimation.kalman_filter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.stream.IntStream;

import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.simple.SimpleMatrix;

import com.ENGO625.models.Observation;
import com.ENGO625.models.Satellite;

public class EKF {

	private KFconfig kfObj;
	private double prObsNoiseVar;

	public EKF() {
		kfObj = new KFconfig();
	}

	public ArrayList<double[]> process(ArrayList<ArrayList<Observation>> baselineObsList, ArrayList<Integer> timeList,
			double[] intialBaseline) throws Exception {

		double[][] x = new double[3][1];
		double[][] P = new double[3][3];
		/*
		 * state XYZ is intialized WLS generated Rx position estimated using first epoch
		 * data, a-priori estimate error covariance matrix for state XYZ is therefore
		 * assigned 4 m^2 value. Other state variables are assigned infinite(big)
		 * variance
		 */
		IntStream.range(0, 3).forEach(i -> x[i][0] = intialBaseline[i]);
		IntStream.range(0, 3).forEach(i -> P[i][i] = 4);

		kfObj.setState_ProcessCov(x, P);
		// Begin iteration or recursion
		return iterate(baselineObsList, timeList);

	}

	private ArrayList<double[]> iterate(ArrayList<ArrayList<Observation>> baselineObsList, ArrayList<Integer> timeList)
			throws Exception {
		ArrayList<double[]> ecefList = new ArrayList<double[]>();
		ArrayList<Integer> order = new ArrayList<Integer>();
		for (int i = 0; i < timeList.size(); i++) {
			int n = baselineObsList.get(i).size();
			SimpleMatrix x = kfObj.getState();
			SimpleMatrix P = kfObj.getCovariance();
			SimpleMatrix _x = new SimpleMatrix(3 + n, 1);
			SimpleMatrix _P = new SimpleMatrix(3 + n, 3 + n);
			for (int j = 0; j < 3; j++) {
				_x.set(j, x.get(j));
				_P.set(j, j, P.get(j, j));
			}
			HashSet<Integer> set = new HashSet();
			HashMap<Integer, Integer> map = new HashMap<Integer, Integer>();
			baselineObsList.get(i).stream().mapToInt(j -> j.getPrn()).forEach(set::add);
			ArrayList<Integer> newOrder = new ArrayList<Integer>();
			for (int j = 0; j < order.size(); j++) {
				int prn = order.get(j);
				if (set.contains(prn)) {
					newOrder.add(prn);
					map.put(prn, j);
					set.remove(prn);
				}
			}
			for (int prn : set) {
				map.put(prn, newOrder.size());
				newOrder.add(prn);
			}

			for (int j = 0; j < n; j++) {
				Observation obs = baselineObsList.get(i).get(j);
				int prn = obs.getPrn();

			}
			// Perform Predict and Update
			runFilter(deltaT, satList, flag);
			// Fetch Posteriori state estimate and estimate error covariance matrix
			SimpleMatrix x = kfObj.getState();
			SimpleMatrix P = kfObj.getCovariance();
			double[] estECEF = new double[] { x.get(0), x.get(1), x.get(2) };
			// Add position estimate to the list
			ecefList.add(estECEF);
			/*
			 * Check whether estimate error covariance matrix is positive semidefinite
			 * before further proceeding
			 */
			if (!MatrixFeatures_DDRM.isPositiveDefinite(P.getMatrix())) {

				throw new Exception("PositiveDefinite test Failed");
			}
			time = currentTime;

		}

		return ecefList;
	}

	private void runFilter(double deltaT, ArrayList<Satellite> satList, Flag flag) {

		// Satellite count
		int n = satList.size();

		// Assign Q and F matrix
		kfObj.config(deltaT, flag);
		kfObj.predict();

		SimpleMatrix x = kfObj.getState();
		double[] estECEF = new double[] { x.get(0), x.get(1), x.get(2) };
		double rxClkOff = x.get(3);// in meters

		/*
		 * H is the Jacobian matrix of partial derivatives Observation StateModel(h) of
		 * with respect to x
		 */
		double[][] H = getJacobian(satList, estECEF, x.numRows());
		// Measurement vector
		double[][] z = new double[n][1];
		IntStream.range(0, n).forEach(i -> z[i][0] = satList.get(i).getPseduorange());
		// Estimated Measurement vector
		double[][] ze = new double[n][1];
		IntStream.range(0, n)
				.forEach(i -> ze[i][0] = Math
						.sqrt(IntStream.range(0, 3).mapToDouble(j -> estECEF[j] - satList.get(i).getECEF()[j])
								.map(j -> j * j).reduce(0, (j, k) -> j + k))
						+ rxClkOff);
		// Measurement Noise
		double[][] R = new double[n][n];
		for (int i = 0; i < n; i++) {
			double elevAngle = Math.toRadians(satList.get(i).getElevation());
			double var = 1 / Math.pow(Math.sin(elevAngle), 2);
			R[i][i] = var;
		}
		// Perform Update Step
		kfObj.update(z, R, ze, H);

	}

	private double[][] getJacobian(ArrayList<Satellite> satList, double[] estECEF, int stateN) {
		int n = satList.size();
		double[][] H = new double[n][stateN];

		for (int i = 0; i < n; i++) {
			Satellite sat = satList.get(i);
			// Line of Sight vector
			double[] LOS = IntStream.range(0, 3).mapToDouble(j -> sat.getECEF()[j] - estECEF[j]).toArray();
			// Geometric Range
			double GR = Math.sqrt(Arrays.stream(LOS).map(j -> j * j).reduce(0.0, (j, k) -> j + k));
			// Converting LOS to unit vector
			final int _i = i;
			IntStream.range(0, 3).forEach(j -> H[_i][j] = -LOS[j] / GR);
			H[i][3] = 1;
		}

		return H;

	}
}
