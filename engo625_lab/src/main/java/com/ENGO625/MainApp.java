package com.ENGO625;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.stream.IntStream;

import org.ejml.simple.SimpleMatrix;
import org.json.JSONObject;

import com.ENGO625.estimation.IF_Weight;
import com.ENGO625.estimation.LinearLeastSquare;
import com.ENGO625.estimation.RobustEstimation;
import com.ENGO625.estimation.kalman_filter.EKF;
import com.ENGO625.models.CxParam;
import com.ENGO625.models.Observation;
import com.ENGO625.models.SatResidual;
import com.ENGO625.models.Satellite;
import com.ENGO625.util.ComputeEleAzm;
import com.ENGO625.util.CycleSlipDetection;
import com.ENGO625.util.GraphPlotter;
import com.ENGO625.util.LatLonUtil;
import com.ENGO625.util.Parser;

public class MainApp {

	private final static double SpeedofLight = 299792458;

	public static void main(String args[]) {

		try {
			// Path to store output file
			String path = "D:\\projects\\eclipse_projects\\UCalgary\\ENGO625\\results\\output_test2";
			File output = new File(path + ".txt");
			PrintStream stream;
			stream = new PrintStream(output);
			System.setOut(stream);
			// Map contains true ecef of remote and base station
			HashMap<String, double[]> trueEcef = Parser.getTrueEcef("TruthCoordinates.txt");
			// Satellite data map -> (k->(k,v)) = (GPSTime->(PRN->Satellite Data))
			HashMap<Integer, HashMap<Integer, Satellite>> satMap = Parser.getSat("Satellites.sat");
			// Base Station Observation data map -> (k,v) = (GPSTime->List of Observation)
			HashMap<Integer, ArrayList<Observation>> baseMap = Parser.getList("BaseL1L2.obs", Observation.class);
			// Remote Station Observation data map -> (k,v) = (GPSTime->List of Observation)
			HashMap<Integer, ArrayList<Observation>> remoteMap = Parser.getList("RemoteL1L2.obs", Observation.class);
			// Map based variable to store enu errors for each algorithm
			HashMap<String, ArrayList<double[]>> EnuMap = new HashMap<String, ArrayList<double[]>>();
			// Map based variable to store DOP and UERE parameters or Variance(dx)
			HashMap<String, ArrayList<CxParam>> CxMap = new HashMap<String, ArrayList<CxParam>>();
			// List variable to store satellite count used in estimation for each epoch
			ArrayList<Integer> satCountList = new ArrayList<Integer>();
			// Map based variable to store Satellite Residuals
			HashMap<String, HashMap<Integer, ArrayList<SatResidual>>> satResMap = new HashMap<String, HashMap<Integer, ArrayList<SatResidual>>>();
			// Map based variable to store Satellite data used in estimation - Pseudorange,
			// Doppler, Phase etc.
			HashMap<Integer, ArrayList<Observation>> satDataMap = new HashMap<Integer, ArrayList<Observation>>();
			// List storing GPS time for each epoch
			ArrayList<Integer> timeList = new ArrayList<Integer>();
			ArrayList<ArrayList<Observation>> baseObsList = new ArrayList<ArrayList<Observation>>();
			ArrayList<ArrayList<Observation>> remObsList = new ArrayList<ArrayList<Observation>>();
			/*
			 * Estimation option - (1. Satellite Trajectory, 2. Least Squares, 3. WLS, 4.
			 * Between Receiver Single Difference(BRSD), 5. Combined of option 2,3 and 4)
			 */
			int opt = 9;
			// Option 1 is to output a json file containing Satellite Coordinates
			if (opt == 1) {
				JSONObject json = new JSONObject(satMap);
				FileWriter file = new FileWriter(
						"D:\\projects\\eclipse_projects\\UCalgary\\ENGO625\\results\\satMap2.json");
				file.write(json.toString());
				file.close();
			}

			TreeMap<Integer, ArrayList<Observation>> remoteOrdMap = new TreeMap<Integer, ArrayList<Observation>>();
			// Copy all data from hashMap into TreeMap
			remoteOrdMap.putAll(remoteMap);
			// Remote Station ECEF
			double[] remoteEcef = trueEcef.get("Remote");
			// Base Station ECEF
			double[] baseEcef = trueEcef.get("Base");
			double dist = Math
					.sqrt(IntStream.range(0, 3).mapToDouble(i -> remoteEcef[i] - baseEcef[i]).map(i -> i * i).sum());
			// First Epoch GPSTime
			int t0 = remoteOrdMap.firstKey();
			// Iterate over remote station observation data
			for (Map.Entry ele : remoteOrdMap.entrySet()) {
				// GPS Time
				int t = (int) ele.getKey();

				if ((t - t0) > 3400) {
					break;
				}

				// Store GPS Time minus GPS Time of first epoch, to set timescale at zero
				timeList.add(t - t0);

				HashMap<Integer, Satellite> subSatMap = satMap.get(t);
				ArrayList<Observation> remObsvs = (ArrayList<Observation>) ele.getValue();
				ArrayList<Observation> baseObsvs = null;
				ArrayList<Observation> invalid = new ArrayList<Observation>();
				// Creating observation list to be used for estimation
				for (Observation obs : remObsvs) {
					int prn = obs.getPrn();
					// Check whether Satellite data is available for the epoch
					if (!subSatMap.containsKey(prn)) {
						invalid.add(obs);
						continue;
					}
					Satellite sat = subSatMap.get(prn);
					obs.setEcef(sat.getEcef());
					obs.setVel(sat.getVel());
					satDataMap.computeIfAbsent(prn, k -> new ArrayList<Observation>()).add(obs);
				}
				remObsvs.removeAll(invalid);
				// Satellite Count
				int n = remObsvs.size();
				/*
				 * If differencing option is selected, create a similar observation for Base
				 * Station data, and check whether same set of satellites is present in Remote
				 * Data
				 */
				if (opt == 4 || opt == 5 || opt == 7 || opt == 8 || opt == 9) {
					baseObsvs = baseMap.get(t);
					baseObsvs = modifyObsList(remObsvs, baseObsvs);
					remObsList.add(remObsvs);
					baseObsList.add(baseObsvs);
				}
				satCountList.add(n);
				if (opt == 6) {
					// Least Squares
					LinearLeastSquare lls = new LinearLeastSquare();
					// Estimated Remote ECEF
					double[] lsEcef = lls.processSP(remObsvs, false);

					EnuMap.computeIfAbsent("LS ", k -> new ArrayList<double[]>()).add(estimateENU(lsEcef, remoteEcef));
					for (int i = 0; i < n; i++) {
						double[] elevAzm = ComputeEleAzm.computeEleAzm(lsEcef, remObsvs.get(i).getEcef());
						remObsvs.get(i).setElevAzmAngle(elevAzm);
					}
					double[] ecef = lls.processSP(remObsvs, true);
					EnuMap.computeIfAbsent("WLS", k -> new ArrayList<double[]>()).add(estimateENU(ecef, remoteEcef));
					double[] robustEcef = RobustEstimation.processSP(remObsvs, ecef, IF_Weight.HUBER);
					EnuMap.computeIfAbsent("Robust LS Huber", k -> new ArrayList<double[]>())
							.add(estimateENU(robustEcef, remoteEcef));
					robustEcef = RobustEstimation.processSP(remObsvs, ecef, IF_Weight.HAMPEL);
					EnuMap.computeIfAbsent("Robust LS Hampel", k -> new ArrayList<double[]>())
							.add(estimateENU(robustEcef, remoteEcef));
					robustEcef = RobustEstimation.processSP(remObsvs, ecef, IF_Weight.ANDREW);
					EnuMap.computeIfAbsent("Robust LS Andrew", k -> new ArrayList<double[]>())
							.add(estimateENU(robustEcef, remoteEcef));

					CxParam Cx = lls.getCxParam();
					CxMap.computeIfAbsent("LS ", k -> new ArrayList<CxParam>()).add(Cx);
					CxMap.computeIfAbsent("WLS", k -> new ArrayList<CxParam>()).add(Cx);
					CxMap.computeIfAbsent("Robust LS Huber", k -> new ArrayList<CxParam>()).add(Cx);
					CxMap.computeIfAbsent("Robust LS Hampel", k -> new ArrayList<CxParam>()).add(Cx);
					CxMap.computeIfAbsent("Robust LS Andrew", k -> new ArrayList<CxParam>()).add(Cx);

				}
				if (opt == 7) {
					// Least Squares
					LinearLeastSquare lls = new LinearLeastSquare();
					// Estimated Remote ECEF
					double[] ecef = lls.processBRSD(baseObsvs, remObsvs, baseEcef);
					EnuMap.computeIfAbsent("BRSD", k -> new ArrayList<double[]>()).add(estimateENU(ecef, remoteEcef));
					double[] robustEcef = RobustEstimation.processBRSD(baseObsvs, remObsvs, baseEcef, ecef,
							IF_Weight.HUBER);
					EnuMap.computeIfAbsent("Robust BRSD Huber", k -> new ArrayList<double[]>())
							.add(estimateENU(robustEcef, remoteEcef));
					robustEcef = RobustEstimation.processBRSD(baseObsvs, remObsvs, baseEcef, ecef, IF_Weight.HAMPEL);
					EnuMap.computeIfAbsent("Robust BRSD Hampel", k -> new ArrayList<double[]>())
							.add(estimateENU(robustEcef, remoteEcef));
					robustEcef = RobustEstimation.processBRSD(baseObsvs, remObsvs, baseEcef, ecef, IF_Weight.ANDREW);
					EnuMap.computeIfAbsent("Robust BRSD Andrew", k -> new ArrayList<double[]>())
							.add(estimateENU(robustEcef, remoteEcef));
					CxParam Cx = lls.getCxParam();
					CxMap.computeIfAbsent("BRSD", k -> new ArrayList<CxParam>()).add(Cx);
					CxMap.computeIfAbsent("Robust BRSD Huber", k -> new ArrayList<CxParam>()).add(Cx);
					CxMap.computeIfAbsent("Robust BRSD Hampel", k -> new ArrayList<CxParam>()).add(Cx);
					CxMap.computeIfAbsent("Robust BRSD Andrew", k -> new ArrayList<CxParam>()).add(Cx);

				}
				switch (opt) {

				case 2, 3, 4, 5:

					// Least Squares
					LinearLeastSquare lls = new LinearLeastSquare();
					// Estimated Remote ECEF
					double[] ecef = lls.processSP(remObsvs, false);
					// Compute Elevation Angle
					for (int i = 0; i < n; i++) {
						double[] elevAzm = ComputeEleAzm.computeEleAzm(ecef, remObsvs.get(i).getEcef());
						remObsvs.get(i).setElevAzmAngle(elevAzm);
					}
					if (opt == 2 || opt == 5) {
						EnuMap.computeIfAbsent("LS", k -> new ArrayList<double[]>()).add(estimateENU(ecef, remoteEcef));
						CxParam Cx = lls.getCxParam();
						CxMap.computeIfAbsent("LS", k -> new ArrayList<CxParam>()).add(Cx);
						double[] residual = lls.getResidual();
						satResMap.computeIfAbsent("LS", k -> new HashMap<Integer, ArrayList<SatResidual>>());
						for (int i = 0; i < n; i++) {
							Observation obs = remObsvs.get(i);
							satResMap.get("LS").computeIfAbsent(obs.getPrn(), k -> new ArrayList<SatResidual>())
									.add(new SatResidual(t - t0, obs.getElevAngle(), residual[i]));
						}
					}
					// Weighted Least Squares
					if (opt == 3 || opt == 5) {

						lls = new LinearLeastSquare();
						ecef = lls.processSP(remObsvs, true);
						EnuMap.computeIfAbsent("WLS", k -> new ArrayList<double[]>())
								.add(estimateENU(ecef, remoteEcef));
						CxParam Cx = lls.getCxParam();
						CxMap.computeIfAbsent("WLS", k -> new ArrayList<CxParam>()).add(Cx);
						double[] residual = lls.getResidual();
						satResMap.computeIfAbsent("WLS", k -> new HashMap<Integer, ArrayList<SatResidual>>());
						for (int i = 0; i < n; i++) {
							Observation obs = remObsvs.get(i);
							satResMap.get("WLS").computeIfAbsent(obs.getPrn(), k -> new ArrayList<SatResidual>())
									.add(new SatResidual(t - t0, obs.getElevAngle(), residual[i]));
						}

					}
					// Between Receiver Single Differencing
					if (opt == 4 || opt == 5) {
						lls = new LinearLeastSquare();
						ecef = lls.processBRSD(baseObsvs, remObsvs, baseEcef);
						EnuMap.computeIfAbsent("BRSD", k -> new ArrayList<double[]>())
								.add(estimateENU(ecef, remoteEcef));
						CxParam Cx = lls.getCxParam();
						CxMap.computeIfAbsent("BRSD", k -> new ArrayList<CxParam>()).add(Cx);
						double[] residual = lls.getResidual();
						satResMap.computeIfAbsent("BRSD", k -> new HashMap<Integer, ArrayList<SatResidual>>());
						for (int i = 0; i < n; i++) {
							Observation obs = remObsvs.get(i);
							satResMap.get("BRSD").computeIfAbsent(obs.getPrn(), k -> new ArrayList<SatResidual>())
									.add(new SatResidual(t - t0, obs.getElevAngle(), residual[i]));
						}

					}
					break;

				}
			}

			// Cycle Slip Detection
			if (opt == 8) {
				CycleSlipDetection.phaseRateMethod(baseObsList, remObsList);
			}
			// RTK
			HashMap<String, HashMap<String, double[]>> rtkAmbMap = null;
			if (opt == 9) {
				LinearLeastSquare lls = new LinearLeastSquare();
				double[] estRemEcef = lls.processBRSD(baseObsList.get(0), remObsList.get(0), baseEcef);

				SimpleMatrix _R = new SimpleMatrix(LatLonUtil.getEnu2EcefRotMat(remoteEcef));
				SimpleMatrix R = new SimpleMatrix(LatLonUtil.getEcef2EnuRotMat(remoteEcef));
				// Base Satellite
				int refPRN = 11;
				// Perform Cycle Slip
				CycleSlipDetection.phaseRateMethod(baseObsList, remObsList);
				ArrayList<ArrayList<Observation>> baselineObsList = new ArrayList<ArrayList<Observation>>();
				int n = baseObsList.size();
				// Reject following satellites
				Set<Integer> reject = Set.of(9, 18, 22, 28, 8, 19);
				for (int i = 0; i < n; i++) {
					int m = baseObsList.get(i).size();
					ArrayList<Observation> obsList = new ArrayList<Observation>();
					int k = 0;
					boolean flag = false;
					for (int j = 0; j < m; j++) {
						if (baseObsList.get(i).get(j).getPrn() == refPRN) {
							flag = true;
							k = j;
							break;
						}
					}
					// Perform Double difference
					double refPR = remObsList.get(i).get(k).getPseudorange()
							- baseObsList.get(i).get(k).getPseudorange();
					double refPhase = remObsList.get(i).get(k).getPhaseL1() - baseObsList.get(i).get(k).getPhaseL1();
					double[] refUnitLOS = getUnitLOS(estRemEcef, remObsList.get(i).get(k).getEcef());
					boolean refIsPhaseLocked = remObsList.get(i).get(k).isPhaseLocked()
							&& baseObsList.get(i).get(k).isPhaseLocked();
					for (int j = 0; j < m; j++) {
						if (j == k) {
							continue;
						}
						int prn = remObsList.get(i).get(j).getPrn();
						int t = remObsList.get(i).get(j).getT();
						if (reject.contains(prn)) {
							continue;
						}
						double pseudorange = remObsList.get(i).get(j).getPseudorange()
								- baseObsList.get(i).get(j).getPseudorange() - refPR;
						double phase = remObsList.get(i).get(j).getPhaseL1() - baseObsList.get(i).get(j).getPhaseL1()
								- refPhase;
						double[] unitLOS = getUnitLOS(estRemEcef, remObsList.get(i).get(j).getEcef());
						boolean isPhaseLocked = remObsList.get(i).get(j).isPhaseLocked()
								&& baseObsList.get(i).get(j).isPhaseLocked() && refIsPhaseLocked;

						IntStream.range(0, 3).forEach(l -> unitLOS[l] = unitLOS[l] - refUnitLOS[l]);

						obsList.add(new Observation(t, prn, pseudorange, phase, unitLOS, isPhaseLocked));

					}
					baselineObsList.add(obsList);

				}
				// Intitiating the Kalman Filter
				double[] intialBaseline = IntStream.range(0, 3).mapToDouble(i -> estRemEcef[i] - baseEcef[i]).toArray();
				EKF ekf = new EKF();
				ArrayList<double[]> baselineList = ekf.process(baselineObsList, timeList, intialBaseline, R, _R);
				ArrayList<double[]> stateInfo = ekf.getStateInfo();
				rtkAmbMap = ekf.getAmbInfoMap();
				for (int i = 0; i < baselineList.size(); i++) {
					double[] baseline = baselineList.get(i);
					double[] estEcef = new double[4];
					IntStream.range(0, 3).forEach(j -> estEcef[j] = baseline[j] + baseEcef[j]);
					EnuMap.computeIfAbsent("RTK-float", k -> new ArrayList<double[]>())
							.add(estimateENU(estEcef, remoteEcef));
					CxParam Cx = new CxParam(stateInfo.get(i));
					CxMap.computeIfAbsent("RTK-float", k -> new ArrayList<CxParam>()).add(Cx);
				}
				HashMap<Integer, Double> intAmbMap = ekf.getIntAmbMap();
				baselineList = ekf.processOnlyPhase(baselineObsList, timeList,
						baselineList.get(baselineList.size() - 1), R, intAmbMap, "");
				stateInfo = ekf.getStateInfo();
				for (int i = 0; i < baselineList.size(); i++) {
					double[] baseline = baselineList.get(i);
					double[] estEcef = new double[4];
					IntStream.range(0, 3).forEach(j -> estEcef[j] = baseline[j] + baseEcef[j]);
					EnuMap.computeIfAbsent("RTK_fixed", k -> new ArrayList<double[]>())
							.add(estimateENU(estEcef, remoteEcef));
					CxParam Cx = new CxParam(stateInfo.get(i));
					CxMap.computeIfAbsent("RTK_fixed", k -> new ArrayList<CxParam>()).add(Cx);
				}

			}

			// Calculate Accuracy Metrics
			HashMap<String, ArrayList<double[]>> GraphEnuMap = new HashMap<String, ArrayList<double[]>>();
			for (String key : EnuMap.keySet()) {
				ArrayList<Double>[] errList = new ArrayList[6];
				IntStream.range(0, 6).forEach(i -> errList[i] = new ArrayList<Double>());
				ArrayList<double[]> enuList = EnuMap.get(key);
				int n = enuList.size();

				for (int i = 0; i < n; i++) {
					double[] enu = enuList.get(i);
					// error in East direction
					errList[0].add(Math.sqrt(enu[0] * enu[0]));
					// error in North direction
					errList[1].add(Math.sqrt(enu[1] * enu[1]));
					// error in Up direction
					errList[2].add(Math.sqrt(enu[2] * enu[2]));
					// 3d error
					errList[3].add(Math.sqrt(IntStream.range(0, 3).mapToDouble(j -> enu[j]).map(j -> j * j).sum()));
					// 2d error
					errList[4].add(Math.sqrt((enu[0] * enu[0]) + (enu[1] * enu[1])));
					// Rcvr Clk Offset*Speed of Light(in m)
					errList[5].add(enu[3]);

				}

				GraphEnuMap.put(key, enuList);

				// RMSE
				System.out.println("\n" + key + " RMSE");

				System.out.println(" E - " + RMS(errList[0]));
				System.out.println(" N - " + RMS(errList[1]));
				System.out.println(" U - " + RMS(errList[2]));
				System.out.println(" 3d Error - " + RMS(errList[3]));
				System.out.println(" 2d Error - " + RMS(errList[4]));
				System.out.println(" Rcvr CLk Off(in m) - " + RMS(errList[5]));

				if (opt == 9) {
					System.out.println("\n" + key + " Converged Value");

					System.out.println(" E - " + errList[0].get(n - 1));
					System.out.println(" N - " + errList[1].get(n - 1));
					System.out.println(" U - " + errList[2].get(n - 1));
					System.out.println(" 3d Error - " + errList[3].get(n - 1));
					System.out.println(" 2d Error - " + errList[4].get(n - 1));
				}
				// 95th Percentile
				IntStream.range(0, 6).forEach(i -> Collections.sort(errList[i]));
				int q95 = (int) (n * 0.95);

			}

			// Plot Graphs
			// GraphPlotter.graphCycleSlip(baseObsList, remObsList, timeList);
			GraphPlotter.graphENU(GraphEnuMap, CxMap, timeList);
			GraphPlotter.graphRTKcov(rtkAmbMap, timeList);
//			GraphPlotter.graphSatData(satDataMap, t0);
//			GraphPlotter.graphDOP(CxMap, satCountList, timeList);
//			GraphPlotter.graphSatRes(satResMap);

		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	// Create Base Station observation list
	public static ArrayList<Observation> modifyObsList(ArrayList<Observation> remObsList,
			ArrayList<Observation> baseObsList) {
		ArrayList<Observation> newBaseObsList = new ArrayList<Observation>();
		HashMap<Integer, Integer> order = new HashMap<Integer, Integer>();
		for (int i = 0; i < baseObsList.size(); i++) {
			Observation obs = baseObsList.get(i);
			order.put(obs.getPrn(), i);
		}
		for (int i = 0; i < remObsList.size(); i++) {
			Observation obs = remObsList.get(i);
			int prn = obs.getPrn();
			newBaseObsList.add(baseObsList.get(order.get(prn)));
		}
		return newBaseObsList;
	}

	// Transform from ECEF to ENU
	public static double[] estimateENU(double[] estEcefClk, double[] rxECEF) {
		double[] enu = LatLonUtil.ecef2enu(estEcefClk, rxECEF);
		return new double[] { enu[0], enu[1], enu[2], SpeedofLight * estEcefClk[3] };
	}

	// perform RMS
	public static double RMS(ArrayList<Double> list) {
		return Math.sqrt(list.stream().mapToDouble(x -> x * x).average().orElse(Double.NaN));
	}

	// perform MAE
	public static double MAE(ArrayList<Double> list) {
		return list.stream().mapToDouble(x -> x).average().orElse(Double.NaN);
	}

	public static double[] getUnitLOS(double[] rxEcef, double[] satEcef) {
		double[] unitLOS = new double[3];
		for (int i = 0; i < 3; i++) {
			unitLOS[i] = satEcef[i] - rxEcef[i];
		}
		double range = Math.sqrt(Arrays.stream(unitLOS).map(j -> j * j).sum());
		IntStream.range(0, 3).forEach(j -> unitLOS[j] = unitLOS[j] / range);

		return unitLOS;
	}

}
