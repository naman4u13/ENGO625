package com.ENGO625;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.IntStream;

import org.json.JSONObject;

import com.ENGO625.estimation.LinearLeastSquare;
import com.ENGO625.models.CxParam;
import com.ENGO625.models.Observation;
import com.ENGO625.models.SatResidual;
import com.ENGO625.models.Satellite;
import com.ENGO625.util.ComputeEleAzm;
import com.ENGO625.util.GraphPlotter;
import com.ENGO625.util.LatLonUtil;
import com.ENGO625.util.Parser;

public class MainApp {

	private final static double SpeedofLight = 299792458;

	public static void main(String args[]) {

		try {
			// Path to store output file
			String path = "D:\\projects\\eclipse_projects\\UCalgary\\ENGO625\\results\\output2";
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
			/*
			 * Estimation option - (1. Satellite Trajectory, 2. Least Squares, 3. WLS, 4.
			 * Between Receiver Single Difference(BRSD), 5. Combined of option 2,3 and 4)
			 */
			int opt = 5;
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
			// First Epoch GPSTime
			int t0 = remoteOrdMap.firstKey();
			// Iterate over remote station observation data
			for (Map.Entry ele : remoteOrdMap.entrySet()) {
				// GPS Time
				int t = (int) ele.getKey();
				// Store GPS Time minus GPS Time of first epoch, to set timescale at zero
				timeList.add(t - t0);
				HashMap<Integer, Satellite> subSatMap = satMap.get(t);
				ArrayList<Observation> remObsList = (ArrayList<Observation>) ele.getValue();
				ArrayList<Observation> baseObsList = null;
				ArrayList<Observation> invalid = new ArrayList<Observation>();
				// Creating observation list to be used for estimation
				for (Observation obs : remObsList) {
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
				remObsList.removeAll(invalid);
				// Satellite Count
				int n = remObsList.size();
				/*
				 * If differencing option is selected, create a similar observation for Base
				 * Station data, and check whether same set of satellites is present in Remote
				 * Data
				 */
				if (opt == 4 || opt == 5) {
					baseObsList = baseMap.get(t);
					baseObsList = modifyObsList(remObsList, baseObsList);
				}
				satCountList.add(n);
				switch (opt) {

				case 2, 3, 4, 5:

					// Least Squares
					LinearLeastSquare lls = new LinearLeastSquare();
					// Estimated Remote ECEF
					double[] ecef = lls.processSP(remObsList, false);
					// Compute Elevation Angle
					for (int i = 0; i < n; i++) {
						double[] elevAzm = ComputeEleAzm.computeEleAzm(ecef, remObsList.get(i).getEcef());
						remObsList.get(i).setElevAzmAngle(elevAzm);
					}
					if (opt == 2 || opt == 5) {
						EnuMap.computeIfAbsent("LS", k -> new ArrayList<double[]>()).add(estimateENU(ecef, remoteEcef));
						CxParam Cx = lls.getCxParam();
						CxMap.computeIfAbsent("LS", k -> new ArrayList<CxParam>()).add(Cx);
						double[] residual = lls.getResidual();
						satResMap.computeIfAbsent("LS", k -> new HashMap<Integer, ArrayList<SatResidual>>());
						for (int i = 0; i < n; i++) {
							Observation obs = remObsList.get(i);
							satResMap.get("LS").computeIfAbsent(obs.getPrn(), k -> new ArrayList<SatResidual>())
									.add(new SatResidual(t - t0, obs.getElevAngle(), residual[i]));
						}
					}
					// Weighted Least Squares
					if (opt == 3 || opt == 5) {

						lls = new LinearLeastSquare();
						ecef = lls.processSP(remObsList, true);
						EnuMap.computeIfAbsent("WLS", k -> new ArrayList<double[]>())
								.add(estimateENU(ecef, remoteEcef));
						CxParam Cx = lls.getCxParam();
						CxMap.computeIfAbsent("WLS", k -> new ArrayList<CxParam>()).add(Cx);
						double[] residual = lls.getResidual();
						satResMap.computeIfAbsent("WLS", k -> new HashMap<Integer, ArrayList<SatResidual>>());
						for (int i = 0; i < n; i++) {
							Observation obs = remObsList.get(i);
							satResMap.get("WLS").computeIfAbsent(obs.getPrn(), k -> new ArrayList<SatResidual>())
									.add(new SatResidual(t - t0, obs.getElevAngle(), residual[i]));
						}

					}
					// Between Receiver Single Differencing
					if (opt == 4 || opt == 5) {
						lls = new LinearLeastSquare();
						ecef = lls.processBRSD(baseObsList, remObsList, baseEcef);
						EnuMap.computeIfAbsent("BRSD", k -> new ArrayList<double[]>())
								.add(estimateENU(ecef, remoteEcef));
						CxParam Cx = lls.getCxParam();
						CxMap.computeIfAbsent("BRSD", k -> new ArrayList<CxParam>()).add(Cx);
						double[] residual = lls.getResidual();
						satResMap.computeIfAbsent("BRSD", k -> new HashMap<Integer, ArrayList<SatResidual>>());
						for (int i = 0; i < n; i++) {
							Observation obs = remObsList.get(i);
							satResMap.get("BRSD").computeIfAbsent(obs.getPrn(), k -> new ArrayList<SatResidual>())
									.add(new SatResidual(t - t0, obs.getElevAngle(), residual[i]));
						}

					}
					break;

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
					errList[3].add(Math.sqrt(Arrays.stream(enu).map(j -> j * j).sum()));
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

				// 95th Percentile
				IntStream.range(0, 6).forEach(i -> Collections.sort(errList[i]));
				int q95 = (int) (n * 0.95);

				System.out.println("\n" + key + " 95%");

				System.out.println(" E - " + errList[0].get(q95));
				System.out.println(" N - " + errList[1].get(q95));
				System.out.println(" U - " + errList[2].get(q95));
				System.out.println(" 3d Error - " + errList[3].get(q95));
				System.out.println(" 2d Error - " + errList[4].get(q95));
				System.out.println(" Rcvr CLk Off(in m) - " + errList[5].get(q95));

			}

			// Plot Graphs
			GraphPlotter.graphENU(GraphEnuMap, CxMap, timeList);
			GraphPlotter.graphSatData(satDataMap, t0);
			GraphPlotter.graphDOP(CxMap, satCountList, timeList);
			GraphPlotter.graphSatRes(satResMap);

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

}
