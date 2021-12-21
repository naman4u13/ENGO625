package com.ENGO625.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.ui.ApplicationFrame;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.RefineryUtilities;

import com.ENGO625.models.CxParam;
import com.ENGO625.models.Observation;
import com.ENGO625.models.SatResidual;

public class GraphPlotter extends ApplicationFrame {
	private static final double freqL1 = 1575.42e6;
	private final static double SpeedofLight = 299792458;

	public GraphPlotter(String applicationTitle, String chartTitle, HashMap<String, ArrayList<double[]>> dataMap,
			ArrayList<Integer> timeList) throws IOException {
		super(applicationTitle);
		// TODO Auto-generated constructor stub

		final JFreeChart chart = ChartFactory.createXYLineChart(chartTitle, "GPS-time", chartTitle,
				createDataset3dErr(dataMap, timeList));
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + chartTitle + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

	}

	public GraphPlotter(int prn, HashMap<Integer, Boolean> dataMap, ArrayList<Integer> timeList) throws IOException {
		super(prn + " Cycle-Slip Detection");
		// TODO Auto-generated constructor stub

		final JFreeChart chart = ChartFactory.createXYLineChart(prn + " Is-Phase-Locked", "GPS-time", "Is-Phase-Locked",
				createCycleSlipDataset(prn, dataMap, timeList));
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + chartTitle + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

	}

	public GraphPlotter(HashMap<String, HashMap<Integer, ArrayList<SatResidual>>> satResMap, boolean flag) {
		super("Satellite-Residual");
		JFreeChart chart;
		if (flag) {
			chart = ChartFactory.createXYLineChart("Satellite-Residual", "GPS-time", "Satellite-Residual(in m)",
					createDatasetSatRes(satResMap, flag));
		} else {
			chart = ChartFactory.createScatterPlot("Satellite-Residual vs Elevation Angle",
					"Elevation-Angle(in degrees)", "Satellite-Residual(in m)", createDatasetSatRes(satResMap, flag));
		}
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + chartTitle + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

	}

	public GraphPlotter(HashMap<String, HashMap<String, ArrayList<Double>>> dataMap, ArrayList<Integer> xList,
			boolean flag) throws IOException {
		super("DOP");
		// TODO Auto-generated constructor stub
		JFreeChart chart = null;
		if (flag) {
			chart = ChartFactory.createXYLineChart("DOP", "GPS-time", "DOP", createDatasetDOP(dataMap, xList));
		} else {
			chart = ChartFactory.createScatterPlot("DOP", "Satellite-Count", "DOP", createDatasetDOP(dataMap, xList));
		}
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + chartTitle + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

	}

	public GraphPlotter(ArrayList<Integer> dataList, ArrayList<Integer> timeList) throws IOException {
		super("Satellite Count");
		// TODO Auto-generated constructor stub

		final JFreeChart chart = ChartFactory.createXYLineChart("Satellite Count", "GPS-time", "Satellite Count",
				createDatasetSatCount(dataList, timeList));
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + chartTitle + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

	}

	public GraphPlotter(String applicationTitle, String chartTitle, HashMap<String, double[][]> dataMap,
			ArrayList<Integer> timeList, boolean flag) throws IOException {
		super(applicationTitle);
		// TODO Auto-generated constructor stub

		final JFreeChart chart = ChartFactory.createXYLineChart(chartTitle, "GPS-time", chartTitle,
				createDatasetENU(dataMap, timeList));
//		XYPlot xyPlot = (XYPlot) chart.getPlot();
//		final XYLineAndShapeRenderer renderer = (XYLineAndShapeRenderer) xyPlot.getRenderer();
//		((AbstractRenderer) renderer).setAutoPopulateSeriesStroke(false);
////		renderer.setDefaultStroke(new BasicStroke(2.0f));
////		renderer.setSeriesStroke(0, new BasicStroke(2.0f));
//		renderer.setSeriesOutlineStroke(0, new BasicStroke(2.0f));

		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + chartTitle + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

	}

	public GraphPlotter(String applicationTitle, HashMap<String, ArrayList<double[]>> dataMap) throws IOException {
		super(applicationTitle);
		// TODO Auto-generated constructor stub

		final JFreeChart chart = ChartFactory.createScatterPlot("2D Error", "East(m)", "North(m)",
				createDataset2dErr(dataMap));
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + "2d Error" + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

	}

	public GraphPlotter(int flag, HashMap<Integer, ArrayList<Observation>> satDataMap, int t0) {
		super("Observed Satellite Data");
		String title;
		if (flag == 0) {
			title = "Doppler(in Hz)";
		} else if (flag == 1) {
			title = "Pseudorange(in m)";
		} else {
			title = "L1 Carrier Phase(in L1 Cycles)";
		}
		final JFreeChart chart = ChartFactory.createXYLineChart(title, "GPS-time", title,
				createDatasetSatData(satDataMap, flag, t0));
		final ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new java.awt.Dimension(560, 370));
		chartPanel.setMouseZoomable(true, false);
		// ChartUtils.saveChartAsJPEG(new File(path + chartTitle + ".jpeg"), chart,
		// 1000, 600);
		setContentPane(chartPanel);

		// TODO Auto-generated constructor stub
	}

	public static void graphENU(HashMap<String, ArrayList<double[]>> EnuMap, HashMap<String, ArrayList<CxParam>> CxMap,
			ArrayList<Integer> timeList) throws IOException {

		String[] chartNames = new String[] { "E", "N", "U" };
		for (int i = 0; i < 3; i++) {
			final int index = i;
			HashMap<String, double[][]> dataMap = new HashMap<String, double[][]>();
			for (String key : EnuMap.keySet()) {
				ArrayList<double[]> enuList = EnuMap.get(key);
				ArrayList<CxParam> CxList = CxMap.get(key);
				double[][] arr = new double[2][];
				arr[0] = enuList.stream().mapToDouble(j -> j[index]).toArray();
				arr[1] = CxList.stream().mapToDouble(j -> Math.sqrt(j.getDopDiag()[index] * j.getVarUERE())).toArray();
				dataMap.put(key, arr);
			}
			GraphPlotter chart = new GraphPlotter("GPS PVT Error - ", chartNames[i] + "(m)", dataMap, timeList, true);
			chart.pack();
			RefineryUtilities.positionFrameRandomly(chart);
			chart.setVisible(true);
		}
		GraphPlotter chart = new GraphPlotter("2D-Error", EnuMap);
		chart.pack();
		RefineryUtilities.positionFrameRandomly(chart);
		chart.setVisible(true);
		chart = new GraphPlotter("3d-Error(in m)", "3d-Error(in m)", EnuMap, timeList);
		chart.pack();
		RefineryUtilities.positionFrameRandomly(chart);
		chart.setVisible(true);

	}

	public static void graphDOP(HashMap<String, ArrayList<CxParam>> dataMap, ArrayList<Integer> satCountList,
			ArrayList<Integer> timeList) throws Exception {

		HashMap<String, HashMap<String, ArrayList<Double>>> dopMap = new HashMap<String, HashMap<String, ArrayList<Double>>>();
		for (String key : dataMap.keySet()) {
			ArrayList<CxParam> dopList = dataMap.get(key);
			ArrayList<Double> pdopList = new ArrayList<Double>();
			ArrayList<Double> hdopList = new ArrayList<Double>();
			ArrayList<Double> vdopList = new ArrayList<Double>();
			int n = dopList.size();
			for (int i = 0; i < n; i++) {
				double[] dopDiag = dopList.get(i).getDopDiag();
				pdopList.add(Math.sqrt(dopDiag[0] + dopDiag[1] + dopDiag[2]));
				hdopList.add(Math.sqrt(dopDiag[0] + dopDiag[1]));
				vdopList.add(Math.sqrt(dopDiag[2]));
			}
			dopMap.computeIfAbsent("PDOP", k -> new HashMap<String, ArrayList<Double>>()).put(key, pdopList);
			dopMap.computeIfAbsent("HDOP", k -> new HashMap<String, ArrayList<Double>>()).put(key, hdopList);
			dopMap.computeIfAbsent("VDOP", k -> new HashMap<String, ArrayList<Double>>()).put(key, vdopList);

		}

		GraphPlotter chart = new GraphPlotter(dopMap, timeList, true);
		chart.pack();
		RefineryUtilities.positionFrameRandomly(chart);
		chart.setVisible(true);
		chart = new GraphPlotter(dopMap, satCountList, false);
		chart.pack();
		RefineryUtilities.positionFrameRandomly(chart);
		chart.setVisible(true);
		chart = new GraphPlotter(satCountList, timeList);
		chart.pack();
		RefineryUtilities.positionFrameRandomly(chart);
		chart.setVisible(true);

	}

	public static void graphCycleSlip(ArrayList<ArrayList<Observation>> baseObsList,
			ArrayList<ArrayList<Observation>> remObsList, ArrayList<Integer> timeList) throws IOException {
		int n = baseObsList.size();
		HashMap<Integer, HashMap<Integer, Boolean>> csMap = new HashMap<Integer, HashMap<Integer, Boolean>>();
		for (int i = 0; i < n; i++) {
			int t = timeList.get(i);
			ArrayList<Observation> baseObsvs = baseObsList.get(i);
			ArrayList<Observation> remObsvs = remObsList.get(i);
			int m = baseObsvs.size();
			for (int j = 0; j < m; j++) {
				int prn = baseObsvs.get(j).getPrn();
				boolean isPhaseLocked = baseObsvs.get(j).isPhaseLocked() && remObsvs.get(j).isPhaseLocked();
				csMap.computeIfAbsent(prn, k -> new HashMap<Integer, Boolean>()).put(t, isPhaseLocked);
			}
		}
		for (int prn : csMap.keySet()) {
			GraphPlotter chart = new GraphPlotter(prn, csMap.get(prn), timeList);
			chart.pack();
			RefineryUtilities.positionFrameRandomly(chart);
			chart.setVisible(true);
		}

	}

	public static void graphSatRes(HashMap<String, HashMap<Integer, ArrayList<SatResidual>>> satResMap) {
		GraphPlotter chart = new GraphPlotter(satResMap, true);
		chart.pack();
		RefineryUtilities.positionFrameRandomly(chart);
		chart.setVisible(true);

		chart = new GraphPlotter(satResMap, false);
		chart.pack();
		RefineryUtilities.positionFrameRandomly(chart);
		chart.setVisible(true);

	}

	public static void graphSatData(HashMap<Integer, ArrayList<Observation>> satDataMap, int t0) {
		for (int i = 0; i < 3; i++) {
			GraphPlotter chart = new GraphPlotter(i, satDataMap, t0);
			chart.pack();
			RefineryUtilities.positionFrameRandomly(chart);
			chart.setVisible(true);
		}
	}

	private XYDataset createDatasetENU(HashMap<String, double[][]> dataMap, ArrayList<Integer> timeList) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		for (String key : dataMap.keySet()) {
			final XYSeries errSeries = new XYSeries(key + " Error");
			final XYSeries psdSeries = new XYSeries(key + " +SD");
			final XYSeries nsdSeries = new XYSeries(key + " -SD");
			double[] data = dataMap.get(key)[0];
			double[] sd = dataMap.get(key)[1];
			for (int i = 0; i < data.length; i++) {
				errSeries.add(timeList.get(i), Double.valueOf(data[i]));
				psdSeries.add(timeList.get(i), Double.valueOf(sd[i]));
				nsdSeries.add(timeList.get(i), Double.valueOf(-sd[i]));
			}
			dataset.addSeries(errSeries);
			// dataset.addSeries(psdSeries);
			// dataset.addSeries(nsdSeries);
		}

		return dataset;

	}

	private static XYDataset createDatasetDOP(HashMap<String, HashMap<String, ArrayList<Double>>> dataMap,
			ArrayList<Integer> timeList) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		for (String key : dataMap.keySet()) {
			for (String subKey : dataMap.get(key).keySet()) {
				final XYSeries series = new XYSeries(key + " " + subKey);
				ArrayList<Double> data = dataMap.get(key).get(subKey);
				for (int i = 0; i < data.size(); i++) {
					series.add(timeList.get(i), data.get(i));
				}
				dataset.addSeries(series);
			}
		}

		return dataset;

	}

	private static XYDataset createDatasetSatCount(ArrayList<Integer> data, ArrayList<Integer> timeList) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		final XYSeries series = new XYSeries("SatCount");
		for (int i = 0; i < data.size(); i++) {
			series.add(timeList.get(i), data.get(i));
		}
		dataset.addSeries(series);

		return dataset;

	}

	private XYDataset createDataset2dErr(HashMap<String, ArrayList<double[]>> dataMap) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		for (String key : dataMap.keySet()) {
			final XYSeries series = new XYSeries(key);
			ArrayList<double[]> list = dataMap.get(key);
			for (int i = 0; i < list.size(); i++) {
				double[] data = list.get(i);
				series.add(data[0], data[1]);
			}
			dataset.addSeries(series);
		}

		return dataset;

	}

	private XYDataset createCycleSlipDataset(int prn, HashMap<Integer, Boolean> dataMap, ArrayList<Integer> timeList) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		final XYSeries series = new XYSeries(prn);
		for (int i = 0; i < timeList.size(); i++) {
			int time = timeList.get(i);
			int flag = 0;
			if (dataMap.containsKey(time)) {
				flag = dataMap.get(time) ? 2 : 1;
			}
			series.add(time, flag);
		}
		dataset.addSeries(series);
		return dataset;

	}

	private XYDataset createDataset3dErr(HashMap<String, ArrayList<double[]>> dataMap, ArrayList<Integer> timeList) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		for (String key : dataMap.keySet()) {
			final XYSeries series = new XYSeries(key);
			ArrayList<double[]> list = dataMap.get(key);
			for (int i = 0; i < list.size(); i++) {
				double[] enu = list.get(i);
				double err = Math.sqrt(Arrays.stream(enu).map(j -> j * j).sum());
				series.add(timeList.get(i), Double.valueOf(err));
			}
			dataset.addSeries(series);
		}

		return dataset;

	}

	private XYDataset createDatasetSatData(HashMap<Integer, ArrayList<Observation>> dataMap, int flag, int t0) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		for (int key : dataMap.keySet()) {
			final XYSeries series = new XYSeries(key);
			ArrayList<Observation> list = dataMap.get(key);
			int prevT = 0;
			for (int i = 1; i < list.size(); i++) {
				Observation obs = list.get(i);
				double data;
				int t = obs.getT() - t0;

				double drDoppler = obs.getDoppler() * (SpeedofLight / freqL1) * (t - prevT);
				double drPR = list.get(i).getPseudorange() - list.get(i - 1).getPseudorange();
				double drCP = -(list.get(i).getPhaseL1() - list.get(i - 1).getPhaseL1()) * (SpeedofLight / freqL1);
				if (Math.abs(drDoppler - drPR) > 1 || Math.abs(drDoppler - drCP) > 1 || Math.abs(drCP - drPR) > 1) {
					System.out.print("");
				}
				if (t > 300) {
					break;
				}
				if (flag == 0) {
					data = obs.getDoppler();
				} else if (flag == 1) {
					data = obs.getPseudorange();
				} else {
					data = obs.getPhaseL1();
				}
				if (t - prevT > 1) {
					series.add(prevT, null);
				}
				series.add(t, data);
				prevT = t;
			}
			dataset.addSeries(series);
		}

		return dataset;

	}

	private XYDataset createDatasetSatRes(HashMap<String, HashMap<Integer, ArrayList<SatResidual>>> dataMap,
			boolean flag) {
		XYSeriesCollection dataset = new XYSeriesCollection();
		for (String key : dataMap.keySet()) {
			for (int subKey : dataMap.get(key).keySet()) {
				final XYSeries series = new XYSeries(key + " " + subKey);
				ArrayList<SatResidual> dataList = dataMap.get(key).get(subKey);
				if (flag) {
					int t0 = 0;
					for (int i = 0; i < dataList.size(); i++) {
						SatResidual data = dataList.get(i);
						int t = data.getT();
						double satRes = data.getResidual();
						if (t - t0 > 1) {
							series.add(t0, null);
						}
						series.add(t, satRes);
						t0 = t;
					}
				} else {
					for (int i = 0; i < dataList.size(); i++) {
						SatResidual data = dataList.get(i);
						double elevAngle = Math.toDegrees(data.getElevAngle());
						double satRes = data.getResidual();

						series.add(elevAngle, satRes);

					}
				}

				dataset.addSeries(series);
			}
		}

		return dataset;
	}

}
