import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Paint;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;
import java.util.Arrays;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.DefaultDrawingSupplier;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.DefaultXYDataset;
import org.jfree.ui.ApplicationFrame;

public class DisplayPointCloud extends ApplicationFrame {

    ArrayList<double[]> seriesHeights;
    DefaultXYDataset dataset;
    int currentKey = 0;

    public DisplayPointCloud(String title) {
        super(title);
        this.setDefaultCloseOperation(ApplicationFrame.EXIT_ON_CLOSE);
        dataset = new DefaultXYDataset();
        JFreeChart chart = createChart();
        ChartPanel chartPanel = new ChartPanel(chart, false);
        chartPanel.setPreferredSize(new Dimension(500, 400));
        this.add(chartPanel, BorderLayout.CENTER);
        seriesHeights = new ArrayList<>();
    }

    public void addSamples(double[][] samples) {
        dataset.addSeries(Integer.toString(currentKey), Arrays.copyOfRange(samples, 0, 2));
        currentKey++;
        seriesHeights.add(samples[2]);
    }

    private JFreeChart createChart() {
        JFreeChart chart = ChartFactory.createScatterPlot(
                "Scatter Plot Demo", "X", "Y", dataset,
                PlotOrientation.VERTICAL, true, true, false);
        chart.setBackgroundPaint(Color.white);
        XYPlot plot = (XYPlot) chart.getPlot();
        Shape[] cross = DefaultDrawingSupplier.createStandardSeriesShapes();
        plot.setBackgroundPaint(new Color(0xffffe0));
        plot.setDomainGridlinesVisible(true);
        plot.setDomainGridlinePaint(Color.lightGray);
        plot.setRangeGridlinePaint(Color.lightGray);
        MyRenderer renderer = new MyRenderer(false, true);
        plot.setRenderer(renderer);
        renderer.setSeriesShape(0, cross[0]);
        plot.setRenderer(renderer);
        return chart;
    }

    private class MyRenderer extends XYLineAndShapeRenderer {

        public MyRenderer(boolean lines, boolean shapes) {
            super(lines, shapes);
        }
        @Override
        public Paint getItemPaint(int row, int column) {
            return new Color(0, (float)seriesHeights.get(row)[column], 0);
        }

        @Override
        public Shape getItemShape(int row, int column) {
            return createCircle(10);
        }
    }

    private static Shape createCircle(double size){
        return new Ellipse2D.Double(-size/2,-size/2,size,size);
    }

    public static void main(String[] args) {
        double[][] trainingSet = {
                {0.18053, 0.55487, 0.94834,},
                {0.401414, 0.401415, 0.401416},
                {0, 1, 0},
        };
        double[][] secondSet = {
                {0.28053, 0.65487, 0.84834,},
                {0.501414, 0.501415, 0.501416},
                {1, 0, 1},
        };
        DisplayPointCloud demo = new DisplayPointCloud("JFreeChartDemo");
        demo.addSamples(trainingSet);
        demo.pack();
        demo.setLocationRelativeTo(null);
        demo.setVisible(true);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) { }
        demo.addSamples(secondSet);
    }
}