package eu.qrobotics.freightfrenzy.teamcode.util;

public class KalmanFilter {
    private double x; // estimate
    private final double initialCovarianceGuess; // your initial covariance guess
    private double p; // estimate error

    public KalmanFilter(double initialState, double initialCovarianceGuess) {
        this.x = initialState;
        this.p = initialCovarianceGuess;
        this.initialCovarianceGuess = initialCovarianceGuess;
    }

    public double update(double reading) {
        return update(reading, initialCovarianceGuess);
    }
    public double update(double reading, double readingError) {
        if(Double.isNaN(readingError)) {
            readingError = initialCovarianceGuess;
        }

        double K = p/(p + readingError);

        x = x + K * (reading - x);

        p = (1 - K) * p;

        return x;
    }
}
