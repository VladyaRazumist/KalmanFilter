using System;

namespace Filter
{

    public class KalmanLatLong
    {
        private float MinAccuracy = 1;

        private double velocity;
        private long TimeStamp_milliseconds;
        private double lat;
        private double lng;
        private double variance; // P matrix.  Negative means object uninitialised.  NB: units irrelevant, as long as same units used throughout
        private int KalmanConst = 150;

        public KalmanLatLong(float Q_metres_per_second) { this.velocity = Q_metres_per_second; variance = -1; }

        public long get_TimeStamp() { return TimeStamp_milliseconds; }
        public double get_lat() { return lat; }
        public double get_lng() { return lng; }
        public float get_accuracy() { return (float)Math.Sqrt(variance); }

        public void SetState(double lat, double lng, double accuracy, long TimeStamp_milliseconds)
        {
            this.lat = lat; this.lng = lng; variance = accuracy * accuracy; this.TimeStamp_milliseconds = TimeStamp_milliseconds;
        }

        /// <summary>
        /// Kalman filter processing for lattitude and longitude
        /// </summary>
        /// <param name="lat_measurement_degrees">new measurement of lattidude</param>
        /// <param name="lng_measurement">new measurement of longitude</param>
        /// <param name="accuracy">measurement of 1 standard deviation error in metres</param>
        /// <param name="TimeStamp_milliseconds">time of measurement</param>
        /// <returns>new state</returns>
        public Location Process(double lat_measurement, double lng_measurement, double accuracy, long TimeStamp_milliseconds)
        {
            if (accuracy < MinAccuracy) accuracy = MinAccuracy;


            // else apply Kalman filter methodology

            long TimeInc_milliseconds = TimeStamp_milliseconds - this.TimeStamp_milliseconds;
            if (TimeInc_milliseconds > 0)
            {
                velocity = CalculateCircleDistance(lat, lng, lat_measurement, lng_measurement)/TimeInc_milliseconds * KalmanConst;
                // time has moved on, so the uncertainty in the current position increases
                variance += TimeInc_milliseconds * velocity * velocity / 1000;
                this.TimeStamp_milliseconds = TimeStamp_milliseconds;
                // TO DO: USE VELOCITY INFORMATION HERE TO GET A BETTER ESTIMATE OF CURRENT POSITION
            }

            // Kalman gain matrix K = Covarariance * Inverse(Covariance + MeasurementVariance)
            // NB: because K is dimensionless, it doesn't matter that variance has different units to lat and lng
            double K = variance / (variance + accuracy * accuracy);
            // apply K
            lat += K * (lat_measurement - lat);
            lng += K * (lng_measurement - lng);
            // new Covarariance  matrix is (IdentityMatrix - K) * Covarariance 
            variance = (1 - K) * variance;

            return new Location(lat, lng, 0.0, 0.0, 0.0, DateTime.Now);
        }

        private double CalculateCircleDistance(double lat1, double lon1, double lat2, double lon2)
        {
            var toRadians = new Func<double, double>(number => number * Math.PI / 180);
            var p1 = toRadians(lat1);
            var p2 = toRadians(lat2);
            var deltagamma = toRadians(lon2 - lon1);
            var R = 6371e3;
            var d = Math.Acos(
                        Math.Sin(p1) * Math.Sin(p2) + Math.Cos(p1) * Math.Cos(p2) * Math.Cos(deltagamma)
                    ) * R;

            return double.IsNaN(d) ? 0 : d;
        }
    }

 
}

