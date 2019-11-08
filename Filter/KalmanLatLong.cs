using System;

namespace Filter
{
    public class KalmanLatLong
    {
        private const float MinAccuracy = 1;
        private double _velocity;
        public long TimeStampMilliseconds { get; private set; }
        public double Lat { get; private set; }
        public double Lng { get; private set; }
        private double _variance; // P matrix.  Negative means object uninitialised.  NB: units irrelevant, as long as same units used throughout
        private int _kalmanConst = 150;

        public KalmanLatLong(float velocity) { this._velocity = velocity; _variance = -1; }

        public float GetAccuracy() { return (float)Math.Sqrt(_variance); }

        public void SetState(double lat, double lng, double accuracy, long timeStampMilliseconds)
        {
            Lat = lat;
            Lng = lng;
            _variance = accuracy * accuracy;
            TimeStampMilliseconds = timeStampMilliseconds;
        }

        /// <summary>
        /// Kalman filter processing for lattitude and longitude
        /// </summary>
        /// <param name="latMeasurement">new measurement of lattidude</param>
        /// <param name="lngMeasurement">new measurement of longitude</param>
        /// <param name="accuracy">measurement of 1 standard deviation error in metres</param>
        /// <param name="timeStampMilliseconds">time of measurement</param>
        /// <returns>new state</returns>
        ///
        
        public Location Process(double latMeasurement, double lngMeasurement, double accuracy, long timeStampMilliseconds)
        {
            if (accuracy < MinAccuracy) accuracy = MinAccuracy;
            var timeIncMilliseconds = timeStampMilliseconds - TimeStampMilliseconds;

            if (timeIncMilliseconds > 0)
            {
                _velocity = CalculateCircleDistance(Lat, Lng, latMeasurement, lngMeasurement) / timeIncMilliseconds * _kalmanConst;
                _variance += timeIncMilliseconds * _velocity * _velocity / 1000;
                TimeStampMilliseconds = timeStampMilliseconds;
            }

            var k = _variance / (_variance + accuracy * accuracy);
            Lat += k * (latMeasurement - Lat);
            Lng += k * (lngMeasurement - Lng);

            _variance = (1 - k) * _variance;
            return new Location(Lat, Lng, 0.0, 0.0, 0.0, DateTime.Now);
        }

        private static double CalculateCircleDistance(double lat1, double lon1, double lat2, double lon2)
        {
            var toRadians = new Func<double, double>(number => number * Math.PI / 180);
            var p1 = toRadians(lat1);
            var p2 = toRadians(lat2);
            var deltaGamma = toRadians(lon2 - lon1);
            const double r = 6371e3;
            var d = Math.Acos(
                        Math.Sin(p1) * Math.Sin(p2) + Math.Cos(p1) * Math.Cos(p2) * Math.Cos(deltaGamma)
                    ) * r;

            return double.IsNaN(d) ? 0 : d;
        }
    }


}

