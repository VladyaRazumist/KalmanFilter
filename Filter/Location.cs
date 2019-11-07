using System;

namespace Filter
{
    public class Location
    {
        public double Latitude { get; }
        public double Longitude { get; }
        public double Altitude { get; }
        public double HorizontalAccuracy { get; }
        public double VerticalAccuracy { get; }
        public DateTimeOffset Timestamp { get; }

        public Location(double latitude, double longitude, double altitude, double horizontalAccuracy,
            double verticalAccuracy, DateTimeOffset timestamp)
        {
            Latitude = latitude;
            Longitude = longitude;
            Altitude = altitude;
            HorizontalAccuracy = horizontalAccuracy;
            VerticalAccuracy = verticalAccuracy;
            Timestamp = timestamp;
        }
    }
}
