using System;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Filter
{
    public class Algorithm
    {
        private int stateMDimension = 6;
        private int stateNDimension = 1;
        private double sigma = 0.625;

        public double RValue { get; set; } = 29;
        private Vector<double> xk1;
        private Matrix<double> Pk1;
        private Matrix<double> A;
        private Matrix<double> Qt;
        private Matrix<double> R;
        private Vector<double> zt;
        private DateTimeOffset previousMeasureTime;
        private Location previousLocation;


        public Algorithm(Location initialLocation)
        {
            Pk1 = new DenseMatrix(rows: stateMDimension, columns: stateMDimension);
            A = new DenseMatrix(rows: stateMDimension, columns: stateMDimension);
            Qt = new DenseMatrix(rows: stateMDimension, columns: stateMDimension);
            R = new DenseMatrix(rows: stateMDimension, columns: stateMDimension);

            InitKalman(initialLocation);
        }

        private void InitKalman(Location initialLocation)
        {
            previousMeasureTime = initialLocation.Timestamp;
            previousLocation = initialLocation;

            xk1 = new DenseVector(new double[]
                {initialLocation.Latitude, 0.0, initialLocation.Longitude, 0.0, initialLocation.Altitude, 0.0});

            Pk1 = GetMatrix(
                new double[][]
                {
                    new double[] {0, 0, 0, 0, 0, 0},
                    new double[] {0, 0, 0, 0, 0, 0},
                    new double[] {0, 0, 0, 0, 0, 0},
                    new double[] {0, 0, 0, 0, 0, 0},
                    new double[] {0, 0, 0, 0, 0, 0},
                    new double[] {0, 0, 0, 0, 0, 0},
                }
            );

            A = GetMatrix(
                new double[][]
                {
                    new double[] {1, 0, 0, 0, 0, 0},
                    new double[] {0, 1, 0, 0, 0, 0},
                    new double[] {0, 0, 1, 0, 0, 0},
                    new double[] {0, 0, 0, 1, 0, 0},
                    new double[] {0, 0, 0, 0, 1, 0},
                    new double[] {0, 0, 0, 0, 0, 1},
                }
            );

            R = GetMatrix(
                new double[][]
                {
                    new double[] {RValue, 0, 0, 0, 0, 0},
                    new double[] {0, RValue, 0, 0, 0, 0},
                    new double[] {0, 0, RValue, 0, 0, 0},
                    new double[] {0, 0, 0, RValue, 0, 0},
                    new double[] {0, 0, 0, 0, RValue, 0},
                    new double[] {0, 0, 0, 0, 0, RValue},
                }
            );
        }

        public Location ProcessState(Location currentLocation)
        {
            var newMeasureTime = currentLocation.Timestamp;
            RValue = currentLocation.HorizontalAccuracy;

            R = GetMatrix(
                new double[][]
                {
                    new double[] {RValue, 0, 0, 0, 0, 0},
                    new double[] {0, RValue, 0, 0, 0, 0},
                    new double[] {0, 0, RValue, 0, 0, 0},
                    new double[] {0, 0, 0, RValue, 0, 0},
                    new double[] {0, 0, 0, 0, RValue, 0},
                    new double[] {0, 0, 0, 0, 0, RValue},
                }
            );

            // Convert measure times to seconds
            var newMeasureTimeSeconds = newMeasureTime.ToUnixTimeSeconds();
            var lastMeasureTimeSeconds = previousMeasureTime.ToUnixTimeSeconds();

            // Calculate timeInterval between last and current measure
              var timeInterval = (double) newMeasureTimeSeconds - lastMeasureTimeSeconds;

              if (timeInterval <= 0)
              {
                  timeInterval = 1;
              }

           

              //if (RValue > 50)
              //{
              //    timeInterval = 1;
              //}

             A = GetMatrix(
                new double[][]
                {
                    new double[] {1, 0, 0, 0, 0, 0},
                    new double[] { timeInterval, 1, 0, 0, 0, 0},
                    new double[] {0, 0, 1, 0, 0, 0},
                    new double[] {0, 0, timeInterval, 1, 0, 0},
                    new double[] {0, 0, 0, 0, 1, 0},
                    new double[] {0, 0, 0, 0, timeInterval, 1},
                }
            );

            // Parts of Acceleration Noise Magnitude Matrix
            var part1 = sigma * ((Math.Pow(timeInterval, 4)) / 4);
            var part2 = sigma * ((Math.Pow(timeInterval, 3)) / 2);
            var part3 = sigma * (Math.Pow(timeInterval, 2));

            Qt = GetMatrix(
                new double[][]
                {
                    new double[] { part1, part2, 0, 0, 0, 0},
                    new double[] { part2, part3, 0, 0, 0, 0},
                    new double[] {0, 0, part1, part2, 0, 0},
                    new double[] {0, 0, part2, part3, 0, 0},
                    new double[] {0, 0, 0, 0, part1, part2},
                    new double[] {0, 0, 0, 0, part2, part3},
                }
            );

            var velocityXComponent = (previousLocation.Latitude - currentLocation.Latitude) / timeInterval;
            var velocityYComponent = (previousLocation.Longitude - currentLocation.Longitude) / timeInterval;
            var velocityZComponent = (previousLocation.Altitude - currentLocation.Altitude) / timeInterval;

            zt = new DenseVector(new double[]
                {currentLocation.Latitude, velocityXComponent, currentLocation.Longitude, velocityYComponent, currentLocation.Altitude, velocityZComponent});

            previousLocation = currentLocation;
            previousMeasureTime = newMeasureTime;

            return KalmanFilter();
        }

        private Location KalmanFilter()
        {
            var xk = A * xk1;
            var Pk = ((A * Pk1) * A.Transpose()) + Qt;

            var tmp = Pk + R;
            var Kt = Pk * (tmp.Inverse());

            var xt = xk + (Kt * (zt - xk));
            var Pt = (GetIdentityMatrix() - Kt) * Pk;

            xk1 = xt;
            Pk1 = Pt;

            var lat = xk1[0];
            var lon = xk1[2];
            var alt = xk1[4];

            var kalmanLocation = new Location(lat, lon, alt, previousLocation.HorizontalAccuracy, previousLocation.VerticalAccuracy, previousMeasureTime);

            return kalmanLocation;
        }

        public DenseMatrix GetMatrix(double[][] matrix)
        {
            var array = matrix.SelectMany(inner => inner).ToArray();
            var result = new DenseMatrix(stateMDimension, stateMDimension, array);
            return result;
        }

        public DenseMatrix GetIdentityMatrix()
        {
            var identityMatrix = new DenseMatrix(stateMDimension, stateMDimension);

            for (var i = 0; i < stateMDimension; i++)
            {
                for (var j = 0; j < stateMDimension; j++)
                {
                    if (i == j)
                    {
                        identityMatrix[i, j] = 1.0;
                    }
                }
            }

            return identityMatrix;
        }

    }
}
