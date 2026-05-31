namespace IAFahim.Math.Kalman.Tests
{
    using Unity.Mathematics;
    using NUnit.Framework;

    public sealed unsafe class ScalarKalmanFilterTests
    {
        [Test]
        public void Predict_NoMotion_ReturnsState()
        {
            float state = 5.0f;
            float velocity = 1.0f;
            float predicted = ScalarKalmanFilter.Predict(state, velocity, 0.1f, 0.016f);
            Assert.IsTrue(predicted > state);
        }

        [Test]
        public void Update_ZeroMeasurement_ReturnsPredicted()
        {
            float updated;
            float result = ScalarKalmanFilter.Update(5.0f, 1.0f, 0.0f, 1.0f, out updated);
            Assert.IsTrue(result < 5.0f);
        }

        [Test]
        public void Update_PerfectMeasurement_ConvergesToMeasurement()
        {
            float state = 0.0f;
            float covariance = 0.01f;
            float measurement = 10.0f;
            float measurementNoise = 0.01f;

            for (int i = 0; i < 100; i++)
            {
                state = ScalarKalmanFilter.Update(state, covariance, measurement, measurementNoise, out covariance);
            }

            Assert.IsTrue(math.abs(state - 10.0f) < 0.1f);
        }

        [Test]
        public void Run_SmoothMeasurements_ProducesSmoothOutput()
        {
            const int count = 50;
            float* measurements = stackalloc float[count];
            float* output = stackalloc float[count];

            for (int i = 0; i < count; i++)
            {
                measurements[i] = 10.0f + (float)(i % 10) + (float)((i * 17) % 5) * 0.1f;
            }

            ScalarKalmanFilter.Run(measurements, count, 0.1f, 0.5f, 1.0f, 0.0f, 0.0f, output);

            float maxJump = 0.0f;
            for (int i = 1; i < count; i++)
            {
                float jump = math.abs(output[i] - output[i - 1]);
                if (jump > maxJump) maxJump = jump;
            }

            Assert.IsTrue(maxJump < 5.0f);
        }
    }

    public sealed unsafe class VectorKalmanFilterTests
    {
        [Test]
        public void Predict_ReturnsUpdatedState()
        {
            float3 state = new float3(1, 2, 3);
            float3 velocity = new float3(1, 0, 0);
            float3 predicted = VectorKalmanFilter.Predict(state, velocity, 0.1f, 0.016f);
            Assert.IsTrue(predicted.x > state.x);
        }

        [Test]
        public void Update_SmoothsMeasurements()
        {
            float3 predictedState = new float3(5, 5, 5);
            float3 predictedCov = new float3(1.0f);
            float3 measurement = new float3(10, 10, 10);
            float3 updated;

            float3 result = VectorKalmanFilter.Update(predictedState, predictedCov, measurement, 1.0f, out updated);

            Assert.IsTrue(result.x > predictedState.x && result.x < measurement.x);
        }
    }
}