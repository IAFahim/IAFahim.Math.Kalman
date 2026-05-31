namespace IAFahim.Math.Kalman
{
    using System;
    using System.Runtime.CompilerServices;
    using Unity.Mathematics;

    public static unsafe class VectorKalmanFilter
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 Predict(float3 state, float3 velocity, float processNoise, float dt)
        {
            return state + velocity * dt;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 PredictCovariance(float3 covariance, float processNoise, float dt)
        {
            return covariance + new float3(processNoise * dt);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 Update(float3 predictedState, float3 predictedCov, float3 measurement, float measurementNoise, out float3 updatedCov)
        {
            float3 kalmanGain = predictedCov / (predictedCov + new float3(measurementNoise));
            updatedCov = (new float3(1.0f) - kalmanGain) * predictedCov;
            return predictedState + kalmanGain * (measurement - predictedState);
        }

        public static void Run(float3* measurements, int count, float processNoise, float measurementNoise,
            float dt, float3 initialState, float3 initialVelocity, float3* outStates)
        {
            float3 state = initialState;
            float3 covariance = new float3(1.0f);
            float3 velocity = initialVelocity;

            for (int i = 0; i < count; i++)
            {
                float3 predictedState = Predict(state, velocity, processNoise, dt);
                float3 predictedCov = PredictCovariance(covariance, processNoise, dt);

                state = Update(predictedState, predictedCov, measurements[i], measurementNoise, out covariance);

                if (i > 0)
                {
                    velocity = (state - outStates[i - 1]) / dt;
                }

                outStates[i] = state;
            }
        }
    }
}
