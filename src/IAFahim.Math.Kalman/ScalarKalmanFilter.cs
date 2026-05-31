namespace IAFahim.Math.Kalman
{
    using System;
    using System.Runtime.CompilerServices;
    using Unity.Mathematics;

    public static unsafe class ScalarKalmanFilter
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Predict(float state, float velocity, float processNoise, float dt)
        {
            return state + velocity * dt;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float PredictCovariance(float covariance, float processNoise, float dt)
        {
            return covariance + processNoise * dt;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Update(float predictedState, float predictedCovariance, float measurement, float measurementNoise, out float updatedCovariance)
        {
            float kalmanGain = predictedCovariance / (predictedCovariance + measurementNoise);
            updatedCovariance = (1.0f - kalmanGain) * predictedCovariance;
            return predictedState + kalmanGain * (measurement - predictedState);
        }

        public static void Run(float* measurements, int count, float processNoise, float measurementNoise,
            float dt, float initialState, float initialVelocity, float* outStates)
        {
            float state = initialState;
            float covariance = 1.0f;
            float velocity = initialVelocity;

            for (int i = 0; i < count; i++)
            {
                float predictedState = Predict(state, velocity, processNoise, dt);
                float predictedCov = PredictCovariance(covariance, processNoise, dt);

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
