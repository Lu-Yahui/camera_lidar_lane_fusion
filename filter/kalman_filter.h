#ifndef CAMERA_LIDAR_LANE_FUSION_FILTER_KALMAN_FILTER_H
#define CAMERA_LIDAR_LANE_FUSION_FILTER_KALMAN_FILTER_H

#include <functional>

namespace camera_lidar_lane_fusion
{

template <typename StateType>
class KalmanFilter
{
  public:
    template <typename ControlDataType>
    StateType Predict(const StateType& last_state,
                      const ControlDataType& control_data,
                      const std::function<typename StateType::MeanType(const typename StateType::MeanType&,
                                                                       const ControlDataType&)>& predict_func);

    template <typename MeasurementType>
    StateType Correct(const StateType& predicted_state,
                      const MeasurementType& measurement,
                      const std::function<typename MeasurementType::MeanType(const typename StateType::MeanType&,
                                                                             const MeasurementType&)>& measure_func);
};

template <typename StateType>
template <typename ControlDataType>
StateType KalmanFilter<StateType>::Predict(
    const StateType& last_state,
    const ControlDataType& control_data,
    const std::function<typename StateType::MeanType(const typename StateType::MeanType&, const ControlDataType&)>&
        predict_func)
{
    // predict mean
    const typename StateType::MeanType predicted_mean = predict_func(last_state.Mean(), control_data);

    // predict covariance
    const auto& F = last_state.Jacobian();
    const typename StateType::CovType predicted_cov = F * last_state.Cov() * F.transpose() + control_data.Cov();

    return StateType(predicted_mean, predicted_cov);
}

template <typename StateType>
template <typename MeasurementType>
StateType KalmanFilter<StateType>::Correct(
    const StateType& predicted_state,
    const MeasurementType& measurement,
    const std::function<typename MeasurementType::MeanType(const typename StateType::MeanType&,
                                                           const MeasurementType&)>& measure_func)
{
    const auto& H = measurement.Jacobian();
    const auto& S = H * predicted_state.Cov() * H.transpose() + measurement.Cov();
    // kalman gain
    const auto& K = predicted_state.Cov() * H.transpose() * S.inverse();

    // correct mean
    const typename MeasurementType::MeanType predicted_measurement = measure_func(predicted_state.Mean(), measurement);
    const typename StateType::MeanType corrected_mean =
        predicted_state.Mean() + K * (measurement - predicted_measurement);

    // correct covariance with Joseph form
    const auto& cov_factor = typename StateType::CovType::Identity() - K * H;
    const typename StateType::CovType corrected_cov =
        cov_factor * predicted_state.Cov() * cov_factor.transpose() + K * measurement.Cov() * K.transpose();

    return StateType(corrected_mean, corrected_cov);
}

}  // namespace camera_lidar_lane_fusion

#endif  // CAMERA_LIDAR_LANE_FUSION_FILTER_KALMAN_FILTER_H