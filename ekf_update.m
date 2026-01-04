function [state, P] = ekf_update(x_pred, P_pred, z, H, R, K)
    % Measurement residual
    y = z - H * x_pred;
    
    % Update state estimate with Kalman gain
    state = x_pred + K * y;
    
    % Update covariance matrix
    P = (eye(size(P_pred)) - K * H) * P_pred;
end
