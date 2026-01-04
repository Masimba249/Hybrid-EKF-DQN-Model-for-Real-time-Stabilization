function K = ekf_gain(P_pred, H, R)
    % Residual covariance
    S = H * P_pred * H' + R;
    
    % Kalman gain
    K = P_pred * H' / S;
end
