function [x_pred, P_pred] = ekf_predict(state, P, F, Q)
    % State prediction
    x_pred = F * state;
    
    % Covariance prediction
    P_pred = F * P * F' + Q;
end
