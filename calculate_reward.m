function reward = calculate_reward(next_state, target_state)
    reward = -norm(next_state(1:3) - target_state(1:3)); % Negative distance to target
end
