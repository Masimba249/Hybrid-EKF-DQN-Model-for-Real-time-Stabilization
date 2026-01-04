function state_index = discretize_state(state)
    position = round(state(1:3) * 10); % Scale position for discrete bins
    state_index = min(max(1, position(1) + 50), 100); % Map to 1-100 range
end
