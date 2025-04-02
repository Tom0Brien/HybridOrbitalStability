function json_str = generate_simulation_history_json(q)
    % q: A matrix with size [n_steps x 4]
    % json_str: A JSON string in the specified format

    n_steps = size(q, 1);

    % Initialize the JSON string
    json_str = '{';

    % Iterate through the rows of the q matrix
    for i = 1:n_steps
        % Concatenate the joint values to the JSON string
        json_str = [json_str, ...
                    sprintf('"%d": {\n', i - 1), ...
                    sprintf('    "floating_base_x": [%.16f],\n', q(i, 1)), ...
                    sprintf('    "floating_base_z": [%.16f],\n', q(i, 2)), ...
                    sprintf('    "left_hip_pitch": [%.16e],\n', q(i, 3)), ...
                    sprintf('    "right_hip_pitch": [%.16e]\n', q(i, 4))];

        % Add a comma separator for all steps except the last
        if i < n_steps
            json_str = [json_str, '},'];
        else
            json_str = [json_str, '}'];
        end
    end

    % Close the JSON object
    json_str = [json_str, '}'];

    fileID = fopen('simulation_history.json', 'w');
    if fileID == -1
        error('Cannot open file for writing');
    end
    fprintf(fileID, '%s', json_str);
    fclose(fileID);

end