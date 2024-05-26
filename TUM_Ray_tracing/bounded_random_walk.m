function [results] = bounded_random_walk(lat0, lon0, alt0, num_steps, step_size, dt, lat_bounds, lon_bounds, alt_bounds)
    % lat0, lon0, alt0: Initial latitude, longitude, and altitude
    % num_steps: Number of steps in the random walk
    % step_size: Step size for the walk (constant for simplicity)
    % dt: Time step for each movement
    % lat_bounds: [min_lat, max_lat]
    % lon_bounds: [min_lon, max_lon]
    % alt_bounds: [min_alt, max_alt]

    % Earth's radius in meters
    R = 6371000;

    % Preallocate arrays for efficiency
    latitudes = zeros(num_steps, 1);
    longitudes = zeros(num_steps, 1);
    altitudes = zeros(num_steps, 1);
    times = zeros(num_steps, 1);

    % Initialize the starting point
    latitudes(1) = lat0;
    longitudes(1) = lon0;
    altitudes(1) = alt0;
    times(1) = 0;

    % Convert step size to degrees (approximation)
    deg_per_meter = 360 / (2 * pi * R);

    for i = 2:num_steps
        % Generate random angles for 2D walk (latitude and longitude)
        theta = 2 * pi * rand; % Random direction in 2D plane

        % Update latitude and longitude based on step size and direction
        dlat = step_size * deg_per_meter * cos(theta);
        dlon = step_size * deg_per_meter * sin(theta) / cosd(latitudes(i-1));

        % Generate random step for altitude
        dalt = step_size * (2 * rand - 1); % Random step in z direction

        % Calculate potential new positions
        new_lat = latitudes(i-1) + dlat;
        new_lon = longitudes(i-1) + dlon;
        new_alt = altitudes(i-1) + dalt;

        % Check bounds for latitude
        if new_lat < lat_bounds(1)
            new_lat = lat_bounds(1) + (lat_bounds(1) - new_lat);
        elseif new_lat > lat_bounds(2)
            new_lat = lat_bounds(2) - (new_lat - lat_bounds(2));
        end

        % Check bounds for longitude
        if new_lon < lon_bounds(1)
            new_lon = lon_bounds(1) + (lon_bounds(1) - new_lon);
        elseif new_lon > lon_bounds(2)
            new_lon = lon_bounds(2) - (new_lon - lon_bounds(2));
        end

        % Check bounds for altitude
        if new_alt < alt_bounds(1)
            new_alt = alt_bounds(1) + (alt_bounds(1) - new_alt);
        elseif new_alt > alt_bounds(2)
            new_alt = alt_bounds(2) - (new_alt - alt_bounds(2));
        end

        % Update positions
        latitudes(i) = new_lat;
        longitudes(i) = new_lon;
        altitudes(i) = new_alt;
        times(i) = times(i-1) + dt;
    end

    % Combine results into a single matrix
    results = [times, latitudes, longitudes, altitudes];

    % % Display results
    % disp('Time, Latitude, Longitude, Altitude');
    % disp(results);
    % 
    % % Optionally, plot the 2D and 3D paths
    % figure;
    % subplot(2,1,1);
    % plot(longitudes, latitudes);
    % xlabel('Longitude');
    % ylabel('Latitude');
    % title('2D Random Walk (Latitude vs Longitude)');
    % grid on;
    % 
    % subplot(2,1,2);
    % plot3(longitudes, latitudes, altitudes);
    % xlabel('Longitude');
    % ylabel('Latitude');
    % zlabel('Altitude');
    % title('3D Random Walk');
    % grid on;

    % Save the results to a CSV file
    %csvwrite('bounded_random_walk_output.csv', results);
end
