% Initial coordinates (example: San Francisco)
lat0 = 48.148934;
lon0 = 11.5673;
alt0 = 0;

% Number of steps and step size in meters
num_steps = 1000;
step_size = 1.5;
dt = 1; % Time step in seconds

% Define boundaries for latitude, longitude, and altitude
lat_bounds = [48.149256973302265, 48.14867172996017];
lon_bounds = [11.567141561605183, 11.567435014244525];
alt_bounds = [0, 2];

% Call the function
walk = bounded_random_walk(lat0, lon0, alt0, num_steps, step_size, dt, lat_bounds, lon_bounds, alt_bounds);
