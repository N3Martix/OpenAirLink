viewer = siteviewer("Buildings","TUM_map.osm","Basemap","topographic");

% Transmitter
tx = txsite("Name","Small cell transmitter", ...
    "Latitude",48.14909567490029,...
    "Longitude",11.566954757894084, ...
    "AntennaHeight",6, ...
    "TransmitterPower",5, ...
    "TransmitterFrequency",37e8);

show(tx)

% Propogation Model
% LOS with fspl, concrete materials
rtpm = propagationModel("raytracing", ...
    "Method","sbr", ...
    "MaxNumReflections",0, ...
    "BuildingsMaterial","concrete", ...
    "TerrainMaterial","concrete");

% Generate Routes, it takes some default positions and creates routes
% between them 
%coods = generate_routes();
%latitude = coods.lats.';
%longitude = coods.lons.';

% Use random Walk function with bounding boxes to create a random walk with
% 100 samples. 
% Initial coordinates (example: San Francisco)
lat0 = 48.148934;
lon0 = 11.5673;
alt0 = 0;

% Number of steps and step size in meters
num_steps = 100;
step_size = 1;
dt = 1; % Time step in seconds

% Define boundaries for latitude, longitude, and altitude
lat_bounds = [48.14867172996017, 48.149256973302265];
lon_bounds = [11.567141561605183, 11.567435014244525];
alt_bounds = [0, 2];

% Call the function
walk = bounded_random_walk(lat0, lon0, alt0, num_steps, step_size, dt, lat_bounds, lon_bounds, alt_bounds);

latitude = walk(:,2);
longitude = walk(:,3);
height = walk(:,4).';

% Receiver 
rxs = rxsite("Name","Small cell receiver", ...
    "Latitude",latitude, ...
    "Longitude",longitude,...
    "AntennaHeight",1);

signalStrength = sigstrength(rxs, tx)';

tbl = table(latitude, longitude, signalStrength);
pd = propagationData(tbl);

legendTitle = "Signal" + newline + "Strength" + newline + "(dB)";
plot(pd, "LegendTitle", legendTitle, "Colormap", parula);

rays = raytrace(tx,rxs,rtpm);

propagationDelays = zeros(1, length(rays));
pathlosses = zeros(1, length(rays));

% Loop through each cell and extract the required properties
for i = 1:length(rays)
    propagationDelays(i) = rays{i}.PropagationDelay;
    pathlosses(i) = rays{i}.PathLoss;
end

resultant_table = table(walk(:,1), propagationDelays', pathlosses');
resultant_table.Properties.VariableNames(1:3) = {'Time','PathDelay','PathLoss'};
writetable(resultant_table,'Time_PathDelay_PathLoss_TUM_campus.csv');

%% Generate OpenAirLink Coeffs
base_loss = 30; % Power difference of input and output
index_len = length(walk(:,1));
coeff_airlink = cell(index_len, 3);

% Time Index
coeff_airlink{1} = walk(:,1);

% Attenuation & Shift Bits
emulate_loss = pathlosses.' - base_loss;
shift_bits = floor(emulate_loss/(20*log10(2)));
fine_att    = mod(emulate_loss, shift_bits*(20*log10(2)));
fir_att = round(32767*10.^(-fine_att/20));

coeff_airlink{3} = shift_bits;

% FIR Taps & Coeff Gen
fir_taps = round(propagationDelays'*1e9/5) + 1;

for i = 1:index_len
    fir_coeff  = zeros(1, 41);
    fir_coeff(fir_taps(i)) = fir_att(i);

    firString = arrayfun(@(x) sprintf('%.6g', x), fir_coeff, 'UniformOutput', false);
    firString = strjoin(firString, ' ');

    coeff_airlink{i,1} = walk(i,1);
    coeff_airlink{i,2} = firString;
    coeff_airlink{i,3} = shift_bits(i,1);
end

writecell(coeff_airlink, 'OpenAirLinkCoeff.csv');