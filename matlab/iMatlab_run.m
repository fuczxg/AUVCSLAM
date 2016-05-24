function [] = iMatlab_run(moosfile, id)


iMatlab('init','MOOSNAME','iMatlab','CONFIG_FILE',moosfile);
iMatlab('MOOS_PAUSE',1);

if (id == 1)
    load ../data/E-W-2_detections_summary.mat
    load ../data/E-W-2_vehicle_telemetry.mat

    detections = total_sonar_detections;
    telemetry = total_vehicle_telemetry;
elseif (id == 2)
    load ../data/N-S-1_detections_summary.mat
    load ../data/N-S-1_vehicle_telemetry.mat

    detections = total_sonar_detections;
    telemetry = total_vehicle_telemetry;

elseif (id == 3)
    load ../data/N-S-1_detections_summary.mat
    load ../data/N-S-1_vehicle_telemetry.mat

    detections = total_sonar_detections;
    telemetry = total_vehicle_telemetry;
end
start_time = telemetry(1,7) + 60*(telemetry(1,6) + 60*(telemetry(1,5)));
detection_time1 = detections(1,7) + 60*(detections(1,6) + 60*(detections(1,5)));

i = 1;
j = 1;
tic
%for i=1:min([length(telemetry1),length(telemetry2),length(telemetry3)])
while 1
    telemetry_time = telemetry(i,7) + 60*(telemetry(i,6) + 60*(telemetry(i,5)));
    current_time = toc;
    if (current_time > (telemetry_time-start_time))
%        iMatlab('MOOS_MAIL_TX','SIM_LAT',telemetry(i,8));
%        iMatlab('MOOS_MAIL_TX','SIM_LON',telemetry(i,9));
        [x,y] = gps_to_xy(telemetry(i,8),telemetry(i,9));
        iMatlab('MOOS_MAIL_TX','SIM_X',x);
        iMatlab('MOOS_MAIL_TX','SIM_Y',y);
        iMatlab('MOOS_MAIL_TX','SIM_HEADING',telemetry(i,10));
        iMatlab('MOOS_MAIL_TX','SIM_SPEED',telemetry(i,11));
        iMatlab('MOOS_MAIL_TX','SIM_PITCH',telemetry(i,12));
        iMatlab('MOOS_MAIL_TX','SIM_ROLL',telemetry(i,13));
        
       if (telemetry_time == detection_time1)
           %publish detection
           % step 1: calculate the 2d range from slant range and altitude:
           % first try just publish as based on GPS coords?
           j = j+1;
       end
       i = i+1;
    end
    iMatlab('MOOS_PAUSE',0.05);
end
end
    
function [x,y] = gps_to_xy(lat, lon)
lat_origin = 46.2954316666700;
lon_origin = -60.1282133333300;

delta_lat = pi*(lat-lat_origin)/180;
delta_lon = pi*(lon-lon_origin)/180;

a1 = sin(delta_lat)^2;
a2 = cos(pi*lat/180)*cos(pi*lat_origin/180)*sin(delta_lon)^2;
c1 = 2*atan2(sqrt(a1),sqrt(1-a1));
c2 = 2*atan2(sqrt(a2),sqrt(1-a2));
y = c1*6371000;
x = c2*6371000;
end