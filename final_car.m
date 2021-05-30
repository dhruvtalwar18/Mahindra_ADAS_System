%Set the camera Parameters
%focal_Length    = [528.82001 528.82001]; % Focal Length
%principal_Point = [383.15302  621.88000]; % The Optical Center
%image_Size      = [720 1280];  

focal_Length = [772.6633 775.6966]; % Focal Length
principal_Point = [622.8061 379.3357]; % The Optical Center
image_Size = [720 1280];
camIntrinsics = cameraIntrinsics(focal_Length, principal_Point, image_Size);
height = 1.0;    % Height of the camera from the gound
pitch  = 15;     % Pitch value of the camera
%The sensor is defined with the above parameters
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);

%The pretrained Aggregate Channel Features, model is loaded
% The full view model is used which is trained on full images of vehicles
Vehicle_detector = vehicleDetectorACF('full-view');

%The pedestrain detector is loaded with is trained on images of pedestrians
Ped_detector = peopleDetectorACF();

% Average width of pedestrains and vehicles is defined
vehicle_Width = [0,3];
ped_Width = [0,1];

% Both the Pedestrian and vehicle detector is then configured with the sensor and the witdth value set 
Ped_detector = configureDetectorMonoCamera(Ped_detector, sensor, ped_Width);
Vehicle_detector = configureDetectorMonoCamera(Vehicle_detector, sensor, vehicle_Width);


% A Multi-Object tracker which includes the Kalman filter is then
% initialized. 
[pedes_tracker, Pedes_position_Selector] = pedestrian_setup_tracker();
[vehicle_tracker, vehicle_position_Selector] = vehicle_setup_tracker();

% The Video file to be run is then loaded
video_File   = 'car_1.mp4';
detect_video_Reader = VideoReader(video_File);
videoPlayer = vision.DeployableVideoPlayer();

current_time_Step = 0;
snapshot = [];
snapTimeStamp = 120;
% The loop continues till the video reader has frames available
cont = hasFrame(detect_video_Reader);
while cont
    % The frame counters are updated 
    current_time_Step = current_time_Step + 1;
    
    % Read a frame
    detect_frame = readFrame(detect_video_Reader);
    
    % Implement both the detectors and the returned results into an object
    % which is required by the multiObjectTracker funtion    
    pedest_detections = pedestrian_detect_objects(Ped_detector, detect_frame, current_time_Step);
    vehicle_detections = vehicle_detect_objects(Vehicle_detector, detect_frame, current_time_Step);
 
        
    % The tracks are returned after being updated  for the 'currentStep' time.
    %After the first frame, a helper function
    % |detectionToTrackCost| is called to compute the customized cost
    % matrix between all tracks and all detections
    if current_time_Step == 1
        costMatrix1 = zeros(0, numel(pedest_detections)); 
        [pedestrian_confirmed_Tracks,~,allTracks] = updateTracks(pedes_tracker, pedest_detections, current_time_Step, ...
            costMatrix1);
    else
        costMatrix1 = ped_detection_Cost(allTracks, pedest_detections, ...
            Pedes_position_Selector, pedes_tracker.AssignmentThreshold);
        [pedestrian_confirmed_Tracks,~,allTracks] = updateTracks(pedes_tracker, pedest_detections, current_time_Step, ...
            costMatrix1);
    end
    vehicle_confirmed_Tracks = updateTracks(vehicle_tracker, vehicle_detections, current_time_Step);
    
       
    % The vehicles and pedestrians which are far away are then removed away.
    pedestrian_confirmed_Tracks = pedestrians_remove_noisy_tracks(pedestrian_confirmed_Tracks, Pedes_position_Selector, sensor.Intrinsics.ImageSize);
    vehicle_confirmed_Tracks = vehicle_remove_noisy_tracks(vehicle_confirmed_Tracks, vehicle_position_Selector, sensor.Intrinsics.ImageSize);
    
    
    % The annotations are added to the frame for it to be visualized
    frame_With_pedestrians = pedestrian_insert_box(detect_frame, pedestrian_confirmed_Tracks, Pedes_position_Selector, sensor);
    % The frame with the pedestrian annotatations is then fed to the
    % vehicle annotattion box inserter
    final_frame = vehicle_insert_box(frame_With_pedestrians, vehicle_confirmed_Tracks, vehicle_position_Selector, sensor);

    % Display the annotated frame.    
    videoPlayer(final_frame);
    
    % Exit the loop if the video player figure is closed by user.
    cont = hasFrame(detect_video_Reader) && isOpen(videoPlayer);
end


% Vehicle_setup_tracker()- creates a multiObjectTracker 
% to track many vehicles using the Kalman filters
function [vehicle_tracker, vehicle_position_selector] = vehicle_setup_tracker()
    % The Tracker object is created
    vehicle_tracker = multiObjectTracker('FilterInitializationFcn', @vehicle_init_Bbox_Filter, ...
        'AssignmentThreshold', 50, ...
        'NumCoastingUpdates', 5, ... 
        'ConfirmationParameters', [3 5]);

    % The State vector is: [x; vx; y; vy; w; vw; h; vh]
    % [x;y;w;h] = positionSelector * State
    vehicle_position_selector = [1 0 0 0 0 0 0 0; ...
                        0 0 1 0 0 0 0 0; ...
                        0 0 0 0 1 0 0 0; ...
                        0 0 0 0 0 0 1 0]; 
end


% The Vehicle_init_Bbox_Filter defines a Kalman filter to help filter bounding box measurement
function vehicle_filter = vehicle_init_Bbox_Filter(vehicle_detection)
% First the model and state is defined
% A constant velocity model is used for making the bounding box
%   The state is [x; vx; y; vy; w; wv; h; hv]
%   The state transition matrix is: 
%       [1 dt 0  0 0  0 0  0;
%        0  1 0  0 0  0 0  0; 
%        0  0 1 dt 0  0 0  0; 
%        0  0 0  1 0  0 0  0; 
%        0  0 0  0 1 dt 0  0; 
%        0  0 0  0 0  1 0  0;
%        0  0 0  0 0  0 1 dt; 
%        0  0 0  0 0  0 0  1]
%   dt = 1 is Assumed. In this case time-variant transition is  not
%   considered 
    deltat = 1;
    cvel =[1 deltat; 0 1];
    A_car = blkdiag(cvel, cvel, cvel, cvel);
 
%  Then noise is defined, this is the part that the model
%  does not take into account. In a non time variant model which is used the 
%  Acceleration is neglected
    G1d = [deltat^2/2; deltat];
    Q1d = G1d*G1d';
    Q = blkdiag(Q1d, Q1d, Q1d, Q1d);
 
%  The measurement model is defined
%  Only the position ([x;y;w;h]) is measured.
%  The measurement model is
    H_model_car = [1 0 0 0 0 0 0 0; ...
         0 0 1 0 0 0 0 0; ...
         0 0 0 0 1 0 0 0; ...
         0 0 0 0 0 0 1 0];
 
%  Then the sensor measurements is mapped to an initial state vector.
%  The velocity components are initialized to 0
    vehicle_state = [vehicle_detection.Measurement(1); ...
             0; ...
             vehicle_detection.Measurement(2); ...
             0; ...
             vehicle_detection.Measurement(3); ...
             0; ...
             vehicle_detection.Measurement(4); ...
             0];
 
% The sensor measurment is mapped to a state covariance matrix
%   For the parts of the state that the sensor measured directly, use the
%   corresponding measurement noise components. For the parts that the
%   sensor does not measure, assume a large initial state covariance. 
%   That way, future detections can be assigned to the track.
    vec_L = 100; % Large value
    vehicle_state_cov = diag([vehicle_detection.MeasurementNoise(1,1), ...
                     vec_L, ...
                     vehicle_detection.MeasurementNoise(2,2), ...
                     vec_L, ...
                     vehicle_detection.MeasurementNoise(3,3), ...
                     vec_L, ...
                     vehicle_detection.MeasurementNoise(4,4), ...
                     vec_L]);
 
% Finally a filter is created, for this model a linear
% Filter is used 
 
    vehicle_filter = trackingKF(...
        'StateTransitionModel', A_car, ...
        'MeasurementModel', H_model_car, ...
        'State', vehicle_state, ...
        'StateCovariance', vehicle_state_cov, ... 
        'MeasurementNoise', vehicle_detection.MeasurementNoise, ...
        'ProcessNoise', Q);
end

% The vehicle_detect_objects- detects vehicles in an image.
function vehicle_detections = vehicle_detect_objects(car_detector, car_frame, frameCount)
    % The detector is run on an image and 
    % a list of bounding boxes is returned: [x, y, w, h]
    vech_bboxes = detect(car_detector, car_frame);
    
    % The measurement noise is defined
    vech_L = 100;
    vech_measurementNoise = [vech_L 0  0  0; ...
                        0 vech_L  0  0; ...
                        0 0 vech_L/2 0; ...
                        0 0  0 vech_L/2];
                    
    % The detections is then formulated as a list of objectDetection reports.
    vehicle_num_detections = size(vech_bboxes, 1);
    vehicle_detections = cell(vehicle_num_detections, 1);                      
    for i = 1:vehicle_num_detections
        vehicle_detections{i} = objectDetection(frameCount, vech_bboxes(i, :), ...
            'MeasurementNoise', vech_measurementNoise);
    end
end


%  vehicle_remove_noisy_tracks function removes noisy tracks.
%  A track might be noisy if its predicted bounding box is quite small
%  The small bounding box implies that the car is quite far
function vechicle_tracks = vehicle_remove_noisy_tracks(vechicle_tracks, car_position_selector, imageSize)

    if isempty(vechicle_tracks)
        return
    end
    
    % The position is extracted from the tracks
    vehicle_positions = getTrackPositions(vechicle_tracks, car_position_selector);
    % The track is not valid if the bounding box is too small.
    invalid = ( vehicle_positions(:, 1) < 1 | ...
                vehicle_positions(:, 1) + vehicle_positions(:, 3) > imageSize(2) | ...
                vehicle_positions(:, 3) <= 20 | ...
                vehicle_positions(:, 4) <= 20 );
    vechicle_tracks(invalid) = [];
end


% The vehicle_insert_box funtion adds bounding boxes in the image and displays
% the track's position in world coordinates. 
function vechicle_I = vehicle_insert_box(vechicle_I, tracks, positionSelector, sensor)

    if isempty(tracks)
        return
    end

    labels = cell(numel(tracks), 1);
    
    % Retrieve positions of bounding boxes.
    bboxes = getTrackPositions(tracks, positionSelector);


    for i = 1:numel(tracks)        
        box = bboxes(i, :);
        
        % Convert to vehicle coordinates using the sensor mono camera object.
        xyVehicle = imageToVehicle(sensor, [box(1)+box(3)/2, box(2)+box(4)]);
        
        % These are the labels which will be displayed on the image
        labels{i} = sprintf('X=%.1f, Y=%.1f',xyVehicle(1),xyVehicle(2));
        
    end
    
    % The Value of alpha for the vehicle is defined here, It is the minimum
    % safe distance after which the vehicle will send message to the ROS
    % controller to apply brakes
    alpha_vehicle = 2.0 ; 
    
    % The minimum distance of the vehicle is used and is compared agaisnt
    % the alpha value
    if sqrt(xyVehicle(1)*xyVehicle(1)+xyVehicle(2)*xyVehicle(2)) < alpha_vehicle
        vechicle_I = insertObjectAnnotation(vechicle_I, 'rectangle', bboxes, labels, 'Color', 'red', ...
        'FontSize', 15, 'TextBoxOpacity', .8, 'LineWidth', 4);
        % Publish Message to ROS here!
        
    elseif sqrt(xyVehicle(1)*xyVehicle(1)+xyVehicle(2)*xyVehicle(2)) > alpha_vehicle
        vechicle_I = insertObjectAnnotation(vechicle_I, 'rectangle', bboxes, labels, 'Color', 'green', ...
        'FontSize', 15, 'TextBoxOpacity', .8, 'LineWidth', 4);
        % Continue current values
    end
       
    
end


% The Pedestrian_setup_tracker function creates a multiObjectTracker to track multiple pedestrians with 
% with Kalman filters. 
function [pedestrian_tracker, pedestrian_position_selector] = pedestrian_setup_tracker()
    % The Tracker object is created
    pedestrian_tracker = multiObjectTracker('FilterInitializationFcn', @pedestrian_init_Bbox_Filter, ...
        'AssignmentThreshold', 0.999, ...
        'NumCoastingUpdates', 5, ... 
        'ConfirmationParameters', [3 5], ...
        'HasCostMatrixInput', true);

    % The State vector is: [x; vx; y; vy; w; vw; h; vh]
    % [x;y;w;h] = positionSelector * State
    pedestrian_position_selector = [1 0 0 0 0 0 0 0; ...
                        0 0 1 0 0 0 0 0; ...
                        0 0 0 0 1 0 0 0; ...
                        0 0 0 0 0 0 1 0]; 
end

% The Pedestrian_init_Bbox_Filter defines a Kalman filter to filter bounding box measurement.
function pedestrian_filter = pedestrian_init_Bbox_Filter(pedes_detection)
% First the model and state is defined
% A constant velocity model is used for making the bounding box
%   The state is [x; vx; y; vy; w; wv; h; hv]
%   The state transition matrix is: 
%       [1 dt 0  0 0  0 0  0;
%        0  1 0  0 0  0 0  0; 
%        0  0 1 dt 0  0 0  0; 
%        0  0 0  1 0  0 0  0; 
%        0  0 0  0 1 dt 0  0; 
%        0  0 0  0 0  1 0  0;
%        0  0 0  0 0  0 1 dt; 
%        0  0 0  0 0  0 0  1]
%   dt = 1 is Assumed. In this case time-variant transition is  not
%   considered
    delta_t = 1;
    cvel =[1 delta_t; 0 1];
    A_Ped = blkdiag(cvel, cvel, cvel, cvel);
 
%  Then noise is defined, this is the part that the model
%  does not take into account. In a non time variant model which is used the 
%  Acceleration is neglected
    G1d = [delta_t^2/2; delta_t];
    Q1d = G1d*G1d';
    Q = blkdiag(Q1d, Q1d, Q1d, Q1d);
 
%  The measurement model is defined
%  Only the position ([x;y;w;h]) is measured.
%  The measurement model is
    H_model_ped = [1 0 0 0 0 0 0 0; ...
         0 0 1 0 0 0 0 0; ...
         0 0 0 0 1 0 0 0; ...
         0 0 0 0 0 0 1 0];
 
%  Then the sensor measurements is mapped to an initial state vector.
%  The velocity components are initialized to 0
    state_ped = [pedes_detection.Measurement(1); ...
             0; ...
             pedes_detection.Measurement(2); ...
             0; ...
             pedes_detection.Measurement(3); ...
             0; ...
             pedes_detection.Measurement(4); ...
             0];
 
% The sensor measurment is mapped to a state covariance matrix
%   For the parts of the state that the sensor measured directly, use the
%   corresponding measurement noise components. For the parts that the
%   sensor does not measure, assume a large initial state covariance. 
%   That way, future detections can be assigned to the track.
    L_ped = 120; % Large value
    pedestrian_state_Cov = diag([pedes_detection.MeasurementNoise(1,1), ...
                     L_ped, ...
                     pedes_detection.MeasurementNoise(2,2), ...
                     L_ped, ...
                     pedes_detection.MeasurementNoise(3,3), ...
                     L_ped, ...
                     pedes_detection.MeasurementNoise(4,4), ...
                     L_ped]);
 
% Finally a filter is created, for this model a linear
% Filter is used
    pedestrian_filter = trackingKF(...
        'StateTransitionModel', A_Ped, ...
        'MeasurementModel', H_model_ped, ...
        'State', state_ped, ...
        'StateCovariance', pedestrian_state_Cov, ... 
        'MeasurementNoise', pedes_detection.MeasurementNoise, ...
        'ProcessNoise', Q);
end

% The function pedestrain_detect_objects then detects people in an image.
function pedestrian_detections = pedestrian_detect_objects(ped_detector, ped_frame, frameCount)
    % The detector is run on an image and 
    % a list of bounding boxes is returned: [x, y, w, h]
    ped_bboxes = detect(ped_detector, ped_frame);
    
    % The measurement noise is defined
    ped_L = 100;
    measurementNoise = [ped_L 0  0  0; ...
                        0 ped_L  0  0; ...
                        0 0 ped_L/2 0; ...
                        0 0  0 ped_L/2];
                    
    % The detections is then formulated as a list of objectDetection reports.
    ped_num_detections = size(ped_bboxes, 1);
    pedestrian_detections = cell(ped_num_detections, 1);                      
    for i = 1:ped_num_detections
        pedestrian_detections{i} = objectDetection(frameCount, ped_bboxes(i, :), ...
            'MeasurementNoise', measurementNoise);
    end
end


%  pedestrians_remove_noisy_tracks function removes noisy tracks.
%  A track might be noisy if its predicted bounding box is quite small
%  The small bounding box implies that the person is quite far
function pedestrian_tracks = pedestrians_remove_noisy_tracks(pedestrian_tracks, ped_position_selector, imageSize)

    if isempty(pedestrian_tracks)
        return
    end
    
    % The position is extracted from the tracks.
    ped_positions = getTrackPositions(pedestrian_tracks, ped_position_selector);
    % The track is not valid if the bounding box is too small
    ped_invalid_track = ( ped_positions(:, 1) < 1 | ...
                ped_positions(:, 1) + ped_positions(:, 3) > imageSize(2) | ...
                ped_positions(:, 3) <= 5 | ...
                ped_positions(:, 4) <= 10 );
    pedestrian_tracks(ped_invalid_track) = [];
end

% The pedestrain_insert_box funtion adds bounding boxes in the image and displays
% the track's position in world coordinates.
function pedestrain_I = pedestrian_insert_box(pedestrain_I, pedes_tracks, pedes_position_selector, sensor)

    if isempty(pedes_tracks)
        return
    end

    labels = cell(numel(pedes_tracks), 1);
    % Retrieve positions of bounding boxes.
    bboxes = getTrackPositions(pedes_tracks, pedes_position_selector);

    for i = 1:numel(pedes_tracks)
        box = bboxes(i, :);
        
        xyImageLoc = [box(1)+box(3)/2, box(2)+box(4)];
        % Constrain the image point to be within the image boarder.
        xyImageLoc(1) = min(max(xyImageLoc(1), 1), size(pedestrain_I, 2));
        xyImageLoc(2) = min(xyImageLoc(2), size(pedestrain_I, 1));
        
        % Convert to vehicle coordinates using the sensor mono camera object.
        xyVehicle = imageToVehicle(sensor, xyImageLoc);
        
        % These are the labels which will be displayed on the image
        labels{i} = sprintf('x=%.1f,y=%.1f',xyVehicle(1),xyVehicle(2));
    end
    
    
    % The Value of alpha for the pedestrian is defined here, It is the minimum
    % safe distance after which the vehicle will send message to the ROS
    % controller to apply brakes
   alpha_pedestrian = 2.0 ;
   if sqrt(xyVehicle(1)*xyVehicle(1)+xyVehicle(2)*xyVehicle(2)) < alpha_pedestrian
        pedestrain_I = insertObjectAnnotation(pedestrain_I, 'rectangle', bboxes, labels, 'Color', 'red', ...
        'FontSize', 15, 'TextBoxOpacity', .8, 'LineWidth', 4);
        %disp("Pedestrian very close!! Collision possible!")
        % Send message to ROS
    elseif sqrt(xyVehicle(1)*xyVehicle(1)+xyVehicle(2)*xyVehicle(2)) > alpha_pedestrian
        pedestrain_I = insertObjectAnnotation(pedestrain_I, 'rectangle', bboxes, labels, 'Color', 'green', ...
        'FontSize', 15, 'TextBoxOpacity', .8, 'LineWidth', 4);
        %disp("All Safe!")
    end
end


% The ped_detection_Cost function detectionToTrackCost computes the
% customized cost for detections to track assignment.
function ped_cost_matrix = ped_detection_Cost(tracks, detections, ...
                                positionSelector, threshold)

    if isempty(tracks) || isempty(detections)
        ped_cost_matrix = zeros(length(tracks), length(detections));
        return
    end
    
    % The overlap ratio is calculated between the predicted boxes
    % and the detected boxes, and a cost is assigned to each detection
    % for each track. The cost is the least when the predicted box is
    % totally aligned with the detected box 
    
    % Retrieve the positions of the bounding boxes.
    ped_track_bboxes = getTrackPositions(tracks, positionSelector);
    % The width and height are checked for a positive value before
    % they are used for computing the box overlap ratio.
    ped_track_bboxes(:, 3) = max(eps, ped_track_bboxes(:, 3));
    ped_track_bboxes(:, 4) = max(eps, ped_track_bboxes(:, 4));
    
    % Extract the detected bounding box from all the detections.
    ped_alldetections = [detections{:}];
    bboxes = reshape([ped_alldetections(:).Measurement], 4, length(detections))';
    
    % Compute all assignment costs.
    ped_cost_matrix = 1 - bboxOverlapRatio(ped_track_bboxes, bboxes);
    % If there is very little overlap the cost is set to infinite
    ped_cost_matrix(ped_cost_matrix(:) > threshold) = Inf;
end

