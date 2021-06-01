%-------Initialize code--------
clc
clear all
close all

%------Reading the mp4 File----- 
Input_vid  = VideoReader('car_1_v_short.mp4');

%----Defining the output File----
Out_vid = VideoWriter('iit_car_res_v_short');
Out_vid.FrameRate = 30;
open(Out_vid);
count=1;
diff_list = [0,0,0,0,0,0,0,0,0,0];
while hasFrame(Input_vid)
    
    %------------------Input stream of frames----------
    frame = readFrame(Input_vid);
    %   imshow(frame);

    frame = imgaussfilt3(frame);
    %    figure('Name','Gauss blurred Image'), imshow(frame);
   
    %-------------- White Mask--------------
    %-------Define optimal range for Hue, Saturation and Value-------------
    HueMin = 150;
    HueMax = 250;

    SatMin = 150;
    SatMax = 250;

    ValMin = 150;
    ValMax = 250;
    
    %-----------Mask based on selected thresholds-----------
    
    White_masked_frame=((frame(:,:,1)>=HueMin)|(frame(:,:,1)<=HueMax))&(frame(:,:,2)>=SatMin)&...
        (frame(:,:,2)<=SatMax)&(frame(:,:,3)>=ValMin)&(frame(:,:,3)<=ValMax);
    % figure('binary_masked'), imshow(White_masked_frame);
    
    %-----------------Edge Detection-------------------------------

    New_frame = edge(White_masked_frame, 'canny');
    New_frame = bwareaopen(New_frame,50);
    
    % imshow(New_frame);
    
    %---------Defining a Region of Interest for Left lane---------
    
     x =[300,650,650,0];
    y = [200,200,540,540];
    left_roi = roipoly(New_frame, x, y);
    [X , Y] = size(left_roi);
    for i = 1:X
        for j = 1:Y
            if left_roi(i,j) == 1
                left_roi_frame(i,j) = New_frame(i,j);
            else
                left_roi_frame(i,j) = 0;  
            end
        end
    end  
    
    %  imshow(left_roi_frame);
    
    %---------Defining a Region of Interest for Right lane----------
    
    x =[700,1300,1300,700];
    y = [200,200,540,540];
    right_roi = roipoly(New_frame, x, y);
    [X , Y] = size(right_roi);
    for i = 1:X
        for j = 1:Y
            if right_roi(i,j) == 1
                right_roi_frame(i,j) = New_frame(i,j);
            else
                right_roi_frame(i,j) = 0;  
            end
        end
    end  
    
    %  imshow(right_roi_frame);
    
    %------Hough Tansform on edge detected images to generate lines--------
    
    [Left_H_matrix,left_theta,left_rho] = hough(left_roi_frame);
    [Right_H_matrix,right_theta,right_rho] = hough(right_roi_frame);
    right_theta
    right_rho
    %-----------------Get 3 Peaks from each Hough Transform----------------
    
    Left_peaks_temp = houghpeaks(Left_H_matrix,3,'threshold',3);
    Right_peaks_temp = houghpeaks(Right_H_matrix,3,'threshold',3);
    
    %-------If peaks not detected, use previous value----------------------
    if sum(size(Left_peaks_temp))==5
        Left_peaks=Left_peaks_temp;
    end
    
    if sum(size(Right_peaks_temp))==5
        Right_peaks=Right_peaks_temp;
    end
      
    %--------------Extracting Lines from Detected Hough Peaks--------------
    
    left_lane_lines_temp = houghlines(left_roi_frame,left_theta,left_rho,Left_peaks,'FillGap',3000,'MinLength',20);
    
    right_lane_lines_temp = houghlines(right_roi_frame,right_theta,right_rho,Right_peaks,'FillGap',3000,'MinLength',20);
    
    if sum(size(left_lane_lines_temp))==4
        left_lane_lines = left_lane_lines_temp;
    end
    
    if sum(size(right_lane_lines_temp))==4
        right_lane_lines = right_lane_lines_temp;
    end
    
    %     max_len = 0;
    %     for k = 1:2
    %        xy = [right_lane_lines(k).point1; right_lane_lines(k).point2];
    %        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
    %     end
    %     hold off 
    
    %-----------------Best fitting lane lines---------------- 
    
    leftp1 = [left_lane_lines(1).point1; left_lane_lines(1).point2];
    leftp2 = [left_lane_lines(2).point1; left_lane_lines(2).point2];  
    
    rightp1 = [right_lane_lines(1).point1; right_lane_lines(1).point2];
    rightp2 = [right_lane_lines(2).point1; right_lane_lines(2).point2];
    
    if leftp1(1,1) < leftp2(1,1)
        left_lane(1,:) = leftp1(1,:);
    else
        left_lane(1,:) = leftp2(1,:);
    end
    
    if leftp1(2,2) < leftp2(2,2)
        left_lane(2,:) = leftp1(2,:);
    else
        left_lane(2,:) = leftp2(2,:);
    end
    
    if rightp1(1,2) < rightp2(1,2)
        right_lane(1,:) = rightp1(1,:);
    else
        right_lane(1,:) = rightp2(1,:);
    end
    
    if rightp1(2,1) > rightp2(2,2)
        right_lane(2,:) = rightp1(2,:);
    else
        right_lane(2,:) = rightp2(2,:);
    end
    
    %---------------Finding lane slopes and extrapolating---------------
    
    %Calculating slope using end points
    slopeL = (left_lane(2,2)-left_lane(1,2))/(left_lane(2,1)-left_lane(1,1));
    if slopeL>-0.2 || slopeL<-1
        slopeL = -0.5;
    end
    slopeR = (right_lane(2,2)-right_lane(1,2))/(right_lane(2,1)-right_lane(1,1));
    if slopeR<0.2 || slopeR>1
        slopeR = 0.5;
    end
    % Specifying X coordinates and then finding Y coordinates using slope
    %Start from left edge
    Left_lane_x_left = 1;
    Left_lane_x_right = 350;
    %Finding corresponsding y coordinates
    %Left_lane_y_left = slopeL * (Left_lane_x_left - left_lane(1,1)) + left_lane(1,2);
    Left_lane_y_left = 516;
    %Left_lane_y_right = slopeL * (Left_lane_x_right - left_lane(2,1)) + left_lane(2,2);
    Left_lane_y_right = Left_lane_y_left + (Left_lane_x_right-Left_lane_x_left)*slopeL;
    
    % Start from right edge
    %Right_lane_x_left = 950; 
    Right_lane_x_left = 914;
    Right_lane_x_right = 1264;
    %Finding corresponsding y coordinates
    %Right_lane_y_left = slopeR * (Right_lane_x_left - right_lane(1,1)) + right_lane(1,2);
    Right_lane_y_right = 516;
    Right_lane_y_left = Right_lane_y_right - (Right_lane_x_right - Right_lane_x_left)*slopeR;
    %Right_lane_y_right = slopeR * (Right_lane_x_right - right_lane(2,1)) + right_lane(2,2);
    
    %------Highlighting the movable region-------
    points = [Left_lane_x_left Left_lane_y_left; Left_lane_x_right Left_lane_y_right ;Right_lane_x_left Right_lane_y_left; Right_lane_x_right Right_lane_y_right ];
    number = [1 2 3 4];
 
    %-----------------Plot the result on original frame--------------------
    
    %figure('Name','Final Output')
    imshow(frame);
    [height, width, dim] = size(frame);
    hold on
    % frame centre marker
    plot((width+60)/2,height/2,'r+', 'MarkerSize', 50); 
    dev =((left_lane(2,1)+left_lane(1,1))/2+(right_lane(1,1)+ right_lane(2,1))/2)/2;
    adding_pos = mod(count,10)+1;
    diff_list(adding_pos)=dev;
    if count>10
        dev = mean(diff_list);
    end
    % deviation marker
    ww = dev - (width+60)/2;
    plot(dev,height/2,'b+', 'MarkerSize', 50);
    % calculating steering angle
    steering = atand(ww/(height/2));
    print_steer = ['Steer: ' num2str(steering,'%0.2f')];
    count=count+1;
    % plotting lane lines
    plot([Left_lane_x_left, Left_lane_x_right], [Left_lane_y_left, Left_lane_y_right], 'LineWidth',8,'Color','red');
    plot([Right_lane_x_left, Right_lane_x_right], [Right_lane_y_left, Right_lane_y_right], 'LineWidth',8,'Color','red');
    
    if steering>=-4 && steering<=4
       % print straight 
       text(650, 65, 'Straight','horizontalAlignment', 'center', 'Color','red','FontSize',30) 
    else
       % print steering angle
       text(650, 65, print_steer,'horizontalAlignment', 'center', 'Color','red','FontSize',30) 
    end
    % plotting trapezoid
    patch('Faces', number, 'Vertices', points, 'FaceColor','green','Edgecolor','green','FaceAlpha',0.4)
    hold off
      
    %------------------Saving frame in output file------------------
    writeVideo(Out_vid,getframe);    
end

%-------------------------------Closing------------------------------------
close(Out_vid)