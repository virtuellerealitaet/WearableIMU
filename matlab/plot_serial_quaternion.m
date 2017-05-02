%clear display
clear
clc
 
%serial port configuration
serialPort = 'COM4';
s = serial(serialPort);
s.BaudRate = 57600;
s.FlowControl = 'hardware';
s.Timeout = 1000;
s.OutputBufferSize = 1000;

%%

% initialize input data
qx = single(0);
qy = single(0);
qz = single(0);
qw = single(0);

%Set up Plot
plotTitle = 'Sensor quaternion Data Log';  % plot title
xLabel = 'Elapsed Time (s)';    % x-axis label
yLabel = 'Data';                % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
min = -1.5;                     % set y-min
max = 1.5;                      % set y-max
scrollWidth = 100;               % display period in plot, plot entire data log if <= 0
delay = .01;                    % make sure sample faster than resolution

time = 0;
data = zeros(100,4);
count = 0;

plotGraph = plot(rand(4));
size(plotGraph)

title(plotTitle,'FontSize',25);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
axis([0 10 min max]);
grid(plotGrid);


%%

fopen(s);
disp('Close Plot to End Session');

tic
 
while ishandle(plotGraph) %Loop when Plot is Active

    % read serial and convert to quaternion values (single float)
    while(s.BytesAvailable > 16)
        X = fread(s,16,'char');

        qx = typecast(uint8(X(1:4)), 'single');
        qy = typecast(uint8(X(5:8)), 'single');
        qz = typecast(uint8(X(9:12)), 'single');
        qw = typecast(uint8(X(13:16)), 'single');
    end
    %fprintf('%.2f %2.f %.2f %.2f\n', qx,qy,qz,qw);
    
    % fill plot data and draw plot
    if(~isempty(qx) && isfloat(qx)) %Make sure Data Type is Correct        
        count = count + 1;    
        time(count) = toc;    %Extract Elapsed Time
        
        data(count,1) = qx;
        data(count,2) = qy;
        data(count,3) = qz;
        data(count,4) = qw;
        
        for k = 1:numel(plotGraph)
            if(scrollWidth > 0)
                set(plotGraph(k), 'XData', time(time > time(count)-scrollWidth), 'YData', data(time > time(count)-scrollWidth,k))
                axis([time(count)-scrollWidth time(count) min max]);
            end        
        end
                 
        %update plots
        pause(delay);
    end
end

%%

%Close Serial COM Port and Delete useless Variables
fclose(s);

clear count dat delay max min plotGraph plotGrid plotTitle s ...
        scrollWidth serialPort xLabel yLabel;

disp('Session Terminated...');