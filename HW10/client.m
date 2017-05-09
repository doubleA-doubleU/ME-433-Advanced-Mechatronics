function client()
data = zeros(100,4); % four values per sample: RAW, MAF, IIR, FIR
times = zeros(1,100);
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
mySerial = serial('COM3', 'BaudRate', 9600,'Timeout',30); 
fopen(mySerial);
clean = onCleanup(@()fclose(mySerial));                                 
fprintf(mySerial,'r'); %send an 'r'
r = fscanf(mySerial,'%c'); %read the 'r' that is returned
for i=1:100
    data(i,:) = fscanf(mySerial,'%d %f %f %f'); % read in data from PIC32 
    times(i) = (i)*0.01; % 0.01 s between samples, 1s of data
end
figure
stairs(times,data(:,1));
hold
stairs(times,data(:,2)); % RAW v MAF
title('RAW vs. MAF');
xlabel('Time (s)');
ylabel('accelZ');
legend('RAW','MAF');

figure
stairs(times,data(:,1));
hold
stairs(times,data(:,3)); % RAW v IIR
title('RAW vs. IIR');
xlabel('Time (s)');
ylabel('accelZ');
legend('RAW','IIR');

figure
stairs(times,data(:,1));
hold
stairs(times,data(:,4)); % RAW v FIR
title('RAW vs. FIR');
xlabel('Time (s)');
ylabel('accelZ');
legend('RAW','FIR');