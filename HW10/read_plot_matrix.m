function data = read_plot_matrix(mySerial)
  data = zeros(100,4);                          % four values per sample: RAW, MAF, IIR, FIR
  for i=1:100
    data(i,:) = fscanf(mySerial,'%d %f %f %f'); % read in data from PIC32;
    times(i) = (i)*0.01;                        % 0.01 s between samples
  end					        
  stairs(times,data(:,1:4));                    % plot RAW and filtered data
  ylabel('accelZ');
  xlabel('Time (s)');  
end