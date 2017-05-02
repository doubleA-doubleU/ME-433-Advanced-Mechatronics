function client()
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
port = 'COM3';
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 30 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 9600,'Timeout',30); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('\nPIC32 ACCELEROMETER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('     q: Quit client                        r: Read accelerometer\n');

    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c',selection);
    
    % take the appropriate action
    switch selection
        case 'q'
            has_quit = true;             % exit client
        case 'r'
            read_plot_matrix(mySerial);
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end
