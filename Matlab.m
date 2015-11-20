clear;
clc;

s = serial('COM4'); %assigns the object s to serial port
set(s, 'BaudRate', 9600);



fopen(s); %opens the serial port
i =1;
data = fscanf(s , '%f' ); 
pause(3);

run = 1;


while( run == 1) 
 data = fscanf(s , '%f' ); %reads into vector 

 if ( ~isempty(data) )
     
 xdata(i) = data(1);
 ydata(i) = data(2);
 thetadata(i) = data(3)
 
 i = i + 1;
 plot( xdata , ydata )
  hold on
 pause(0.05);
 
 end
end

fclose(s);

