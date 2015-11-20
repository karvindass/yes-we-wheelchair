fh = fopen('data_jump_1723_04212014.txt','r');

line = '';
x = zeros(1,10000);
y = zeros(1,10000);
theta = zeros(1,10000);
i = 1;

% while ischar(line)
%     line = fgetl(fh);
%     if (isempty(line))
%         line = fgetl(fh);
%     end
%     [xstr, line] = strtok(line);
%     x(i) = str2double(xstr);
%     [ystr, line] = strtok(line);
%     y(i) = str2double(ystr);
%     [thetaStr, line] = strtok(line);
%     theta(i) = str2double(thetaStr);
%     i = i + 1;
% end
%     
fclose(fh);