function tdof
   %pkg load instrument-control
   clear
   clc
   
   serialport = 'COM97';
   baudrade = '115200';
   ptitle = 'Serial Data';
   delay = 0.0001;
   
   markersize = 16;
   l0 = 1.4;
   l1 = 1.5;
   l2 = 1.2;
   a1 = pi/4;
   a2 = -pi/6;
   
   function [x1, y1, x2, y2] = forkin(a1, a2, l1, l2)
     x1 = l1*cos(a1);
     y1 = l1*sin(a1);
     x2 = l1*cos(a1) + l2*cos(a1+a2);
     y2 = l1*sin(a1) + l2*sin(a1+a2);
   end
   
   
   [x1, y1, x2, y2] = forkin(a1, a2, l1, l2);
   hold on
   line ([-l0, 0], [-l0 -l0],'Linewidth',4,'Color','g');
   line ([0, 0], [-l0 0],'Linewidth',4,'Color','g');
   plot(0,0,'g.','MarkerSize',markersize);
   line([0 x1],[0 y1],'Linewidth',3,'Color','b');
   plot(x1,y1,'b.','MarkerSize',markersize);
   line([x1 x2],[y1 y2],'Linewidth',3,'Color','r');
   p = plot(x2,y2,'r.','MarkerSize',markersize);
   
   s = serial(serialport, 'BaudRate', 115200);
   fopen(s);
   
   %tic
   
   while (1)
       
     data = fscanf(s, '%s');
     if(~isempty(data))
       clf;
       d = strsplit(data,',');
       a = cell2mat(d);
       disp(str2double(d(2)));
       [x1, y1, x2, y2] = forkin(str2double(d(1)), str2double(d(2)), l1, l2);
       hold on;
       line ([-l0, 0], [-l0 -l0],'Linewidth',4,'Color','g');
       line ([0, 0], [-l0 0],'Linewidth',4,'Color','g');
       plot(0,0,'g.','MarkerSize',markersize);
       line([0 x1],[0 y1],'Linewidth',3,'Color','b');
       plot(x1,y1,'b.','MarkerSize',markersize);
       line([x1 x2],[y1 y2],'Linewidth',3,'Color','r');
       plot(x2,y2,'r.','MarkerSize',markersize);
       pause(0.01);
     end
   
   end
   fclose(s);
 end 