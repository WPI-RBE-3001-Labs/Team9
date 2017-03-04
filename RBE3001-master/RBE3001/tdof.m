function tdof
   %pkg load instrument-control
    clear
    clc

    serialport = 'COM57';
    baudrade = '115200';
    ptitle = 'Serial Data';
    delay = 0.0001;

    theta1 = (-pi/6):0.1:(pi+pi/6);
    theta2 = (-pi/6):0.1:(pi+pi/6);

    [theta1, theta2] = meshgrid(theta1, theta2);
    markersize = 16;
    l0 = 1.4;
    l1 = 1.5;
    l2 = 1.2;
    a1 = pi/4;
    a2 = -pi/6;
    OFF = 1;
    OFF2 = 1.5;
    function [a, b] = invkin(x, y, l1, l2)
        v = (((x*x) + (y*y) - (l2*l2) - (l1*l1))/(2*l1*l2));
        try
        b = atan2(sqrt(1 - (2*v)),v);
        k = [l1+l2*cos(b), l2*sin(b)];
        catch
            a = NaN(1,1);
            b = NaN(1,1);
            return;
        end
        try
        a = atan2(y,x) - atan2(k(1,2), k(1,1));

        catch
            a = NaN(1,1);
            b = NaN(1,1);
            return
        end
    end

   function [x1, y1, x2, y2] = forkin(a1, a2, l1, l2)
     x1 = l1*cos(a1);
     y1 = l1*sin(a1);
     x2 = l1*cos(a1) + l2*cos(a1+a2);
     y2 = l1*sin(a1) + l2*sin(a1+a2);
   end
   [x1, y1, x2, y2] = forkin(theta1, theta2, l1, l2);
   
   plot(x2, y2,'.m');
   axis([-l0-OFF2 (l1+l2)+OFF -l0-OFF (l1+l2)+OFF])
   grid('on')
   title('Position Plot')
   xlabel('X-pos')
   ylabel('Y-pos')
   hold on;
   [x1, y1, x2, y2] = forkin(a1, a2, l1, l2);
   line ([-l0, 0], [-l0 -l0],'Linewidth',4,'Color','g');
   line ([0, 0], [-l0 0],'Linewidth',4,'Color','g');
   plot(0,0,'g.','MarkerSize',markersize);
   line([0 x1],[0 y1],'Linewidth',3,'Color','b');
   plot(x1,y1,'b.','MarkerSize',markersize);
   line([x1 x2],[y1 y2],'Linewidth',3,'Color','r');
   p = plot(x2,y2,'r.','MarkerSize',markersize);
   
   s = serial(serialport, 'BaudRate', 115200);
   
   try fopen(s);
   catch
   end
   

      
   while(ishandle(p))
       data = ginput(1);
       %disp(size(data));
       x = size(data);
       if(x(2) == 2 && x(1) == 1)
           %disp(data);
           
           hold on;
           axis([-l0-OFF2 (l1+l2)+OFF -l0-OFF (l1+l2)+OFF])
           grid('on')
           xlabel('X-pos')
           ylabel('Y-pos')
           [a1, a2] = invkin((data(1,1)),(data(1,2)), l1, l2);
           [x1, y1, x2, y2] = forkin(a1, a2, l1, l2);
           plot(x2,y2,'r.','MarkerSize',markersize);
           plot(data(1,1),data(1,2),'b.','MarkerSize',20);


           line ([-l0, 0], [-l0 -l0],'Linewidth',4,'Color','g');
           line ([0, 0], [-l0 0],'Linewidth',4,'Color','g');
           plot(0,0,'g.','MarkerSize',markersize);
           line([0 x1],[0 y1],'Linewidth',3,'Color','b');
           plot(x1,y1,'b.','MarkerSize',markersize);
           line([x1 x2],[y1 y2],'Linewidth',3,'Color','r');
           p = plot(x2,y2,'r.','MarkerSize',markersize);
           hold off;
           try
               fwrite(s,data(1,1),'double');
               fwrite(s,data(1,2),'double');
           catch
           end
           

           
           pause(0.01);
       end
   end
   %p = plot(x2,y2,'r.','MarkerSize',markersize);
   hold off;
   s = serial(serialport, 'BaudRate', 115200);
   try
   fopen(s);
   figure;
   
   %tic
   b  = -pi/2;
   while (ishandle(p))
       
     data = fscanf(s, '%s');
     c = 0;
     if(~isempty(data))
       clf;
       d = strsplit(data,',');
       if(c > 100)
          disp(str2double(d));
          c = 0; 
       end
       c = c+1;
       [x1, y1, x2, y2] = forkin(str2double(d(1))-b, str2double(d(2)), l1, l2);
       hold on;
       axis([-l0 (l1+l2) -l0 (l1+l2)])
       grid('on')
       xlabel('X-pos')
       ylabel('Y-pos')
       line ([-l0, 0], [-l0 -l0],'Linewidth',4,'Color','g');
       line ([0, 0], [-l0 0],'Linewidth',4,'Color','g');
       plot(0,0,'g.','MarkerSize',markersize);
       line([0 x1],[0 y1],'Linewidth',3,'Color','b');
       plot(x1,y1,'b.','MarkerSize',markersize);
       line([x1 x2],[y1 y2],'Linewidth',3,'Color','r');
       p = plot(x2,y2,'r.','MarkerSize',markersize);
       hold off;
       pause(0.01);
     end
   
   end
   fclose(s);
  catch
   end
end 
