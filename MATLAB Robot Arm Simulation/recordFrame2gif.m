function recordFrame2gif(h,filename,ii)
%Given the figure and the filename, this appends a frame of a figure 'h' to
%a gif file: filename.gif for every iteration ii of the figure 
% Get the figure object like so h = figure();
  drawnow 
  % Capture the plot as an image 
  frame = getframe(h); 
  im = frame2im(frame); 
  [imind,cm] = rgb2ind(im,256); 
  % Write to the GIF File for first iteration
  if ii == 1 
      imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
  else 
      imwrite(imind,cm,filename,'gif','WriteMode','append'); 
  end 