%% Magnetometer Calibration
clc; clear all;

[sec,nsec,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz,rawdat]=data('data_going_in_circles.bag');
quat=[qw,qx,qy,qz];
eul = quat2eul(quat);%quat=[w,x,y,z]; euler outputs in ZYX
yaw=eul(:,1); yaw=yaw(600:end,:);
pitch=eul(:,2);pitch=pitch(600:end,:);
roll=eul(:,3);roll=roll(600:end,:);

time = sec + 10^(-9)*nsec;
time = time - time(1);time=time(600:end,:);


mx=mx(600:end,:);
my=my(600:end,:);
mz=mz(600:end,:);

figure(1)
subplot(2,2,1)
scatter(mx,my);
title('Magnetic Field X vs Y')
xlabel('magnetic field X (Gauss)');ylabel('magnetic field Y (Gauss)')
grid on; axis equal;
hold on;
[x0,y0,a,b,phi]=plotellipse(mx,my)
hold off;

%Shift Center
subplot(2,2,2)
tmx=mx-x0;
tmy=my-y0;
scatter(tmx,tmy)
grid on; axis equal;
hold on;
[Tx0,Ty0,Ta,Tb,Tphi]=plotellipse(tmx,tmy)
hold off;
title('Translated Magnetic Field data')
xlabel('magnetic field X (Gauss)');ylabel('magnetic field Y (Gauss)')

%Rotation
subplot(2,2,3)
theta=Tphi
mrot=[tmx,tmy]*[cos(theta),sin(theta);-sin(theta),cos(theta)];
rmx=mrot(:,1); rmy=mrot(:,2);
scatter(rmx,rmy)
grid on; axis equal;
hold on;
[Rx0,Ry0,Ra,Rb,Rphi]=plotellipse(rmx,rmy)
hold off;
title('Rotated and Translated Magnetic Field data')
xlabel('magnetic field X (Gauss)');ylabel('magnetic field Y (Gauss)')

%Scaling
subplot(2,2,4)
sigma=Rb/Ra
mscale=[sigma,0;0,1]*mrot';
smx=mscale(1,:);smy=mscale(2,:);
scatter(smx,smy)
grid on; axis equal;
hold on;
[Sx0,Sy0,Sa,Sb,Sphi]=plotellipse(smx,smy)
scatter(Sx0,Sy0)
hold off;
title('Calibrated Magnetic Field data')
xlabel('magnetic field X (Gauss)');ylabel('magnetic field Y (Gauss)')
%% Testing CalibrateMf function
[y,u]=calibrateMf(mx,my,x0,y0,theta,Rb,Ra)
figure
scatter(mx,my,'.')
hold on;
scatter(y,u,'.');scatter(x0,y0);scatter(Sx0,Sy0);
hold off; grid on; axis equal;
title('Magnetometer Calibration');xlabel('magnetic field X (Gauss)');ylabel('magnetic field Y (Gauss)')
%% DRIVING DATA INITITALIZATIONS

clc;
[sGps,nsGps,xP, yP, zP]=latLon(['data_driving.bag']);
[sec,nsec,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz,rawdat]=data('data_driving.bag');
time = sec + 10^(-9)*nsec;
time = time - time(1);
gpsTime=sGps + 10^(-9)*nsGps;
gpsTime = gpsTime - gpsTime(1);
quat=[qw,qx,qy,qz];
eul = quat2eul(quat);%quat=[w,x,y,z]; euler outputs in ZYX
yaw=eul(:,1);
pitch=eul(:,2); roll=eul(:,3);
%% ======================= Calculating Yaw ================================
figure(1)
subplot(2,1,1)
magyaw = atan2(-my,mx);
plot(time,unwrap(magyaw));hold on;
ylabel('Yaw (radians)');xlabel('time (seconds)')
[calmx,calmy]=calibrateMf(mx,my,x0,y0,theta,Rb,Ra);
calmagyaw =  atan2(-calmy,calmx);
plot(time,unwrap(calmagyaw));
legend('Uncalibrated Magnetometer Yaw','Calibrated Magetometer Yaw')
ylabel('Yaw (radians)');xlabel('time (seconds)')
grid on;title("Yaw Calculated from Magnetometer Readings")

subplot(2,1,2)
plot(time,unwrap(calmagyaw));ylabel('Yaw (radians)');xlabel('time (seconds)')
grid on; title("Yaw")
hold on;

cumgyroyaw=cumtrapz(time,(gz));
%temp=[0;cumgyroyaw];
%gyroyaw=[cumgyroyaw-temp(1:end-1,:)];
plot(time,cumgyroyaw);ylabel('Yaw (radians)');xlabel('time (seconds)')
legend('Calibrated Magnetometer Yaw','Yaw Integrated from Gyro')

%LPF on Corrected Mag Yaw
lpf_mag_yaw=lowpass(unwrap(calmagyaw),0.00001,40);
figure(2)
subplot(2,1,1)
plot(time,unwrap(lpf_mag_yaw));grid on;
hold on;ylabel('Yaw (radians)');xlabel('time (seconds)')

%HPF on Gyro Yaw
hpf_gyro_yaw=highpass(unwrap(cumgyroyaw),0.07,40);

compyaw=lpf_mag_yaw' + hpf_gyro_yaw;
plot(time,hpf_gyro_yaw);ylabel('Yaw (radians)');xlabel('time (seconds)')
plot(time,compyaw,'black');ylabel('Yaw (radians)');xlabel('time (seconds)')
hold off; legend("LPF Mag Yaw",'HPF Gyro Yaw','Complementary Filter Yaw');
title('Complementary Filtering')

subplot(2,1,2)
plot(time,compyaw); grid on; hold on; ylabel('Yaw (radians)');xlabel('time (seconds)')
plot(time,unwrap(yaw));ylabel('Yaw (radians)');xlabel('time (seconds)')
legend('Yaw from Complementary Filter','IMU yaw'); title('Complementaary filter Yaw vs. IMU Yaw')
%Remove offset in comp filter yaw
%% VELOCITY ESTIMATE

velAx=cumtrapz(time,ax);
figure

%LPF
lpf=lowpass(ax,0.1);

%Velocity Estimate from GPS
UTM=[xP,yP];
velGPS=[];
UTM1=UTM(2:end,:);
gpsTime1=gpsTime(2:end,:);
xy2_1=UTM1-UTM(1:end-1,:);
velGPS=sqrt((xy2_1(:,1).^2)+(xy2_1(:,2).^2))./(gpsTime1-gpsTime(1:end-1,:));
velGPS=[0;velGPS]; velGPS=interp(velGPS,40);

plot(time,velAx); xlabel('Time (seconds)');ylabel('Velocity (m/s)')
grid on; hold on; plot(interp(gpsTime,40),velGPS);
xlabel('Time (seconds)');ylabel('Velocity (m/s)');
legend('Velocity integrated from accelerometer data','Velocity estimate from GPS');
title('Forward Velocity Comparision');

figure
subplot(2,1,1)
plot(time,ax)
grid on;
subplot(2,1,2)
plot(time,lpf);grid on;

%Stationary Points
sp=[1,1659,3794,4955,8220,9051,17282,17282];



ax1=[];
index=length(sp)
for i=1:index-2
    offset = ax(sp(1,i):sp(1,i+1));
    ax1(sp(1,i):sp(1,i+2)) = ax(sp(1,i):sp(1,i+2))-mean(offset);
end
figure;plot(time,ax1);grid on; title("Corrected ax");
fixVelx=cumtrapz(ax1*(1/40))';
for j=1:length(fixVelx)
    if fixVelx(j,:)<0
        fixVelx(j,:)=0;
    end
end
figure;plot(time,fixVelx);grid on; xlabel('Time (seconds)');ylabel('Velocity (m/s)');
title("Corrected Velocity Comparision");
hold on;
plot(interp(gpsTime,40),velGPS);
grid on; legend('Corrected Accelerometer velocity','Velocity Estimate from GPS'); hold off;

%% DEAD RECKONING WITH IMU
disAx=cumtrapz(fixVelx);
figure; plot(disAx); grid on; title("Displacement"); hold on;
%UTM1=UTM(2:end,:);
%xy2_1=UTM1-UTM(1:end-1,:);
%disGPS=[];
%disGPS=sqrt((xy2_1(:,1).^2)+(xy2_1(:,2).^2));disGPS=interp([0;disGPS],40);
disGPS=cumtrapz(velGPS);
plot(disGPS); legend('IMU Displacement','GPS Displacement'); hold off;

d2x_obs=ax1;
dx=fixVelx;
w=gz;
d2y_obs=w.*dx;
figure; plot(time,lowpass(d2y_obs,0.1),LineWidth=1.5);grid on;title("Dead Reckoning")
int_d2x_obs=cumtrapz(time,lowpass(d2x_obs,0.1));
int_d2y_obs=w.*int_d2x_obs';
hold on; plot(time,int_d2y_obs); legend('d^2y𝑜𝑏𝑠','𝜔d𝑋');hold off;
xlabel('Time (seconds)');ylabel('m/s^2')

ve=fixVelx.*sin(compyaw); vn=fixVelx.*cos(compyaw);
xe=cumtrapz(ve./40);xn=cumtrapz(vn/40);
figure; plot(xe,xn);grid on; title("Trajectory"); xlabel('Easting (meters)'); ylabel('Northing (Meters)');hold on;
plot(xP-xP(1),yP-yP(1));legend('IMU estimated track','GPS track');hold off;
%% Functions


function [se,nse,xP, yP, zP] = latLon(name)
    stBag=rosbag(name);
    stSel=select(stBag,'Topic',"/gps");
    stMsg=readMessages(stSel,"DataFormat","struct");
    %stMsg{2}
    %stTs=timeseries(stSel,'Latitude')
    se=cellfun(@(m) double(m.Header.Stamp.Sec),stMsg);
    nse=cellfun(@(m) double(m.Header.Stamp.Nsec),stMsg);
    xP = cellfun(@(m) double(m.UTMEasting),stMsg);
    yP = cellfun(@(m) double(m.UTMNorthing),stMsg);
    zP = cellfun(@(m) double(m.Altitude),stMsg);
end

function [s,ns,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz,raw] = data(name)
    stBag=rosbag(name)
    stSel=select(stBag,'Topic',"/imu")
    stMsg=readMessages(stSel,"DataFormat","struct");
    stMsg{2}
    %stTs=timeseries(stSel,'Latitude')
    s=cellfun(@(m) double(m.Header.Stamp.Sec),stMsg);
    ns=cellfun(@(m) double(m.Header.Stamp.Nsec),stMsg);
    qw = cellfun(@(m) double(m.IMU.Orientation.W),stMsg);
    qx = cellfun(@(m) double(m.IMU.Orientation.X),stMsg);
    qy = cellfun(@(m) double(m.IMU.Orientation.Y),stMsg);
    qz = cellfun(@(m) double(m.IMU.Orientation.Z),stMsg);
    gx = cellfun(@(m) double(m.IMU.AngularVelocity.X),stMsg);
    gy = cellfun(@(m) double(m.IMU.AngularVelocity.Y),stMsg);
    gz = cellfun(@(m) double(m.IMU.AngularVelocity.Z),stMsg);
    ax = cellfun(@(m) double(m.IMU.LinearAcceleration.X),stMsg);
    ay = cellfun(@(m) double(m.IMU.LinearAcceleration.Y),stMsg);
    az = cellfun(@(m) double(m.IMU.LinearAcceleration.Z),stMsg);
    mx = cellfun(@(m) double(m.MagField.MagneticField_.X),stMsg);
    my = cellfun(@(m) double(m.MagField.MagneticField_.Y),stMsg);
    mz = cellfun(@(m) double(m.MagField.MagneticField_.Z),stMsg);
    raw = cellfun(@(m) (m.VNYMR),stMsg,UniformOutput=false);
    raw=cell2mat(raw);
    rawdim=size(raw);
    rawstr=strsplit(raw(1,:),{',','*'});
    gyro=str2num(char(rawstr(1,11:13)))
    for i=2:rawdim(1)
        appendraw=strsplit(raw(i,:),{',','*'});
    %rawstr=[rawstr;appendraw];
        gyro=[gyro,str2num(char(appendraw(11:13)))];
    end
    gx=gyro(1,:)';
    gy=gyro(2,:)';
    gz=gyro(3,:)';
    
end

function [x0,y0,a,b,theta]=plotellipse(mx,my)
    ellipsefit=fit_ellipse(mx,my)
    x0=ellipsefit.X0_in; y0=ellipsefit.Y0_in;
    a=ellipsefit.long_axis/2;
    b=ellipsefit.short_axis/2;
    theta=ellipsefit.phi;
    t=-pi:0.01:pi;
    x = x0 + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
    y = y0 + b*sin(t)*cos(theta) + a*cos(t)*sin(theta);
    plot(x,y,LineWidth=1.5);
end

function [cmx,cmy]=calibrateMf(magx,magy,x0,y0,theta,Rb,Ra)
    tmagx=magx-x0;
    tmagy=magy-y0;
    angle=theta;
    Mrot=[tmagx,tmagy]*[cos(angle),sin(angle);-sin(angle),cos(angle)];
    rmagx=Mrot(:,1); rmagy=Mrot(:,2);
    sig=Rb/Ra;
    mscale=[sig,0;0,1]*Mrot';
    cmx=mscale(1,:);cmy=mscale(2,:);
end
%% Best Fit Ellipse function  
% Reference: Ohad Gal (2022). fit_ellipse 
% (https://www.mathworks.com/matlabcentral/fileexchange/3215-fit_ellipse), 
% MATLAB Central File Exchange. Retrieved November 5, 2022. 


function ellipse_t = fit_ellipse( x,y,axis_handle )
%
% fit_ellipse - finds the best fit to an ellipse for the given set of points.
%
% Format:   ellipse_t = fit_ellipse( x,y,axis_handle )
%
% Input:    x,y         - a set of points in 2 column vectors. AT LEAST 5 points are needed !
%           axis_handle - optional. a handle to an axis, at which the estimated ellipse 
%                         will be drawn along with it's axes
%
% Output:   ellipse_t - structure that defines the best fit to an ellipse
%                       a           - sub axis (radius) of the X axis of the non-tilt ellipse
%                       b           - sub axis (radius) of the Y axis of the non-tilt ellipse
%                       phi         - orientation in radians of the ellipse (tilt)
%                       X0          - center at the X axis of the non-tilt ellipse
%                       Y0          - center at the Y axis of the non-tilt ellipse
%                       X0_in       - center at the X axis of the tilted ellipse
%                       Y0_in       - center at the Y axis of the tilted ellipse
%                       long_axis   - size of the long axis of the ellipse
%                       short_axis  - size of the short axis of the ellipse
%                       status      - status of detection of an ellipse
%
% Note:     if an ellipse was not detected (but a parabola or hyperbola), then
%           an empty structure is returned
% =====================================================================================
%                  Ellipse Fit using Least Squares criterion
% =====================================================================================
% We will try to fit the best ellipse to the given measurements. the mathematical
% representation of use will be the CONIC Equation of the Ellipse which is:
% 
%    Ellipse = a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
%   
% The fit-estimation method of use is the Least Squares method (without any weights)
% The estimator is extracted from the following equations:
%
%    g(x,y;A) := a*x^2 + b*x*y + c*y^2 + d*x + e*y = f
%
%    where:
%       A   - is the vector of parameters to be estimated (a,b,c,d,e)
%       x,y - is a single measurement
%
% We will define the cost function to be:
%
%   Cost(A) := (g_c(x_c,y_c;A)-f_c)'*(g_c(x_c,y_c;A)-f_c)
%            = (X*A+f_c)'*(X*A+f_c) 
%            = A'*X'*X*A + 2*f_c'*X*A + N*f^2
%
%   where:
%       g_c(x_c,y_c;A) - vector function of ALL the measurements
%                        each element of g_c() is g(x,y;A)
%       X              - a matrix of the form: [x_c.^2, x_c.*y_c, y_c.^2, x_c, y_c ]
%       f_c            - is actually defined as ones(length(f),1)*f
%
% Derivation of the Cost function with respect to the vector of parameters "A" yields:
%
%   A'*X'*X = -f_c'*X = -f*ones(1,length(f_c))*X = -f*sum(X)
%
% Which yields the estimator:
%
%       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%       |  A_least_squares = -f*sum(X)/(X'*X) ->(normalize by -f) = sum(X)/(X'*X)  |
%       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%
% (We will normalize the variables by (-f) since "f" is unknown and can be accounted for later on)
%  
% NOW, all that is left to do is to extract the parameters from the Conic Equation.
% We will deal the vector A into the variables: (A,B,C,D,E) and assume F = -1;
%
%    Recall the conic representation of an ellipse:
% 
%       A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
% 
% We will check if the ellipse has a tilt (=orientation). The orientation is present
% if the coefficient of the term "x*y" is not zero. If so, we first need to remove the
% tilt of the ellipse.
%
% If the parameter "B" is not equal to zero, then we have an orientation (tilt) to the ellipse.
% we will remove the tilt of the ellipse so as to remain with a conic representation of an 
% ellipse without a tilt, for which the math is more simple:
%
% Non tilt conic rep.:  A`*x^2 + C`*y^2 + D`*x + E`*y + F` = 0
%
% We will remove the orientation using the following substitution:
%   
%   Replace x with cx+sy and y with -sx+cy such that the conic representation is:
%   
%   A(cx+sy)^2 + B(cx+sy)(-sx+cy) + C(-sx+cy)^2 + D(cx+sy) + E(-sx+cy) + F = 0
%
%   where:      c = cos(phi)    ,   s = sin(phi)
%
%   and simplify...
%
%       x^2(A*c^2 - Bcs + Cs^2) + xy(2A*cs +(c^2-s^2)B -2Ccs) + ...
%           y^2(As^2 + Bcs + Cc^2) + x(Dc-Es) + y(Ds+Ec) + F = 0
%
%   The orientation is easily found by the condition of (B_new=0) which results in:
% 
%   2A*cs +(c^2-s^2)B -2Ccs = 0  ==> phi = 1/2 * atan( b/(c-a) )
%   
%   Now the constants   c=cos(phi)  and  s=sin(phi)  can be found, and from them
%   all the other constants A`,C`,D`,E` can be found.
%
%   A` = A*c^2 - B*c*s + C*s^2                  D` = D*c-E*s
%   B` = 2*A*c*s +(c^2-s^2)*B -2*C*c*s = 0      E` = D*s+E*c 
%   C` = A*s^2 + B*c*s + C*c^2
%
% Next, we want the representation of the non-tilted ellipse to be as:
%
%       Ellipse = ( (X-X0)/a )^2 + ( (Y-Y0)/b )^2 = 1
%
%       where:  (X0,Y0) is the center of the ellipse
%               a,b     are the ellipse "radiuses" (or sub-axis)
%
% Using a square completion method we will define:
%       
%       F`` = -F` + (D`^2)/(4*A`) + (E`^2)/(4*C`)
%
%       Such that:    a`*(X-X0)^2 = A`(X^2 + X*D`/A` + (D`/(2*A`))^2 )
%                     c`*(Y-Y0)^2 = C`(Y^2 + Y*E`/C` + (E`/(2*C`))^2 )
%
%       which yields the transformations:
%       
%           X0  =   -D`/(2*A`)
%           Y0  =   -E`/(2*C`)
%           a   =   sqrt( abs( F``/A` ) )
%           b   =   sqrt( abs( F``/C` ) )
%
% And finally we can define the remaining parameters:
%
%   long_axis   = 2 * max( a,b )
%   short_axis  = 2 * min( a,b )
%   Orientation = phi
%
%
% initialize
orientation_tolerance = 1e-3;
% empty warning stack
warning( '' );
% prepare vectors, must be column vectors
x = x(:);
y = y(:);
% remove bias of the ellipse - to make matrix inversion more accurate. (will be added later on).
mean_x = mean(x);
mean_y = mean(y);
x = x-mean_x;
y = y-mean_y;
% the estimation for the conic equation of the ellipse
X = [x.^2, x.*y, y.^2, x, y ];
a = sum(X)/(X'*X);
% check for warnings
if ~isempty( lastwarn )
    disp( 'stopped because of a warning regarding matrix inversion' );
    ellipse_t = [];
    return
end
% extract parameters from the conic equation
[a,b,c,d,e] = deal( a(1),a(2),a(3),a(4),a(5) );
% remove the orientation from the ellipse
if ( min(abs(b/a),abs(b/c)) > orientation_tolerance )
    
    orientation_rad = 1/2 * atan( b/(c-a) );
    cos_phi = cos( orientation_rad );
    sin_phi = sin( orientation_rad );
    [a,b,c,d,e] = deal(...
        a*cos_phi^2 - b*cos_phi*sin_phi + c*sin_phi^2,...
        0,...
        a*sin_phi^2 + b*cos_phi*sin_phi + c*cos_phi^2,...
        d*cos_phi - e*sin_phi,...
        d*sin_phi + e*cos_phi );
    [mean_x,mean_y] = deal( ...
        cos_phi*mean_x - sin_phi*mean_y,...
        sin_phi*mean_x + cos_phi*mean_y );
else
    orientation_rad = 0;
    cos_phi = cos( orientation_rad );
    sin_phi = sin( orientation_rad );
end
% check if conic equation represents an ellipse
test = a*c;
switch (1)
case (test>0),  status = '';
case (test==0), status = 'Parabola found';  warning( 'fit_ellipse: Did not locate an ellipse' );
case (test<0),  status = 'Hyperbola found'; warning( 'fit_ellipse: Did not locate an ellipse' );
end
% if we found an ellipse return it's data
if (test>0)
    
    % make sure coefficients are positive as required
    if (a<0), [a,c,d,e] = deal( -a,-c,-d,-e ); end
    
    % final ellipse parameters
    X0          = mean_x - d/2/a;
    Y0          = mean_y - e/2/c;
    F           = 1 + (d^2)/(4*a) + (e^2)/(4*c);
    [a,b]       = deal( sqrt( F/a ),sqrt( F/c ) );    
    long_axis   = 2*max(a,b);
    short_axis  = 2*min(a,b);
    % rotate the axes backwards to find the center point of the original TILTED ellipse
    R           = [ cos_phi sin_phi; -sin_phi cos_phi ];
    P_in        = R * [X0;Y0];
    X0_in       = P_in(1);
    Y0_in       = P_in(2);
    
    % pack ellipse into a structure
    ellipse_t = struct( ...
        'a',a,...
        'b',b,...
        'phi',orientation_rad,...
        'X0',X0,...
        'Y0',Y0,...
        'X0_in',X0_in,...
        'Y0_in',Y0_in,...
        'long_axis',long_axis,...
        'short_axis',short_axis,...
        'status','' );
else
    % report an empty structure
    ellipse_t = struct( ...
        'a',[],...
        'b',[],...
        'phi',[],...
        'X0',[],...
        'Y0',[],...
        'X0_in',[],...
        'Y0_in',[],...
        'long_axis',[],...
        'short_axis',[],...
        'status',status );
end
% check if we need to plot an ellipse with it's axes.
if (nargin>2) & ~isempty( axis_handle ) & (test>0)
    
    % rotation matrix to rotate the axes with respect to an angle phi
    R = [ cos_phi sin_phi; -sin_phi cos_phi ];
    
    % the axes
    ver_line        = [ [X0 X0]; Y0+b*[-1 1] ];
    horz_line       = [ X0+a*[-1 1]; [Y0 Y0] ];
    new_ver_line    = R*ver_line;
    new_horz_line   = R*horz_line;
    
    % the ellipse
    theta_r         = linspace(0,2*pi);
    ellipse_x_r     = X0 + a*cos( theta_r );
    ellipse_y_r     = Y0 + b*sin( theta_r );
    rotated_ellipse = R * [ellipse_x_r;ellipse_y_r];
    
    % draw
    hold_state = get( axis_handle,'NextPlot' );
    set( axis_handle,'NextPlot','add' );
    plot( new_ver_line(1,:),new_ver_line(2,:),'r' );
    plot( new_horz_line(1,:),new_horz_line(2,:),'r' );
    plot( rotated_ellipse(1,:),rotated_ellipse(2,:),'r' );
    set( axis_handle,'NextPlot',hold_state );
end
end


