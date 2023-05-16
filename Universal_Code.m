%% Following this code developing in https://github.com/J-Rezvani/Universal-Robot-Code.git

clc
clear 
close all


%% Parameter Configuration 
n=input("\nHow Many links does your robot? ");

a_dist = sym('a_dist_',[n 1]);
alpha = sym('alpha_',[n 1]);
d_dist = sym('d_dist_',[n 1]);
theta = sym('theta_',[n 1]);

DH= sym('DH_',[1 4]);

Joint_type=sym('JT_',[n 1]);
Joint_type=str2mat(upper((input("\nWhat's kind of joint 'P' for Parismatic and 'R' for Revolot Joint with Double Quotation..." + ...
    "(for example ""RRR"" or ""RRP"" and etc.):  "))));


%% Deviant Hartenberg Parameter Calculation

for i=1:n
    disp(['Please Input Deviant Hartenberg Parameter for link No.',num2str(i)]);
    fprintf("\n");
    DH=input("\nPlease Enter DH Parameter as a 1x4 Matrix Like [a_dist alpha d_dist theta]..." + ...
        " and ""x"" for variable parameters like [a_dist ""x"" d_dist ""x""]:  ");
    fprintf("\n");
    for j=1:4
        if (j==1 && DH(j)~="x")
            a_dist(i)=DH(j);
        elseif (j==2 && DH(j)~="x")
            alpha(i)=DH(j);
        elseif (j==3 && DH(j)~="x")
            d_dist(i)=DH(j);
        elseif (j==4 && DH(j)~="x")
            theta(i)=DH(j);
        end
    end
end
DHM = [a_dist,alpha,d_dist,theta]


%% T Matrix Calculation for Forward Kinematics

A=sym('A',[4 4 n]);
I=eye(4,4);
T=I;
 for i=1:n
     A(:,:,i)=[cosd(theta(i))   -sind(theta(i))*cosd(alpha(i))    sind(theta(i))*sind(alpha(i))    a_dist(i)*cosd(theta(i));...
               sind(theta(i))    cosd(theta(i))*cosd(alpha(i))   -cosd(theta(i))*sind(alpha(i))    a_dist(i)*sind(theta(i));...
               0                 sind(alpha(i))                   cosd(alpha(i))                   d_dist(i);...
               0                 0                                0                                1];
     T=T*A(:,:,i);
 end
 T
 P=T(1:3,end)


 %% T Generator
TT=sym('TT',[4 4 n-1]);
 for i=1:n-1
     for j=i+1:n
          if j==(i+1)
            TT(:,:,i)=I*A(:,:,j);
          else
            TT(:,:,i)=TT(:,:,i)*A(:,:,j);
          end
     end
 end


%% Inverse Kinematics (Pieper Technique)
for i=1:n
    for j=1:n
        if j==1
            PT(:,:,i)=I*inv(A(:,:,j));
        else
            PT(:,:,i)=inv(A(:,:,j))*PT(:,:,i);
        end
    end
    ITT(:,:,i)=PT(:,:,i)*T;
end





