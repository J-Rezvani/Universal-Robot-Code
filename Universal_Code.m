clc
clear 
close all

n=input("How Many links does your robot? ");
a_dist = sym('a_dist_',[n 1]);
alpha = sym('alpha_',[n 1]);
d_dist = sym('d_dist_',[n 1]);
theta = sym('theta_',[n 1]);

% Joint_type=sym('JT_',[n 1]);
Joint_type=str2mat(upper((input("\n What's kind of joint 'P' for Parismatic and 'R' for Revolot Joint with Double Quotation..." + ...
    "(for example ""RRR"" or ""RRP"" and etc.):  "))));
for i=1:n
    disp(['Please Input Deviant Hartenberg Parameter for link No.',num2str(i)]);
%     Joint_type(i)=input("\n What's kind of joint 'P' for Parismatic and 'R' for Revolot Joint with Double Quotation:  ");
    for j=1:4
        if j==1
            a_dist(i)=input("Please Enter a_dist for link =  ");
        elseif j==2
            alpha(i)=input("Please Enter alpha for link =  ");
        elseif (j==3 & Joint_type(i)~="P")
            d_dist(i)=input("Please Enter d_dist for link =  ");
        elseif (j==4 & Joint_type(i)~="R") 
            theta(i)=input("Please Enter theta for link =  ");
        end
    end
end
DHM = [a_dist,alpha,d_dist,theta]

A=sym('A',[4 4 n]);
T=eye(4,4);
 for i=1:n
     A(:,:,i)=[cosd(theta(i))   -sind(theta(i))*cosd(alpha(i))    sind(theta(i))*sind(alpha(i))    a_dist(i)*cosd(theta(i));...
               sind(theta(i))    cosd(theta(i))*cosd(alpha(i))   -cosd(theta(i))*sind(alpha(i))    a_dist(i)*sind(theta(i));...
               0                 sind(alpha(i))                   cosd(alpha(i))                   d_dist(i);...
               0                 0                                0                                0];
     T=T*A(:,:,i);
 end
 T
 P=T(:,end)
