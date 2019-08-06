clc
clear all
close all

global d K N data

% x(1:100) = normrnd(4,2,[100 1]);
% y(1:100) = normrnd(2,4,[100 1]);
% 
% x(101:145) = normrnd(8,3,[45 1]);
% y(101:145) = normrnd(10,1,[45 1]);

mu1 = [10 7];
sig1 = [1 .75; .75 1];

R1 = mvnrnd(mu1,sig1,100);

mu2 = [15 10];
sig2 = [1 .75; .75 1];

R2 = mvnrnd(mu2,sig2,45);

data = [R1; R2]';

d = 2;
K = 2;
N = length(data);

[alphs,mus,Sigs,gam,llh] = GMMEM;

for ii = 1:N
    if ii<101
        myColors(ii, :) = [1,0,0];
    else
        myColors(ii, :) = [0,0,1];
    end
end

STD = 2;

conf = 2*normcdf(STD)-1;
scale = chi2inv(conf,2);

[V1 D1] = eig(Sigs(:,:,1)*scale);
[D1 order] = sort(diag(D1), 'descend');
D1 = diag(D1);
V1 = V1(:,order);
t = linspace(0,2*pi,100);
e1= [cos(t); sin(t)];
VV1 = V1*sqrt(D1);
e1 = VV1*e1+mus(:,:,1);
    

[V2 D2] = eig(Sigs(:,:,2)*scale);
[D2 order] = sort(diag(D2), 'descend');
D2 = diag(D2);
V2 = V2(:,order);
t = linspace(0,2*pi,100);
e2= [cos(t); sin(t)];
VV2 = V2*sqrt(D2);
e2 = VV2*e2+mus(:,:,2);

figure(1)
hold on
scatter(data(1,:),data(2,:), 5, myColors, 'Filled')
plot(e1(1,:), e1(2,:), 'LineWidth', 1.5, 'Color', [0.25, 0.25, 0.25]);
plot(e2(1,:), e2(2,:), 'LineWidth', 1.5, 'Color', [0.4940, 0.1840, 0.5560]);
xlabel('x')
ylabel('y')
title('Gaussian Mixture Model fitting using Expectation Max')
grid on