x=  rand(1,100).*10
noise = randn(1,100)
y = 2*x +5 + noise;
% y = 

figure

plot(x,y,'b*')
grid on

constant = lsqcurvefit(@f, [0;-0.1], x, y);

m = constant(1)
c = constant(2)

xfit = 0: 0.1 : 10;
yfit = f(constant, xfit);

figure
plot(x,y,'b*')
hold on
plot(xfit, yfit,'r','linewidth',2)
grid on