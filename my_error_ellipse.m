function  my_error_ellipse(mu,P)
    
    conf1 = 0.70;
    %conf2 = 0.95;
    scale = 1;
    [r,c] = size(P);
    if r~=c
        disp("matrix must be squared!");
        return;
    else
        x0=mu(1);
        y0=mu(2);
        % Compute quantile for the desired percentile
        k1 = sqrt(qchisq(conf1,r)); % r is the number of dimensions (degrees of freedom)
        %k2 = sqrt(qchisq(conf2,r)); % r is the number of dimensions (degrees of freedom)
        %hold_state = get(gca,'nextplot');

        [x,y] = getpoints(P);
        figure(4)
        plot(scale*(x0+k1*x),scale*(y0+k1*y), 'b');
        %hold on
        %plot(scale*(x0+k2*x),scale*(y0+k2*y), 'g');
        %hold off
        %axis([-6 8 -10 15])

        %set(gca,'nextplot',hold_state);
        drawnow
    end
end
% getpoints - Generate x and y points that define an ellipse, given a 2x2
%   covariance matrix, C. z, if requested, is all zeros with same shape as
%   x and y.
function [x,y] = getpoints(C)
    n=100; % Number of points around ellipse
    p=0:pi/n:2*pi; % angles around a circle
    [eigvec,eigval] = eig(C); % Compute eigen-stuff
    xy = [cos(p'),sin(p')] * sqrt(eigval) * eigvec'; % Transformation
    x = xy(:,1);
    y = xy(:,2);
    
end
%---------------------------------------------------------------
function x=qchisq(P,n)
% QCHISQ(P,N) - quantile of the chi-square distribution.
    if nargin<2
      n=1;
    end
    s0 = P==0;
    s1 = P==1;
    s = P>0 & P<1;
    x = 0.5*ones(size(P));
    x(s0) = -inf;
    x(s1) = inf;
    x(~(s0|s1|s))=nan;
    for ii=1:14
      dx = -(pchisq(x(s),n)-P(s))./dchisq(x(s),n);
      x(s) = x(s)+dx;
      if all(abs(dx) < 1e-6)
        break;
      end
    end
end
%---------------------------------------------------------------
function F=pchisq(x,n)
% PCHISQ(X,N) - Probability function of the chi-square distribution.
    if nargin<2
      n=1;
    end
    F=zeros(size(x));
    if rem(n,2) == 0
      s = x>0;
      k = 0;
      for jj = 0:n/2-1
        k = k + (x(s)/2).^jj/factorial(jj);
      end
      F(s) = 1-exp(-x(s)/2).*k;
    else
      for ii=1:numel(x)
        if x(ii) > 0
          F(ii) = quadl(@dchisq,0,x(ii),1e-6,0,n);
        else
          F(ii) = 0;
        end
      end
    end
end
%---------------------------------------------------------------
function f=dchisq(x,n)
% DCHISQ(X,N) - Density function of the chi-square distribution.
    if nargin<2
      n=1;
    end
    f=zeros(size(x));
    s = x>=0;
    f(s) = x(s).^(n/2-1).*exp(-x(s)/2)./(2^(n/2)*gamma(n/2));
end