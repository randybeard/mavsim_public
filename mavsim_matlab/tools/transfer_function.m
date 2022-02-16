% transfer_function
%   implement transfer function
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         2/15/2019 - RWB
classdef transfer_function < handle
   %--------------------------------
    properties
        m
        n
        num
        den
        A
        B
        C
        D
        state
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = transfer_function(num, den, Ts)
            m = length(num);
            n = length(den);
            self.num = num;
            self.den = den;
            self.A = eye(n-1);
            for i=1:n-1
                self.A(1,i) = self.A(1,i) - Ts*den(i+1);
            end
            for i=2:n-1
                self.A(i,i-1) = self.A(i,i-1)+Ts;
            end
            self.B = zeros(n-1, 1);
            self.B(1) = Ts;
            self.C = zeros(1, n-1);
            if m==n
                self.D = num(1);
                for i=0:m-2
                    self.C(n-i-1) = num(m-i)-num(1)*den(n-i);
                end
            else
                self.D = 0;
                for i=0:m-2
                    self.C(n-i-1) = num(m-i);
                end
            end
            self.state = zeros(n-1,1);
        end
        %---------------------------
        function y=update(self, u)
            self.state = self.A * self.state + self.B * u;
            y = self.C * self.state + self.D * u;
        end
    end
end