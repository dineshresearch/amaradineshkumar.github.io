
# Introduction

## Controllability

## State controllability
The state of a deterministic system, which is the set of values of all the system's state variables (those variables characterized by dynamic equations), completely describes the system at any given time. In particular, no information on the past of a system is needed to help in predicting the future, if the states at the present time are known and all current and future values of the control variables (those whose values can be chosen) are known.


## Complete state controllability (or simply controllability if no other context is given) 
describes the ability of an external input (the vector of control variables) to move the internal state of a system from any initial state to any other final state in a finite time interval.

By giving the external control input we can move the internal state of the system from any initial state to final state within in a finite time period.By measuring the full state feedback we can get back the full state space of the control system (We can keep the eignen values any where we want) then we can tell that the system is controllable.


## Output controllability

Controllability is the related notion for the output of the system (denoted y in the previous equations); the output controllability describes the ability of an external input to move the output from any initial condition to any final condition in a finite time interval. It is not necessary that there is any relationship between state controllability and output controllability. 

In our case: By changing the value of K (gain) we should be able to change the eigen values of the closed loop system to our desired eigen values then we can tell that the system is controllable. <br/> <br/>


Ref https://www.youtube.com/embed/M_jchYsTZvM?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m

# Pole placement method
Full state feedback (FSF), or pole placement, is a method employed in feedback control system theory to place the closed-loop poles of a plant in pre-determined locations in the s-plane. Placing poles is desirable because the location of the poles corresponds directly to the eigenvalues of the system, which control the characteristics of the response of the system. The system must be considered controllable in order to implement this method.

\begin{equation}
\dot{X}=Ax+Bu
\end{equation}
\begin{equation}
u=-Kx
\end{equation}
\begin{equation}
\dot{X}=(A-BK)x
\end{equation}

\begin{equation}
y=\left(\begin{array}{c}x\\ \dot{x} \\ \theta \\ \dot{\theta} \end{array}\right) 
\end{equation}

Finally it should lead to this for inverted pendulum to get to stable state

\begin{equation}
y=\left(\begin{array}{c}1\\ 0 \\ \pi \\ 0 \end{array}\right) 
\end{equation}





```python
Code:

clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % pendulum up (s=1)

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];
eig(A)

rank(ctrb(A,B))  % is it controllable

%%  Pole placement

% p is a vector of desired eigenvalues
% p = [-.01; -.02; -.03; -.04]; % not enough
% p = [-.3; -.4; -.5; -.6];  % just barely
% p = [-1; -1.1; -1.2; -1.3]; % good
% p = [-2; -2.1; -2.2; -2.3]; % aggressive
%p = [-3; -3.1; -3.2; -3.3]; % aggressive
% p = [-3.5; -3.6; -3.7; -3.8]; % br
K = place(A,B,p);
% K = lqr(A,B,Q,R);

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
%     [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[1; 0; pi; 0])),tspan,y0);
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[1; 0; pi; 0])),tspan,y0);
else 
end

for k=1:100:length(t)
    drawcartpend(y(k,:),m,M,L);
end

```

# Linear Quadratic Regulator ( LQR )

**Linear** means the regulator is linear 

**Quadratic** means the cost function is quadratic in nature

**Regulator** means it is the controller which stabilizes the control system by regulating (controlling the closed loop feedback system) by using the U=-Kx. Our aim is to find the optimal K value for which the system becomes stable.

|![Figure 1-1](https://www.researchgate.net/publication/262484498/figure/fig6/AS:324805573267474@1454451091285/The-structure-of-LQR-control-system.png)|
|:--:| 
| *Control System with Closed Loop Feedback* |

Optimal Regulation
The LQR problem is defined as follows: Find the control input u(t) $t ∈ [0,∞)$ that makes
the following criterion as small as possible
\begin{equation}
J_{LQR} := \int_{0}^{\infty} ||z(t)||^{2}+\rho ||u(t)||^{2} dt
\end{equation}
where ρ is a positive constant. The term
\begin{equation}
\int_{0}^{\infty} ||z(t)||^{2}
\end{equation}
corresponds to the energy of the controlled output and the term
\begin{equation}
\int_{0}^{\infty} ||u(t)||^{2}
\end{equation}
corresponds to the energy of the control signal. In LQR one seeks a controller that minimizes both
energies. However, decreasing the energy of the controlled output will require a large control
signal and a small control signal will lead to large controlled outputs. The role of the constant
ρ is to establish a trade-off between these conflicting goals:

1. When we chose ρ very large, the most effective way to decrease J LQR is to use little control, at the expense of a large controlled output.

2. When we chose ρ very small, the most effective way to decrease J LQR is to obtain a very small controlled output, even if this is achieved at the expense of a large controlled output.

often the optimal LQR problem is defined more generally and consists of finding the control input that minimizes

\begin{equation}
J = \int_{0}^{\infty}(x^{T}Qx+u^{T}Ru)dt
\end{equation}

LQR is a Linear full state feedback controller and it has a quadratic cost function 



where:
$x^{T}Qx$ is the State Cost with weight Q
$u^{T}Ru$ is called the Control Cost with weight R
Basic form of Linear Quadratic Regulator problem.

K gives the optimal state feedback

 <iframe width="1519" height="554" src="https://www.youtube.com/embed/1_UobILf3cc?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

K = place(A,B,p) in MATLAB places the desired closed-loop poles p by computing a state-feedback gain matrix K. All the inputs of the plant are assumed to be control inputs. The length of p must match the row size of A. place works for multi-input systems and is based on the algorithm from [1]. This algorithm uses the extra degrees of freedom to find a solution that minimizes the sensitivity of the closed-loop poles to perturbations in A or B.

Reference

https://www.cds.caltech.edu/~murray/courses/cds110/wi06/lqr.pdf



```python
clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = 1; % pendulum up (s=1)

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];



B = [0; 1/M; 0; s*1/(M*L)];
eig(A)

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];

% play with R by changing the values
% R=1;
R = .0001;

%%
det(ctrb(A,B))

%%
K = lqr(A,B,Q,R);

tspan = 0:.001:10;
if(s==-1)
    y0 = [0; 0; 0; 0];
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[4; 0; 0; 0])),tspan,y0);
elseif(s==1)
    y0 = [-3; 0; pi+.1; 0];
% % [t,y] = ode45(@(t,y)((A-B*K)*(y-[0; 0; pi; 0])),tspan,y0);
    [t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,-K*(y-[1; 0; pi; 0])),tspan,y0);
else
    
end

for k=1:100:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
end

% function dy = pendcart(y,m,M,L,g,d,u)

% [eigvect,eigval]= eig(A-B*K)
% For eigen values 
diag(real(eigval))
% For the correcponding eigen vector 
eigvect(:,1)
% we can see the most stabilizing directions





```

# Intutive Questions
## What is the difference between Kalman filter and LQR?
LQR is a regulator and kalman filter is a estimator. In LQR we assume that we can know the full state vector and the dynamic control system should be controllable. Our aim is to identify the optimal control to apply (generally K) for controlling (stabilizing) the system. 
Where as Kalman filter has to estimate the full state vector then we can pass that info the the controller like LQR.  



```python

```
