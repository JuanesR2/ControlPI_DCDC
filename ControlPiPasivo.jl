using LinearAlgebra, DifferentialEquations, Plots

# Define system matrices and parameters with Float64 type
A = [0.0 0.0;
     0.0 -0.3]

B = [0.0 -1.0;
     1.0 0.0]

d = [1.0;
     0.0]

u_ref = 0.5

# Compute x_ref 
x_ref = -(A + u_ref * B) \ d

kp = 1  #
ki = 10

# Define y_d(x) function
function y_d(x)
    (x - x_ref)' * (kp * A + ki * B) * x_ref
end

# Define u_control(x_e) function
function u_control(x_e)
    -kp * y_d(x_e[1:2]) - ki * x_e[3]
end

# Define convertidor! function for in-place derivative calculation
function convertidor!(du, u, p, t)
    x_e = u
    du[1:2] .= A * x_e[1:2] + u_control(x_e) * B * x_e[1:2] + d
    du[3] = y_d(x_e[1:2])
end

# Initial conditions for differential equation
u0 = [0.0; 0.0; 0.0]  # Initial state [x1(0); x2(0); x3(0)]

# Time span for simulation
tspan = (0.0, 20.0)

# Create the ODE problem using convertidor! function
prob = ODEProblem(convertidor!, u0, tspan)

# Solve the ODE problem 
sol = solve(prob)

# # Extract the solution
tode = sol.t  # Time points
xode = hcat(sol.u...)'

# # Plot the results using Plots.jl
plot(tode, xode, xlabel="Time", ylabel="State Variables", label=["x1" "x2" "u_control"])