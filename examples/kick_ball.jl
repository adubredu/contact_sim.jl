using contact_walk 
using Plots

window_xmin = -3; window_xmax = 3
window_ymin = -0.1; window_ymax = 1.1 
rampref = [-3. 3.; -0.015 -0.015]

plot(rampref[1,:], rampref[2,:],  legend=false, linecolor=:purple, linewidth=4,xlims=[window_xmin, window_xmax], ylims=[window_ymin, window_ymax])

