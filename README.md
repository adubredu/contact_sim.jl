# contact_sim

2D Contact physics simulation by solving Linear Complementarity Problems. 

For more details, see my related paper: https://arxiv.org/abs/2112.08426

## Installation
1. Open your Julia REPL by typing  `julia` in your terminal.
2. Press `]` on your keyboard to enter the package manager
3. Enter command `add https://github.com/adubredu/contact_walk.jl` and press 
`Enter` on your keyboard to install this package.
4. Press the `Backspace` key on your keyboard to return to the REPL

## Usage
Example scripts can be found in the [examples](examples) folder in this repo.

The [block_sim.jl](examples/block_sim.jl) script simulates physics of a 2D square block dropped from a height, as shown below:  

![](media/block_sim.gif)


The [ball_sim.jl](examples/ball_sim.jl) script simulates physics of a 2D ball, as shown below:  

![](media/ball_sim.gif)

The [walker_sim.jl](examples/walker_sim.jl) script simulates physics of a 2D dynamic walker, as shown below. The walking controller uses partial feedback linearization:  

![](media/walker_sim_ctrl.gif)


The [kick_ball_sim.jl](examples/kick_ball_sim.jl) script simulates physics of a 2D dynamic walker kicking a ball, as shown below. The walking controller uses partial feedback linearization.:  

![](media/kick_sim_makie.gif)
