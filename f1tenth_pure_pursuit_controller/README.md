# Pure Pursuit Controller (Group-1)

### Contributers
- Aryaan Sinha
- Harsh Vibhor Sharma
- Utsab Karan

## Simulation Environement Setup
- It is important to setup the simulation env. Use the following links.
- Link 1
- doc link 

## Available Controllers

- **Pure Pursuit Spielberg**
	- It is for the Spielberg map and uses the spielberg_waypoints.csv file.
	- You can always edit the file path in the code to use some other csv file as long as the file 		format is same.
	- The waypoints are generated using the min curvature algorithm.
	- It goes at max speed of 8.
	- To run this node just edit the map path in sim.yaml and change "levine" to "Spielberg_map"
	- Build and run.
	- It has a looping function so the bot keeps going in a loop around the map.
	- same code can be used for any map as long as the waypoints are provided. Just tweak the 		parameters for the best result.
