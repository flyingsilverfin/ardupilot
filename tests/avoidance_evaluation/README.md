## Contents

In this directory you'll find:
1. `copter_parameters/` directory containing parameters for SITL instances. Modify the file `copter_params.parm`, then run `generate_params.py`
1. `experiments/` directory containing experimental setup files. `plan.json` is fed into `experiment_runner.py` to start and execute an experiment
1. `adsb_connector.py` this script simply forwards packets between SITL instances. Currently doesn't have connection removals by age, might be good to restart to clear connections once in a while
1. `experiment_runner.py` reads a `plan.json` file and executes commands indicated. Spawns the required number of SITL (spawned off) and Dronekit (internal) instances required.
1. `generate_experiment.py` a larger script which generates experiments. It's a bit of a mess since it evolved from an interactive script to help fill out a skeleton by hand to a fully automatic one which generates a given type of collision. Explained further below.
1. `log_analyzer.py` Reads logs produced by each SITL instance, and finds the closest each copter gets to another. This could be extended in the future. I might add a component which spits out XML path files for a graphical view of flight paths in google earth - mission planner already has a feature doing this for one log a time I believe!

I've tried to document these as I could in the code. The most complicated is likely the `generate_experiment.py` script.