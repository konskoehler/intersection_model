junction_id = 'junc_01'

pathfinding_distribution = [57, 43]  # cf. thesis 5.3.2
#pathfinding_distribution = [100, 0]

directory = './example_scenario'
cnfg_file = 'alex.sumocfg'
network_file = directory + '/alex.net.xml'

sumoBinary = "path/to/sumo/bin/sumo-gui"
#sumoBinary = "path/to/sumo/bin/sumo"

eval_hyperparameters = False  # Flag to indicate, whether grid search for hyperparameter evaluation shall be obtained
