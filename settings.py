junction_id = 'junc_01'

pathfinding_distribution = [57, 43]  # cf. thesis 5.3.2
#pathfinding_distribution = [100, 0]

directory = 'path/to/im_impl/alex'
cnfg_file = 'alex_2.sumocfg'
network_file = directory + '/alex_2_blunt.net.xml'

#directory = 'path/to/im_impl/warschauer'
#cnfg_file = 'warschauer.sumocfg'
#network_file = directory + '/osm.net.xml'

#directory = 'path/to/im_impl/stralauer'
#cnfg_file = 'stralauer.sumocfg'
#network_file = directory + '/osm.net.xml'

#directory = 'path/to/im_impl/mehringdamm'
#cnfg_file = 'mehringdamm.sumocfg'
#network_file = directory + '/osm.net.xml'

#directory = 'path/to/im_impl/bismarck'
#cnfg_file = 'bismarck.sumocfg'
#network_file = directory + '/osm.net.xml'

sumoBinary = "path/to/sumo/bin/sumo-gui"
#sumoBinary = "path/to/sumo/bin/sumo"

eval_hyperparameters = False  # Flag to indicate, whether grid search for hyperparameter evaluation shall be obtained