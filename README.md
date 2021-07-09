# intersection_model

External simulation model that can be linked to SUMO to enable realistic left turning behavior of cyclists at a single signaled intersection. To enable realisitc bicycle behavior at multiple intersections, multiple intersection_model instances need to be run. In that case, the given implementation needs to be forked so that the intersection_model does not start the SUMO simulation when executed, but is linked to a running SUMO simulation. 

The model was implemented and tested using Python3.6.

To run the model, the respective variables in `settings.py` need to be set. Then run `main.py`. 

By default, the model runs an exemplary scenario featuring the intersection Alexanderstr./Karl-Marx-Str. in Berlin.  

Note that when evaluating the model's hyperparameters (`settings.eval_hyperparameters=True`), the sumoBinary should be set to `path/to/sumo/bin/sumo` to speed up the evaluation. Furthermore, it was found that the model performs sufficiently faster when being started from outside the IDE.
