# intersection_model

External simulation model that can be linked to SUMO to enable realistic left turning behavior of cyclists at signaled intersections.

The model was implemented and tested using Python3.6.

To run the model, the respective variables in `settings.py` need to be set. 

Note that when evaluating the model's hyperparameters (`settings.eval_hyperparameters=True`), the sumoBinary should be set to `path/to/sumo/bin/sumo` to speed up the evaluation. Furthermore, it was found that the model performs sufficiently faster when being started from outside the IDE.
