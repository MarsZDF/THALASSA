# THALASSA
THALASSA stands for Technologies for Hazard Avoidance and Landing for Autonomous Spacecraft with Situational Awareness. THALASSA was developed with the general scope of developing, testing, and eventually deploying autonomous navigation technologies. The package has been developed to be able to control as many aspects as possible of the simulations.

In this first iteration, THALASSA is built upon two software tools: MATLAB and Blender.
MATLAB is used for the generation, analysis and visualisation of numerical data (PATH_ESTIMATION_FILTER folder). 
Blender is used to generate synthetic images from MATLAB's numerical data (SENSOR folder).  

THALASSA currently has five distinct modules, named blocks. 
These are Path, Sensor, Estimation, Filtering and Decision. 

Path generates a set of states. 
Sensor builds synthetic sensor acquisitions from the Path states. 
Estimation converts raw data into state information. 
Filtering pools together various information to either obtain new actionable information or reduce the uncertainty associated to the initial one. 
Decision refers to control actions, which have not been implemented as of now. 

For more information, and for a visual reference, please refer to:

Di Fraia, M. Z., Cuartielles, J. P., Felicetti, L., & Chermak, L. (2021, April). Autonomous Visual Trajectory Reconstruction in the Proximity of Small Celestial Objects. In 7th IAA Planetary Defense Conference (p. 164).

The package was developed during Marco Z. Di Fraia's PhD at Cranfield University (sponsored by Thales Alenia Space). 
It is still very experimental, so any feedback/suggestion is more than welcome!:D 
