![Build Status](https://github.com/sbslab/BICEPS/actions/workflows/formatting.yml/badge.svg)

# BICEPS Modelica Library
An open-source Modelica library for Biomimetic Integrated Community Energy and Power System (BICEPS)

## Description
This repository contains a Modelica library for modeling heterogenous community energy systems -- 
involving thermo-fluid, electrical, and biofuel systems -- and their controls.

## Setup

### Main Library
This model uses the Modelica Buildings Library Version 8.0.0, which can be downloaded here:
https://simulationresearch.lbl.gov/modelica/download.html.

LBNL recommends to install the offically released library in a different location than where models
are developed. Their install guide can be seen here: https://simulationresearch.lbl.gov/modelica/installLibrary.html.

### BuildingsPy
The unit tests require `buildingspy`. Information on installation and use can be found here: 
https://simulationresearch.lbl.gov/modelica/buildingspy/


## Running Unit Tests

Unit tests can be run from the DES folder that contains the top level `package.mo` with the following script:

`python ../bin/runUnitTests.py`

The above script will simulate all unit tests for the entire package. Individual packages can be simulated 
separately with the `-s` option, as in:

`python ../bin/runUnitTests.py -s BICEPS.Fluid.Examples`

# Publications

Hinkelman, Kathryn, Wangda Zuo, Jing Wang, Sen Huang, Michael Wetter. 2022. “Ecosystem-Level Biomimicry for the Built Environment: Adopting Systems Ecology Principles for the Control of Heterogeneous Energy Systems.” The 5th International Conference on Building Energy and Environment. Montreal, Canada. 10.1007/978-981-19-9822-5_284.
