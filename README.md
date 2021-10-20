# BICEPS
The Biomimetic Integrated Community Energy and Power System

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
