within BICEPS.Combined.ElectricalAndFluid.Examples.COBEE;
model NormalZero
  "SingleFamilyResidentialBuilding with normal control, net zero"
  extends BiomimeticZero(bld(
    biomimeticControl=false));
  annotation (experiment(
      StartTime=1728000,
      StopTime=2419200,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));
end NormalZero;
