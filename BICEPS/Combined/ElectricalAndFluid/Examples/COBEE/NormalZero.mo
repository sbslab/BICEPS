within BICEPS.Combined.ElectricalAndFluid.Examples.COBEE;
model NormalZero
  "SingleFamilyResidentialBuilding with normal control, net zero"
  extends BiomimeticZero(bld(
    biomimeticControl=false));
end NormalZero;
