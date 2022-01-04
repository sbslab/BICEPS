within BICEPS.Combined.ElectricalAndFluid.Examples.COBEE;
model BiomimeticZero
  "SingleFamilyResidentialBuilding with biomimetic control, net zero"
  extends SingleFamilyResidentialBuilding(bld(
    biomimeticControl=true,
    PPV_nominal=4000*3.36,
    PWin_nominal=2000*3.36));
end BiomimeticZero;
