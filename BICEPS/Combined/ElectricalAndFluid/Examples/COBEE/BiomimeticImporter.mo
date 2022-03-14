within BICEPS.Combined.ElectricalAndFluid.Examples.COBEE;
model BiomimeticImporter
  "SingleFamilyResidentialBuilding with biomimetic control, net importer"
  extends SingleFamilyResidentialBuilding(bld(
    biomimeticControl=true,
    PPV_nominal=3000,
    PWin_nominal=1000));
end BiomimeticImporter;
