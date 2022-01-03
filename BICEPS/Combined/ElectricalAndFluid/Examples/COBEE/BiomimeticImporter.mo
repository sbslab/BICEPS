within BICEPS.Combined.ElectricalAndFluid.Examples.COBEE;
model BiomimeticImporter
  "SingleFamilyResidentialBuilding with biomimetic control, net importer"
  extends SingleFamilyResidentialBuilding(bld(
    biomimeticControl=true,
    PPV_nominal=4000,
    PWin_nominal=2000));
end BiomimeticImporter;
