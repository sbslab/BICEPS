within BICEPS.Combined.ElectricalAndFluid.Examples.COBEE;
model NormalImporter
  "SingleFamilyResidentialBuilding with normal control, net importer"
  extends BiomimeticImporter(bld(
    biomimeticControl=false));
end NormalImporter;
