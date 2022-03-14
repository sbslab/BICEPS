within BICEPS.Combined.ElectricalAndFluid.Examples.COBEE;
model NormalExporter
  "SingleFamilyResidentialBuilding with normal control, net exporter"
  extends BiomimeticExporter(bld(
    biomimeticControl=false));
end NormalExporter;
