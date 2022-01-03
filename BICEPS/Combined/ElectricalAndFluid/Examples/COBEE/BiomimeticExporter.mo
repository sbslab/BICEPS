within BICEPS.Combined.ElectricalAndFluid.Examples.COBEE;
model BiomimeticExporter
  "SingleFamilyResidentialBuilding with biomimetic control, net exporter"
  extends SingleFamilyResidentialBuilding(bld(
    biomimeticControl=true,
    PPV_nominal=4000*9,
    PWin_nominal=2000*9));
end BiomimeticExporter;
