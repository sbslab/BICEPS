within BICEPS.Combined.ElectricalAndFluid.Examples.COBEE;
model BiomimeticExporter
  "SingleFamilyResidentialBuilding with biomimetic control, net exporter"
  extends SingleFamilyResidentialBuilding(bld(
    biomimeticControl=true,
    PPV_nominal=4000*7,
    PWin_nominal=2000*7));
  annotation (__Dymola_Commands(
    file="modelica://BICEPS/Resources/Scripts/CaseStudy/COBEE.mos"
        "COBEE"), experiment(
      StartTime=1728000,
      StopTime=2419200,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));
end BiomimeticExporter;
