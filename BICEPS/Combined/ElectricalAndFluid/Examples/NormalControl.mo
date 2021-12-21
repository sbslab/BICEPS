within BICEPS.Combined.ElectricalAndFluid.Examples;
model NormalControl
  "SingleFamilyResidentailBuilding with normal control"
  extends SingleFamilyResidentialBuilding(bld(biomimeticControl=false));
  annotation (__Dymola_Commands(
    file="modelica://BICEPS/Resources/Scripts/Dymola/Combined/ElectricalAndFluid/Examples//NormalAndBiomimeticControl.mos"
        "Normal and Biomimetic"), experiment(
      StopTime=2592000,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));
end NormalControl;
